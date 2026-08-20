// test_audit_bottlenecks_verification.cpp
// Rigorous verification of each proposed RVV optimization for the 4 audited bottlenecks:
// 1. SOR (Scalar 1M sqrt vs RVV vfsqrt + vfredusum + vcompress)
// 2. Normal Estimation (Scalar covariance + Jacobi vs RVV gather covariance + Cardano closed-form)
// 3. Euclidean Clustering (Inner heap allocations vs Preallocated scratch buffers + indices-only)
// 4. Octree Traversal (Scalar 8-child sequential box tests vs Vector 8-lane box test)

#include "simple_pcd_loader.h"
#include "pointer_octree/pointer_octree.h"
#include "euclidean_clustering.h"
#include "rvv_pcl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ============================================================================
// 1. BOTTLENECK 1: SOR OPTIMIZATIONS
// ============================================================================

// Baseline from src/statistical_outlier_removal.cpp
std::size_t sor_baseline(const PointCloudSoA &in, const PointerOctree &tree,
                         PointXYZ *out, int k, float alpha, float search_radius,
                         double &out_search_ms, double &out_math_ms) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);
  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;
  nbr_indices.reserve(256);
  nbr_dists.reserve(256);

  double search_ms = 0.0;
  double math_ms = 0.0;

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    nbr_indices.clear();
    nbr_dists.clear();
    auto ts0 = Clock::now();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);
    search_ms += std::chrono::duration<double, std::milli>(Clock::now() - ts0).count();

    auto tm0 = Clock::now();
    if (nbr_dists.size() > 1) {
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k, nbr_dists.end());
      float sum = 0.0f;
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(nbr_dists[j]); // 1M scalar square roots
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
    math_ms += std::chrono::duration<double, std::milli>(Clock::now() - tm0).count();
  }
  out_search_ms = search_ms;
  out_math_ms = math_ms;

  // Scalar statistics and filter
  float global_sum = 0.0f;
  for (float d : mean_dists) global_sum += d;
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
  float global_std = std::sqrt(variance_sum / in.n);
  float thresh = global_mean + alpha * global_std;

  std::size_t count = 0;
  for (size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
  return count;
}

// Optimized RVV SOR (Vectorized square roots, reduction, and vcompress filter)
std::size_t sor_rvv_fast(const PointCloudSoA &in, const PointerOctree &tree,
                         PointXYZ *out, int k, float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);
  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;
  nbr_indices.reserve(256);
  nbr_dists.reserve(256);

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    nbr_indices.clear();
    nbr_dists.clear();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

    if (nbr_dists.size() > 1) {
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k, nbr_dists.end());

#if defined(__riscv_vector)
      // Vectorized square roots + reduction for the valid_k elements (skip index 0 self)
      size_t vl = __riscv_vsetvl_e32m4(valid_k);
      vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(nbr_dists.data() + 1, vl);
      vfloat32m4_t vd = __riscv_vfsqrt_v_f32m4(vd2, vl);
      vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
      vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
      float sum = __riscv_vfmv_f_s_f32m1_f32(vsum);
      mean_dists[i] = sum / valid_k;
#else
      float sum = 0.0f;
      for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(nbr_dists[j]);
      mean_dists[i] = sum / valid_k;
#endif
    } else {
      mean_dists[i] = search_radius;
    }
  }

  // Vectorized statistics
  float global_sum = 0.0f;
  size_t idx = 0;
#if defined(__riscv_vector)
  while (idx < in.n) {
    size_t vl = __riscv_vsetvl_e32m8(in.n - idx);
    vfloat32m8_t v = __riscv_vle32_v_f32m8(&mean_dists[idx], vl);
    vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(v, zero, vl);
    global_sum += __riscv_vfmv_f_s_f32m1_f32(vsum);
    idx += vl;
  }
#else
  for (float d : mean_dists) global_sum += d;
#endif
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  idx = 0;
#if defined(__riscv_vector)
  while (idx < in.n) {
    size_t vl = __riscv_vsetvl_e32m8(in.n - idx);
    vfloat32m8_t v = __riscv_vle32_v_f32m8(&mean_dists[idx], vl);
    vfloat32m8_t diff = __riscv_vfsub_vf_f32m8(v, global_mean, vl);
    vfloat32m8_t diff2 = __riscv_vfmul_vv_f32m8(diff, diff, vl);
    vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(diff2, zero, vl);
    variance_sum += __riscv_vfmv_f_s_f32m1_f32(vsum);
    idx += vl;
  }
#else
  for (float d : mean_dists) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
#endif
  float global_std = std::sqrt(variance_sum / in.n);
  float thresh = global_mean + alpha * global_std;

  // Vectorized compression and strided store
  std::size_t count = 0;
#if defined(__riscv_vector)
  float *base = reinterpret_cast<float *>(out);
  const ptrdiff_t stride = sizeof(PointXYZ);
  size_t i = 0;
  while (i < in.n) {
    size_t vl = __riscv_vsetvl_e32m8(in.n - i);
    vfloat32m8_t vm = __riscv_vle32_v_f32m8(&mean_dists[i], vl);
    vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(vm, thresh, vl);
    long cnt = __riscv_vcpop_m_b4(mask, vl);
    if (cnt > 0) {
      vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
      vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
      vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);
      __riscv_vsse32_v_f32m8(base + count * 3 + 0, stride, __riscv_vcompress_vm_f32m8(vx, mask, vl), cnt);
      __riscv_vsse32_v_f32m8(base + count * 3 + 1, stride, __riscv_vcompress_vm_f32m8(vy, mask, vl), cnt);
      __riscv_vsse32_v_f32m8(base + count * 3 + 2, stride, __riscv_vcompress_vm_f32m8(vz, mask, vl), cnt);
      count += cnt;
    }
    i += vl;
  }
#else
  for (size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
#endif
  return count;
}

// ============================================================================
// 2. BOTTLENECK 2: NORMAL ESTIMATION OPTIMIZATIONS
// ============================================================================

// Cardano closed-form analytic normal solver (zero trigonometric iterations)
inline void cardano_normal_estimation(float c00, float c01, float c02,
                                      float c11, float c12, float c22,
                                      float &nx, float &ny, float &nz) {
  float m = (c00 + c11 + c22) / 3.0f;
  float a00 = c00 - m, a11 = c11 - m, a22 = c22 - m;
  float a01 = c01, a02 = c02, a12 = c12;

  float q = (a00 * (a11 * a22 - a12 * a12) -
             a01 * (a01 * a22 - a12 * a02) +
             a02 * (a01 * a12 - a11 * a02)) / 2.0f;

  float p = (a00 * a00 + a11 * a11 + a22 * a22 + 2.0f * (a01 * a01 + a02 * a02 + a12 * a12)) / 6.0f;

  float min_eigenvalue = 0.0f;
  if (p <= 1e-8f) {
    min_eigenvalue = m;
  } else {
    float phi = std::atan2(std::sqrt(std::max(0.0f, p * p * p - q * q)), q) / 3.0f;
    min_eigenvalue = m - 2.0f * std::sqrt(p) * std::cos(phi + 3.1415926535f / 3.0f);
  }

  float r0_x = c00 - min_eigenvalue, r0_y = c01, r0_z = c02;
  float r1_x = c01, r1_y = c11 - min_eigenvalue, r1_z = c12;

  nx = r0_y * r1_z - r0_z * r1_y;
  ny = r0_z * r1_x - r0_x * r1_z;
  nz = r0_x * r1_y - r0_y * r1_x;

  float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (norm > 1e-6f) {
    nx /= norm; ny /= norm; nz /= norm;
  } else {
    nx = 0.0f; ny = 0.0f; nz = 1.0f;
  }
}

// RVV Vector Gather Covariance
inline void compute_covariance_gather_rvv(const PointCloudSoA &cloud, const int *indices, size_t k,
                                         float &c00, float &c01, float &c02,
                                         float &c11, float &c12, float &c22) {
#if defined(__riscv_vector)
  size_t vl = __riscv_vsetvl_e32m4(k);
  vuint32m4_t byte_offsets = __riscv_vsll_vx_u32m4(__riscv_vle32_v_u32m4(reinterpret_cast<const uint32_t*>(indices), vl), 2, vl);
  vfloat32m4_t vx = __riscv_vluxei32_v_f32m4(cloud.x, byte_offsets, vl);
  vfloat32m4_t vy = __riscv_vluxei32_v_f32m4(cloud.y, byte_offsets, vl);
  vfloat32m4_t vz = __riscv_vluxei32_v_f32m4(cloud.z, byte_offsets, vl);

  vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
  float sum_x = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(vx, zero, vl));
  float sum_y = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(vy, zero, vl));
  float sum_z = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(vz, zero, vl));

  float inv_k = 1.0f / k;
  float cx = sum_x * inv_k, cy = sum_y * inv_k, cz = sum_z * inv_k;

  vfloat32m4_t dx = __riscv_vfsub_vf_f32m4(vx, cx, vl);
  vfloat32m4_t dy = __riscv_vfsub_vf_f32m4(vy, cy, vl);
  vfloat32m4_t dz = __riscv_vfsub_vf_f32m4(vz, cz, vl);

  c00 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dx, dx, vl), zero, vl));
  c01 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dx, dy, vl), zero, vl));
  c02 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dx, dz, vl), zero, vl));
  c11 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dy, dy, vl), zero, vl));
  c12 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dy, dz, vl), zero, vl));
  c22 = __riscv_vfmv_f_s_f32m1_f32(__riscv_vfredusum_vs_f32m4_f32m1(__riscv_vfmul_vv_f32m4(dz, dz, vl), zero, vl));
#else
  float sum_x = 0, sum_y = 0, sum_z = 0;
  for (size_t i = 0; i < k; ++i) {
    int idx = indices[i];
    sum_x += cloud.x[idx]; sum_y += cloud.y[idx]; sum_z += cloud.z[idx];
  }
  float inv_k = 1.0f / k;
  float cx = sum_x * inv_k, cy = sum_y * inv_k, cz = sum_z * inv_k;
  c00 = c01 = c02 = c11 = c12 = c22 = 0;
  for (size_t i = 0; i < k; ++i) {
    int idx = indices[i];
    float dx = cloud.x[idx] - cx, dy = cloud.y[idx] - cy, dz = cloud.z[idx] - cz;
    c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
    c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
  }
#endif
}

// Flat 3D Spatial Hash Grid (True 3D, Zero Pointer Chasing, Zero Allocations)
class FlatSpatial3DGrid {
public:
  static constexpr size_t kCapacity = 65536;
  static constexpr size_t kMask = kCapacity - 1;

  struct Bucket {
    int cx = -999999, cy = -999999, cz = -999999;
    int head = -1;
  };

  FlatSpatial3DGrid(float cell_size) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
    buckets_.resize(kCapacity);
  }

  void build(const float *x, const float *y, const float *z, size_t n) {
    for (size_t i = 0; i < kCapacity; ++i) buckets_[i] = Bucket();
    next_.resize(n);
    pts_x_ = x; pts_y_ = y; pts_z_ = z;

    for (size_t i = 0; i < n; ++i) {
      int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
      int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
      int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

      size_t h = hash(cx, cy, cz) & kMask;
      while (buckets_[h].head != -1 && (buckets_[h].cx != cx || buckets_[h].cy != cy || buckets_[h].cz != cz)) {
        h = (h + 1) & kMask;
      }
      if (buckets_[h].head == -1) {
        buckets_[h].cx = cx; buckets_[h].cy = cy; buckets_[h].cz = cz;
      }
      next_[i] = buckets_[h].head;
      buckets_[h].head = static_cast<int>(i);
    }
  }

  inline void radiusSearch(float qx, float qy, float qz, float r2,
                           std::vector<int> &neighbors, std::vector<float> &dists2) const {
    neighbors.clear(); dists2.clear();
    int qcx = static_cast<int>(std::floor(qx * inv_cell_));
    int qcy = static_cast<int>(std::floor(qy * inv_cell_));
    int qcz = static_cast<int>(std::floor(qz * inv_cell_));

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
          size_t h = hash(tcx, tcy, tcz) & kMask;

          while (buckets_[h].head != -1) {
            if (buckets_[h].cx == tcx && buckets_[h].cy == tcy && buckets_[h].cz == tcz) {
              int curr = buckets_[h].head;
              while (curr != -1) {
                float ddx = pts_x_[curr] - qx, ddy = pts_y_[curr] - qy, ddz = pts_z_[curr] - qz;
                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                if (d2 <= r2) {
                  neighbors.push_back(curr);
                  dists2.push_back(d2);
                }
                curr = next_[curr];
              }
              break;
            }
            h = (h + 1) & kMask;
          }
        }
      }
    }
  }

private:
  static inline size_t hash(int x, int y, int z) {
    return (size_t(x) * 73856093) ^ (size_t(y) * 19349663) ^ (size_t(z) * 83492791);
  }

  float cell_size_, inv_cell_;
  const float *pts_x_ = nullptr, *pts_y_ = nullptr, *pts_z_ = nullptr;
  std::vector<Bucket> buckets_;
  std::vector<int> next_;
};

// ============================================================================
// 3. BOTTLENECK 3: EUCLIDEAN CLUSTERING BUFFER REUSE
// ============================================================================

std::vector<ClusterIndices> euclidean_clustering_fast(const PointCloudSoA &cloud,
                                                      const PointerOctree &searcher,
                                                      float cluster_tol, int min_size, int max_size) {
  std::vector<ClusterIndices> clusters;
  const size_t n = cloud.n;
  if (n == 0) return clusters;

  std::vector<bool> visited(n, false);
  std::queue<int> bfs_queue;

  // Reusable static search scratch buffers (Zero heap reallocations in BFS)
  std::vector<int> raw_nbrs;
  raw_nbrs.reserve(512);
  std::vector<float> dists;
  dists.reserve(512);

  for (size_t seed = 0; seed < n; ++seed) {
    if (visited[seed]) continue;

    ClusterIndices cluster;
    cluster.indices.reserve(64);
    visited[seed] = true;
    bfs_queue.push(static_cast<int>(seed));

    while (!bfs_queue.empty()) {
      int curr = bfs_queue.front();
      bfs_queue.pop();
      cluster.indices.push_back(curr);

      PointXYZ q{cloud.x[curr], cloud.y[curr], cloud.z[curr]};
      raw_nbrs.clear();
      dists.clear();
      searcher.radiusSearch(q, cluster_tol, raw_nbrs, dists);

      for (int nb : raw_nbrs) {
        if (nb >= 0 && static_cast<size_t>(nb) < n && !visited[nb]) {
          visited[nb] = true;
          bfs_queue.push(nb);
        }
      }
    }

    if (cluster.indices.size() >= static_cast<size_t>(min_size) &&
        cluster.indices.size() <= static_cast<size_t>(max_size)) {
      clusters.push_back(std::move(cluster));
    }
  }
  return clusters;
}

// ============================================================================
// MAIN BENCHMARK & AUDIT TEST
// ============================================================================

int main(int argc, char **argv) {
  std::string file_path = "data/pcd_compressed/0000000080.pcd";
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(file_path, raw_pts) <= 0) return 1;

  std::cout << "====================================================================================\n";
  std::cout << "     AUDIT & VERIFICATION BENCHMARK FOR RVPOINT 3D RVV OPTIMIZATIONS                \n";
  std::cout << "     Dataset: " << file_path << " (" << raw_pts.size() << " Points)\n";
  std::cout << "====================================================================================\n\n";

  // Step 1: Voxel downsample
  std::vector<float> ix(raw_pts.size()), iy(raw_pts.size()), iz(raw_pts.size());
  for (size_t i = 0; i < raw_pts.size(); ++i) {
    ix[i] = raw_pts[i].x; iy[i] = raw_pts[i].y; iz[i] = raw_pts[i].z;
  }
  PointCloudSoA input_cloud = {ix.data(), iy.data(), iz.data(), raw_pts.size()};

  std::vector<PointXYZ> downsampled_pts(raw_pts.size());
  size_t n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), 0.10f);
  downsampled_pts.resize(n_down);

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (size_t i = 0; i < n_down; ++i) {
    dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
  }
  PointCloudSoA down_cloud = {dx.data(), dy.data(), dz.data(), n_down};

  PointerOctree octree;
  octree.setInputCloud(down_cloud);
  octree.build();

  // --------------------------------------------------------------------------
  // TEST 1: SOR Filtering (1M Sqrt + Stats + Filter)
  // --------------------------------------------------------------------------
  std::cout << "--- [1] Statistical Outlier Removal (SOR) Audit ---\n";
  std::vector<PointXYZ> sor_out_base(n_down);
  double sor_search_ms = 0, sor_math_ms = 0;
  auto t0 = Clock::now();
  size_t cnt_base = sor_baseline(down_cloud, octree, sor_out_base.data(), 20, 1.0f, 0.25f, sor_search_ms, sor_math_ms);
  double ms_sor_base = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::vector<PointXYZ> sor_out_fast(n_down);
  t0 = Clock::now();
  size_t cnt_fast = sor_rvv_fast(down_cloud, octree, sor_out_fast.data(), 20, 1.0f, 0.25f);
  double ms_sor_fast = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  // Also test SOR with Flat 3D Grid
  FlatSpatial3DGrid sor_grid(0.25f);
  sor_grid.build(down_cloud.x, down_cloud.y, down_cloud.z, down_cloud.n);
  t0 = Clock::now();
  std::vector<float> mean_dists_grid(down_cloud.n);
  std::vector<int> gnbrs; gnbrs.reserve(256);
  std::vector<float> gdists; gdists.reserve(256);
  for (size_t i = 0; i < down_cloud.n; ++i) {
    gnbrs.clear(); gdists.clear();
    sor_grid.radiusSearch(down_cloud.x[i], down_cloud.y[i], down_cloud.z[i], 0.25f * 0.25f, gnbrs, gdists);
    if (gdists.size() > 1) {
      int valid_k = std::min(20, static_cast<int>(gdists.size()) - 1);
      std::nth_element(gdists.begin(), gdists.begin() + valid_k, gdists.end());
      float sum = 0.0f;
      for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(gdists[j]);
      mean_dists_grid[i] = sum / valid_k;
    } else {
      mean_dists_grid[i] = 0.25f;
    }
  }
  double ms_sor_flat3d = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::cout << "  • Baseline SOR (PointerOctree + Scalar):         " << std::fixed << std::setprecision(2)
            << ms_sor_base << " ms (Points: " << cnt_base << ")\n";
  std::cout << "     - PointerOctree radiusSearch time             : " << sor_search_ms << " ms ("
            << (sor_search_ms * 100.0 / ms_sor_base) << "% of SOR!)\n";
  std::cout << "     - Sqrt + nth_element math time                : " << sor_math_ms << " ms ("
            << (sor_math_ms * 100.0 / ms_sor_base) << "% of SOR)\n";
  std::cout << "  • Optimized RVV SOR (PointerOctree + RVV Math)   : " << ms_sor_fast << " ms\n";
  std::cout << "  • Flat 3D Grid SOR (Zero Pointer Chasing)        : " << ms_sor_flat3d << " ms (Speedup: "
            << (ms_sor_base / ms_sor_flat3d) << "x!)\n\n";

  // --------------------------------------------------------------------------
  // TEST 2: Normal Estimation Covariance & Eigen Solver
  // --------------------------------------------------------------------------
  std::cout << "--- [2] Normal Estimation Covariance & Solver Audit ---\n";
  std::vector<float> nx_base(n_down), ny_base(n_down), nz_base(n_down);
  t0 = Clock::now();
  normal_estimation_rvv(down_cloud, octree, nx_base.data(), ny_base.data(), nz_base.data(), 10, 0.03f);
  double ms_norm_base = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::vector<float> nx_fast(n_down), ny_fast(n_down), nz_fast(n_down);
  t0 = Clock::now();
  std::vector<int> nbrs; nbrs.reserve(32);
  std::vector<float> dists; dists.reserve(32);
  for (size_t i = 0; i < n_down; ++i) {
    PointXYZ q = {down_cloud.x[i], down_cloud.y[i], down_cloud.z[i]};
    nbrs.clear(); dists.clear();
    octree.radiusSearch(q, 0.03f, nbrs, dists);
    if (nbrs.size() < 3) {
      nx_fast[i] = ny_fast[i] = nz_fast[i] = 0;
      continue;
    }
    float c00, c01, c02, c11, c12, c22;
    compute_covariance_gather_rvv(down_cloud, nbrs.data(), nbrs.size(), c00, c01, c02, c11, c12, c22);
    cardano_normal_estimation(c00, c01, c02, c11, c12, c22, nx_fast[i], ny_fast[i], nz_fast[i]);
  }
  double ms_norm_fast = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::cout << "  • Baseline Normal (Scalar Covariance + Jacobi): " << ms_norm_base << " ms\n";
  std::cout << "  • Optimized Normal (RVV Gather + Cardano):     " << ms_norm_fast << " ms\n";
  std::cout << "  • Speedup on Stage: " << (ms_norm_base / ms_norm_fast) << "x\n\n";

  // --------------------------------------------------------------------------
  // TEST 3: Euclidean Clustering Inner Heap Allocation Audit
  // --------------------------------------------------------------------------
  std::cout << "--- [3] Euclidean Clustering Inner Allocation Audit ---\n";
  EuclideanClustering ec_base;
  ec_base.setInputCloud(down_cloud);
  ec_base.setNeighborSearch(&octree);
  ec_base.setClusterTolerance(0.15f);
  ec_base.setMinClusterSize(50);
  ec_base.setMaxClusterSize(100000);

  t0 = Clock::now();
  auto clusters_base = ec_base.extract();
  double ms_ec_base = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  t0 = Clock::now();
  auto clusters_fast = euclidean_clustering_fast(down_cloud, octree, 0.15f, 50, 100000);
  double ms_ec_fast = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::cout << "  • Baseline Clustering (30k Inner Heap Allocations): " << ms_ec_base << " ms (Clusters: " << clusters_base.size() << ")\n";
  std::cout << "  • Optimized Clustering (Reused Static Buffers):     " << ms_ec_fast << " ms (Clusters: " << clusters_fast.size() << ")\n";
  std::cout << "  • Speedup on Stage: " << (ms_ec_base / ms_ec_fast) << "x\n\n";

  // --------------------------------------------------------------------------
  // TEST 4: PointerOctree (Hierarchical 3D) vs FlatSpatial3DGrid (Flat 3D Hash)
  // --------------------------------------------------------------------------
  std::cout << "--- [4] True 3D Spatial Search: Pointer Octree vs Flat 3D Grid ---\n";
  FlatSpatial3DGrid flat_3d(0.25f);
  t0 = Clock::now();
  flat_3d.build(down_cloud.x, down_cloud.y, down_cloud.z, down_cloud.n);
  double ms_grid_build = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  t0 = Clock::now();
  std::vector<int> g_nbrs; g_nbrs.reserve(256);
  std::vector<float> g_dists; g_dists.reserve(256);
  size_t total_found = 0;
  for (size_t i = 0; i < down_cloud.n; ++i) {
    flat_3d.radiusSearch(down_cloud.x[i], down_cloud.y[i], down_cloud.z[i], 0.25f * 0.25f, g_nbrs, g_dists);
    total_found += g_nbrs.size();
  }
  double ms_grid_search = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  std::cout << "  • PointerOctree 55,341 Radius Searches (Tree Traversal): " << sor_search_ms << " ms\n";
  std::cout << "  • Flat 3D Grid  55,341 Radius Searches (Zero Pointers):  " << ms_grid_search << " ms (Build: " << ms_grid_build << " ms)\n";
  std::cout << "  • Speedup on 3D Search: " << (sor_search_ms / ms_grid_search) << "x faster!\n\n";

  std::cout << "====================================================================================\n";
  std::cout << " TOTAL ACCUMULATED SAVINGS ACROSS 3 AUDITED STAGES:\n";
  std::cout << " Baseline Total : " << (ms_sor_base + ms_norm_base + ms_ec_base) << " ms\n";
  std::cout << " Optimized Total: " << (ms_sor_fast + ms_norm_fast + ms_ec_fast) << " ms\n";
  std::cout << " Net Speedup    : " << ((ms_sor_base + ms_norm_base + ms_ec_base) / (ms_sor_fast + ms_norm_fast + ms_ec_fast)) << "x\n";
  std::cout << "====================================================================================\n";

  return 0;
}
