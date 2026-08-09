// caravan_strategies.cpp
// Implementations of the 4 Caravan Query-Pack Strategies for Statistical Outlier Removal

#include "caravan_strategies.h"
#include "rvv_pcl.h"
#include <algorithm>
#include <cmath>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

// Helper: Filter cloud based on computed mean distances and alpha threshold
static std::size_t filter_by_mean_dists(const PointCloudSoA &in,
                                        const std::vector<float> &mean_dists,
                                        PointXYZ *out, float alpha) {
  if (in.n == 0) return 0;
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

// ----------------------------------------------------------------------------
// Strategy 1: Grid-Caravan Bounding Box Pruned SOR
// ----------------------------------------------------------------------------
std::size_t sor_grid_caravan(const PointCloudSoA &in, PointXYZ *out, int k,
                             float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n, 0.0f);

  SpatialHash hash;
  hash.setInputCloud(in, search_radius);
  hash.build();

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    hash.radiusSearch(query, search_radius, nbr_indices, nbr_dists, k + 1);

    if (nbr_dists.size() > 1) {
      std::sort(nbr_dists.begin(), nbr_dists.end());
      float sum = 0.0f;
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(nbr_dists[j]);
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
  }
  return filter_by_mean_dists(in, mean_dists, out, alpha);
}

// ----------------------------------------------------------------------------
// Strategy 2: Pure Hardware SIMD Register Selection SOR
// ----------------------------------------------------------------------------
std::size_t sor_caravan_simd_select(const PointCloudSoA &in, PointXYZ *out,
                                    int k, float alpha) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n, 0.0f);
  const int max_k = k + 1;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  size_t q = 0;
  while (q < in.n) {
    size_t vl = __riscv_vsetvl_e32m1(in.n - q);
    vfloat32m1_t vqx = __riscv_vle32_v_f32m1(&in.x[q], vl);
    vfloat32m1_t vqy = __riscv_vle32_v_f32m1(&in.y[q], vl);
    vfloat32m1_t vqz = __riscv_vle32_v_f32m1(&in.z[q], vl);

    float heap_buf[32][64];
    int heap_cnt[32] = {0};

    for (size_t p = 0; p < in.n; ++p) {
      float px = in.x[p];
      float py = in.y[p];
      float pz = in.z[p];

      vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
      vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
      vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

      vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
      d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
      d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

      float dist_buf[32];
      __riscv_vse32_v_f32m1(dist_buf, d2, vl);

      for (size_t lane = 0; lane < vl; ++lane) {
        float val = dist_buf[lane];
        int &cnt = heap_cnt[lane];
        float *h = heap_buf[lane];

        if (cnt < max_k) {
          h[cnt++] = val;
          std::push_heap(h, h + cnt);
        } else if (val < h[0]) {
          std::pop_heap(h, h + max_k);
          h[max_k - 1] = val;
          std::push_heap(h, h + max_k);
        }
      }
    }

    for (size_t lane = 0; lane < vl; ++lane) {
      float *h = heap_buf[lane];
      int cnt = heap_cnt[lane];
      std::sort_heap(h, h + cnt);
      float sum = 0.0f;
      int valid_k = std::min(k, cnt - 1);
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(h[j]);
      }
      mean_dists[q + lane] = (valid_k > 0) ? (sum / valid_k) : 0.5f;
    }

    q += vl;
  }
#else
  for (size_t i = 0; i < in.n; ++i) {
    std::vector<float> dists(in.n);
    for (size_t j = 0; j < in.n; ++j) {
      float dx = in.x[i] - in.x[j];
      float dy = in.y[i] - in.y[j];
      float dz = in.z[i] - in.z[j];
      dists[j] = dx * dx + dy * dy + dz * dz;
    }
    std::partial_sort(dists.begin(), dists.begin() + k + 1, dists.end());
    float sum = 0.0f;
    for (int j = 1; j <= k; ++j) sum += std::sqrt(dists[j]);
    mean_dists[i] = sum / k;
  }
#endif

  return filter_by_mean_dists(in, mean_dists, out, alpha);
}

// ----------------------------------------------------------------------------
// Strategy 3: Coarse Voxel Centroid Streaming SOR
// ----------------------------------------------------------------------------
std::size_t sor_caravan_voxel(const PointCloudSoA &in, PointXYZ *out, int k,
                              float alpha, float leaf_size) {
  if (in.n == 0) return 0;
  std::vector<PointXYZ> centroids(in.n);
  std::size_t num_centroids = voxel_grid_downsamp_rvv_v2(in, centroids.data(), leaf_size);

  PointCloudSoA vox_soa;
  std::vector<float> vx(num_centroids), vy(num_centroids), vz(num_centroids);
  for (size_t i = 0; i < num_centroids; ++i) {
    vx[i] = centroids[i].x;
    vy[i] = centroids[i].y;
    vz[i] = centroids[i].z;
  }
  vox_soa.x = vx.data();
  vox_soa.y = vy.data();
  vox_soa.z = vz.data();
  vox_soa.n = num_centroids;

  return sor_caravan_simd_select(vox_soa, out, k, alpha);
}

// ----------------------------------------------------------------------------
// Strategy 4: Fixed-Radius Bitmask & Distance Sum Reduction SOR
// ----------------------------------------------------------------------------
std::size_t sor_caravan_radius_bitmask(const PointCloudSoA &in, PointXYZ *out,
                                       float search_radius, float alpha) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n, 0.0f);
  const float r_sq = search_radius * search_radius;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
  size_t q = 0;
  while (q < in.n) {
    size_t vl = __riscv_vsetvl_e32m1(in.n - q);
    vfloat32m1_t vqx = __riscv_vle32_v_f32m1(&in.x[q], vl);
    vfloat32m1_t vqy = __riscv_vle32_v_f32m1(&in.y[q], vl);
    vfloat32m1_t vqz = __riscv_vle32_v_f32m1(&in.z[q], vl);

    float sum_dists[32] = {0.0f};
    int count_nbrs[32] = {0};

    for (size_t p = 0; p < in.n; ++p) {
      if (p == q) continue; // Skip self
      float px = in.x[p];
      float py = in.y[p];
      float pz = in.z[p];

      vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
      vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
      vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

      vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
      d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
      d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

      vbool32_t mask = __riscv_vmfle_vf_f32m1_b32(d2, r_sq, vl);

      float dist_buf[32];
      __riscv_vse32_v_f32m1(dist_buf, d2, vl);

      uint8_t mask_bytes[64] = {0};
      __riscv_vsm_v_b32(mask_bytes, mask, vl);

      for (size_t lane = 0; lane < vl; ++lane) {
        if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
          sum_dists[lane] += std::sqrt(dist_buf[lane]);
          count_nbrs[lane]++;
        }
      }
    }

    for (size_t lane = 0; lane < vl; ++lane) {
      mean_dists[q + lane] = (count_nbrs[lane] > 0) ? (sum_dists[lane] / count_nbrs[lane]) : search_radius;
    }

    q += vl;
  }
#else
  for (size_t i = 0; i < in.n; ++i) {
    float sum = 0.0f;
    int cnt = 0;
    for (size_t j = 0; j < in.n; ++j) {
      if (i == j) continue;
      float dx = in.x[i] - in.x[j];
      float dy = in.y[i] - in.y[j];
      float dz = in.z[i] - in.z[j];
      float d2 = dx * dx + dy * dy + dz * dz;
      if (d2 <= r_sq) {
        sum += std::sqrt(d2);
        cnt++;
      }
    }
    mean_dists[i] = (cnt > 0) ? (sum / cnt) : search_radius;
  }
#endif

  return filter_by_mean_dists(in, mean_dists, out, alpha);
}

} // namespace rvv_pcl
