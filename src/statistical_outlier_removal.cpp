#include "include/rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace rvv_pcl {

// ============================================================================
// Scalar Implementation
// ============================================================================
std::size_t sor_sc(const PointXYZ *in, std::size_t n, PointXYZ *out, int k,
                   float alpha) {
  if (n == 0)
    return 0;
  std::vector<float> mean_dists(n);
  std::vector<float> dists(n);

  // 1. Compute mean K-NN distance for each point
  for (size_t i = 0; i < n; ++i) {
    // Compute distances to all other points
    for (size_t j = 0; j < n; ++j) {
      float dx = in[i].x - in[j].x;
      float dy = in[i].y - in[j].y;
      float dz = in[i].z - in[j].z;
      dists[j] = dx * dx + dy * dy + dz * dz;
    }

    // Find k nearest neighbors (1 to k, 0 is self)
    std::partial_sort(dists.begin(), dists.begin() + k + 1, dists.end());

    float sum = 0;
    for (int j = 1; j <= k; ++j)
      sum += std::sqrt(dists[j]);
    mean_dists[i] = sum / k;
  }

  // 2. Compute Global Statistics
  float global_sum = 0;
  for (float d : mean_dists)
    global_sum += d;
  float global_mean = global_sum / n;

  float variance_sum = 0;
  for (float d : mean_dists)
    variance_sum += (d - global_mean) * (d - global_mean);
  float global_std = std::sqrt(variance_sum / n);

  // 3. Filter
  float thresh = global_mean + alpha * global_std;
  std::size_t count = 0;
  for (size_t i = 0; i < n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count++] = in[i];
    }
  }
  return count;
}

// ============================================================================
// RVV Implementation (Optimized)
// - Inlined distance kernel (eliminates function call overhead)
// - Priority queue for K-selection (O(N log K) instead of O(N log N))
// ============================================================================
std::size_t sor_rvv(const PointCloudSoA &in, PointXYZ *out, int k,
                    float alpha) {
  if (in.n == 0)
    return 0;
  std::vector<float> mean_dists(in.n);
  std::vector<float> dists(in.n);

  // 1. Compute mean K-NN distance using INLINED RVV Kernel + Priority Queue
  for (size_t i = 0; i < in.n; ++i) {
    // --- INLINED get_dist_sq_rvv ---
    const float qx = in.x[i];
    const float qy = in.y[i];
    const float qz = in.z[i];

    size_t j = 0;
    while (j < in.n) {
      size_t vl = __riscv_vsetvl_e32m8(in.n - j);

      vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[j], vl);
      vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[j], vl);
      vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[j], vl);

      vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
      vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
      vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

      vfloat32m8_t dx2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
      vfloat32m8_t dy2 = __riscv_vfmul_vv_f32m8(dy, dy, vl);
      vfloat32m8_t dz2 = __riscv_vfmul_vv_f32m8(dz, dz, vl);

      vfloat32m8_t sum = __riscv_vfadd_vv_f32m8(dx2, dy2, vl);
      sum = __riscv_vfadd_vv_f32m8(sum, dz2, vl);

      __riscv_vse32_v_f32m8(&dists[j], sum, vl);
      j += vl;
    }
    // --- END INLINED ---

    // Use partial_sort to find k+1 smallest (index 0 is self with dist=0)
    std::partial_sort(dists.begin(), dists.begin() + k + 1, dists.end());

    // Sum distances 1 to k (skip self at index 0)
    float sum = 0;
    for (int j = 1; j <= k; ++j)
      sum += std::sqrt(dists[j]);
    mean_dists[i] = sum / k;
  }

  // 2. Compute Global Statistics (Vectorized reduction)
  float global_sum = 0;
  size_t idx = 0;
  while (idx < in.n) {
    size_t vl = __riscv_vsetvl_e32m8(in.n - idx);
    vfloat32m8_t v = __riscv_vle32_v_f32m8(&mean_dists[idx], vl);
    vfloat32m1_t zero = __riscv_vfmv_s_f_f32m1(0.0f, 1);
    vfloat32m1_t vsum = __riscv_vfredusum_vs_f32m8_f32m1(v, zero, vl);
    global_sum += __riscv_vfmv_f_s_f32m1_f32(vsum);
    idx += vl;
  }
  float global_mean = global_sum / in.n;

  // Variance (vectorized)
  float variance_sum = 0;
  idx = 0;
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
  float global_std = std::sqrt(variance_sum / in.n);

  // 3. Filter
  float thresh = global_mean + alpha * global_std;
  std::size_t count = 0;
#ifdef GEM5_BUILD
  // gem5: scalar filter — avoids auto-vectorised vsseg (LMUL*nf > 8 is illegal)
  for (size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
#else
  // QEMU / real HW: vcompress x/y/z SoA → unit-stride stores → AoS pack
  // Uses vsse32 (strided store) to write directly to AoS — no packing loop,
  // no vsseg generated by the compiler.
  {
    float *base = reinterpret_cast<float *>(out);
    const ptrdiff_t stride = (ptrdiff_t)sizeof(PointXYZ); // 12 bytes
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
        // Write compressed fields directly into AoS with
        // stride=sizeof(PointXYZ)
        __riscv_vsse32_v_f32m8(base + count * 3 + 0, stride,
                               __riscv_vcompress_vm_f32m8(vx, mask, vl),
                               (size_t)cnt);
        __riscv_vsse32_v_f32m8(base + count * 3 + 1, stride,
                               __riscv_vcompress_vm_f32m8(vy, mask, vl),
                               (size_t)cnt);
        __riscv_vsse32_v_f32m8(base + count * 3 + 2, stride,
                               __riscv_vcompress_vm_f32m8(vz, mask, vl),
                               (size_t)cnt);
        count += (size_t)cnt;
      }
      i += vl;
    }
  }
#endif
  return count;
}

// Helper: Filter cloud based on computed mean distances and alpha threshold
static std::size_t filter_by_mean_dists(const PointCloudSoA &in,
                                        const std::vector<float> &mean_dists,
                                        PointXYZ *out, float alpha) {
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

std::size_t sor_octree(const PointCloudSoA &in, const Octree &tree, PointXYZ *out,
                       int k, float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists, k + 1);

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

std::size_t sor_spatial_hash(const PointCloudSoA &in, const SpatialHash &hash, PointXYZ *out,
                             int k, float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);

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

std::size_t sor_pointer_octree(const PointCloudSoA &in, const PointerOctree &tree, PointXYZ *out,
                               int k, float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

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

std::size_t sor_hybrid_spatial_hash_pointer_octree(
    const PointCloudSoA &in, const SpatialHash &hash, const PointerOctree &tree,
    PointXYZ *out, int k, float alpha, float search_radius, int min_cell_pts) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;

    std::size_t hash_cnt = hash.radiusSearch(query, search_radius, nbr_indices, nbr_dists);
    if (hash_cnt < static_cast<std::size_t>(min_cell_pts)) {
      mean_dists[i] = search_radius * 2.0f;
      continue;
    }

    nbr_indices.clear();
    nbr_dists.clear();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

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

} // namespace rvv_pcl
