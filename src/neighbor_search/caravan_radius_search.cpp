// caravan_radius_search.cpp
// Caravan Query-Pack Radius Search & Grid-Caravan Strategy Implementation

#include "caravan_radius_search.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

static void caravanTile(
    const float *qx, const float *qy, const float *qz,
    std::size_t vl,
    float px, float py, float pz,
    float r_sq,
    std::size_t point_index,
    std::vector<std::vector<int32_t>> &results,
    std::size_t q_offset
) {
    vfloat32m1_t vqx = __riscv_vle32_v_f32m1(qx, vl);
    vfloat32m1_t vqy = __riscv_vle32_v_f32m1(qy, vl);
    vfloat32m1_t vqz = __riscv_vle32_v_f32m1(qz, vl);

    vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
    vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
    vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

    vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

    vbool32_t mask = __riscv_vmfle_vf_f32m1_b32(d2, r_sq, vl);

    uint8_t mask_bytes[64] = {0};
    __riscv_vsm_v_b32(mask_bytes, mask, vl);

    for (std::size_t lane = 0; lane < vl; ++lane) {
        if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
            results[q_offset + lane].push_back(static_cast<int32_t>(point_index));
        }
    }
}

#endif

void CaravanRadiusSearch::batchRadiusSearch(
    const PointCloudSoA &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
    if (cloud_.n == 0 || queries.n == 0) {
        results.clear();
        return;
    }

    const std::size_t num_points  = cloud_.n;
    const std::size_t num_queries = queries.n;
    results.assign(num_queries, {});

    const float r_sq = radius * radius;

    const float *const px = cloud_.x;
    const float *const py = cloud_.y;
    const float *const pz = cloud_.z;

    const float *const qx = queries.x;
    const float *const qy = queries.y;
    const float *const qz = queries.z;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

    for (std::size_t i = 0; i < num_points; ++i) {
        const float cx = px[i];
        const float cy = py[i];
        const float cz = pz[i];

        std::size_t q = 0;
        while (q < num_queries) {
            std::size_t vl = __riscv_vsetvl_e32m1(num_queries - q);
            caravanTile(qx + q, qy + q, qz + q, vl,
                        cx, cy, cz, r_sq,
                        i, results, q);
            q += vl;
        }
    }

#else

    for (std::size_t q = 0; q < num_queries; ++q) {
        const float qcx = qx[q], qcy = qy[q], qcz = qz[q];
        for (std::size_t i = 0; i < num_points; ++i) {
            const float dx = px[i] - qcx;
            const float dy = py[i] - qcy;
            const float dz = pz[i] - qcz;
            if (dx*dx + dy*dy + dz*dz <= r_sq) {
                results[q].push_back(static_cast<int32_t>(i));
            }
        }
    }

#endif
}

std::size_t CaravanRadiusSearch::radiusSearch(
    const PointXYZ &query,
    float radius,
    std::vector<int32_t> &indices
) const {
    indices.clear();
    if (cloud_.n == 0) return 0;

    const float r_sq = radius * radius;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
    std::size_t i = 0;
    while (i < cloud_.n) {
        std::size_t vl = __riscv_vsetvl_e32m8(cloud_.n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud_.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud_.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud_.z[i], vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, query.x, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, query.y, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, query.z, vl);

        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r_sq, vl);

        uint8_t mask_bytes[64] = {0};
        __riscv_vsm_v_b4(mask_bytes, mask, vl);

        for (std::size_t lane = 0; lane < vl; ++lane) {
            if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
                indices.push_back(static_cast<int32_t>(i + lane));
            }
        }

        i += vl;
    }
#else
    for (std::size_t i = 0; i < cloud_.n; ++i) {
        float dx = cloud_.x[i] - query.x;
        float dy = cloud_.y[i] - query.y;
        float dz = cloud_.z[i] - query.z;
        if (dx*dx + dy*dy + dz*dz <= r_sq) {
            indices.push_back(static_cast<int32_t>(i));
        }
    }
#endif

    return indices.size();
}

// ----------------------------------------------------------------------------
// Strategy 1: Grid-Caravan Bounding Box Pruned SOR
// ----------------------------------------------------------------------------
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

} // namespace rvv_pcl
