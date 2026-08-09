// caravan_pointer_octree.cpp
// Implementation of Caravan-PointerOctree Hybrid Algorithm

#include "caravan_pointer_octree.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

void CaravanPointerOctree::setInputCloud(const PointCloudSoA &cloud) {
  cloud_ = cloud;
  ptr_octree_.setInputCloud(cloud);
}

void CaravanPointerOctree::build() {
  ptr_octree_.build();
}

static inline bool tileOverlapsNode(const PointerOctreeNode *node,
                                   float min_qx, float max_qx,
                                   float min_qy, float max_qy,
                                   float min_qz, float max_qz,
                                   float radius) {
  if (node->min_x > max_qx + radius || node->max_x < min_qx - radius) return false;
  if (node->min_y > max_qy + radius || node->max_y < min_qy - radius) return false;
  if (node->min_z > max_qz + radius || node->max_z < min_qz - radius) return false;
  return true;
}

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

static void caravanLeafTileKernel(
    const PointerOctreeNode *leaf,
    const float *qx, const float *qy, const float *qz,
    size_t vl,
    float r_sq,
    size_t q_offset,
    std::vector<std::vector<int32_t>> &results
) {
  const size_t leaf_n = leaf->indices.size();
  if (leaf_n == 0) return;

  const float *const lx = leaf->leaf_x.data();
  const float *const ly = leaf->leaf_y.data();
  const float *const lz = leaf->leaf_z.data();
  const int *const l_indices = leaf->indices.data();

  vfloat32m1_t vqx = __riscv_vle32_v_f32m1(qx, vl);
  vfloat32m1_t vqy = __riscv_vle32_v_f32m1(qy, vl);
  vfloat32m1_t vqz = __riscv_vle32_v_f32m1(qz, vl);

  for (size_t p = 0; p < leaf_n; ++p) {
    float px = lx[p];
    float py = ly[p];
    float pz = lz[p];
    int pt_idx = l_indices[p];

    vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
    vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
    vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

    vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

    vbool32_t mask = __riscv_vmfle_vf_f32m1_b32(d2, r_sq, vl);

    uint8_t mask_bytes[64] = {0};
    __riscv_vsm_v_b32(mask_bytes, mask, vl);

    for (size_t lane = 0; lane < vl; ++lane) {
      if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
        results[q_offset + lane].push_back(pt_idx);
      }
    }
  }
}

#endif

void CaravanPointerOctree::batchRadiusSearch(
    const PointCloudSoA &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
  if (cloud_.n == 0 || queries.n == 0) {
    results.clear();
    return;
  }

  const std::size_t num_queries = queries.n;
  results.assign(num_queries, {});
  const float r_sq = radius * radius;

  const PointerOctreeNode *root = ptr_octree_.getRoot();
  if (!root) return;

  const float *const qx = queries.x;
  const float *const qy = queries.y;
  const float *const qz = queries.z;

  size_t q = 0;
  while (q < num_queries) {
    size_t vl = std::min(static_cast<size_t>(16), num_queries - q);

    // 1. Compute Tile AABB
    float min_qx = qx[q], max_qx = qx[q];
    float min_qy = qy[q], max_qy = qy[q];
    float min_qz = qz[q], max_qz = qz[q];

    for (size_t lane = 1; lane < vl; ++lane) {
      min_qx = std::min(min_qx, qx[q + lane]);
      max_qx = std::max(max_qx, qx[q + lane]);
      min_qy = std::min(min_qy, qy[q + lane]);
      max_qy = std::max(max_qy, qy[q + lane]);
      min_qz = std::min(min_qz, qz[q + lane]);
      max_qz = std::max(max_qz, qz[q + lane]);
    }

    // 2. Tile AABB Pruned PointerOctree Traversal
    const PointerOctreeNode *stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root;

    while (stack_ptr > 0) {
      const PointerOctreeNode *curr = stack[--stack_ptr];

      if (!tileOverlapsNode(curr, min_qx, max_qx, min_qy, max_qy, min_qz, max_qz, radius)) {
        continue;
      }

      if (curr->is_leaf) {
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
        caravanLeafTileKernel(curr, qx + q, qy + q, qz + q, vl, r_sq, q, results);
#else
        for (size_t lane = 0; lane < vl; ++lane) {
          float cqx = qx[q + lane], cqy = qy[q + lane], cqz = qz[q + lane];
          for (size_t p = 0; p < curr->indices.size(); ++p) {
            float dx = curr->leaf_x[p] - cqx;
            float dy = curr->leaf_y[p] - cqy;
            float dz = curr->leaf_z[p] - cqz;
            if (dx * dx + dy * dy + dz * dz <= r_sq) {
              results[q + lane].push_back(curr->indices[p]);
            }
          }
        }
#endif
      } else {
        for (int i = 7; i >= 0; --i) {
          if (curr->children[i]) {
            stack[stack_ptr++] = curr->children[i];
          }
        }
      }
    }

    q += vl;
  }
}

std::size_t CaravanPointerOctree::sorFilter(PointXYZ *out, int k, float alpha, float search_radius) const {
  if (cloud_.n == 0) return 0;
  std::vector<std::vector<int32_t>> batch_results;
  batchRadiusSearch(cloud_, search_radius, batch_results);

  std::vector<float> mean_dists(cloud_.n, 0.0f);
  for (size_t i = 0; i < cloud_.n; ++i) {
    const auto &nbrs = batch_results[i];
    if (nbrs.size() > 1) {
      std::vector<float> dists;
      dists.reserve(nbrs.size());
      for (int nbr_idx : nbrs) {
        float dx = cloud_.x[i] - cloud_.x[nbr_idx];
        float dy = cloud_.y[i] - cloud_.y[nbr_idx];
        float dz = cloud_.z[i] - cloud_.z[nbr_idx];
        dists.push_back(dx * dx + dy * dy + dz * dz);
      }
      std::sort(dists.begin(), dists.end());
      float sum = 0.0f;
      int valid_k = std::min(k, static_cast<int>(dists.size()) - 1);
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(dists[j]);
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  // Filter inliers
  float global_sum = 0.0f;
  for (float d : mean_dists) global_sum += d;
  float global_mean = global_sum / cloud_.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
  float global_std = std::sqrt(variance_sum / cloud_.n);
  float thresh = global_mean + alpha * global_std;

  std::size_t count = 0;
  for (size_t i = 0; i < cloud_.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = cloud_.x[i];
      out[count].y = cloud_.y[i];
      out[count].z = cloud_.z[i];
      count++;
    }
  }
  return count;
}

} // namespace rvv_pcl
