#include "search/caravan_pointer_octree.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

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

static inline uint32_t expandBits(uint32_t v) {
  v = (v | (v << 16)) & 0x030000FF;
  v = (v | (v <<  8)) & 0x0300F00F;
  v = (v | (v <<  4)) & 0x030C30C3;
  v = (v | (v <<  2)) & 0x09249249;
  return v;
}

static inline uint32_t morton3D(float x, float y, float z,
                                float min_x, float min_y, float min_z,
                                float inv_range) {
  uint32_t ix = std::clamp(static_cast<uint32_t>((x - min_x) * inv_range * 1023.0f), 0u, 1023u);
  uint32_t iy = std::clamp(static_cast<uint32_t>((y - min_y) * inv_range * 1023.0f), 0u, 1023u);
  uint32_t iz = std::clamp(static_cast<uint32_t>((z - min_z) * inv_range * 1023.0f), 0u, 1023u);
  return expandBits(ix) | (expandBits(iy) << 1) | (expandBits(iz) << 2);
}

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

static void caravanLeafTileKernelIndexed(
    const PointerOctreeNode *leaf,
    const float *tile_qx, const float *tile_qy, const float *tile_qz,
    const size_t *tile_q_indices,
    size_t vl,
    float r_sq,
    std::vector<std::vector<int32_t>> &results
) {
  const size_t leaf_n = leaf->indices.size();
  if (leaf_n == 0) return;

  const float *const lx = leaf->leaf_x.data();
  const float *const ly = leaf->leaf_y.data();
  const float *const lz = leaf->leaf_z.data();
  const int *const l_indices = leaf->indices.data();

  vfloat32m1_t vqx = __riscv_vle32_v_f32m1(tile_qx, vl);
  vfloat32m1_t vqy = __riscv_vle32_v_f32m1(tile_qy, vl);
  vfloat32m1_t vqz = __riscv_vle32_v_f32m1(tile_qz, vl);

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
        results[tile_q_indices[lane]].push_back(pt_idx);
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

  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float min_z = std::numeric_limits<float>::max();
  float max_x = -std::numeric_limits<float>::max();
  float max_y = -std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();

  for (size_t i = 0; i < num_queries; ++i) {
    min_x = std::min(min_x, queries.x[i]);
    min_y = std::min(min_y, queries.y[i]);
    min_z = std::min(min_z, queries.z[i]);
    max_x = std::max(max_x, queries.x[i]);
    max_y = std::max(max_y, queries.y[i]);
    max_z = std::max(max_z, queries.z[i]);
  }

  float range = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 1e-5f});
  float inv_range = 1.0f / range;

  std::vector<uint32_t> morton_codes(num_queries);
  for (size_t i = 0; i < num_queries; ++i) {
    morton_codes[i] = morton3D(queries.x[i], queries.y[i], queries.z[i], min_x, min_y, min_z, inv_range);
  }

  std::vector<size_t> sorted_order(num_queries);
  std::iota(sorted_order.begin(), sorted_order.end(), 0);
  std::sort(sorted_order.begin(), sorted_order.end(), [&](size_t a, size_t b) {
    return morton_codes[a] < morton_codes[b];
  });

  size_t q = 0;
  while (q < num_queries) {
    size_t vl = std::min(static_cast<size_t>(16), num_queries - q);

    float tile_qx[16], tile_qy[16], tile_qz[16];
    size_t tile_indices[16];

    float min_qx = std::numeric_limits<float>::max();
    float max_qx = -std::numeric_limits<float>::max();
    float min_qy = std::numeric_limits<float>::max();
    float max_qy = -std::numeric_limits<float>::max();
    float min_qz = std::numeric_limits<float>::max();
    float max_qz = -std::numeric_limits<float>::max();

    for (size_t lane = 0; lane < vl; ++lane) {
      size_t orig_idx = sorted_order[q + lane];
      tile_indices[lane] = orig_idx;
      float x = queries.x[orig_idx];
      float y = queries.y[orig_idx];
      float z = queries.z[orig_idx];

      tile_qx[lane] = x;
      tile_qy[lane] = y;
      tile_qz[lane] = z;

      min_qx = std::min(min_qx, x);
      max_qx = std::max(max_qx, x);
      min_qy = std::min(min_qy, y);
      max_qy = std::max(max_qy, y);
      min_qz = std::min(min_qz, z);
      max_qz = std::max(max_qz, z);
    }

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
        caravanLeafTileKernelIndexed(curr, tile_qx, tile_qy, tile_qz, tile_indices, vl, r_sq, results);
#else
        for (size_t lane = 0; lane < vl; ++lane) {
          size_t orig_idx = tile_indices[lane];
          float cqx = tile_qx[lane], cqy = tile_qy[lane], cqz = tile_qz[lane];
          for (size_t p = 0; p < curr->indices.size(); ++p) {
            float dx = curr->leaf_x[p] - cqx;
            float dy = curr->leaf_y[p] - cqy;
            float dz = curr->leaf_z[p] - cqz;
            if (dx * dx + dy * dy + dz * dz <= r_sq) {
              results[orig_idx].push_back(curr->indices[p]);
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

} // namespace rvpoint
