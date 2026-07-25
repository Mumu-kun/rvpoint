#include "custom_pointer_octree.h"
#include "custom_rvv_intrinsics.h"
#include <algorithm>
#include <limits>
#include <cmath>

namespace rvv_pcl {

CustomPointerOctreeNode::CustomPointerOctreeNode() : min_x(0.0f), min_y(0.0f), min_z(0.0f), max_x(0.0f), max_y(0.0f), max_z(0.0f), is_leaf(true) {
    for (int i = 0; i < 8; ++i) children[i] = nullptr;
}

CustomPointerOctreeNode::~CustomPointerOctreeNode() {
    for (int i = 0; i < 8; ++i) {
        if (children[i]) delete children[i];
    }
}

CustomPointerOctree::CustomPointerOctree() : root_(nullptr) {}

CustomPointerOctree::~CustomPointerOctree() {
    if (root_) delete root_;
}

void CustomPointerOctree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
}

void CustomPointerOctree::build() {
    if (cloud_.n == 0) return;
    if (root_) {
        delete root_;
        root_ = nullptr;
    }

    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for (size_t i = 0; i < cloud_.n; ++i) {
        min_x = std::min(min_x, cloud_.x[i]);
        min_y = std::min(min_y, cloud_.y[i]);
        min_z = std::min(min_z, cloud_.z[i]);
        max_x = std::max(max_x, cloud_.x[i]);
        max_y = std::max(max_y, cloud_.y[i]);
        max_z = std::max(max_z, cloud_.z[i]);
    }

    root_ = new CustomPointerOctreeNode();
    root_->min_x = min_x - build_epsilon_;
    root_->min_y = min_y - build_epsilon_;
    root_->min_z = min_z - build_epsilon_;
    root_->max_x = max_x + build_epsilon_;
    root_->max_y = max_y + build_epsilon_;
    root_->max_z = max_z + build_epsilon_;

    std::vector<int> all_indices(cloud_.n);
    for (size_t i = 0; i < cloud_.n; ++i) {
        all_indices[i] = i;
    }

    buildParams(root_, all_indices, 0);
}

void CustomPointerOctree::buildParams(CustomPointerOctreeNode* node, const std::vector<int>& indices, int depth) {
    if (indices.size() <= (size_t)max_points_per_leaf_ || depth >= max_depth_) {
        node->is_leaf = true;
        node->indices = indices;
        
        node->leaf_x.resize(indices.size());
        node->leaf_y.resize(indices.size());
        node->leaf_z.resize(indices.size());
        for (size_t i = 0; i < indices.size(); ++i) {
            int idx = indices[i];
            node->leaf_x[i] = cloud_.x[idx];
            node->leaf_y[i] = cloud_.y[idx];
            node->leaf_z[i] = cloud_.z[idx];
        }
        return;
    }

    node->is_leaf = false;
    float mid_x = (node->min_x + node->max_x) * 0.5f;
    float mid_y = (node->min_y + node->max_y) * 0.5f;
    float mid_z = (node->min_z + node->max_z) * 0.5f;

    std::vector<int> child_indices[8];

    for (int idx : indices) {
        int code = 0;
        if (cloud_.x[idx] >= mid_x) code |= 1;
        if (cloud_.y[idx] >= mid_y) code |= 2;
        if (cloud_.z[idx] >= mid_z) code |= 4;
        child_indices[code].push_back(idx);
    }

    for (int i = 0; i < 8; ++i) {
        if (!child_indices[i].empty()) {
            node->children[i] = new CustomPointerOctreeNode();
            node->children[i]->min_x = (i & 1) ? mid_x : node->min_x;
            node->children[i]->max_x = (i & 1) ? node->max_x : mid_x;
            node->children[i]->min_y = (i & 2) ? mid_y : node->min_y;
            node->children[i]->max_y = (i & 2) ? node->max_y : mid_y;
            node->children[i]->min_z = (i & 4) ? mid_z : node->min_z;
            node->children[i]->max_z = (i & 4) ? node->max_z : mid_z;

            buildParams(node->children[i], child_indices[i], depth + 1);
        }
    }
}

static inline bool boxOverlapsSphere(const CustomPointerOctreeNode* node, const PointXYZ& center, float r2) {
    float closest_x = std::max(node->min_x, std::min(center.x, node->max_x));
    float closest_y = std::max(node->min_y, std::min(center.y, node->max_y));
    float closest_z = std::max(node->min_z, std::min(center.z, node->max_z));

    float dx = center.x - closest_x;
    float dy = center.y - closest_y;
    float dz = center.z - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= r2;
}

static inline void get_inds_in_radius_custom_rvv(
    const float* lx, const float* ly, const float* lz,
    const int* indices, size_t n,
    float qx, float qy, float qz, float r2,
    std::vector<int>& out_indices,
    std::vector<float>& out_dists)
{
    size_t i = 0;
#ifdef USE_CUSTOM_RVV
    float query_arr[3] = { qx, qy, qz };
    while (i < n) {
        size_t vl = 0;
        size_t count = 0;
        
        size_t batch_size = std::min((size_t)32, n - i);
        size_t old_size = out_indices.size();
        
        out_indices.resize(old_size + batch_size);
        out_dists.resize(old_size + batch_size);

        asm volatile (
            // Set vector config: e32, m8
            "vsetvli %0, %5, e32, m8, ta, ma\n\t"
            
            // Load x, y, z coordinates
            "vle32.v v8, (%2)\n\t"
            "vle32.v v16, (%3)\n\t"
            "vle32.v v24, (%4)\n\t"
            
            // Custom instruction: 3D distance -> v8
            // Uses integer register names x8, x16, x24 as aliases for v8, v16, v24
            ".insn r4 0x0b, 6, 0, x8, x16, x24, %6\n\t" // vdist3d.vf v8, v16, v24, query_arr
            
            // Compare: v8 <= r2 -> v0 mask
            "vmfle.vf v0, v8, %9\n\t"
            
            // Count matching points -> count
            "vcpop.m %1, v0\n\t"
            
            // Load indices -> v16
            "vle32.v v16, (%7)\n\t"
            
            // Custom instruction: Store compressed indices -> out_indices pointer (%8)
            ".insn r 0x2b, 1, 0, x0, v16, %8\n\t"
            
            // Custom instruction: Store compressed distances -> out_dists pointer (%10)
            ".insn r 0x2b, 0, 0, x0, v8, %10\n\t"
            
            : "=r" (vl), "=r" (count)
            : "r" (lx + i), "r" (ly + i), "r" (lz + i), "r" (n - i), "r" (query_arr),
              "r" (indices + i), "r" (out_indices.data() + old_size), "f" (r2), "r" (out_dists.data() + old_size)
            : "memory", "v0", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15",
              "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23",
              "v24", "v25", "v26", "v27", "v28", "v29", "v30", "v31"
        );

        out_indices.resize(old_size + count);
        out_dists.resize(old_size + count);

        i += vl;
    }
#else
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(lx + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(ly + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(lz + i, vl);

        vfloat32m8_t d2 = __riscv_vdist3d_vf_f32m8(vx, vy, vz, qx, qy, qz, vl);

        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
        size_t count = __riscv_vcpop_m_b4(mask, vl);

        if (count > 0) {
            vint32m8_t v_indices = __riscv_vle32_v_i32m8(indices + i, vl);
            
            size_t old_size = out_indices.size();
            out_indices.resize(old_size + count);
            out_dists.resize(old_size + count);

            __riscv_vstore_compressed_v_i32m8(out_indices.data() + old_size, v_indices, mask, vl, count);
            __riscv_vstore_compressed_v_f32m8(out_dists.data() + old_size, d2, mask, vl, count);
        }

        i += vl;
    }
#endif
}

std::size_t CustomPointerOctree::radiusSearch(const PointXYZ& query, float radius,
                                             std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;
    
    float radius_sq = radius * radius;
    if (!boxOverlapsSphere(root_, query, radius_sq)) return 0;
    
    const CustomPointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const CustomPointerOctreeNode* curr = stack[--stack_ptr];

        if (curr->is_leaf) {
            size_t n = curr->indices.size();
            if (n > 0) {
                get_inds_in_radius_custom_rvv(
                    curr->leaf_x.data(), curr->leaf_y.data(), curr->leaf_z.data(),
                    curr->indices.data(), n,
                    query.x, query.y, query.z, radius_sq,
                    indices, dists
                );
            }
        } else {
            for (int i = 7; i >= 0; --i) {
                if (curr->children[i] && boxOverlapsSphere(curr->children[i], query, radius_sq)) {
                    stack[stack_ptr++] = curr->children[i];
                }
            }
        }
    }
    
    return indices.size();
}

std::size_t CustomPointerOctree::radiusSearchScalar(const PointXYZ& query, float radius,
                                                   std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;

    float radius_sq = radius * radius;
    if (!boxOverlapsSphere(root_, query, radius_sq)) return 0;

    const CustomPointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const CustomPointerOctreeNode* curr = stack[--stack_ptr];

        if (curr->is_leaf) {
            for (size_t i = 0; i < curr->indices.size(); ++i) {
                float dx = curr->leaf_x[i] - query.x;
                float dy = curr->leaf_y[i] - query.y;
                float dz = curr->leaf_z[i] - query.z;
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 <= radius_sq) {
                    indices.push_back(curr->indices[i]);
                    dists.push_back(d2);
                }
            }
        } else {
            for (int i = 7; i >= 0; --i) {
                if (curr->children[i] && boxOverlapsSphere(curr->children[i], query, radius_sq)) {
                    stack[stack_ptr++] = curr->children[i];
                }
            }
        }
    }

    return indices.size();
}

} // namespace rvv_pcl
