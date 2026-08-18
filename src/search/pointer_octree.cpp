#include "search/pointer_octree.h"
#include <algorithm>
#include <limits>
#include <cmath>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

PointerOctreeNode::PointerOctreeNode() : min_x(0.0f), min_y(0.0f), min_z(0.0f), max_x(0.0f), max_y(0.0f), max_z(0.0f), is_leaf(true) {
    for (int i = 0; i < 8; ++i) children[i] = nullptr;
}

PointerOctreeNode::~PointerOctreeNode() {
    for (int i = 0; i < 8; ++i) {
        if (children[i]) delete children[i];
    }
}

PointerOctree::PointerOctree() : root_(nullptr) {}

PointerOctree::~PointerOctree() {
    if (root_) delete root_;
}

void PointerOctree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
}

void PointerOctree::build() {
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

    root_ = new PointerOctreeNode();
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

void PointerOctree::buildParams(PointerOctreeNode* node, const std::vector<int>& indices, int depth) {
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
            node->children[i] = new PointerOctreeNode();
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

static inline bool boxOverlapsSphere(const PointerOctreeNode* node, const PointXYZ& center, float r2) {
    float closest_x = std::max(node->min_x, std::min(center.x, node->max_x));
    float closest_y = std::max(node->min_y, std::min(center.y, node->max_y));
    float closest_z = std::max(node->min_z, std::min(center.z, node->max_z));

    float dx = center.x - closest_x;
    float dy = center.y - closest_y;
    float dz = center.z - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= r2;
}

static inline void get_inds_in_radius_contiguous_rvv(
    const float* lx, const float* ly, const float* lz,
    const int* indices, size_t n,
    float qx, float qy, float qz, float r2,
    std::vector<int>& out_indices,
    std::vector<float>& out_dists)
{
#if defined(__riscv_vector)
    if (n < 16) {
        for (size_t i = 0; i < n; ++i) {
            float dx = lx[i] - qx;
            float dy = ly[i] - qy;
            float dz = lz[i] - qz;
            float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 <= r2) {
                out_indices.push_back(indices[i]);
                out_dists.push_back(d2);
            }
        }
        return;
    }

    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(lx + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(ly + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(lz + i, vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
        size_t count = __riscv_vcpop_m_b4(mask, vl);

        if (count > 0) {
            vint32m8_t v_indices = __riscv_vle32_v_i32m8(indices + i, vl);
            
            size_t old_size = out_indices.size();
            out_indices.resize(old_size + count);
            out_dists.resize(old_size + count);

            vint32m8_t v_match_indices = __riscv_vcompress_vm_i32m8(v_indices, mask, vl);
            vfloat32m8_t v_match_dists = __riscv_vcompress_vm_f32m8(d2, mask, vl);

            __riscv_vse32_v_i32m8(out_indices.data() + old_size, v_match_indices, count);
            __riscv_vse32_v_f32m8(out_dists.data() + old_size, v_match_dists, count);
        }

        i += vl;
    }
#else
    for (size_t i = 0; i < n; ++i) {
        float dx = lx[i] - qx;
        float dy = ly[i] - qy;
        float dz = lz[i] - qz;
        float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 <= r2) {
            out_indices.push_back(indices[i]);
            out_dists.push_back(d2);
        }
    }
#endif
}

std::size_t PointerOctree::radiusSearch(const PointXYZ& query, float radius,
                                       std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;
    
    const PointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;
    
    float radius_sq = radius * radius;

    while (stack_ptr > 0) {
        const PointerOctreeNode* curr = stack[--stack_ptr];
        
        if (!boxOverlapsSphere(curr, query, radius_sq)) {
            continue;
        }

        if (curr->is_leaf) {
            size_t n = curr->indices.size();
            if (n > 0) {
                get_inds_in_radius_contiguous_rvv(
                    curr->leaf_x.data(), curr->leaf_y.data(), curr->leaf_z.data(),
                    curr->indices.data(), n,
                    query.x, query.y, query.z, radius_sq,
                    indices, dists
                );
            }
        } else {
            for (int i = 7; i >= 0; --i) {
                if (curr->children[i]) {
                    stack[stack_ptr++] = curr->children[i];
                }
            }
        }
    }
    
    return indices.size();
}

std::size_t PointerOctree::radiusSearchScalar(const PointXYZ& query, float radius,
                                             std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;

    const PointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    float radius_sq = radius * radius;

    while (stack_ptr > 0) {
        const PointerOctreeNode* curr = stack[--stack_ptr];

        if (!boxOverlapsSphere(curr, query, radius_sq)) {
            continue;
        }

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
                if (curr->children[i]) {
                    stack[stack_ptr++] = curr->children[i];
                }
            }
        }
    }

    return indices.size();
}

void PointerOctree::recursiveSearchRVV(PointerOctreeNode* node, const PointXYZ& query, float radius_sq,
                                       std::vector<int>& indices, std::vector<float>& dists) const {
    (void)node; (void)query; (void)radius_sq; (void)indices; (void)dists;
}

void PointerOctree::recursiveSearchScalar(PointerOctreeNode* node, const PointXYZ& query, float radius_sq,
                                         std::vector<int>& indices, std::vector<float>& dists) const {
    (void)node; (void)query; (void)radius_sq; (void)indices; (void)dists;
}

} // namespace rvpoint
