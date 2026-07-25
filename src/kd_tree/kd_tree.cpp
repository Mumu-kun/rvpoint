#include "kd_tree.h"
#include <algorithm>
#include <limits>
#include <cmath>

namespace rvv_pcl {

// ===========================================
// KdTreeNode Implementation
// ===========================================
KdTreeNode::KdTreeNode() : split_val(0.0f), split_axis(0), is_leaf(true) {}

KdTreeNode::~KdTreeNode() {
    if (left) delete left;
    if (right) delete right;
}

// ===========================================
// KdTree Implementation
// ===========================================
KdTree::KdTree() : root_(nullptr) {}

KdTree::~KdTree() {
    if (root_) delete root_;
}

void KdTree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
}

void KdTree::build() {
    if (cloud_.n == 0) return;
    if (root_) {
        delete root_;
        root_ = nullptr;
    }

    std::vector<int> all_indices(cloud_.n);
    for (size_t i = 0; i < cloud_.n; ++i) {
        all_indices[i] = i;
    }

    root_ = buildRecursive(all_indices, 0);
}

KdTreeNode* KdTree::buildRecursive(std::vector<int>& indices, int depth) {
    if (indices.empty()) return nullptr;

    KdTreeNode* node = new KdTreeNode();

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
        return node;
    }

    node->is_leaf = false;

    // Cycle through X, Y, Z axes
    int axis = depth % 3;
    node->split_axis = axis;

    // Find split coordinate (median along the split axis)
    size_t median_idx = indices.size() / 2;
    std::nth_element(indices.begin(), indices.begin() + median_idx, indices.end(),
        [this, axis](int lhs, int rhs) {
            if (axis == 0) return cloud_.x[lhs] < cloud_.x[rhs];
            if (axis == 1) return cloud_.y[lhs] < cloud_.y[rhs];
            return cloud_.z[lhs] < cloud_.z[rhs];
        });

    int median_pt = indices[median_idx];
    node->split_val = (axis == 0) ? cloud_.x[median_pt] :
                      ((axis == 1) ? cloud_.y[median_pt] : cloud_.z[median_pt]);

    std::vector<int> left_indices(indices.begin(), indices.begin() + median_idx);
    std::vector<int> right_indices(indices.begin() + median_idx, indices.end());

    node->left = buildRecursive(left_indices, depth + 1);
    node->right = buildRecursive(right_indices, depth + 1);

    return node;
}

// Optimized contiguous unit-stride load RVV radius check kernel (LMUL=8)
static inline void get_inds_in_radius_contiguous_rvv(
    const float* lx, const float* ly, const float* lz,
    const int* indices, size_t n,
    float qx, float qy, float qz, float r2,
    std::vector<int>& out_indices,
    std::vector<float>& out_dists)
{
    // Scalar fallback for tiny leaves to avoid vector execution setup overhead
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

        // Unit-stride load coordinate vectors
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(lx + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(ly + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(lz + i, vl);

        // Sub query coordinates
        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

        // Compute d2 = dx*dx + dy*dy + dz*dz
        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        // Compare: d2 <= r2
        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);

        // Count matching points
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
}

std::size_t KdTree::radiusSearch(const PointXYZ& query, float radius,
                                std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;

    float radius_sq = radius * radius;

    // Flattened non-recursive stack-based traversal
    const KdTreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const KdTreeNode* curr = stack[--stack_ptr];

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
            float q_val = (curr->split_axis == 0) ? query.x :
                          ((curr->split_axis == 1) ? query.y : query.z);

            // Splitting Plane Checks - visited in right-to-left order (so left popped first)
            if (q_val + radius > curr->split_val) {
                if (curr->right) {
                    stack[stack_ptr++] = curr->right;
                }
            }
            if (q_val - radius <= curr->split_val) {
                if (curr->left) {
                    stack[stack_ptr++] = curr->left;
                }
            }
        }
    }

    return indices.size();
}

std::size_t KdTree::radiusSearchScalar(const PointXYZ& query, float radius,
                                      std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;

    float radius_sq = radius * radius;

    // Flattened non-recursive stack-based traversal
    const KdTreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const KdTreeNode* curr = stack[--stack_ptr];

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
            float q_val = (curr->split_axis == 0) ? query.x :
                          ((curr->split_axis == 1) ? query.y : query.z);

            if (q_val + radius > curr->split_val) {
                if (curr->right) {
                    stack[stack_ptr++] = curr->right;
                }
            }
            if (q_val - radius <= curr->split_val) {
                if (curr->left) {
                    stack[stack_ptr++] = curr->left;
                }
            }
        }
    }

    return indices.size();
}

} // namespace rvv_pcl
