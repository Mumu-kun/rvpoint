#include "morton_pointer_octree.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <riscv_vector.h>

extern "C" uint64_t morton_encode(uint32_t ix, uint32_t iy, uint32_t iz);

namespace rvv_pcl {

MortonPointerOctreeNode::MortonPointerOctreeNode() 
    : min_x(0.0f), min_y(0.0f), min_z(0.0f), max_x(0.0f), max_y(0.0f), max_z(0.0f),
      point_start(0), point_count(0), is_leaf(true) 
{
    for (int i = 0; i < 8; ++i) children[i] = nullptr;
}

MortonPointerOctreeNode::~MortonPointerOctreeNode() {
    for (int i = 0; i < 8; ++i) {
        if (children[i]) delete children[i];
    }
}

MortonPointerOctree::MortonPointerOctree() : root_(nullptr) {}

MortonPointerOctree::~MortonPointerOctree() {
    if (root_) delete root_;
}

void MortonPointerOctree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
}

void MortonPointerOctree::build() {
    if (cloud_.n == 0) return;
    if (root_) {
        delete root_;
        root_ = nullptr;
    }

    // Determine bounding box of the cloud to normalize coordinates for Morton encoding
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

    // Sort cloud points by Morton code
    std::vector<uint64_t> morton_codes(cloud_.n);
    std::vector<int> order(cloud_.n);
    
    float span_x = max_x - min_x;
    float span_y = max_y - min_y;
    float span_z = max_z - min_z;
    
    if (span_x == 0.0f) span_x = 1.0f;
    if (span_y == 0.0f) span_y = 1.0f;
    if (span_z == 0.0f) span_z = 1.0f;

    // Map coordinates to [0, 2097151] (21 bits) for Morton encoding
    const float scale_factor = (float)(1 << 21) - 1.0f;

    for (size_t i = 0; i < cloud_.n; ++i) {
        float nx = (cloud_.x[i] - min_x) / span_x;
        float ny = (cloud_.y[i] - min_y) / span_y;
        float nz = (cloud_.z[i] - min_z) / span_z;
        
        nx = std::max(0.0f, std::min(1.0f, nx));
        ny = std::max(0.0f, std::min(1.0f, ny));
        nz = std::max(0.0f, std::min(1.0f, nz));
        
        uint32_t ix = (uint32_t)(nx * scale_factor);
        uint32_t iy = (uint32_t)(ny * scale_factor);
        uint32_t iz = (uint32_t)(nz * scale_factor);
        
        morton_codes[i] = morton_encode(ix, iy, iz);
        order[i] = (int)i;
    }

    // Sort order based on Morton code
    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        return morton_codes[lhs] < morton_codes[rhs];
    });

    // Populate sorted coordinate arrays
    sorted_x_.resize(cloud_.n);
    sorted_y_.resize(cloud_.n);
    sorted_z_.resize(cloud_.n);
    sorted_indices_.resize(cloud_.n);

    for (size_t i = 0; i < cloud_.n; ++i) {
        int src = order[i];
        sorted_x_[i] = cloud_.x[src];
        sorted_y_[i] = cloud_.y[src];
        sorted_z_[i] = cloud_.z[src];
        sorted_indices_[i] = src; // map sorted -> original
    }

    // Recursively build the octree
    root_ = new MortonPointerOctreeNode();
    root_->min_x = min_x - build_epsilon_;
    root_->min_y = min_y - build_epsilon_;
    root_->min_z = min_z - build_epsilon_;
    root_->max_x = max_x + build_epsilon_;
    root_->max_y = max_y + build_epsilon_;
    root_->max_z = max_z + build_epsilon_;
    root_->point_start = 0;
    root_->point_count = (int)cloud_.n;

    buildParams(root_, 0, (int)cloud_.n, 0);
}

void MortonPointerOctree::buildParams(MortonPointerOctreeNode* node, int start, int count, int depth) {
    if (count <= max_points_per_leaf_ || depth >= max_depth_) {
        node->is_leaf = true;
        node->point_start = start;
        node->point_count = count;
        return;
    }

    node->is_leaf = false;
    float mid_x = (node->min_x + node->max_x) * 0.5f;
    float mid_y = (node->min_y + node->max_y) * 0.5f;
    float mid_z = (node->min_z + node->max_z) * 0.5f;

    // Partition elements in range [start, start + count) locally by child octant to maintain contiguity
    std::vector<int> temp_indices(count);
    for (int i = 0; i < count; ++i) {
        temp_indices[i] = start + i;
    }
    
    std::vector<int> buckets[8];
    for (int idx : temp_indices) {
        int code = 0;
        if (sorted_x_[idx] >= mid_x) code |= 1;
        if (sorted_y_[idx] >= mid_y) code |= 2;
        if (sorted_z_[idx] >= mid_z) code |= 4;
        buckets[code].push_back(idx);
    }
    
    std::vector<float> tx(count), ty(count), tz(count);
    std::vector<int> tindices(count);
    
    int write_idx = 0;
    int child_start[8];
    int child_count[8];
    for (int i = 0; i < 8; ++i) {
        child_start[i] = start + write_idx;
        child_count[i] = (int)buckets[i].size();
        for (int idx : buckets[i]) {
            tx[write_idx] = sorted_x_[idx];
            ty[write_idx] = sorted_y_[idx];
            tz[write_idx] = sorted_z_[idx];
            tindices[write_idx] = sorted_indices_[idx];
            write_idx++;
        }
    }
    
    // Copy binned elements back into global sorted arrays
    for (int i = 0; i < count; ++i) {
        sorted_x_[start + i] = tx[i];
        sorted_y_[start + i] = ty[i];
        sorted_z_[start + i] = tz[i];
        sorted_indices_[start + i] = tindices[i];
    }

    // Recurse for children
    for (int i = 0; i < 8; ++i) {
        if (child_count[i] > 0) {
            node->children[i] = new MortonPointerOctreeNode();
            node->children[i]->min_x = (i & 1) ? mid_x : node->min_x;
            node->children[i]->max_x = (i & 1) ? node->max_x : mid_x;
            node->children[i]->min_y = (i & 2) ? mid_y : node->min_y;
            node->children[i]->max_y = (i & 2) ? node->max_y : mid_y;
            node->children[i]->min_z = (i & 4) ? mid_z : node->min_z;
            node->children[i]->max_z = (i & 4) ? node->max_z : mid_z;
            node->children[i]->point_start = child_start[i];
            node->children[i]->point_count = child_count[i];

            buildParams(node->children[i], child_start[i], child_count[i], depth + 1);
        }
    }
}

static inline bool boxOverlapsSphere(const MortonPointerOctreeNode* node, const PointXYZ& center, float r2) {
    float closest_x = std::max(node->min_x, std::min(center.x, node->max_x));
    float closest_y = std::max(node->min_y, std::min(center.y, node->max_y));
    float closest_z = std::max(node->min_z, std::min(center.z, node->max_z));

    float dx = center.x - closest_x;
    float dy = center.y - closest_y;
    float dz = center.z - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= r2;
}

static inline void get_inds_in_radius_contiguous_sorted_rvv(
    const float* global_x, const float* global_y, const float* global_z,
    const int* global_indices, int start, int count,
    float qx, float qy, float qz, float r2,
    std::vector<int>& out_indices,
    std::vector<float>& out_dists)
{
    // Scalar fallback for tiny leaves to avoid vector execution setup overhead
    if (count < 16) {
        for (int i = 0; i < count; ++i) {
            int idx = start + i;
            float dx = global_x[idx] - qx;
            float dy = global_y[idx] - qy;
            float dz = global_z[idx] - qz;
            float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 <= r2) {
                out_indices.push_back(global_indices[idx]);
                out_dists.push_back(d2);
            }
        }
        return;
    }

    int i = 0;
    while (i < count) {
        size_t remaining = count - i;
        if (remaining >= 32) {
            size_t vl = __riscv_vsetvl_e32m4(16);
            int idx0 = start + i;
            int idx1 = start + i + 16;

            // Load coordinates
            vfloat32m4_t vx0 = __riscv_vle32_v_f32m4(global_x + idx0, vl);
            vfloat32m4_t vx1 = __riscv_vle32_v_f32m4(global_x + idx1, vl);

            vfloat32m4_t vy0 = __riscv_vle32_v_f32m4(global_y + idx0, vl);
            vfloat32m4_t vy1 = __riscv_vle32_v_f32m4(global_y + idx1, vl);

            vfloat32m4_t vz0 = __riscv_vle32_v_f32m4(global_z + idx0, vl);
            vfloat32m4_t vz1 = __riscv_vle32_v_f32m4(global_z + idx1, vl);

            // Subtractions
            vfloat32m4_t dx0 = __riscv_vfsub_vf_f32m4(vx0, qx, vl);
            vfloat32m4_t dx1 = __riscv_vfsub_vf_f32m4(vx1, qx, vl);

            vfloat32m4_t dy0 = __riscv_vfsub_vf_f32m4(vy0, qy, vl);
            vfloat32m4_t dy1 = __riscv_vfsub_vf_f32m4(vy1, qy, vl);

            vfloat32m4_t dz0 = __riscv_vfsub_vf_f32m4(vz0, qz, vl);
            vfloat32m4_t dz1 = __riscv_vfsub_vf_f32m4(vz1, qz, vl);

            // Distances
            vfloat32m4_t d2_0 = __riscv_vfmul_vv_f32m4(dx0, dx0, vl);
            vfloat32m4_t d2_1 = __riscv_vfmul_vv_f32m4(dx1, dx1, vl);

            d2_0 = __riscv_vfmacc_vv_f32m4(d2_0, dy0, dy0, vl);
            d2_1 = __riscv_vfmacc_vv_f32m4(d2_1, dy1, dy1, vl);

            d2_0 = __riscv_vfmacc_vv_f32m4(d2_0, dz0, dz0, vl);
            d2_1 = __riscv_vfmacc_vv_f32m4(d2_1, dz1, dz1, vl);

            // Compares
            vbool8_t mask0 = __riscv_vmfle_vf_f32m4_b8(d2_0, r2, vl);
            vbool8_t mask1 = __riscv_vmfle_vf_f32m4_b8(d2_1, r2, vl);

            size_t match_count0 = __riscv_vcpop_m_b8(mask0, vl);
            size_t match_count1 = __riscv_vcpop_m_b8(mask1, vl);

            if (match_count0 > 0) {
                uint8_t mask_bytes0[2] = {0};
                __riscv_vsm_v_b8(mask_bytes0, mask0, vl);
                float dists_scratch0[16];
                __riscv_vse32_v_f32m4(dists_scratch0, d2_0, vl);
                for (size_t lane = 0; lane < 16; ++lane) {
                    if (mask_bytes0[lane / 8] & (1u << (lane % 8))) {
                        out_indices.push_back(global_indices[idx0 + lane]);
                        out_dists.push_back(dists_scratch0[lane]);
                    }
                }
            }

            if (match_count1 > 0) {
                uint8_t mask_bytes1[2] = {0};
                __riscv_vsm_v_b8(mask_bytes1, mask1, vl);
                float dists_scratch1[16];
                __riscv_vse32_v_f32m4(dists_scratch1, d2_1, vl);
                for (size_t lane = 0; lane < 16; ++lane) {
                    if (mask_bytes1[lane / 8] & (1u << (lane % 8))) {
                        out_indices.push_back(global_indices[idx1 + lane]);
                        out_dists.push_back(dists_scratch1[lane]);
                    }
                }
            }

            i += 32;
        } else {
            size_t vl = __riscv_vsetvl_e32m4(remaining);
            int idx = start + i;

            vfloat32m4_t vx = __riscv_vle32_v_f32m4(global_x + idx, vl);
            vfloat32m4_t vy = __riscv_vle32_v_f32m4(global_y + idx, vl);
            vfloat32m4_t vz = __riscv_vle32_v_f32m4(global_z + idx, vl);

            vfloat32m4_t dx = __riscv_vfsub_vf_f32m4(vx, qx, vl);
            vfloat32m4_t dy = __riscv_vfsub_vf_f32m4(vy, qy, vl);
            vfloat32m4_t dz = __riscv_vfsub_vf_f32m4(vz, qz, vl);

            vfloat32m4_t d2 = __riscv_vfmul_vv_f32m4(dx, dx, vl);
            d2 = __riscv_vfmacc_vv_f32m4(d2, dy, dy, vl);
            d2 = __riscv_vfmacc_vv_f32m4(d2, dz, dz, vl);

            vbool8_t mask = __riscv_vmfle_vf_f32m4_b8(d2, r2, vl);
            size_t match_count = __riscv_vcpop_m_b8(mask, vl);

            if (match_count > 0) {
                uint8_t mask_bytes[4] = {0};
                __riscv_vsm_v_b8(mask_bytes, mask, vl);
                float dists_scratch[32];
                __riscv_vse32_v_f32m4(dists_scratch, d2, vl);
                for (size_t lane = 0; lane < vl; ++lane) {
                    if (mask_bytes[lane / 8] & (1u << (lane % 8))) {
                        out_indices.push_back(global_indices[idx + lane]);
                        out_dists.push_back(dists_scratch[lane]);
                    }
                }
            }

            i += vl;
        }
    }
}

std::size_t MortonPointerOctree::radiusSearch(const PointXYZ& query, float radius,
                                             std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;
    
    float radius_sq = radius * radius;
    if (!boxOverlapsSphere(root_, query, radius_sq)) return 0;
    
    const MortonPointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const MortonPointerOctreeNode* curr = stack[--stack_ptr];

        if (curr->is_leaf) {
            if (curr->point_count > 0) {
                get_inds_in_radius_contiguous_sorted_rvv(
                    sorted_x_.data(), sorted_y_.data(), sorted_z_.data(),
                    sorted_indices_.data(), curr->point_start, curr->point_count,
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

std::size_t MortonPointerOctree::radiusSearchScalar(const PointXYZ& query, float radius,
                                                   std::vector<int>& indices, std::vector<float>& dists) const {
    indices.clear();
    dists.clear();
    if (!root_) return 0;

    float radius_sq = radius * radius;
    if (!boxOverlapsSphere(root_, query, radius_sq)) return 0;

    const MortonPointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = root_;

    while (stack_ptr > 0) {
        const MortonPointerOctreeNode* curr = stack[--stack_ptr];

        if (curr->is_leaf) {
            for (int i = 0; i < curr->point_count; ++i) {
                int idx = curr->point_start + i;
                float dx = sorted_x_[idx] - query.x;
                float dy = sorted_y_[idx] - query.y;
                float dz = sorted_z_[idx] - query.z;
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 <= radius_sq) {
                    indices.push_back(sorted_indices_[idx]);
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
