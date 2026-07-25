#include "packet_pointer_octree.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <riscv_vector.h>

extern "C" uint64_t morton_encode(uint32_t ix, uint32_t iy, uint32_t iz);

namespace rvv_pcl {

PacketPointerOctreeNode::PacketPointerOctreeNode() 
    : min_x(0.0f), min_y(0.0f), min_z(0.0f), max_x(0.0f), max_y(0.0f), max_z(0.0f),
      point_start(0), point_count(0), is_leaf(true) 
{
    for (int i = 0; i < 8; ++i) children[i] = nullptr;
}

PacketPointerOctreeNode::~PacketPointerOctreeNode() {
    for (int i = 0; i < 8; ++i) {
        if (children[i]) delete children[i];
    }
}

PacketPointerOctree::PacketPointerOctree() : root_(nullptr) {}

PacketPointerOctree::~PacketPointerOctree() {
    if (root_) delete root_;
}

void PacketPointerOctree::setInputCloud(const PointCloudSoA& cloud) {
    cloud_ = cloud;
}

void PacketPointerOctree::build() {
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
    root_ = new PacketPointerOctreeNode();
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

void PacketPointerOctree::buildParams(PacketPointerOctreeNode* node, int start, int count, int depth) {
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
            node->children[i] = new PacketPointerOctreeNode();
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

static inline bool boxOverlapsSphere(const PacketPointerOctreeNode* node, const PointXYZ& center, float r2) {
    float closest_x = std::max(node->min_x, std::min(center.x, node->max_x));
    float closest_y = std::max(node->min_y, std::min(center.y, node->max_y));
    float closest_z = std::max(node->min_z, std::min(center.z, node->max_z));

    float dx = center.x - closest_x;
    float dy = center.y - closest_y;
    float dz = center.z - closest_z;

    return (dx * dx + dy * dy + dz * dz) <= r2;
}

static inline uint8_t boxOverlapsSpherePacket(
    const PacketPointerOctreeNode* node,
    const float* qx, const float* qy, const float* qz,
    uint8_t active_mask, float r2, size_t vl)
{
    vfloat32m2_t vqx = __riscv_vle32_v_f32m2(qx, vl);
    vfloat32m2_t vqy = __riscv_vle32_v_f32m2(qy, vl);
    vfloat32m2_t vqz = __riscv_vle32_v_f32m2(qz, vl);

    // closest point on box
    vfloat32m2_t cx = __riscv_vfmin_vf_f32m2(vqx, node->max_x, vl);
    vfloat32m2_t cy = __riscv_vfmin_vf_f32m2(vqy, node->max_y, vl);
    vfloat32m2_t cz = __riscv_vfmin_vf_f32m2(vqz, node->max_z, vl);

    cx = __riscv_vfmax_vf_f32m2(cx, node->min_x, vl);
    cy = __riscv_vfmax_vf_f32m2(cy, node->min_y, vl);
    cz = __riscv_vfmax_vf_f32m2(cz, node->min_z, vl);

    vfloat32m2_t dx = __riscv_vfsub_vv_f32m2(cx, vqx, vl);
    vfloat32m2_t dy = __riscv_vfsub_vv_f32m2(cy, vqy, vl);
    vfloat32m2_t dz = __riscv_vfsub_vv_f32m2(cz, vqz, vl);

    vfloat32m2_t d2 = __riscv_vfmul_vv_f32m2(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m2(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m2(d2, dz, dz, vl);

    vbool16_t overlap = __riscv_vmfle_vf_f32m2_b16(d2, r2, vl);

    uint8_t overlap_mask_bytes[32] = {0};
    __riscv_vsm_v_b16(overlap_mask_bytes, overlap, vl);

    return active_mask & overlap_mask_bytes[0];
}

static inline void get_inds_in_radius_contiguous_sorted_rvv_packet(
    const float* global_x, const float* global_y, const float* global_z,
    const int* global_indices, int start, int count,
    const float* qx, const float* qy, const float* qz,
    uint8_t active_mask, float r2,
    std::vector<std::vector<int>>& out_indices,
    std::vector<std::vector<float>>& out_dists,
    const std::vector<int>& query_indices)
{
    // Scalar fallback for tiny leaf nodes
    if (count < 16) {
        for (int q = 0; q < 8; ++q) {
            if (active_mask & (1 << q)) {
                int qi = query_indices[q];
                float sx = qx[q];
                float sy = qy[q];
                float sz = qz[q];
                for (int i = 0; i < count; ++i) {
                    int idx = start + i;
                    float dx = global_x[idx] - sx;
                    float dy = global_y[idx] - sy;
                    float dz = global_z[idx] - sz;
                    float d2 = dx * dx + dy * dy + dz * dz;
                    if (d2 <= r2) {
                        out_indices[qi].push_back(global_indices[idx]);
                        out_dists[qi].push_back(d2);
                    }
                }
            }
        }
        return;
    }

    // Load leaf points coordinates once into vector registers (LMUL=8)
    int i = 0;
    while (i < count) {
        size_t vl = __riscv_vsetvl_e32m8(count - i);
        int idx = start + i;

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(global_x + idx, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(global_y + idx, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(global_z + idx, vl);

        // Loop over active queries, reusing vx, vy, vz registers
        for (int q = 0; q < 8; ++q) {
            if (active_mask & (1 << q)) {
                int qi = query_indices[q];
                float sx = qx[q];
                float sy = qy[q];
                float sz = qz[q];

                vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, sx, vl);
                vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, sy, vl);
                vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, sz, vl);

                vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
                d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
                d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

                vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
                size_t match_count = __riscv_vcpop_m_b4(mask, vl);

                if (match_count > 0) {
                    uint8_t mask_bytes[32] = {0};
                    __riscv_vsm_v_b4(mask_bytes, mask, vl);
                    float dists_scratch[256];
                    __riscv_vse32_v_f32m8(dists_scratch, d2, vl);
                    for (size_t lane = 0; lane < vl; ++lane) {
                        if (mask_bytes[lane / 8] & (1u << (lane % 8))) {
                            out_indices[qi].push_back(global_indices[idx + lane]);
                            out_dists[qi].push_back(dists_scratch[lane]);
                        }
                    }
                }
            }
        }
        i += vl;
    }
}

static void radiusSearchScalarSubtree(
    const PacketPointerOctreeNode* subtree_root,
    const PointXYZ& query, float radius_sq,
    std::vector<int>& out_indices,
    std::vector<float>& out_dists,
    const std::vector<float>& sorted_x,
    const std::vector<float>& sorted_y,
    const std::vector<float>& sorted_z,
    const std::vector<int>& sorted_indices)
{
    const PacketPointerOctreeNode* stack[64];
    int stack_ptr = 0;
    stack[stack_ptr++] = subtree_root;

    while (stack_ptr > 0) {
        const PacketPointerOctreeNode* curr = stack[--stack_ptr];

        if (curr->is_leaf) {
            for (int i = 0; i < curr->point_count; ++i) {
                int idx = curr->point_start + i;
                float dx = sorted_x[idx] - query.x;
                float dy = sorted_y[idx] - query.y;
                float dz = sorted_z[idx] - query.z;
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 <= radius_sq) {
                    out_indices.push_back(sorted_indices[idx]);
                    out_dists.push_back(d2);
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
}

void PacketPointerOctree::radiusSearchBatch(const std::vector<PointXYZ>& queries, float radius,
                                           std::vector<std::vector<int>>& out_indices,
                                           std::vector<std::vector<float>>& out_dists) const {
    const int Q = (int)queries.size();
    out_indices.resize(Q);
    out_dists.resize(Q);
    for (int q = 0; q < Q; ++q) {
        out_indices[q].clear();
        out_dists[q].clear();
    }

    if (!root_) return;
    float radius_sq = radius * radius;

    // Loop queries in packets of 8
    for (int q_start = 0; q_start < Q; q_start += 8) {
        int q_count = std::min(8, Q - q_start);
        
        alignas(32) float qx[8] = {0.0f};
        alignas(32) float qy[8] = {0.0f};
        alignas(32) float qz[8] = {0.0f};
        std::vector<int> query_indices(8, 0);

        for (int i = 0; i < q_count; ++i) {
            qx[i] = queries[q_start + i].x;
            qy[i] = queries[q_start + i].y;
            qz[i] = queries[q_start + i].z;
            query_indices[i] = q_start + i;
        }

        // Overlap mask at the root
        size_t vl = __riscv_vsetvl_e32m2(q_count);
        uint8_t initial_active_mask = (1 << q_count) - 1;
        uint8_t root_active_mask = boxOverlapsSpherePacket(root_, qx, qy, qz, initial_active_mask, radius_sq, vl);
        if (root_active_mask == 0) {
            continue;
        }

        // Stack-based packet traversal
        TraversalItem stack[64];
        int stack_ptr = 0;
        stack[stack_ptr++] = {root_, root_active_mask};

        while (stack_ptr > 0) {
            TraversalItem curr = stack[--stack_ptr];

            int active_count = __builtin_popcount(curr.active_mask);
            if (active_count <= 7) {
                for (int q = 0; q < 8; ++q) {
                    if (curr.active_mask & (1 << q)) {
                        int qi = query_indices[q];
                        radiusSearchScalarSubtree(
                            curr.node, queries[qi], radius_sq,
                            out_indices[qi], out_dists[qi],
                            sorted_x_, sorted_y_, sorted_z_, sorted_indices_
                        );
                    }
                }
                continue;
            }

            if (curr.node->is_leaf) {
                if (curr.node->point_count > 0) {
                    get_inds_in_radius_contiguous_sorted_rvv_packet(
                        sorted_x_.data(), sorted_y_.data(), sorted_z_.data(),
                        sorted_indices_.data(), curr.node->point_start, curr.node->point_count,
                        qx, qy, qz,
                        curr.active_mask, radius_sq,
                        out_indices, out_dists,
                        query_indices
                    );
                }
            } else {
                for (int i = 7; i >= 0; --i) {
                    if (curr.node->children[i]) {
                        uint8_t child_active_mask = boxOverlapsSpherePacket(
                            curr.node->children[i], qx, qy, qz, curr.active_mask, radius_sq, vl
                        );
                        if (child_active_mask != 0) {
                            stack[stack_ptr++] = {curr.node->children[i], child_active_mask};
                        }
                    }
                }
            }
        }
    }
}

void PacketPointerOctree::radiusSearchBatchScalar(const std::vector<PointXYZ>& queries, float radius,
                                                std::vector<std::vector<int>>& out_indices,
                                                std::vector<std::vector<float>>& out_dists) const {
    const int Q = (int)queries.size();
    out_indices.resize(Q);
    out_dists.resize(Q);
    for (int q = 0; q < Q; ++q) {
        out_indices[q].clear();
        out_dists[q].clear();
    }
    
    if (!root_) return;
    float radius_sq = radius * radius;

    for (int q = 0; q < Q; ++q) {
        const PointXYZ& query = queries[q];
        if (!boxOverlapsSphere(root_, query, radius_sq)) continue;

        const PacketPointerOctreeNode* stack[64];
        int stack_ptr = 0;
        stack[stack_ptr++] = root_;

        while (stack_ptr > 0) {
            const PacketPointerOctreeNode* curr = stack[--stack_ptr];

            if (curr->is_leaf) {
                for (int i = 0; i < curr->point_count; ++i) {
                    int idx = curr->point_start + i;
                    float dx = sorted_x_[idx] - query.x;
                    float dy = sorted_y_[idx] - query.y;
                    float dz = sorted_z_[idx] - query.z;
                    float d2 = dx * dx + dy * dy + dz * dz;
                    if (d2 <= radius_sq) {
                        out_indices[q].push_back(sorted_indices_[idx]);
                        out_dists[q].push_back(d2);
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
    }
}

} // namespace rvv_pcl
