#pragma once
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

struct PacketPointerOctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    PacketPointerOctreeNode* children[8] = {nullptr};
    
    // Range mapping to the global sorted coordinates
    int point_start = 0;
    int point_count = 0;
    
    bool is_leaf = true;

    PacketPointerOctreeNode();
    ~PacketPointerOctreeNode();
};

struct TraversalItem {
    const PacketPointerOctreeNode* node;
    uint8_t active_mask; // Bit i is 1 if query i overlaps the node's ancestor path
};

class PacketPointerOctree {
public:
    PacketPointerOctree();
    ~PacketPointerOctree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();
    
    // Batch query interfaces
    void radiusSearchBatch(const std::vector<PointXYZ>& queries, float radius,
                           std::vector<std::vector<int>>& out_indices,
                           std::vector<std::vector<float>>& out_dists) const;

    void radiusSearchBatchScalar(const std::vector<PointXYZ>& queries, float radius,
                                 std::vector<std::vector<int>>& out_indices,
                                 std::vector<std::vector<float>>& out_dists) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }
    void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
    PointCloudSoA cloud_;
    PacketPointerOctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    // The globally sorted coordinates and index mapping
    std::vector<float> sorted_x_;
    std::vector<float> sorted_y_;
    std::vector<float> sorted_z_;
    std::vector<int> sorted_indices_;

    void buildParams(PacketPointerOctreeNode* node, int start, int count, int depth);
};

} // namespace rvv_pcl
