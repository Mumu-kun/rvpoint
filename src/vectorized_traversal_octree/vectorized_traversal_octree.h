#pragma once
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

struct VectorizedTraversalOctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    VectorizedTraversalOctreeNode* children[8] = {nullptr};
    std::vector<int> indices;
    
    // Contiguous child bounding boxes for parallel vector pruning
    alignas(32) float child_min_x[8];
    alignas(32) float child_min_y[8];
    alignas(32) float child_min_z[8];
    alignas(32) float child_max_x[8];
    alignas(32) float child_max_y[8];
    alignas(32) float child_max_z[8];

    // Contiguous point coordinates for leaf nodes to optimize vector loading
    std::vector<float> leaf_x;
    std::vector<float> leaf_y;
    std::vector<float> leaf_z;
    
    bool is_leaf = true;

    VectorizedTraversalOctreeNode();
    ~VectorizedTraversalOctreeNode();
};

class VectorizedTraversalOctree {
public:
    VectorizedTraversalOctree();
    ~VectorizedTraversalOctree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();
    
    // RVV-accelerated search (RVV child pruning + RVV leaf checks)
    std::size_t radiusSearch(const PointXYZ& query, float radius,
                             std::vector<int>& indices, std::vector<float>& dists) const;

    // Pure scalar search for benchmarking
    std::size_t radiusSearchScalar(const PointXYZ& query, float radius,
                                   std::vector<int>& indices, std::vector<float>& dists) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }
    void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
    PointCloudSoA cloud_;
    VectorizedTraversalOctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    void buildParams(VectorizedTraversalOctreeNode* node, const std::vector<int>& indices, int depth);
};

} // namespace rvv_pcl
