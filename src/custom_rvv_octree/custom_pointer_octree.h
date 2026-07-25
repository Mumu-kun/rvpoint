#pragma once
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

struct CustomPointerOctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    CustomPointerOctreeNode* children[8] = {nullptr};
    std::vector<int> indices;
    
    // Contiguous point coordinates for leaf nodes
    std::vector<float> leaf_x;
    std::vector<float> leaf_y;
    std::vector<float> leaf_z;
    
    bool is_leaf = true;

    CustomPointerOctreeNode();
    ~CustomPointerOctreeNode();
};

class CustomPointerOctree {
public:
    CustomPointerOctree();
    ~CustomPointerOctree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();
    
    // Custom instruction based search (calls custom intrinsics)
    std::size_t radiusSearch(const PointXYZ& query, float radius,
                             std::vector<int>& indices, std::vector<float>& dists) const;

    // Pure scalar search
    std::size_t radiusSearchScalar(const PointXYZ& query, float radius,
                                   std::vector<int>& indices, std::vector<float>& dists) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }
    void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
    PointCloudSoA cloud_;
    CustomPointerOctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    void buildParams(CustomPointerOctreeNode* node, const std::vector<int>& indices, int depth);
};

} // namespace rvv_pcl
