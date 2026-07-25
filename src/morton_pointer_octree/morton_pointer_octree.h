#pragma once
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

struct MortonPointerOctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    MortonPointerOctreeNode* children[8] = {nullptr};
    
    // Contiguous indices mapping to the global sorted array
    int point_start = 0;
    int point_count = 0;
    
    bool is_leaf = true;

    MortonPointerOctreeNode();
    ~MortonPointerOctreeNode();
};

class MortonPointerOctree {
public:
    MortonPointerOctree();
    ~MortonPointerOctree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();
    
    // RVV-accelerated search (direct unit-stride global array loads)
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
    MortonPointerOctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    // The globally sorted coordinates and index mapping
    std::vector<float> sorted_x_;
    std::vector<float> sorted_y_;
    std::vector<float> sorted_z_;
    std::vector<int> sorted_indices_;

    void buildParams(MortonPointerOctreeNode* node, int start, int count, int depth);
};

} // namespace rvv_pcl
