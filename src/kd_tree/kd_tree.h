#pragma once
#include "../include/rvv_pcl.h"
#include <vector>

namespace rvv_pcl {

struct KdTreeNode {
    float split_val;
    int split_axis; // 0 for X, 1 for Y, 2 for Z
    KdTreeNode* left = nullptr;
    KdTreeNode* right = nullptr;
    bool is_leaf = false;
    std::vector<int> indices;
    
    // Contiguous point coordinates for leaf nodes
    std::vector<float> leaf_x;
    std::vector<float> leaf_y;
    std::vector<float> leaf_z;

    KdTreeNode();
    ~KdTreeNode();
};

class KdTree {
public:
    KdTree();
    ~KdTree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();

    std::size_t radiusSearch(const PointXYZ& query, float radius,
                             std::vector<int>& indices, std::vector<float>& dists) const;

    std::size_t radiusSearchScalar(const PointXYZ& query, float radius,
                                   std::vector<int>& indices, std::vector<float>& dists) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }

private:
    PointCloudSoA cloud_;
    KdTreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 20;

    KdTreeNode* buildRecursive(std::vector<int>& indices, int depth);
};

} // namespace rvv_pcl
