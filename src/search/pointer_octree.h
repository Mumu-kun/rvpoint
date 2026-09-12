#pragma once

#include "core/point_types.h"
#include <vector>
#include <cstddef>

namespace rvpoint {

struct PointerOctreeNode {
    float min_x, min_y, min_z;
    float max_x, max_y, max_z;
    PointerOctreeNode* children[8] = {nullptr};
    std::vector<int> indices;

    // Contiguous point coordinates for leaf nodes to optimize vector loading
    std::vector<float> leaf_x;
    std::vector<float> leaf_y;
    std::vector<float> leaf_z;

    bool is_leaf = true;

    PointerOctreeNode();
    ~PointerOctreeNode();
};

class PointerOctree {
public:
    PointerOctree();
    ~PointerOctree();

    void setInputCloud(const PointCloudSoA& cloud);
    void build();

    // Modern Concept-Compliant Spatial Search APIs
    void radius_search(float qx, float qy, float qz, float radius,
                       NeighborQueryResult& result) const;
    void nearest_k_search(float qx, float qy, float qz, int k,
                          NeighborQueryResult& result) const;

    // RVV-accelerated search (scalar tree traversal, unit-stride RVV leaf checks)
    std::size_t radiusSearch(const PointXYZ& query, float radius,
                             std::vector<int>& indices, std::vector<float>& dists) const;

    // Pure scalar search for benchmarking
    std::size_t radiusSearchScalar(const PointXYZ& query, float radius,
                                   std::vector<int>& indices, std::vector<float>& dists) const;

    void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
    void setMaxDepth(int d) { max_depth_ = d; }
    void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

    const PointerOctreeNode* getRoot() const { return root_; }

private:
    PointCloudSoA cloud_;
    PointerOctreeNode* root_ = nullptr;
    int max_points_per_leaf_ = 64;
    int max_depth_ = 8;
    float build_epsilon_ = 1e-4f;

    void buildParams(PointerOctreeNode* node, const std::vector<int>& indices, int depth);

    void recursiveSearchRVV(PointerOctreeNode* node, const PointXYZ& query, float radius_sq,
                            std::vector<int>& indices, std::vector<float>& dists) const;

    void recursiveSearchScalar(PointerOctreeNode* node, const PointXYZ& query, float radius_sq,
                               std::vector<int>& indices, std::vector<float>& dists) const;
};

} // namespace rvpoint
