#pragma once

#include "core/point_types.h"
#include <vector>
#include <limits>
#include <cstddef>

namespace rvpoint {

struct OctreeNode {
  float min_x, min_y, min_z;
  float max_x, max_y, max_z;
  OctreeNode *children[8] = {nullptr};
  std::vector<int> indices;
  bool is_leaf = true;

  ~OctreeNode();
};

class Octree {
public:
  Octree();
  ~Octree();

  void setInputCloud(const PointCloudSoA &cloud);
  void build();

  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int> &indices, std::vector<float> &dists,
                           int max_nn = 0) const;

  void setMaxPointsPerLeaf(int n) { max_points_per_leaf_ = n; }
  void setMaxDepth(int d) { max_depth_ = d; }
  void setBuildEpsilon(float eps) { build_epsilon_ = eps; }

private:
  PointCloudSoA cloud_;
  OctreeNode *root_ = nullptr;
  int max_points_per_leaf_ = 64;
  int max_depth_ = 8;
  float build_epsilon_ = 1e-4f;

  void buildParams(OctreeNode *node, const std::vector<int> &indices,
                   int depth);
  void recursiveSearch(OctreeNode *node, const PointXYZ &query, float radius_sq,
                       std::vector<int> &indices,
                       std::vector<float> &dists) const;
};

} // namespace rvpoint
