#include "include/rvv_pcl.h"

#include <algorithm>
#include <limits>

namespace rvv_pcl {

OctreeNeighborSearch::Node::~Node() {
  for (Node *child : children) {
    delete child;
  }
}

OctreeNeighborSearch::OctreeNeighborSearch() = default;

OctreeNeighborSearch::~OctreeNeighborSearch() { delete root_; }

void OctreeNeighborSearch::setMaxDepth(int depth) { maxDepth_ = depth; }

void OctreeNeighborSearch::setLeafCapacity(int capacity) { leafCapacity_ = capacity; }

void OctreeNeighborSearch::buildIndex() { buildTree(); }

void OctreeNeighborSearch::buildTree() {
  delete root_;
  root_ = nullptr;
  if (!pointCloud_ || pointCloud_->empty()) {
    return;
  }

  root_ = new Node();
  RVVHelper::computeBoundingBox(*pointCloud_, root_->min_x, root_->min_y, root_->min_z,
                                root_->max_x, root_->max_y, root_->max_z);
  root_->max_x += buildEpsilon_;
  root_->max_y += buildEpsilon_;
  root_->max_z += buildEpsilon_;

  for (std::size_t i = 0; i < pointCloud_->size(); ++i) {
    insertPoint(root_, static_cast<int>(i), 0);
  }
}

void OctreeNeighborSearch::insertPoint(Node *node, int pointIndex, int depth) {
  if (node->is_leaf) {
    node->indices.push_back(pointIndex);
    if (static_cast<int>(node->indices.size()) > leafCapacity_ && depth < maxDepth_) {
      subdivide(node, depth);
    }
    return;
  }

  const PointXYZ point = pointCloud_->point(static_cast<std::size_t>(pointIndex));
  const float mid_x = (node->min_x + node->max_x) * 0.5f;
  const float mid_y = (node->min_y + node->max_y) * 0.5f;
  const float mid_z = (node->min_z + node->max_z) * 0.5f;
  int code = 0;
  if (point.x >= mid_x) {
    code |= 1;
  }
  if (point.y >= mid_y) {
    code |= 2;
  }
  if (point.z >= mid_z) {
    code |= 4;
  }

  insertPoint(node->children[code], pointIndex, depth + 1);
}

void OctreeNeighborSearch::subdivide(Node *node, int depth) {
  node->is_leaf = false;
  const float mid_x = (node->min_x + node->max_x) * 0.5f;
  const float mid_y = (node->min_y + node->max_y) * 0.5f;
  const float mid_z = (node->min_z + node->max_z) * 0.5f;

  for (int i = 0; i < 8; ++i) {
    node->children[i] = new Node();
    node->children[i]->min_x = (i & 1) ? mid_x : node->min_x;
    node->children[i]->max_x = (i & 1) ? node->max_x : mid_x;
    node->children[i]->min_y = (i & 2) ? mid_y : node->min_y;
    node->children[i]->max_y = (i & 2) ? node->max_y : mid_y;
    node->children[i]->min_z = (i & 4) ? mid_z : node->min_z;
    node->children[i]->max_z = (i & 4) ? node->max_z : mid_z;
  }

  const std::vector<int> existing = node->indices;
  node->indices.clear();
  for (const int index : existing) {
    insertPoint(node, index, depth);
  }
}

bool OctreeNeighborSearch::boxOverlapsSphere(const Node *node, const PointXYZ &center,
                                             float r2) {
  float dist_sq = 0.0f;
  if (center.x < node->min_x) {
    dist_sq += (center.x - node->min_x) * (center.x - node->min_x);
  } else if (center.x > node->max_x) {
    dist_sq += (center.x - node->max_x) * (center.x - node->max_x);
  }
  if (center.y < node->min_y) {
    dist_sq += (center.y - node->min_y) * (center.y - node->min_y);
  } else if (center.y > node->max_y) {
    dist_sq += (center.y - node->max_y) * (center.y - node->max_y);
  }
  if (center.z < node->min_z) {
    dist_sq += (center.z - node->min_z) * (center.z - node->min_z);
  } else if (center.z > node->max_z) {
    dist_sq += (center.z - node->max_z) * (center.z - node->max_z);
  }
  return dist_sq <= r2;
}

void OctreeNeighborSearch::recursiveSearch(Node *node, const PointXYZ &query,
                                           float radius_sq, std::vector<int> &indices,
                                           std::vector<float> *dists) const {
  if (!boxOverlapsSphere(node, query, radius_sq)) {
    return;
  }

  if (node->is_leaf) {
    std::vector<float> local_dists;
    std::vector<float> &distance_output = dists ? *dists : local_dists;
    RVVHelper::gatherIndicesInRadius(*pointCloud_, node->indices.data(), node->indices.size(),
                                     query.x, query.y, query.z, radius_sq, indices,
                                     distance_output);
    return;
  }

  for (Node *child : node->children) {
    if (child) {
      recursiveSearch(child, query, radius_sq, indices, dists);
    }
  }
}

std::size_t OctreeNeighborSearch::radiusSearch(int queryPointIndex,
                                               std::vector<int> &resultIndices,
                                               std::vector<float> *resultDistances,
                                               int maxResults) const {
  resultIndices.clear();
  if (resultDistances) {
    resultDistances->clear();
  }
  if (!root_ || !pointCloud_ || queryPointIndex < 0 ||
      static_cast<std::size_t>(queryPointIndex) >= pointCloud_->size()) {
    return 0;
  }

  recursiveSearch(root_, queryPoint(queryPointIndex), searchRadius_ * searchRadius_,
                  resultIndices, resultDistances);
  if (resultDistances && !resultIndices.empty()) {
    std::vector<std::pair<float, int>> zipped;
    zipped.reserve(resultIndices.size());
    for (std::size_t i = 0; i < resultIndices.size(); ++i) {
      zipped.emplace_back((*resultDistances)[i], resultIndices[i]);
    }
    std::sort(zipped.begin(), zipped.end());
    for (std::size_t i = 0; i < zipped.size(); ++i) {
      (*resultDistances)[i] = zipped[i].first;
      resultIndices[i] = zipped[i].second;
    }
  }

  if (maxResults > 0 && resultIndices.size() > static_cast<std::size_t>(maxResults)) {
    resultIndices.resize(static_cast<std::size_t>(maxResults));
    if (resultDistances) {
      resultDistances->resize(static_cast<std::size_t>(maxResults));
    }
  }
  return resultIndices.size();
}

} // namespace rvv_pcl
