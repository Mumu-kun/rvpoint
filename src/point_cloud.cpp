#include "include/rvv_pcl.h"
#include "include/simple_pcd_loader.h"
#include "include/caravan_radius_search.h"

#include <fstream>
#include <sstream>

namespace rvv_pcl {

void PointCloudSoA::reserve(std::size_t capacity) {
  xCoords_.reserve(capacity);
  yCoords_.reserve(capacity);
  zCoords_.reserve(capacity);
  intensities_.reserve(capacity);
}

void PointCloudSoA::clear() {
  xCoords_.clear();
  yCoords_.clear();
  zCoords_.clear();
  intensities_.clear();
  pointCount_ = 0;
}

void PointCloudSoA::resize(std::size_t new_size) {
  xCoords_.resize(new_size);
  yCoords_.resize(new_size);
  zCoords_.resize(new_size);
  intensities_.resize(new_size, 0.0f);
  pointCount_ = new_size;
}

bool PointCloudSoA::loadFromPCD(const std::string &filename) {
  clear();
  std::vector<PointXYZ> points;
  int count = loadPCD(filename, points);
  if (count <= 0) {
    return false;
  }
  assign(points);
  return true;
}

void PointCloudSoA::push_back(const PointXYZ &point, float intensity) {
  xCoords_.push_back(point.x);
  yCoords_.push_back(point.y);
  zCoords_.push_back(point.z);
  intensities_.push_back(intensity);
  pointCount_ = xCoords_.size();
}

PointXYZ PointCloudSoA::point(std::size_t index) const {
  return {xCoords_[index], yCoords_[index], zCoords_[index]};
}

void PointCloudSoA::setPoint(std::size_t index, const PointXYZ &point) {
  xCoords_[index] = point.x;
  yCoords_[index] = point.y;
  zCoords_[index] = point.z;
}

std::vector<PointXYZ> PointCloudSoA::toAoS() const {
  std::vector<PointXYZ> points(pointCount_);
  for (std::size_t i = 0; i < pointCount_; ++i) {
    points[i] = point(i);
  }
  return points;
}

void PointCloudSoA::assign(const std::vector<PointXYZ> &points) {
  resize(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    setPoint(i, points[i]);
  }
}

void NormalCloud::resize(std::size_t n) {
  nx_.resize(n);
  ny_.resize(n);
  nz_.resize(n);
  pointCount_ = n;
}

PointXYZ NormalCloud::normal(std::size_t index) const {
  return {nx_[index], ny_[index], nz_[index]};
}

void NormalCloud::setNormal(std::size_t index, float nx, float ny, float nz) {
  nx_[index] = nx;
  ny_[index] = ny;
  nz_[index] = nz;
}

void Filter::setInput(const PointCloudSoA &input) { input_ = &input; }

void NeighborSearch::setInputCloud(const PointCloudSoA &cloud) { pointCloud_ = &cloud; }

void NeighborSearch::setSearchRadius(float radius) { searchRadius_ = radius; }

PointXYZ NeighborSearch::queryPoint(int queryPointIndex) const {
  return pointCloud_->point(static_cast<std::size_t>(queryPointIndex));
}

int NeighborSearch::nearestNeighborSearch(int queryPointIndex) const {
  if (!pointCloud_ || pointCloud_->empty()) {
    return -1;
  }

  const PointXYZ query = queryPoint(queryPointIndex);
  int best_index = -1;
  float best_dist = 0.0f;
  for (std::size_t i = 0; i < pointCloud_->size(); ++i) {
    if (static_cast<int>(i) == queryPointIndex) {
      continue;
    }
    const PointXYZ candidate = pointCloud_->point(i);
    const float dx = candidate.x - query.x;
    const float dy = candidate.y - query.y;
    const float dz = candidate.z - query.z;
    const float dist = dx * dx + dy * dy + dz * dz;
    if (best_index < 0 || dist < best_dist) {
      best_index = static_cast<int>(i);
      best_dist = dist;
    }
  }
  return best_index;
}

void NeighborSearch::batchRadiusSearch(const PointCloudSoA &queries, float radius,
                                       std::vector<std::vector<int>> &results) const {
  results.resize(queries.size());
  for (std::size_t i = 0; i < queries.size(); ++i) {
    radiusSearch(static_cast<int>(i), results[i], nullptr, 0);
  }
}

void FeatureEstimator::setInputCloud(const PointCloudSoA &cloud) { input_ = &cloud; }

void FeatureEstimator::setK(int k) { k_ = k; }

void FeatureEstimator::setNeighborSearch(NeighborSearch *search) { searcher_ = search; }

bool FeatureEstimator::validateInputs() const {
  return input_ != nullptr && searcher_ != nullptr && k_ > 0;
}

void FeatureEstimator::estimate(FeatureCloud &featureCloud) const {
  if (!validateInputs()) {
    return;
  }

  featureCloud.resize(input_->size());

  std::vector<std::vector<int>> batch_neighbors;
  searcher_->batchRadiusSearch(*input_, searcher_->searchRadius(), batch_neighbors);

  for (std::size_t i = 0; i < input_->size(); ++i) {
    auto &neighbors = batch_neighbors[i];
    if (neighbors.size() > static_cast<std::size_t>(k_)) {
      neighbors.resize(static_cast<std::size_t>(k_));
    }
    computeFeature(static_cast<int>(i), neighbors, featureCloud);
  }
}

void ModelFitter::setInputCloud(const PointCloudSoA &cloud) { input_ = &cloud; }

} // namespace rvv_pcl
