#include "include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace rvv_pcl {

void SORFilter::setMeanK(int k) { meanK_ = k; }

void SORFilter::setStdThreshold(float threshold) { stdThreshold_ = threshold; }

void SORFilter::setNeighborSearch(NeighborSearch *search) { searcher_ = search; }

float SORFilter::computeMeanDistanceToNeighbors(
    int pointIndex, const std::vector<int> &neighborIndices) const {
  if (!input_ || neighborIndices.empty()) {
    return 0.0f;
  }

  const PointXYZ query = input_->point(static_cast<std::size_t>(pointIndex));
  float sum = 0.0f;
  int used = 0;
  for (const int index : neighborIndices) {
    if (index == pointIndex) {
      continue;
    }
    const PointXYZ neighbor = input_->point(static_cast<std::size_t>(index));
    const float dx = neighbor.x - query.x;
    const float dy = neighbor.y - query.y;
    const float dz = neighbor.z - query.z;
    sum += std::sqrt(dx * dx + dy * dy + dz * dz);
    ++used;
    if (used == meanK_) {
      break;
    }
  }

  return used > 0 ? sum / static_cast<float>(used) : 0.0f;
}

void SORFilter::computeGlobalStatistics(const std::vector<float> &meanDistances,
                                        float &meanDist, float &stddev) const {
  if (meanDistances.empty()) {
    meanDist = 0.0f;
    stddev = 0.0f;
    return;
  }

  meanDist = RVVHelper::vsum(meanDistances.data(), meanDistances.size()) /
             static_cast<float>(meanDistances.size());

  std::vector<float> squared(meanDistances.size());
  for (std::size_t i = 0; i < meanDistances.size(); ++i) {
    const float diff = meanDistances[i] - meanDist;
    squared[i] = diff * diff;
  }
  stddev = std::sqrt(RVVHelper::vsum(squared.data(), squared.size()) /
                     static_cast<float>(squared.size()));
}

void SORFilter::filter(PointCloudSoA &output) const {
  output.clear();
  if (!input_ || input_->empty() || !searcher_) {
    return;
  }

  const std::size_t total_points = input_->size();
  std::vector<float> meanDistances(total_points, 0.0f);

  for (std::size_t i = 0; i < total_points; ++i) {
    std::vector<int> neighbors;
    std::vector<float> dists;
    searcher_->radiusSearch(static_cast<int>(i), neighbors, &dists, 0);
    if (neighbors.size() < static_cast<std::size_t>(meanK_ + 1)) {
      std::vector<float> all_dists(total_points, 0.0f);
      const PointXYZ point = input_->point(i);
      RVVHelper::distanceSquared(input_->xData(), input_->yData(), input_->zData(), total_points,
                                 point.x, point.y, point.z, all_dists.data());
      std::vector<int> order(total_points);
      for (std::size_t j = 0; j < total_points; ++j) {
        order[j] = static_cast<int>(j);
      }
      std::partial_sort(order.begin(),
                        order.begin() +
                            std::min<std::size_t>(order.size(), static_cast<std::size_t>(meanK_ + 1)),
                        order.end(), [&all_dists](int a, int b) {
                          return all_dists[a] < all_dists[b];
                        });
      neighbors = order;
    }
    meanDistances[i] = computeMeanDistanceToNeighbors(static_cast<int>(i), neighbors);
  }

  float meanDist = 0.0f;
  float stddev = 0.0f;
  computeGlobalStatistics(meanDistances, meanDist, stddev);
  const float threshold = meanDist + stdThreshold_ * stddev;

  for (std::size_t i = 0; i < total_points; ++i) {
    if (meanDistances[i] <= threshold) {
      output.push_back(input_->point(i));
    }
  }
}

} // namespace rvv_pcl
