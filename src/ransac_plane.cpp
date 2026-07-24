#include "include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace rvv_pcl {

int PlaneModel::minSamples() { return 3; }

bool PlaneModel::computeModel(const PointCloudSoA &points, const std::vector<int> &samples,
                              std::array<float, 4> &coeffs) {
  if (samples.size() < static_cast<std::size_t>(minSamples())) {
    return false;
  }

  const PointXYZ p0 = points.point(static_cast<std::size_t>(samples[0]));
  const PointXYZ p1 = points.point(static_cast<std::size_t>(samples[1]));
  const PointXYZ p2 = points.point(static_cast<std::size_t>(samples[2]));
  const float v1x = p1.x - p0.x;
  const float v1y = p1.y - p0.y;
  const float v1z = p1.z - p0.z;
  const float v2x = p2.x - p0.x;
  const float v2y = p2.y - p0.y;
  const float v2z = p2.z - p0.z;
  float a = v1y * v2z - v1z * v2y;
  float b = v1z * v2x - v1x * v2z;
  float c = v1x * v2y - v1y * v2x;
  const float norm = std::sqrt(a * a + b * b + c * c);
  if (norm < 1e-6f) {
    return false;
  }

  a /= norm;
  b /= norm;
  c /= norm;
  coeffs = {a, b, c, -(a * p0.x + b * p0.y + c * p0.z)};
  return true;
}

float PlaneModel::evaluatePoint(const PointCloudSoA &points, int pointIdx,
                                const std::array<float, 4> &coeffs) {
  const PointXYZ point = points.point(static_cast<std::size_t>(pointIdx));
  return std::abs(coeffs[0] * point.x + coeffs[1] * point.y + coeffs[2] * point.z +
                  coeffs[3]);
}

int PlaneModel::evaluateAll(const PointCloudSoA &points, const std::array<float, 4> &coeffs,
                            std::vector<int> &inliers, int maxInliers, float threshold) {
  inliers.clear();
  const std::size_t n = points.size();
  std::vector<float> dists(n);
  RVVHelper::planeDistances(points.xData(), points.yData(), points.zData(), n, coeffs, dists.data());

  int count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (std::abs(dists[i]) <= threshold) {
      ++count;
      inliers.push_back(static_cast<int>(i));
      if (maxInliers > 0 && inliers.size() >= static_cast<std::size_t>(maxInliers)) {
        break;
      }
    }
  }
  return count;
}

void RANSACFitter::setDistanceThreshold(float threshold) {
  distanceThreshold_ = threshold;
}

void RANSACFitter::setMaxIterations(int iter) { maxIterations_ = iter; }

void RANSACFitter::setProbability(float prob) { probability_ = prob; }

void RANSACFitter::sampleRandomIndices(std::vector<int> &sample) const {
  sample.clear();
  while (sample.size() < static_cast<std::size_t>(PlaneModel::minSamples())) {
    const int index = std::rand() % static_cast<int>(input_->size());
    if (std::find(sample.begin(), sample.end(), index) == sample.end()) {
      sample.push_back(index);
    }
  }
}

int RANSACFitter::evaluateModel(const std::array<float, 4> &coeffs,
                                std::vector<int> &inliers, int maxInliers) const {
  return PlaneModel::evaluateAll(*input_, coeffs, inliers, maxInliers, distanceThreshold_);
}

bool RANSACFitter::fit(std::array<float, 4> &coefficients, std::vector<int> &inlierIndices,
                       int maxInliers) const {
  if (!input_ || input_->size() < static_cast<std::size_t>(PlaneModel::minSamples())) {
    return false;
  }

  std::srand(0);
  int effective_iterations = maxIterations_;
  if (probability_ > 0.0f && probability_ < 1.0f) {
    effective_iterations = std::max(1, static_cast<int>(maxIterations_ * probability_));
  }

  int best_inliers = 0;
  std::array<float, 4> best_coeffs = {0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<int> sample;
  std::vector<int> current_inliers;
  for (int iter = 0; iter < effective_iterations; ++iter) {
    sampleRandomIndices(sample);
    std::array<float, 4> current_coeffs;
    if (!PlaneModel::computeModel(*input_, sample, current_coeffs)) {
      continue;
    }
    const int count = evaluateModel(current_coeffs, current_inliers, maxInliers);
    if (count > best_inliers) {
      best_inliers = count;
      best_coeffs = current_coeffs;
      inlierIndices = current_inliers;
    }
  }

  coefficients = best_coeffs;
  return best_inliers > 0;
}

} // namespace rvv_pcl
