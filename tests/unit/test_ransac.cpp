#include "rvv_pcl.h"

#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;

int main() {
  constexpr std::size_t N_INLIERS = 500;
  constexpr std::size_t N_OUTLIERS = 500;
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist_xy(-10.0f, 10.0f);
  std::normal_distribution<float> dist_z(10.0f, 0.01f);
  std::uniform_real_distribution<float> outlier(-20.0f, 20.0f);

  PointCloudSoA cloud;
  cloud.reserve(N_INLIERS + N_OUTLIERS);
  for (std::size_t i = 0; i < N_INLIERS + N_OUTLIERS; ++i) {
    if (i < N_INLIERS) {
      cloud.push_back({dist_xy(gen), dist_xy(gen), dist_z(gen)});
    } else {
      cloud.push_back({outlier(gen), outlier(gen), outlier(gen)});
    }
  }

  RANSACFitter fitter;
  fitter.setInputCloud(cloud);
  fitter.setDistanceThreshold(0.1f);
  fitter.setMaxIterations(1000);
  fitter.setProbability(0.99f);

  std::array<float, 4> model;
  std::vector<int> inliers;
  if (!fitter.fit(model, inliers, 0)) {
    std::cerr << "[FAIL] RANSAC returned no model." << std::endl;
    return 1;
  }

  const bool model_ok = std::abs(model[2]) > 0.9f && std::abs(model[3]) >= 9.0f;
  const bool inlier_ok = inliers.size() >= 450;
  if (!model_ok || !inlier_ok) {
    std::cerr << "[FAIL] Unexpected model or inlier count." << std::endl;
    return 1;
  }

  std::cout << "[PASS] RANSACFitter finds the dominant plane." << std::endl;
  return 0;
}

