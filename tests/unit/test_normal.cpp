#include "rvv_pcl.h"

#include <cmath>
#include <iostream>
#include <random>

using namespace rvv_pcl;

namespace {

PointXYZ normalize(PointXYZ normal) {
  const float mag =
      std::sqrt(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z);
  return {normal.x / mag, normal.y / mag, normal.z / mag};
}

int run_plane_case(float slope_x, float slope_y, float z_bias,
                   const PointXYZ &expected_normal, const char *label) {
  constexpr std::size_t N = 1000;
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist_xy(-10.0f, 10.0f);
  std::normal_distribution<float> noise(0.0f, 0.01f);

  PointCloudSoA cloud;
  cloud.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    const float x = dist_xy(gen);
    const float y = dist_xy(gen);
    const float z = slope_x * x + slope_y * y + z_bias + noise(gen);
    cloud.push_back({x, y, z});
  }

  OctreeNeighborSearch search;
  search.setInputCloud(cloud);
  search.setSearchRadius(2.0f);
  search.buildTree();

  NormalEstimation estimator;
  estimator.setInputCloud(cloud);
  estimator.setNeighborSearch(&search);
  estimator.setK(10);

  NormalCloud normals;
  estimator.estimate(normals);

  int pass_count = 0;
  for (std::size_t i = 0; i < normals.size(); ++i) {
    const float dot = normals.nx()[i] * expected_normal.x +
                      normals.ny()[i] * expected_normal.y +
                      normals.nz()[i] * expected_normal.z;
    if (std::abs(dot) > 0.9f) {
      ++pass_count;
    }
  }

  if (pass_count < static_cast<int>(N * 0.95f)) {
    std::cerr << "[FAIL] " << label << " produced too many incorrect normals: "
              << pass_count << std::endl;
    return 1;
  }

  return 0;
}

} // namespace

int main() {
  if (run_plane_case(0.0f, 0.0f, 0.0f, {0.0f, 0.0f, 1.0f}, "Axis-aligned plane") != 0) {
    return 1;
  }

  const PointXYZ tilted_normal = normalize({0.4f, -0.25f, -1.0f});
  if (run_plane_case(0.4f, -0.25f, 5.0f, tilted_normal, "Tilted plane") != 0) {
    return 1;
  }

  std::cout << "[PASS] NormalEstimation recovers axis-aligned and tilted plane normals."
            << std::endl;
  return 0;
}

