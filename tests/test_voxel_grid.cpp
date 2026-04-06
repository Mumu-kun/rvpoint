#include "../src/include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
#include <random>
#include <tuple>
#include <vector>

using namespace rvv_pcl;

namespace {

bool close_point(const PointXYZ &a, const PointXYZ &b, float eps = 1e-4f) {
  return std::abs(a.x - b.x) < eps && std::abs(a.y - b.y) < eps &&
         std::abs(a.z - b.z) < eps;
}

PointCloudSoA make_cloud(std::size_t n, std::mt19937 &gen) {
  std::uniform_real_distribution<float> dist(-10.0f, 10.0f);
  PointCloudSoA cloud;
  cloud.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    cloud.push_back({dist(gen), dist(gen), dist(gen)});
  }
  return cloud;
}

std::vector<PointXYZ> voxel_reference(const PointCloudSoA &cloud, float leaf) {
  std::map<std::tuple<int, int, int>, std::pair<PointXYZ, int>> grid;
  const float inv_leaf = 1.0f / leaf;
  for (std::size_t i = 0; i < cloud.size(); ++i) {
    const PointXYZ point = cloud.point(i);
    const auto key = std::make_tuple(static_cast<int>(std::floor(point.x * inv_leaf)),
                                     static_cast<int>(std::floor(point.y * inv_leaf)),
                                     static_cast<int>(std::floor(point.z * inv_leaf)));
    auto &entry = grid[key];
    entry.first.x += point.x;
    entry.first.y += point.y;
    entry.first.z += point.z;
    entry.second += 1;
  }

  std::vector<PointXYZ> result;
  for (const auto &entry : grid) {
    const float inv_count = 1.0f / static_cast<float>(entry.second.second);
    result.push_back({entry.second.first.x * inv_count, entry.second.first.y * inv_count,
                      entry.second.first.z * inv_count});
  }
  return result;
}

} // namespace

int main() {
  std::mt19937 gen(42);
  PointCloudSoA cloud = make_cloud(1000, gen);

  VoxelGridFilter filter;
  filter.setInput(cloud);
  filter.setLeafSize(0.5f);

  PointCloudSoA output;
  filter.filter(output);
  std::vector<PointXYZ> actual = output.toAoS();
  std::vector<PointXYZ> expected = voxel_reference(cloud, 0.5f);

  auto sort_points = [](std::vector<PointXYZ> &points) {
    std::sort(points.begin(), points.end(), [](const PointXYZ &a, const PointXYZ &b) {
      if (a.x != b.x) {
        return a.x < b.x;
      }
      if (a.y != b.y) {
        return a.y < b.y;
      }
      return a.z < b.z;
    });
  };

  sort_points(actual);
  sort_points(expected);
  if (actual.size() != expected.size()) {
    std::cerr << "[FAIL] Voxel count mismatch: " << actual.size() << " vs "
              << expected.size() << std::endl;
    return 1;
  }

  for (std::size_t i = 0; i < actual.size(); ++i) {
    if (!close_point(actual[i], expected[i])) {
      std::cerr << "[FAIL] Voxel centroid mismatch at " << i << std::endl;
      return 1;
    }
  }

  std::cout << "[PASS] VoxelGridFilter matches reference output." << std::endl;
  return 0;
}
