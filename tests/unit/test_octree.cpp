#include "../src/include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;

int main() {
  constexpr std::size_t N = 5000;
  std::mt19937 gen(1234);
  std::uniform_real_distribution<float> dist(0.0f, 100.0f);

  PointCloudSoA cloud;
  cloud.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    cloud.push_back({dist(gen), dist(gen), dist(gen)});
  }

  OctreeNeighborSearch octree;
  octree.setInputCloud(cloud);
  octree.setSearchRadius(5.0f);
  octree.buildTree();

  int mismatches = 0;
  for (int query_index = 0; query_index < 100; ++query_index) {
    std::vector<int> oct_indices;
    octree.radiusSearch(query_index, oct_indices, nullptr, 0);

    std::vector<int> brute_force;
    const PointXYZ query = cloud.point(static_cast<std::size_t>(query_index));
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      const PointXYZ point = cloud.point(i);
      const float dx = point.x - query.x;
      const float dy = point.y - query.y;
      const float dz = point.z - query.z;
      if (dx * dx + dy * dy + dz * dz <= 25.0f) {
        brute_force.push_back(static_cast<int>(i));
      }
    }

    std::sort(oct_indices.begin(), oct_indices.end());
    std::sort(brute_force.begin(), brute_force.end());
    if (oct_indices != brute_force) {
      ++mismatches;
    }
  }

  if (mismatches != 0) {
    std::cerr << "[FAIL] Octree search mismatches: " << mismatches << std::endl;
    return 1;
  }

  std::cout << "[PASS] OctreeNeighborSearch matches brute force." << std::endl;
  return 0;
}
