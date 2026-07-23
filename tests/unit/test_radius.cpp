#include "rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set>

using namespace rvv_pcl;

int main() {
  constexpr std::size_t N = 1000;
  std::mt19937 gen(42);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  PointCloudSoA cloud;
  cloud.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    cloud.push_back({dist(gen), dist(gen), dist(gen)});
  }

  OctreeNeighborSearch octree;
  octree.setInputCloud(cloud);
  octree.setSearchRadius(2.0f);
  octree.buildTree();

  SpatialHashNeighborSearch hash;
  hash.setInputCloud(cloud);
  hash.setSearchRadius(2.0f);
  hash.setCellSize(2.0f);
  hash.buildHashTable();

  const int query_index = 0;
  std::vector<int> oct_indices;
  std::vector<int> hash_indices;
  octree.radiusSearch(query_index, oct_indices, nullptr, 0);
  hash.radiusSearch(query_index, hash_indices, nullptr, 0);

  std::set<int> oct_set(oct_indices.begin(), oct_indices.end());
  std::set<int> hash_set(hash_indices.begin(), hash_indices.end());
  if (oct_set != hash_set) {
    std::cerr << "[FAIL] Octree and spatial hash radius results differ." << std::endl;
    return 1;
  }

  std::cout << "[PASS] Neighbor search implementations agree." << std::endl;
  return 0;
}

