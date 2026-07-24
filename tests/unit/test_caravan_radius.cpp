#include "rvv_pcl.h"
#include "caravan_radius_search.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <set>
#include <vector>

using namespace rvv_pcl;

int main() {
  constexpr std::size_t N = 1000;
  constexpr std::size_t Q = 20;
  constexpr float search_radius = 2.0f;

  std::mt19937 gen(42);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  auto cloud = std::make_shared<PointCloudSoA>();
  cloud->reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    cloud->push_back({dist(gen), dist(gen), dist(gen)});
  }

  auto queries = std::make_shared<PointCloudSoA>();
  queries->reserve(Q);
  for (std::size_t i = 0; i < Q; ++i) {
    queries->push_back({dist(gen), dist(gen), dist(gen)});
  }

  OctreeNeighborSearch octree;
  octree.setInputCloud(*cloud);
  octree.setSearchRadius(search_radius);
  octree.buildTree();

  CaravanRadiusSearch caravan;
  caravan.setInputCloud(cloud);
  caravan.setSearchRadius(search_radius);

  // 1. Test single query radius search
  const int query_index = 0;
  std::vector<int> oct_indices;
  std::vector<int> caravan_single_indices;

  octree.radiusSearch(query_index, oct_indices, nullptr, 0);
  caravan.radiusSearch(query_index, caravan_single_indices, nullptr, 0);

  std::set<int> oct_set(oct_indices.begin(), oct_indices.end());
  std::set<int> caravan_set(caravan_single_indices.begin(), caravan_single_indices.end());

  if (oct_set != caravan_set) {
    std::cerr << "[FAIL] Caravan single query radius search mismatch with Octree." << std::endl;
    return 1;
  }

  // 2. Test batch radius search across query cloud
  std::vector<std::vector<int32_t>> caravan_batch_res;
  caravan.batchRadiusSearch(queries, search_radius, caravan_batch_res);

  if (caravan_batch_res.size() != Q) {
    std::cerr << "[FAIL] Caravan batch search returned incorrect query result size." << std::endl;
    return 1;
  }

  for (std::size_t q = 0; q < Q; ++q) {
    PointXYZ q_pt = queries->point(q);
    std::vector<int32_t> naive_indices;
    float r2 = search_radius * search_radius;
    for (std::size_t p = 0; p < cloud->size(); ++p) {
      PointXYZ p_pt = cloud->point(p);
      float dx = p_pt.x - q_pt.x;
      float dy = p_pt.y - q_pt.y;
      float dz = p_pt.z - q_pt.z;
      if (dx * dx + dy * dy + dz * dz <= r2) {
        naive_indices.push_back(static_cast<int32_t>(p));
      }
    }

    std::set<int32_t> naive_set(naive_indices.begin(), naive_indices.end());
    std::set<int32_t> batch_q_set(caravan_batch_res[q].begin(), caravan_batch_res[q].end());

    if (naive_set != batch_q_set) {
      std::cerr << "[FAIL] Caravan batch query " << q << " mismatch with naive search." << std::endl;
      return 1;
    }
  }

  std::cout << "[PASS] Caravan radius search implementation verified successfully." << std::endl;
  return 0;
}
