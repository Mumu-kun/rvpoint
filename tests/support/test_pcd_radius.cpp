#include "rvv_pcl.h"
#include "simple_pcd_loader.h"
#include "caravan_radius_search.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace rvv_pcl;

void printUsage() {
  std::cerr << "Usage: test_pcd_radius <pcd_file> <search_type> <radius> [iterations] [query_index]\n"
            << "  search_type : caravan|octree|spatial_hash|rvv_helper\n"
            << "  radius      : search radius float (e.g. 0.5)\n"
            << "  iterations  : number of benchmark iterations (default: 10)\n"
            << "  query_index : index of point to query (default: 0)\n";
}

int main(int argc, char **argv) {
  if (argc < 4 || argc > 6) {
    printUsage();
    return 1;
  }

  const std::string filename = argv[1];
  const std::string search_type = argv[2];
  float radius = 0.5f;
  try {
    radius = std::stof(argv[3]);
  } catch (...) {
    std::cerr << "Invalid radius: " << argv[3] << "\n";
    return 1;
  }

  std::size_t iterations = 10;
  if (argc >= 5) {
    try {
      iterations = std::stoull(argv[4]);
      if (iterations == 0) iterations = 1;
    } catch (...) {}
  }

  int query_index = 0;
  if (argc >= 6) {
    try {
      query_index = std::stoi(argv[5]);
    } catch (...) {}
  }

  std::vector<PointXYZ> raw_points;
  int loaded_count = loadPCD(filename, raw_points);
  if (loaded_count <= 0 || raw_points.empty()) {
    std::cerr << "[FAIL] Could not load PCD file or file is empty: " << filename << std::endl;
    return 1;
  }

  PointCloudSoA cloud;
  cloud.assign(raw_points);

  if (query_index < 0 || static_cast<std::size_t>(query_index) >= cloud.size()) {
    query_index = 0;
  }

  PointXYZ query_pt = cloud.point(query_index);
  std::vector<int> result_indices;
  std::size_t total_found = 0;

  // Instantiate selected search algorithm
  std::unique_ptr<CaravanRadiusSearch> caravan;
  std::unique_ptr<OctreeNeighborSearch> octree;
  std::unique_ptr<SpatialHashNeighborSearch> hash_search;

  if (search_type == "caravan" || search_type == "carvan") {
    caravan = std::make_unique<CaravanRadiusSearch>();
    caravan->setInputCloud(cloud);
    caravan->setSearchRadius(radius);
  } else if (search_type == "octree") {
    octree = std::make_unique<OctreeNeighborSearch>();
    octree->setInputCloud(cloud);
    octree->setSearchRadius(radius);
    octree->buildTree();
  } else if (search_type == "spatial_hash" || search_type == "hash") {
    hash_search = std::make_unique<SpatialHashNeighborSearch>();
    hash_search->setInputCloud(cloud);
    hash_search->setSearchRadius(radius);
    hash_search->setCellSize(radius);
    hash_search->buildHashTable();
  }

  // Warmup run
  if (caravan) {
    caravan->radiusSearch(query_index, result_indices, nullptr, 0);
  } else if (octree) {
    octree->radiusSearch(query_index, result_indices, nullptr, 0);
  } else if (hash_search) {
    hash_search->radiusSearch(query_index, result_indices, nullptr, 0);
  } else {
    // rvv_helper fallback
    std::vector<float> d2(cloud.size());
    const float r2 = radius * radius;
    RVVHelper::distanceSquared(cloud.xData(), cloud.yData(), cloud.zData(), cloud.size(),
                               query_pt.x, query_pt.y, query_pt.z, d2.data());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
      if (d2[i] <= r2) result_indices.push_back(static_cast<int>(i));
    }
  }
  total_found = result_indices.size();

  // Timed benchmark loop
  auto start_time = std::chrono::high_resolution_clock::now();
  for (std::size_t iter = 0; iter < iterations; ++iter) {
    result_indices.clear();
    if (caravan) {
      caravan->radiusSearch(query_index, result_indices, nullptr, 0);
    } else if (octree) {
      octree->radiusSearch(query_index, result_indices, nullptr, 0);
    } else if (hash_search) {
      hash_search->radiusSearch(query_index, result_indices, nullptr, 0);
    } else {
      std::vector<float> d2(cloud.size());
      const float r2 = radius * radius;
      RVVHelper::distanceSquared(cloud.xData(), cloud.yData(), cloud.zData(), cloud.size(),
                                 query_pt.x, query_pt.y, query_pt.z, d2.data());
      for (std::size_t i = 0; i < cloud.size(); ++i) {
        if (d2[i] <= r2) result_indices.push_back(static_cast<int>(i));
      }
    }
  }
  auto end_time = std::chrono::high_resolution_clock::now();

  const auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  const double avg_us = static_cast<double>(total_us) / static_cast<double>(iterations);

  std::cout << "[PCD_RADIUS_TEST] PCD: " << filename
            << " | Points: " << cloud.size()
            << " | Algorithm: " << search_type
            << " | Radius: " << radius
            << " | Found: " << total_found
            << " | Total: " << total_us << " us"
            << " | Avg: " << std::fixed << std::setprecision(2) << avg_us << " us/iter\n";

  std::cout << "PCD_RESULT pcd=" << filename
            << " points=" << cloud.size()
            << " algorithm=" << search_type
            << " radius=" << radius
            << " found=" << total_found
            << " iterations=" << iterations
            << " total_us=" << total_us
            << " avg_us=" << std::fixed << std::setprecision(2) << avg_us << "\n";

  return 0;
}
