#include "../src/include/rvv_pcl.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;

namespace {

PointCloudSoA make_cloud() {
  constexpr std::size_t N = 1000;
  std::mt19937 gen(42);
  std::normal_distribution<float> cluster(0.0f, 1.0f);
  std::uniform_real_distribution<float> outlier(-20.0f, 20.0f);

  PointCloudSoA cloud;
  cloud.reserve(N);
  for (std::size_t i = 0; i < N; ++i) {
    if (i < N * 9 / 10) {
      cloud.push_back({cluster(gen), cluster(gen), cluster(gen)});
    } else {
      cloud.push_back({outlier(gen), outlier(gen), outlier(gen)});
    }
  }
  return cloud;
}

} // namespace

int main() {
  PointCloudSoA cloud = make_cloud();
  OctreeNeighborSearch search;
  search.setInputCloud(cloud);
  search.setSearchRadius(5.0f);
  search.buildTree();

  SORFilter filter;
  filter.setInput(cloud);
  filter.setNeighborSearch(&search);
  filter.setMeanK(10);
  filter.setStdThreshold(1.0f);

  PointCloudSoA output;
  filter.filter(output);

  if (output.size() >= cloud.size() || output.size() < cloud.size() * 7 / 10) {
    std::cerr << "[FAIL] Unexpected SOR output size: " << output.size() << std::endl;
    return 1;
  }

  std::cout << "[PASS] SORFilter removed outliers and kept a dense core." << std::endl;
  return 0;
}
