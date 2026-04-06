#include "../src/include/rvv_pcl.h"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <vector>

using namespace rvv_pcl;

namespace {

struct Timer {
  std::chrono::high_resolution_clock::time_point start;

  void reset() { start = std::chrono::high_resolution_clock::now(); }

  double elapsedMs() const {
    const auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
  }
};

PointCloudSoA generateCloud(std::size_t n) {
  PointCloudSoA cloud;
  cloud.reserve(n);
  std::srand(42);
  for (std::size_t i = 0; i < n; ++i) {
    cloud.push_back({static_cast<float>(std::rand() % 1000) / 10.0f,
                     static_cast<float>(std::rand() % 1000) / 10.0f,
                     static_cast<float>(std::rand() % 1000) / 10.0f});
  }
  return cloud;
}

} // namespace

int main() {
  constexpr std::size_t N = 1024;
  PointCloudSoA cloud = generateCloud(N);
  std::system("mkdir -p results");
  std::stringstream filename;
  filename << "results/benchmark_report_" << std::time(nullptr) << ".txt";
  std::ofstream out(filename.str());
  if (!out.is_open()) {
    std::cerr << "Failed to open benchmark output file." << std::endl;
    return 1;
  }

  struct Result {
    std::string name;
    double ms;
  };
  std::vector<Result> results;
  Timer timer;

  VoxelGridFilter voxel;
  voxel.setInput(cloud);
  voxel.setLeafSize(0.5f);
  PointCloudSoA voxel_output;
  timer.reset();
  voxel.filter(voxel_output);
  results.push_back({"voxel", timer.elapsedMs()});

  OctreeNeighborSearch octree;
  octree.setInputCloud(cloud);
  octree.setSearchRadius(2.0f);
  timer.reset();
  octree.buildTree();
  results.push_back({"octree_build", timer.elapsedMs()});

  SORFilter sor;
  sor.setInput(cloud);
  sor.setNeighborSearch(&octree);
  sor.setMeanK(10);
  PointCloudSoA sor_output;
  timer.reset();
  sor.filter(sor_output);
  results.push_back({"sor", timer.elapsedMs()});

  NormalEstimation normals;
  normals.setInputCloud(cloud);
  normals.setNeighborSearch(&octree);
  normals.setK(10);
  NormalCloud normal_output;
  timer.reset();
  normals.estimate(normal_output);
  results.push_back({"normal", timer.elapsedMs()});

  std::vector<int> radius_indices;
  timer.reset();
  octree.radiusSearch(0, radius_indices, nullptr, 100);
  results.push_back({"radius", timer.elapsedMs()});

  RANSACFitter fitter;
  fitter.setInputCloud(cloud);
  fitter.setDistanceThreshold(0.1f);
  fitter.setMaxIterations(500);
  std::array<float, 4> coeffs;
  std::vector<int> inliers;
  timer.reset();
  fitter.fit(coeffs, inliers, 0);
  results.push_back({"ransac", timer.elapsedMs()});

  out << "Backend: "
#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
      << "RVV"
#else
      << "Scalar fallback"
#endif
      << "\n";
  out << "Algorithm | Time(ms)\n";
  out << "--------------------\n";
  for (const Result &result : results) {
    out << std::left << std::setw(10) << result.name << " | " << std::fixed
        << std::setprecision(4) << result.ms << "\n";
    std::cout << result.name << ": " << std::fixed << std::setprecision(4) << result.ms
              << " ms" << std::endl;
  }

  return 0;
}
