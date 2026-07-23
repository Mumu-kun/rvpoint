#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <chrono>
#include <iostream>
#include <vector>

using namespace rvv_pcl;

namespace {

class Timer {
public:
  void start() { start_ = std::chrono::high_resolution_clock::now(); }
  double elapsedMs() const {
    const auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<double, std::milli>(end - start_).count();
  }

private:
  std::chrono::high_resolution_clock::time_point start_;
};

} // namespace

int main(int argc, char **argv) {
  std::string input_file = "bunny.pcd";
  if (argc > 1) {
    input_file = argv[1];
  }

  std::vector<PointXYZ> loaded_points;
  int count = loadPCD(input_file, loaded_points);
  if (count < 0) {
    count = loadPCD("data/" + input_file, loaded_points);
  }
  if (count < 0) {
    count = loadPCD("../data/" + input_file, loaded_points);
  }
  if (count < 0) {
    count = loadPCD("/workspace/" + input_file, loaded_points);
  }
  if (count < 0) {
    count = loadPCD("/workspace/data/" + input_file, loaded_points);
  }
  if (count < 0) {
    std::cerr << "[FAIL] Could not load input point cloud." << std::endl;
    return 1;
  }

  PointCloudSoA cloud;
  cloud.assign(loaded_points);

  VoxelGridFilter voxel;
  voxel.setInput(cloud);
  voxel.setLeafSize(0.01f);
  PointCloudSoA filtered;
  voxel.filter(filtered);

  Timer timer;
  OctreeNeighborSearch octree;
  octree.setInputCloud(filtered);
  octree.setSearchRadius(0.05f);
  timer.start();
  octree.buildTree();
  const double octree_build_ms = timer.elapsedMs();

  SpatialHashNeighborSearch hash;
  hash.setInputCloud(filtered);
  hash.setSearchRadius(0.05f);
  hash.setCellSize(0.05f);
  timer.start();
  hash.buildHashTable();
  const double hash_build_ms = timer.elapsedMs();

  std::vector<int> octree_indices;
  std::vector<int> hash_indices;
  octree.radiusSearch(static_cast<int>(filtered.size() / 2), octree_indices, nullptr, 0);
  hash.radiusSearch(static_cast<int>(filtered.size() / 2), hash_indices, nullptr, 0);

  std::cout << "Octree build: " << octree_build_ms << " ms" << std::endl;
  std::cout << "Hash build: " << hash_build_ms << " ms" << std::endl;
  std::cout << "Octree neighbors: " << octree_indices.size() << std::endl;
  std::cout << "Hash neighbors: " << hash_indices.size() << std::endl;

  if (octree_indices.size() != hash_indices.size()) {
    std::cerr << "[FAIL] Search structures disagree on neighbor count." << std::endl;
    return 1;
  }

  std::cout << "[PASS] Spatial hash and octree are both usable in the new API."
            << std::endl;
  return 0;
}

