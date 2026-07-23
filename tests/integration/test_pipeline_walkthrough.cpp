#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <filesystem>
#include <iostream>
#include <vector>

using namespace rvv_pcl;

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
  PointCloudSoA voxelized;
  voxel.filter(voxelized);

  OctreeNeighborSearch search;
  search.setInputCloud(voxelized);
  search.setSearchRadius(0.03f);
  search.buildTree();

  SORFilter sor;
  sor.setInput(voxelized);
  sor.setNeighborSearch(&search);
  sor.setMeanK(20);
  PointCloudSoA filtered;
  sor.filter(filtered);

  search.setInputCloud(filtered);
  search.buildTree();

  NormalEstimation estimation;
  estimation.setInputCloud(filtered);
  estimation.setNeighborSearch(&search);
  estimation.setK(10);
  NormalCloud normals;
  estimation.estimate(normals);

  if (filtered.empty() || normals.size() != filtered.size()) {
    std::cerr << "[FAIL] Pipeline outputs are inconsistent." << std::endl;
    return 1;
  }

  const std::filesystem::path output_dir("results");
  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::cerr << "[FAIL] Could not create output directory: " << ec.message()
              << std::endl;
    return 1;
  }

  const std::filesystem::path output_path = output_dir / "pipeline_voxelized.pcd";
  savePCD(output_path.string(), voxelized.toAoS());
  if (!std::filesystem::exists(output_path)) {
    std::cerr << "[FAIL] Walkthrough did not produce output artifact." << std::endl;
    return 1;
  }

  std::cout << "[PASS] Pipeline walkthrough completed on class-based API." << std::endl;
  return 0;
}

