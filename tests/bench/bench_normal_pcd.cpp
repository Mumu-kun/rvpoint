#include "core/point_types.h"
#include "core/rvv_common.h"
#include "search/caravan_radius_search.h"
#include "filters/voxel_grid.h"
#include "features/normal_estimation.h"
#include "io/simple_pcd_loader.h"


#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cmath>  // for std::cbrt

using namespace rvpoint;

int main(int argc, char **argv) {
  std::string input_file = "output/results/1/01_downsampled.pcd";
  bool verbose = true; // always verbose

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    // ignore command‑line flags for verbosity – always verbose
    input_file = arg; // treat any non‑flag as input file
  }

  /* -----------------------------------------------------------
   * Load original point cloud
   * ----------------------------------------------------------- */
  PointCloudSoA cloud;
  std::vector<std::string> search_paths = {
      input_file,
      "data/bunny.pcd",
      "bunny.pcd",
      "../data/bunny.pcd",
      "/workspace/data/bunny.pcd"
  };

  bool loaded = false;
  for (const auto &path : search_paths) {
    if (cloud.loadFromPCD(path) && cloud.size() > 0) {
      if (verbose) std::cout << "[INFO] Loaded PCD from: " << path << std::endl;
      loaded = true;
      break;
    }
  }
  if (!loaded) {
    if (verbose) std::cerr << "[ERROR] Could not load PCD file!" << std::endl;
    return 1;
  }

  /* -----------------------------------------------------------
   * Adaptive down‑sample to ~10 000 points using Voxel Grid
   * ----------------------------------------------------------- */
   PointCloudSoA downsampled;
   VoxelGridFilter voxel;
   constexpr std::size_t TARGET_POINTS = 10000;
   if (cloud.size() > TARGET_POINTS) {
     // Compute bounding box to estimate volume
     float min_x = RVVHelper::vmin(cloud.xData(), cloud.size());
     float min_y = RVVHelper::vmin(cloud.yData(), cloud.size());
     float min_z = RVVHelper::vmin(cloud.zData(), cloud.size());
     float max_x = RVVHelper::vmax(cloud.xData(), cloud.size());
     float max_y = RVVHelper::vmax(cloud.yData(), cloud.size());
     float max_z = RVVHelper::vmax(cloud.zData(), cloud.size());
     float volume = (max_x - min_x) * (max_y - min_y) * (max_z - min_z);
      // Compute dimensions
      float dim_x = max_x - min_x;
      float dim_y = max_y - min_y;
      float dim_z = max_z - min_z;
      float max_dim = std::max(dim_x, std::max(dim_y, dim_z));
      // Approximate leaf size to reach TARGET_POINTS
      float leaf = std::cbrt(volume / static_cast<float>(TARGET_POINTS));
      // Clamp leaf size: not too small, not larger than a fraction of the largest dimension
      leaf = std::max(leaf, 0.001f);
      leaf = std::min(leaf, max_dim / 10.0f);
      voxel.setInput(cloud);
      voxel.setLeafSize(leaf);
      voxel.filter(downsampled);
      // If filter removed everything, fall back to original cloud
      if (downsampled.size() == 0) {
        downsampled = cloud;
      }
   } else {
     // Cloud is already small enough; just copy
     downsampled = cloud;
   }

   if (verbose) {
     std::cout << "[INFO] Down‑sampled to " << downsampled.size() << " points\n";
     // Re‑compute bounding box for the down‑sampled cloud
     float min_x = RVVHelper::vmin(downsampled.xData(), downsampled.size());
     float min_y = RVVHelper::vmin(downsampled.yData(), downsampled.size());
     float min_z = RVVHelper::vmin(downsampled.zData(), downsampled.size());
     float max_x = RVVHelper::vmax(downsampled.xData(), downsampled.size());
     float max_y = RVVHelper::vmax(downsampled.yData(), downsampled.size());
     float max_z = RVVHelper::vmax(downsampled.zData(), downsampled.size());

     std::cout << "[INFO] Points: " << downsampled.size() << std::endl;
    std::cout << "[INFO] Bounding Box: [" << min_x << ", " << min_y << ", "
              << min_z << "] to [" << max_x << ", " << max_y << ", "
              << max_z << "]\n";
    std::cout << "[INFO] Starting Normal Estimation (Caravan RVV, K=10)...\n";
  }

  /* -----------------------------------------------------------
   * Normal estimation on the down‑sampled cloud
   * ----------------------------------------------------------- */
  auto caravan = std::make_shared<CaravanRadiusSearch>();
  caravan->setInputCloud(std::make_shared<PointCloudSoA>(downsampled));
  caravan->setSearchRadius(0.05f);

  NormalEstimation estimator;
  estimator.setInputCloud(downsampled);
  estimator.setNeighborSearch(caravan.get());
  estimator.setK(10);

  NormalCloud normals;
  estimator.estimate(normals);

  if (verbose) {
    std::cout << "[BENCH] Normal Estimation complete: " << normals.size()
              << " normals calculated.\n";
  }
  return 0;
}
