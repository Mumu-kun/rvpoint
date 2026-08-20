#include "core/point_types.h"
#include "core/rvv_common.h"
#include "search/caravan_radius_search.h"
#include "filters/statistical_outlier_removal.h"
#include "io/simple_pcd_loader.h"

#include <iostream>
#include <memory>
#include <vector>
#include <string>

using namespace rvpoint;

int main(int argc, char **argv) {
  std::string input_file = "output/results/1/01_downsampled.pcd";
  bool verbose = false;

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-v" || arg == "--verbose") {
      verbose = true;
    } else {
      input_file = arg;
    }
  }

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
      if (verbose) {
        std::cout << "[INFO] Loaded PCD from: " << path << std::endl;
      }
      loaded = true;
      break;
    }
  }

  if (!loaded) {
    if (verbose) {
      std::cerr << "[ERROR] Could not load PCD file!" << std::endl;
    }
    return 1;
  }

  if (verbose) {
    float min_x = RVVHelper::vmin(cloud.xData(), cloud.size());
    float min_y = RVVHelper::vmin(cloud.yData(), cloud.size());
    float min_z = RVVHelper::vmin(cloud.zData(), cloud.size());
    float max_x = RVVHelper::vmax(cloud.xData(), cloud.size());
    float max_y = RVVHelper::vmax(cloud.yData(), cloud.size());
    float max_z = RVVHelper::vmax(cloud.zData(), cloud.size());

    std::cout << "[INFO] Points: " << cloud.size() << std::endl;
    std::cout << "[INFO] Bounding Box: [" << min_x << ", " << min_y << ", " << min_z << "] to ["
              << max_x << ", " << max_y << ", " << max_z << "]" << std::endl;
  }

  auto caravan = std::make_shared<CaravanRadiusSearch>();
  caravan->setInputCloud(std::make_shared<PointCloudSoA>(cloud));
  caravan->setSearchRadius(0.05f);

  SORFilter sor;
  sor.setInput(cloud);
  sor.setNeighborSearch(caravan.get());
  sor.setMeanK(10);
  sor.setStdThreshold(1.0f);

  PointCloudSoA output;
  sor.filter(output);

  if (verbose) {
    std::cout << "[BENCH] SOR caravan PCD input_points=" << cloud.size()
              << " filtered_points=" << output.size() << std::endl;
  }

  return 0;
}
