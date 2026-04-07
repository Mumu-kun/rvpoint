#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace rvv_pcl;

namespace {

constexpr int kStageCount = 8;

struct StageTiming {
  int index;
  const char *label;
  double ms;
};

struct PipelineConfig {
  // Voxel grid downsampling
  float voxel_leaf_size = 0.01f; // Smaller leaf size keeps more points.

  // Neighbor search
  float search_radius = 0.03f; // Larger radius finds more neighbors.

  // Statistical outlier removal
  int sor_mean_k = 20;           // Larger K smooths local density statistics.
  float sor_std_threshold = 1.0f; // Lower threshold removes more outliers.

  // Normal estimation
  int normal_k = 10; // Larger K smooths normals, smaller K keeps detail.

  // RANSAC dominant plane fitting
  float ransac_distance_threshold = 0.02f; // Larger threshold accepts more inliers.
  int ransac_max_iterations = 1000; // More iterations improve robustness but cost time.
  float ransac_probability = 0.99f; // Higher probability increases expected iterations.
};

constexpr PipelineConfig kPipelineConfig;

std::string resolveInputPath(const std::string &input) {
  const std::vector<std::string> candidates = {
      input,
      "/workspace/" + input,
      "/workspace/data/" + input,
  };

  for (const std::string &candidate : candidates) {
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return input;
}

PointCloudSoA subsetCloud(const PointCloudSoA &input, const std::vector<int> &indices) {
  PointCloudSoA output;
  output.reserve(indices.size());
  for (const int index : indices) {
    output.push_back(input.point(static_cast<std::size_t>(index)));
  }
  return output;
}

PointCloudSoA complementCloud(const PointCloudSoA &input, const std::vector<int> &indices) {
  std::vector<bool> selected(input.size(), false);
  for (const int index : indices) {
    if (index >= 0 && static_cast<std::size_t>(index) < input.size()) {
      selected[static_cast<std::size_t>(index)] = true;
    }
  }

  PointCloudSoA output;
  output.reserve(input.size() - indices.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    if (!selected[i]) {
      output.push_back(input.point(i));
    }
  }
  return output;
}

void saveStage(const std::filesystem::path &path, const PointCloudSoA &cloud,
               const char *label) {
  savePCD(path.string(), cloud.toAoS(), true);
  std::cout << label << ": " << cloud.size() << " points -> " << path.string() << std::endl;
}

void beginStage(int index, const char *label, bool enabled) {
  if (!enabled) {
    return;
  }

  std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label << "..."
            << std::endl;
}

double endStage(int index, const char *label,
                const std::chrono::high_resolution_clock::time_point &start, bool enabled) {
  const auto end = std::chrono::high_resolution_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << ms << " ms" << std::endl;
  }
  return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
  std::cout << "[progress] Final timing breakdown:" << std::endl;
  double stages_sum_ms = 0.0;
  for (const StageTiming &stage : stages) {
    stages_sum_ms += stage.ms;
    const double pct = total_ms > 0.0 ? (stage.ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [" << stage.index << "/" << kStageCount << "] " << stage.label
              << ": " << std::fixed << std::setprecision(3) << stage.ms << " ms ("
              << std::setprecision(2) << pct << "%)" << std::endl;
  }

  const double overhead_ms = total_ms - stages_sum_ms;
  if (std::abs(overhead_ms) > 0.01) {
    const double overhead_pct = total_ms > 0.0 ? (overhead_ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [--] Outside timed stages: " << std::fixed
              << std::setprecision(3) << overhead_ms << " ms (" << std::setprecision(2)
              << overhead_pct << "%)" << std::endl;
  }

  std::cout << "[progress] [--] Total: " << std::fixed << std::setprecision(3) << total_ms
            << " ms (100.00%)" << std::endl;
}

} // namespace

int main(int argc, char **argv) {
  bool progress_enabled = false;
  std::vector<StageTiming> stage_timings;
  stage_timings.reserve(kStageCount);
  std::vector<std::string> positional_args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.size() < 1 || positional_args.size() > 2) {
    std::cerr << "Usage: " << argv[0] << " [--progress] <input.pcd> [output_dir]"
              << std::endl;
    return 1;
  }

  const auto overall_start = std::chrono::high_resolution_clock::now();

  const std::string input_path = resolveInputPath(positional_args[0]);
  const std::filesystem::path input_stem = std::filesystem::path(positional_args[0]).stem();
  const std::filesystem::path output_dir =
      positional_args.size() == 2
          ? std::filesystem::path(positional_args[1])
          : std::filesystem::path("results") / (input_stem.string() + "_pipeline");

  std::error_code ec;
  std::filesystem::create_directories(output_dir, ec);
  if (ec) {
    std::cerr << "Failed to create output directory: " << ec.message() << std::endl;
    return 1;
  }

  std::vector<PointXYZ> loaded_points;
  beginStage(1, "Load input cloud", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  const int count = loadPCD(input_path, loaded_points);
  if (count < 0) {
    std::cerr << "Failed to load input cloud: " << positional_args[0] << std::endl;
    return 1;
  }
  stage_timings.push_back(
      {1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled)});

  PointCloudSoA input_cloud;
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  input_cloud.assign(loaded_points);
  saveStage(output_dir / "00_input.pcd", input_cloud, "Input");
  stage_timings.push_back(
      {2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled)});

  VoxelGridFilter voxel;
  voxel.setInput(input_cloud);
  voxel.setLeafSize(kPipelineConfig.voxel_leaf_size);
  PointCloudSoA downsampled_cloud;
  beginStage(3, "Downsampling", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  voxel.filter(downsampled_cloud);
  saveStage(output_dir / "01_downsampled.pcd", downsampled_cloud, "Downsampled");
  stage_timings.push_back(
      {3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled)});

  OctreeNeighborSearch search;
  search.setInputCloud(downsampled_cloud);
  search.setSearchRadius(kPipelineConfig.search_radius);
  beginStage(4, "Build search index for downsampled cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  search.buildTree();
  stage_timings.push_back({4, "Build search index for downsampled cloud",
                           endStage(4, "Build search index for downsampled cloud", stage_start,
                                    progress_enabled)});

  SORFilter sor;
  sor.setInput(downsampled_cloud);
  sor.setNeighborSearch(&search);
  sor.setMeanK(kPipelineConfig.sor_mean_k);
  sor.setStdThreshold(kPipelineConfig.sor_std_threshold);
  PointCloudSoA sor_cloud;
  beginStage(5, "Statistical outlier removal", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  sor.filter(sor_cloud);
  saveStage(output_dir / "02_sor_filtered.pcd", sor_cloud, "SOR");
  stage_timings.push_back({5, "Statistical outlier removal",
                           endStage(5, "Statistical outlier removal", stage_start,
                                    progress_enabled)});

  search.setInputCloud(sor_cloud);
  beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  search.buildTree();
  stage_timings.push_back({6, "Rebuild search index for filtered cloud",
                           endStage(6, "Rebuild search index for filtered cloud", stage_start,
                                    progress_enabled)});

  NormalEstimation normals;
  normals.setInputCloud(sor_cloud);
  normals.setNeighborSearch(&search);
  normals.setK(kPipelineConfig.normal_k);
  NormalCloud normal_cloud;
  beginStage(7, "Normal estimation", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  normals.estimate(normal_cloud);
  stage_timings.push_back(
      {7, "Normal estimation", endStage(7, "Normal estimation", stage_start, progress_enabled)});

  RANSACFitter fitter;
  fitter.setInputCloud(sor_cloud);
  fitter.setDistanceThreshold(kPipelineConfig.ransac_distance_threshold);
  fitter.setMaxIterations(kPipelineConfig.ransac_max_iterations);
  fitter.setProbability(kPipelineConfig.ransac_probability);

  std::array<float, 4> coefficients = {0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<int> inliers;
  beginStage(8, "RANSAC primitive fitting", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  if (!fitter.fit(coefficients, inliers, 0)) {
    std::cerr << "RANSAC failed to fit a model." << std::endl;
    return 1;
  }

  PointCloudSoA plane_cloud = subsetCloud(sor_cloud, inliers);
  saveStage(output_dir / "04_ransac_inliers.pcd", plane_cloud, "RANSAC inliers");

  PointCloudSoA plane_removed_cloud = complementCloud(sor_cloud, inliers);
  saveStage(output_dir / "05_ground_plane_removed.pcd", plane_removed_cloud,
            "Dominant plane removed");
  stage_timings.push_back({8, "RANSAC primitive fitting",
                           endStage(8, "RANSAC primitive fitting", stage_start,
                                    progress_enabled)});

  std::cout << "Final plane coefficients: [" << coefficients[0] << ", " << coefficients[1]
            << ", " << coefficients[2] << ", " << coefficients[3] << "]" << std::endl;
  if (progress_enabled) {
    const auto overall_end = std::chrono::high_resolution_clock::now();
    const double total_ms =
        std::chrono::duration<double, std::milli>(overall_end - overall_start).count();
    printFinalBreakdown(stage_timings, total_ms);
    std::cout << "[progress] Pipeline complete in " << total_ms << " ms" << std::endl;
  }
  return 0;
}
