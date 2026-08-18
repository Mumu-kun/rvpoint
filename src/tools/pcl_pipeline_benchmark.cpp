#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// PCL Headers
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  std::string label;
  double ms;
  std::size_t point_count;
};

struct PCLPipelineConfig {
  float voxel_leaf_size = 0.01f;
  float search_radius = 0.03f;
  int sor_mean_k = 20;
  float sor_std_threshold = 1.0f;
  int normal_k = 10;
  float ransac_distance_threshold = 0.2f;
  int ransac_max_iterations = 1000;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;
};

void beginStage(int index, const std::string &label, bool enabled) {
  if (!enabled) return;
  std::cout << "[pcl-progress] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const std::string &label,
                const std::chrono::high_resolution_clock::time_point &start,
                bool enabled) {
  const auto end = std::chrono::high_resolution_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[pcl-progress] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << std::fixed << std::setprecision(3) << ms << " ms" << std::endl;
  }
  return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
  std::cout << "\n==========================================================================" << std::endl;
  std::cout << " OFFICIAL PCL PIPELINE TIMING BREAKDOWN" << std::endl;
  std::cout << "==========================================================================" << std::endl;
  double stages_sum_ms = 0.0;
  for (const auto &st : stages) {
    stages_sum_ms += st.ms;
    const double pct = total_ms > 0.0 ? (st.ms * 100.0 / total_ms) : 0.0;
    std::cout << " [" << std::setw(2) << st.index << "/" << kStageCount << "] "
              << std::left << std::setw(42) << st.label << ": "
              << std::right << std::fixed << std::setw(10) << std::setprecision(3) << st.ms << " ms "
              << "(" << std::setw(6) << std::setprecision(2) << pct << "%), pts="
              << st.point_count << std::endl;
  }

  const double overhead_ms = total_ms - stages_sum_ms;
  std::cout << " [--] " << std::left << std::setw(42) << "Outside timed stages" << ": "
            << std::right << std::fixed << std::setw(10) << std::setprecision(3) << overhead_ms << " ms" << std::endl;
  std::cout << "--------------------------------------------------------------------------" << std::endl;
  std::cout << " [--] " << std::left << std::setw(42) << "Total PCL Pipeline Time" << ": "
            << std::right << std::fixed << std::setw(10) << std::setprecision(3) << total_ms << " ms (100.00%)" << std::endl;
  std::cout << "==========================================================================\n" << std::endl;
}

void saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages,
                     double total_ms, float leaf_size, bool skip_sor, float cluster_tolerance) {
  std::ofstream ofs(out_path);
  if (!ofs.is_open()) return;

  ofs << "{\n";
  ofs << "  \"engine\": \"official_pcl\",\n";
  ofs << "  \"leaf_size\": " << leaf_size << ",\n";
  ofs << "  \"skip_sor\": " << (skip_sor ? "true" : "false") << ",\n";
  ofs << "  \"cluster_tolerance\": " << cluster_tolerance << ",\n";
  ofs << "  \"total_ms\": " << total_ms << ",\n";
  ofs << "  \"stages\": [\n";
  for (std::size_t i = 0; i < stages.size(); ++i) {
    const auto &st = stages[i];
    const double pct = total_ms > 0.0 ? (st.ms * 100.0 / total_ms) : 0.0;
    ofs << "    {\n";
    ofs << "      \"index\": " << st.index << ",\n";
    ofs << "      \"label\": \"" << st.label << "\",\n";
    ofs << "      \"ms\": " << st.ms << ",\n";
    ofs << "      \"pct\": " << pct << ",\n";
    ofs << "      \"point_count\": " << st.point_count << "\n";
    ofs << "    }" << (i + 1 < stages.size() ? "," : "") << "\n";
  }
  ofs << "  ]\n";
  ofs << "}\n";
}

} // namespace

int main(int argc, char **argv) {
  bool progress_enabled = true;
  bool skip_sor = false;
  bool export_json = false;
  PCLPipelineConfig config;
  std::vector<std::string> positional_args;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
    } else if (arg == "--json") {
      export_json = true;
    } else if (arg == "--skip-sor" || arg == "--no-sor" || arg == "--without-sor") {
      skip_sor = true;
    } else if (arg == "--leaf-size") {
      if (i + 1 < argc) config.voxel_leaf_size = std::stof(argv[++i]);
    } else if (arg == "--cluster-tolerance") {
      if (i + 1 < argc) config.cluster_tolerance = std::stof(argv[++i]);
    } else if (arg == "--min-cluster") {
      if (i + 1 < argc) config.min_cluster_size = std::stoi(argv[++i]);
    } else if (arg == "--max-cluster") {
      if (i + 1 < argc) config.max_cluster_size = std::stoi(argv[++i]);
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.empty()) {
    std::cerr << "Usage: pcl_pipeline_benchmark <input.pcd> [output_dir] [options...]\n"
              << "Options:\n"
              << "  --leaf-size <val>         Voxel leaf size (default: 0.01)\n"
              << "  --skip-sor                Bypass Statistical Outlier Removal\n"
              << "  --cluster-tolerance <val> Euclidean cluster tolerance (default: 0.15)\n"
              << "  --min-cluster <val>       Min points per cluster (default: 50)\n"
              << "  --max-cluster <val>       Max points per cluster (default: 100000)\n"
              << "  --json                    Export pipeline_metrics_pcl.json\n"
              << "  --progress                Show stage-by-stage timings\n";
    return 1;
  }

  const std::string input_path = positional_args[0];
  const std::filesystem::path input_stem = std::filesystem::path(input_path).stem();
  const std::filesystem::path output_dir = positional_args.size() > 1
      ? std::filesystem::path(positional_args[1])
      : std::filesystem::path("results") / (input_stem.string() + "_pcl_pipeline");

  std::filesystem::create_directories(output_dir);
  std::filesystem::create_directories("results/pcl_pipeline");

  std::vector<StageTiming> stages;
  stages.reserve(kStageCount);

  const auto overall_start = std::chrono::high_resolution_clock::now();

  // 1. Load Input PCD
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  beginStage(1, "Load input cloud (PCL)", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_path, *input_cloud) == -1) {
    std::cerr << "Error: Could not read PCD file " << input_path << std::endl;
    return 1;
  }
  const std::size_t n_input = input_cloud->size();
  stages.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

  // 2. Write input stage
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::io::savePCDFileBinary((output_dir / "00_input.pcd").string(), *input_cloud);
  stages.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

  // 3. Voxel Grid Downsampling
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  beginStage(3, "Downsampling (pcl::VoxelGrid)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(input_cloud);
  vg.setLeafSize(config.voxel_leaf_size, config.voxel_leaf_size, config.voxel_leaf_size);
  vg.filter(*downsampled_cloud);
  const std::size_t n_down = downsampled_cloud->size();
  pcl::io::savePCDFileBinary((output_dir / "01_downsampled.pcd").string(), *downsampled_cloud);
  stages.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});

  // 4. Build search index for downsampled cloud
  beginStage(4, "Build KdTree index for downsampled cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  tree->setInputCloud(downsampled_cloud);
  stages.push_back({4, "Build search index for downsampled cloud", endStage(4, "Build search index", stage_start, progress_enabled), n_down});

  // 5. Statistical Outlier Removal (SOR)
  pcl::PointCloud<pcl::PointXYZ>::Ptr sor_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  beginStage(5, "Statistical outlier removal (pcl::SOR)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  if (skip_sor) {
    sor_cloud = downsampled_cloud;
  } else {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(downsampled_cloud);
    sor.setMeanK(config.sor_mean_k);
    sor.setStddevMulThresh(config.sor_std_threshold);
    sor.filter(*sor_cloud);
  }
  const std::size_t n_sor = sor_cloud->size();
  pcl::io::savePCDFileBinary((output_dir / "02_sor_filtered.pcd").string(), *sor_cloud);
  stages.push_back({5, "Statistical outlier removal", endStage(5, "Statistical outlier removal", stage_start, progress_enabled), n_sor});

  // 6. Rebuild search index for filtered cloud
  beginStage(6, "Rebuild KdTree index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::search::KdTree<pcl::PointXYZ>::Ptr sor_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  sor_tree->setInputCloud(sor_cloud);
  stages.push_back({6, "Rebuild search index for filtered cloud", endStage(6, "Rebuild search index", stage_start, progress_enabled), n_sor});

  // 7. Normal Estimation
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
  beginStage(7, "Normal estimation (pcl::NormalEstimation)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(sor_cloud);
  ne.setSearchMethod(sor_tree);
  ne.setRadiusSearch(config.search_radius);
  ne.compute(*normals);
  stages.push_back({7, "Normal estimation", endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

  // 8. RANSAC Plane Fitting
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_pts(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_pts(new pcl::PointCloud<pcl::PointXYZ>());

  beginStage(8, "RANSAC primitive fitting (pcl::SACSegmentation)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(config.ransac_max_iterations);
  seg.setDistanceThreshold(config.ransac_distance_threshold);
  seg.setInputCloud(sor_cloud);
  seg.segment(*inliers, *coefficients);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(sor_cloud);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*inlier_pts);

  extract.setNegative(true);
  extract.filter(*outlier_pts);

  pcl::io::savePCDFileBinary((output_dir / "04_ransac_inliers.pcd").string(), *inlier_pts);
  pcl::io::savePCDFileBinary((output_dir / "05_ground_plane_removed.pcd").string(), *outlier_pts);

  stages.push_back({8, "RANSAC primitive fitting", endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), outlier_pts->size()});

  // 9. Euclidean Clustering
  std::vector<pcl::PointIndices> cluster_indices;
  beginStage(9, "Euclidean clustering (pcl::EuclideanClusterExtraction)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();

  pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree(new pcl::search::KdTree<pcl::PointXYZ>());
  cluster_tree->setInputCloud(outlier_pts);

  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(config.cluster_tolerance);
  ec.setMinClusterSize(config.min_cluster_size);
  ec.setMaxClusterSize(config.max_cluster_size);
  ec.setSearchMethod(cluster_tree);
  ec.setInputCloud(outlier_pts);
  ec.extract(cluster_indices);

  stages.push_back({9, "Euclidean clustering", endStage(9, "Euclidean clustering", stage_start, progress_enabled), cluster_indices.size()});

  // 10. Write Cluster Stage (Colored PCD)
  beginStage(10, "Write cluster stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clusters(new pcl::PointCloud<pcl::PointXYZRGB>());
  const float golden_ratio = 0.618033988749895f;
  float hue = 0.35f;

  for (const auto &indices : cluster_indices) {
    hue = std::fmod(hue + golden_ratio, 1.0f);
    float s = 0.85f, v = 0.95f;
    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
    float m = v - c;
    float r_f = 0.0f, g_f = 0.0f, b_f = 0.0f;
    int h_i = static_cast<int>(hue * 6.0f) % 6;
    switch (h_i) {
      case 0: r_f = c; g_f = x; b_f = 0.0f; break;
      case 1: r_f = x; g_f = c; b_f = 0.0f; break;
      case 2: r_f = 0.0f; g_f = c; b_f = x; break;
      case 3: r_f = 0.0f; g_f = x; b_f = c; break;
      case 4: r_f = x; g_f = 0.0f; b_f = c; break;
      case 5: r_f = c; g_f = 0.0f; b_f = x; break;
    }
    uint8_t r = static_cast<uint8_t>((r_f + m) * 255.0f);
    uint8_t g = static_cast<uint8_t>((g_f + m) * 255.0f);
    uint8_t b = static_cast<uint8_t>((b_f + m) * 255.0f);

    for (int idx : indices.indices) {
      const auto &pt = outlier_pts->points[idx];
      pcl::PointXYZRGB pt_rgb;
      pt_rgb.x = pt.x;
      pt_rgb.y = pt.y;
      pt_rgb.z = pt.z;
      pt_rgb.r = r;
      pt_rgb.g = g;
      pt_rgb.b = b;
      colored_clusters->points.push_back(pt_rgb);
    }
  }
  colored_clusters->width = colored_clusters->points.size();
  colored_clusters->height = 1;
  colored_clusters->is_dense = true;

  const std::filesystem::path cluster_out_path = output_dir / "06_clusters.pcd";
  pcl::io::savePCDFileBinary(cluster_out_path.string(), *colored_clusters);
  pcl::io::savePCDFileBinary("results/pcl_pipeline/06_clusters.pcd", *colored_clusters);
  std::cout << "Clusters: " << colored_clusters->points.size() << " points ("
            << cluster_indices.size() << " clusters) -> " << cluster_out_path.string() << std::endl;
  stages.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), colored_clusters->size()});

  const auto overall_end = std::chrono::high_resolution_clock::now();
  const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

  printFinalBreakdown(stages, total_ms);

  if (export_json) {
    saveJSONMetrics(output_dir / "pipeline_metrics_pcl.json", stages, total_ms, config.voxel_leaf_size, skip_sor, config.cluster_tolerance);
    std::cout << "Metrics saved to: " << (output_dir / "pipeline_metrics_pcl.json").string() << std::endl;
  }

  std::cout.flush();
  std::cerr.flush();
  std::quick_exit(0);
}
