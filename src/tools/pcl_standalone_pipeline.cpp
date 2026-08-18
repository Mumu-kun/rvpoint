#include "core/point_types.h"
#include "filters/voxel_grid.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "search/octree.h"
#include "io/simple_pcd_loader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <vector>

using namespace rvpoint;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  std::string label;
  double ms;
  std::size_t point_count;
};

struct PCLConfig {
  float voxel_leaf_size = 0.10f;
  float search_radius = 0.03f;
  int sor_mean_k = 20;
  float sor_std_threshold = 1.0f;
  int normal_k = 10;
  float ransac_distance_threshold = 0.20f;
  int ransac_max_iterations = 1000;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;
};

void beginStage(int index, const std::string &label, bool enabled) {
  if (!enabled) return;
  std::cout << "[pcl-riscv] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const std::string &label,
                const std::chrono::high_resolution_clock::time_point &start,
                bool enabled) {
  const auto end = std::chrono::high_resolution_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[pcl-riscv] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << std::fixed << std::setprecision(3) << ms << " ms" << std::endl;
  }
  return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
  std::cout << "\n==========================================================================" << std::endl;
  std::cout << " INDEPENDENT PCL PIPELINE (RISC-V QEMU SCALAR RUNTIME)" << std::endl;
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
  std::cout << " [--] " << std::left << std::setw(42) << "Total PCL RISC-V Pipeline Time" << ": "
            << std::right << std::fixed << std::setw(10) << std::setprecision(3) << total_ms << " ms (100.00%)" << std::endl;
  std::cout << "==========================================================================\n" << std::endl;
}

void saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages,
                     double total_ms, float leaf_size, bool skip_sor,
                     float cluster_tolerance) {
  std::ofstream ofs(out_path);
  if (!ofs.is_open()) return;

  ofs << "{\n";
  ofs << "  \"engine\": \"pcl_riscv_scalar\",\n";
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

void aosToSoa(const std::vector<PointXYZ> &aos, std::vector<float> &x,
              std::vector<float> &y, std::vector<float> &z, PointCloudSoA &soa) {
  const std::size_t n = aos.size();
  x.resize(n);
  y.resize(n);
  z.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    x[i] = aos[i].x;
    y[i] = aos[i].y;
    z[i] = aos[i].z;
  }
  soa.x = x.data();
  soa.y = y.data();
  soa.z = z.data();
  soa.n = n;
}

// Pure Scalar Statistical Outlier Removal using standard Octree radius search
std::size_t sor_octree_scalar(const PointCloudSoA &in, const Octree &tree,
                              PointXYZ *out, int k, float alpha, float search_radius = 0.5f) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);
  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;

  for (std::size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);
    if (nbr_dists.size() > 1) {
      std::sort(nbr_dists.begin(), nbr_dists.end());
      float sum = 0.0f;
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(nbr_dists[j]);
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  float global_sum = 0.0f;
  for (float d : mean_dists) global_sum += d;
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists) variance_sum += (d - global_mean) * (d - global_mean);
  float global_std = std::sqrt(variance_sum / in.n);
  float thresh = global_mean + alpha * global_std;

  std::size_t count = 0;
  for (std::size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count++] = {in.x[i], in.y[i], in.z[i]};
    }
  }
  return count;
}

void saveStagePoints(const std::filesystem::path &out_file,
                     const std::vector<PointXYZ> &pts,
                     const std::string &label) {
  savePCD(out_file.string(), pts, true);
  std::cout << label << ": " << pts.size() << " points -> "
            << out_file.string() << std::endl;
}

void simple_eigen3x3_smallest_sc(float cov[3][3], float &nx, float &ny, float &nz, int iters = 15) {
  float v[3] = {1.0f, 1.0f, 1.0f};
  float norm = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] /= norm; v[1] /= norm; v[2] /= norm;
  for (int iter = 0; iter < iters; ++iter) {
    float nv[3] = {
      cov[0][0] * v[0] + cov[0][1] * v[1] + cov[0][2] * v[2],
      cov[1][0] * v[0] + cov[1][1] * v[1] + cov[1][2] * v[2],
      cov[2][0] * v[0] + cov[2][1] * v[1] + cov[2][2] * v[2]
    };
    float nlen = std::sqrt(nv[0] * nv[0] + nv[1] * nv[1] + nv[2] * nv[2]);
    if (nlen > 1e-6f) {
      v[0] = nv[0] / nlen; v[1] = nv[1] / nlen; v[2] = nv[2] / nlen;
    }
  }
  nx = v[0]; ny = v[1]; nz = v[2];
}

void flip_viewpoint_sc(const PointXYZ &q, float &nx, float &ny, float &nz) {
  if ((-q.x) * nx + (-q.y) * ny + (-q.z) * nz < 0.0f) {
    nx = -nx; ny = -ny; nz = -nz;
  }
}

// Pure Scalar Normal Estimation using standard Octree covariance
void normal_estimation_octree_scalar(const PointCloudSoA &in, const Octree &tree,
                                     float *nx, float *ny, float *nz,
                                     int k, float search_radius) {
  if (in.n == 0) return;
  std::vector<int> indices;
  std::vector<float> dists;

  for (std::size_t i = 0; i < in.n; ++i) {
    PointXYZ q = {in.x[i], in.y[i], in.z[i]};
    tree.radiusSearch(q, search_radius, indices, dists);

    if (indices.size() < 3) {
      nx[i] = 0.0f; ny[i] = 0.0f; nz[i] = 1.0f;
      continue;
    }

    // Centroid
    float cx = 0, cy = 0, cz = 0;
    for (int idx : indices) {
      cx += in.x[idx]; cy += in.y[idx]; cz += in.z[idx];
    }
    const float inv_pts = 1.0f / indices.size();
    cx *= inv_pts; cy *= inv_pts; cz *= inv_pts;

    // Covariance matrix
    float cov[3][3] = {0};
    for (int idx : indices) {
      float dx = in.x[idx] - cx;
      float dy = in.y[idx] - cy;
      float dz = in.z[idx] - cz;
      cov[0][0] += dx * dx; cov[0][1] += dx * dy; cov[0][2] += dx * dz;
      cov[1][1] += dy * dy; cov[1][2] += dy * dz;
      cov[2][2] += dz * dz;
    }
    cov[1][0] = cov[0][1]; cov[2][0] = cov[0][2]; cov[2][1] = cov[1][2];

    // Power iteration / smallest eigenvector
    float n_x, n_y, n_z;
    simple_eigen3x3_smallest_sc(cov, n_x, n_y, n_z, 15);
    flip_viewpoint_sc(q, n_x, n_y, n_z);

    nx[i] = n_x; ny[i] = n_y; nz[i] = n_z;
  }
}

} // namespace

int main(int argc, char **argv) {
  bool progress_enabled = true;
  bool skip_sor = false;
  bool export_json = false;
  PCLConfig config;
  std::vector<std::string> positional_args;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
    } else if (arg == "--json") {
      export_json = true;
    } else if (arg == "--skip-sor") {
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
    std::cerr << "Usage: pcl_standalone_pipeline <input.pcd> [output_dir] [options...]\n";
    return 1;
  }

  const std::string input_path = positional_args[0];
  const std::filesystem::path output_dir = positional_args.size() > 1
      ? std::filesystem::path(positional_args[1])
      : std::filesystem::path("results/pcl_riscv_pipeline");

  std::filesystem::create_directories(output_dir);

  std::vector<StageTiming> stages;
  stages.reserve(kStageCount);

  const auto overall_start = std::chrono::high_resolution_clock::now();

  // Stage 1: Load Input PCD
  beginStage(1, "Load input cloud (RISC-V Scalar)", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> raw_pts;
  int loaded = loadPCD(input_path, raw_pts);
  if (loaded <= 0 || raw_pts.empty()) {
    std::cerr << "Error: Failed to load PCD file from " << input_path << std::endl;
    return 1;
  }
  const std::size_t n_input = raw_pts.size();
  stages.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

  // Stage 2: Write input stage
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  saveStagePoints(output_dir / "00_input.pcd", raw_pts, "Input");
  stages.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

  // Stage 3: Voxel Grid Downsampling (Pure Scalar)
  beginStage(3, "Downsampling (Scalar VoxelGrid)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> down_pts(n_input);
  std::size_t n_down = voxel_grid_downsamp_sc(raw_pts.data(), n_input, down_pts.data(), config.voxel_leaf_size);
  down_pts.resize(n_down);
  saveStagePoints(output_dir / "01_downsampled.pcd", down_pts, "Downsampled");
  stages.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});

  // Stage 4: Build standard Octree for downsampled cloud
  std::vector<float> dx, dy, dz;
  PointCloudSoA down_soa;
  aosToSoa(down_pts, dx, dy, dz, down_soa);

  beginStage(4, "Build Octree index for downsampled cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  Octree search_down;
  search_down.setInputCloud(down_soa);
  search_down.build();
  stages.push_back({4, "Build search index for downsampled cloud", endStage(4, "Build search index", stage_start, progress_enabled), n_down});

  // Stage 5: Statistical Outlier Removal (Pure Scalar)
  beginStage(5, "Statistical outlier removal (Scalar SOR)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> sor_pts(n_down);
  std::size_t n_sor = n_down;
  if (skip_sor) {
    sor_pts = down_pts;
  } else {
    n_sor = sor_octree_scalar(down_soa, search_down, sor_pts.data(), config.sor_mean_k, config.sor_std_threshold);
    sor_pts.resize(n_sor);
  }
  saveStagePoints(output_dir / "02_sor_filtered.pcd", sor_pts, "SOR");
  stages.push_back({5, "Statistical outlier removal", endStage(5, "Statistical outlier removal", stage_start, progress_enabled), n_sor});

  // Stage 6: Rebuild Octree for filtered cloud
  std::vector<float> fx, fy, fz;
  PointCloudSoA filt_soa;
  aosToSoa(sor_pts, fx, fy, fz, filt_soa);

  beginStage(6, "Rebuild Octree index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  Octree search_filt;
  search_filt.setInputCloud(filt_soa);
  search_filt.build();
  stages.push_back({6, "Rebuild search index for filtered cloud", endStage(6, "Rebuild search index", stage_start, progress_enabled), n_sor});

  // Stage 7: Normal Estimation (Pure Scalar)
  beginStage(7, "Normal estimation (Scalar)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
  normal_estimation_octree_scalar(filt_soa, search_filt, nx.data(), ny.data(), nz.data(), config.normal_k, config.search_radius);
  stages.push_back({7, "Normal estimation", endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

  // Stage 8: RANSAC Ground Plane Fitting (Pure Scalar)
  beginStage(8, "RANSAC primitive fitting (Scalar)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  int inliers_count = ransac_plane_sc(sor_pts.data(), n_sor, config.ransac_distance_threshold,
                                      config.ransac_max_iterations, model);

  std::vector<PointXYZ> inlier_pts, outlier_pts;
  inlier_pts.reserve(n_sor);
  outlier_pts.reserve(n_sor);
  for (const auto &p : sor_pts) {
    float d = std::abs(model[0] * p.x + model[1] * p.y + model[2] * p.z + model[3]);
    if (d <= config.ransac_distance_threshold) {
      inlier_pts.push_back(p);
    } else {
      outlier_pts.push_back(p);
    }
  }
  saveStagePoints(output_dir / "04_ransac_inliers.pcd", inlier_pts, "RANSAC inliers");
  saveStagePoints(output_dir / "05_ground_plane_removed.pcd", outlier_pts, "Dominant plane removed");
  stages.push_back({8, "RANSAC primitive fitting", endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), outlier_pts.size()});

  std::cout << "Final plane coefficients: [" << model[0] << ", " << model[1]
            << ", " << model[2] << ", " << model[3] << "]" << std::endl;

  // Stage 9: Euclidean Clustering (Pure Scalar BFS)
  beginStage(9, "Euclidean clustering (Scalar BFS)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<float> ox, oy, oz;
  PointCloudSoA obs_soa;
  aosToSoa(outlier_pts, ox, oy, oz, obs_soa);

  Octree search_obs;
  search_obs.setInputCloud(obs_soa);
  search_obs.build();

  EuclideanClustering ec;
  ec.setInputCloud(obs_soa);
  ec.setNeighborSearch(&search_obs);
  ec.setClusterTolerance(config.cluster_tolerance);
  ec.setMinClusterSize(config.min_cluster_size);
  ec.setMaxClusterSize(config.max_cluster_size);
  std::vector<ClusterIndices> clusters = ec.extract();
  stages.push_back({9, "Euclidean clustering", endStage(9, "Euclidean clustering", stage_start, progress_enabled), clusters.size()});
  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // Stage 10: Write Cluster Stage
  beginStage(10, "Write cluster stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::size_t total_clustered_points = 0;
  for (const auto &c : clusters) total_clustered_points += c.indices.size();
  stages.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", stage_start, progress_enabled), total_clustered_points});

  const auto overall_end = std::chrono::high_resolution_clock::now();
  const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

  printFinalBreakdown(stages, total_ms);

  if (export_json) {
    saveJSONMetrics(output_dir / "pipeline_metrics_pcl.json", stages, total_ms, config.voxel_leaf_size, skip_sor, config.cluster_tolerance);
    std::cout << "Metrics saved to: " << (output_dir / "pipeline_metrics_pcl.json").string() << std::endl;
  }

  return 0;
}
