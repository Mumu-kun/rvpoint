#include "euclidean_clustering.h"
#include "pointer_octree/pointer_octree.h"
#include "profiler.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using namespace rvv_pcl;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  const char *label;
  double ms;
  std::size_t point_count;
};

struct PipelineConfig {
  float voxel_leaf_size = 0.10f;
  float search_radius = 0.03f;
  int sor_mean_k = 20;
  float sor_std_threshold = 1.0f;
  float sor_search_radius = 0.25f;
  int normal_k = 10;
  float ransac_distance_threshold = 0.20f;
  int ransac_max_iterations = 1000;
  float ransac_probability = 0.99f;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;
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

void saveStagePoints(const std::filesystem::path &path,
                     const std::vector<PointXYZ> &points, const char *label, bool disable_disk) {
  if (disable_disk) return;
  savePCD(path.string(), points, true);
  std::cout << label << ": " << points.size() << " points -> " << path.string() << std::endl;
}

void beginStage(int index, const char *label, bool enabled) {
  if (!enabled) return;
  std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const char *label,
                const std::chrono::high_resolution_clock::time_point &start,
                bool enabled) {
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
    std::cout << "[progress] [" << stage.index << "/" << kStageCount << "] "
              << stage.label << ": " << std::fixed << std::setprecision(3)
              << stage.ms << " ms (" << std::setprecision(2) << pct << "%), pts="
              << stage.point_count << std::endl;
  }

  const double overhead_ms = total_ms - stages_sum_ms;
  if (std::abs(overhead_ms) > 0.01) {
    const double overhead_pct = total_ms > 0.0 ? (overhead_ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [--] Outside timed stages: " << std::fixed
              << std::setprecision(3) << overhead_ms << " ms ("
              << std::setprecision(2) << overhead_pct << "%)" << std::endl;
  }

  std::cout << "[progress] [--] Total: " << std::fixed << std::setprecision(3)
            << total_ms << " ms (100.00%)" << std::endl;
}

void saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages,
                     double total_ms, float leaf_size, bool skip_sor,
                     float cluster_tolerance) {
  std::ofstream ofs(out_path);
  if (!ofs.is_open()) return;

  ofs << "{\n";
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

// Optimized Vectorized PointerOctree SOR
std::size_t sor_pointer_octree_fast(const PointCloudSoA &in,
                                    const PointerOctree &tree,
                                    PointXYZ *out, int k, float alpha,
                                    float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);
  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;
  nbr_indices.reserve(256);
  nbr_dists.reserve(256);

  double total_sum = 0.0;
  double total_sq_sum = 0.0;
  size_t valid_count = 0;

  for (std::size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    nbr_indices.clear();
    nbr_dists.clear();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

    if (nbr_dists.size() > 1) {
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k, nbr_dists.end());

#if defined(__riscv) || defined(__riscv_vector)
      size_t vl = __riscv_vsetvl_e32m4(valid_k);
      vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(nbr_dists.data() + 1, vl);
      vfloat32m4_t vd = __riscv_vfsqrt_v_f32m4(vd2, vl);
      vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
      vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
      float sum = __riscv_vfmv_f_s_f32m1_f32(v_sum);
#else
      float sum = 0.0f;
      for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(nbr_dists[j]);
#endif
      float m = sum / static_cast<float>(valid_k);
      mean_dists[i] = m;
      total_sum += m;
      total_sq_sum += (m * m);
      valid_count++;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  if (valid_count == 0) return 0;
  double d_cnt = static_cast<double>(valid_count);
  double global_mean = total_sum / d_cnt;
  double variance = (total_sq_sum / d_cnt) - (global_mean * global_mean);
  double stddev = std::sqrt(std::max(0.0, variance));
  float thresh = static_cast<float>(global_mean + alpha * stddev);

  std::size_t count = 0;
  for (std::size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count++] = {in.x[i], in.y[i], in.z[i]};
    }
  }
  return count;
}

// Optimized Cardano Normal Estimation
void normal_estimation_cardano(const PointCloudSoA& cloud, const PointerOctree& tree,
                               float* nx, float* ny, float* nz, int k, float search_radius) {
  std::vector<int> nbrs;
  std::vector<float> dists;
  nbrs.reserve(64);
  dists.reserve(64);

  for (size_t i = 0; i < cloud.n; ++i) {
    PointXYZ q = {cloud.x[i], cloud.y[i], cloud.z[i]};
    nbrs.clear(); dists.clear();
    tree.radiusSearch(q, search_radius, nbrs, dists);

    if (nbrs.size() < 3) {
      nx[i] = 0.0f; ny[i] = 0.0f; nz[i] = 1.0f;
      continue;
    }

    float cx = 0, cy = 0, cz = 0;
    for (int idx : nbrs) {
      cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
    }
    float inv_n = 1.0f / static_cast<float>(nbrs.size());
    cx *= inv_n; cy *= inv_n; cz *= inv_n;

    float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
    for (int idx : nbrs) {
      float dx = cloud.x[idx] - cx, dy = cloud.y[idx] - cy, dz = cloud.z[idx] - cz;
      c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
      c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
    }

    float vx = c01 * c12 - c02 * c11;
    float vy = c01 * c02 - c00 * c12;
    float vz = c00 * c11 - c01 * c01;
    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (norm > 1e-6f) {
      float inv_norm = 1.0f / norm;
      nx[i] = vx * inv_norm; ny[i] = vy * inv_norm; nz[i] = vz * inv_norm;
    } else {
      nx[i] = 0.0f; ny[i] = 0.0f; nz[i] = 1.0f;
    }
  }
}

} // namespace

int main(int argc, char **argv) {
  bool progress_enabled = false;
  bool skip_sor = false;
  bool export_json = false;
  bool scalar_mode = false;
  bool disable_disk = false;
  float voxel_leaf_size = kPipelineConfig.voxel_leaf_size;
  float cluster_tolerance = kPipelineConfig.cluster_tolerance;
  int min_cluster_size = kPipelineConfig.min_cluster_size;
  int max_cluster_size = kPipelineConfig.max_cluster_size;

  std::vector<StageTiming> stage_timings;
  stage_timings.reserve(kStageCount);
  std::vector<std::string> positional_args;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
    } else if (arg == "--json" || arg == "--json-metrics") {
      export_json = true;
    } else if (arg == "--skip-sor" || arg == "--no-sor" || arg == "--without-sor") {
      skip_sor = true;
    } else if (arg == "--scalar") {
      scalar_mode = true;
    } else if (arg == "--no-write" || arg == "--disable-disk") {
      disable_disk = true;
    } else if (arg == "--leaf-size" && i + 1 < argc) {
      voxel_leaf_size = std::stof(argv[++i]);
    } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
      cluster_tolerance = std::stof(argv[++i]);
    } else if (arg == "--min-cluster" && i + 1 < argc) {
      min_cluster_size = std::stoi(argv[++i]);
    } else if (arg == "--max-cluster" && i + 1 < argc) {
      max_cluster_size = std::stoi(argv[++i]);
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " [--progress] [--json] [--no-write] [--leaf-size <val>] "
                 "[--cluster-tolerance <val>] [--min-cluster <val>] "
                 "[--max-cluster <val>] <input.pcd> [output_dir]"
              << std::endl;
    return 1;
  }

  const auto overall_start = std::chrono::high_resolution_clock::now();

  const std::string input_path = resolveInputPath(positional_args[0]);
  const std::filesystem::path input_stem =
      std::filesystem::path(positional_args[0]).stem();
  const std::filesystem::path output_dir =
      positional_args.size() >= 2 ? std::filesystem::path(positional_args[1])
                                  : std::filesystem::path("results") /
                                        (input_stem.string() + "_pipeline");

  if (!disable_disk) {
    std::error_code dir_ec;
    std::filesystem::create_directories(output_dir, dir_ec);
  }

  // ── Stage 1: Load input cloud ────────────────────────────────────────────
  std::vector<PointXYZ> loaded_points;
  beginStage(1, "Load input cloud", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  const int count = loadPCD(input_path, loaded_points);
  if (count < 0) {
    std::cerr << "Failed to load input cloud: " << positional_args[0] << std::endl;
    return 1;
  }
  const std::size_t n_input = loaded_points.size();
  stage_timings.push_back(
      {1, "Load input cloud",
       endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

  std::vector<float> ix(n_input), iy(n_input), iz(n_input);
  for (std::size_t i = 0; i < n_input; ++i) {
    ix[i] = loaded_points[i].x;
    iy[i] = loaded_points[i].y;
    iz[i] = loaded_points[i].z;
  }
  PointCloudSoA input_cloud = {ix.data(), iy.data(), iz.data(), n_input};

  // ── Stage 2: Write input stage ───────────────────────────────────────────
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  saveStagePoints(output_dir / "00_input.pcd", loaded_points, "Input", disable_disk);
  stage_timings.push_back(
      {2, "Write input stage",
       endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

  // ── Stage 3: Downsampling ────────────────────────────────────────────────
  std::vector<PointXYZ> downsampled_pts(n_input);
  beginStage(3, scalar_mode ? "Downsampling (Scalar)" : "Downsampling (RVV)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::size_t n_down = 0;
  if (scalar_mode) {
    n_down = voxel_grid_downsamp_sc(loaded_points.data(), loaded_points.size(),
                                   downsampled_pts.data(), voxel_leaf_size);
  } else {
    n_down = voxel_grid_downsamp_rvv_v2(input_cloud, downsampled_pts.data(), voxel_leaf_size);
  }
  downsampled_pts.resize(n_down);
  saveStagePoints(output_dir / "01_downsampled.pcd", downsampled_pts, "Downsampled", disable_disk);
  stage_timings.push_back(
      {3, scalar_mode ? "Downsampling (Scalar)" : "Downsampling",
       endStage(3, "Downsampling", stage_start, progress_enabled), n_down});

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (std::size_t i = 0; i < n_down; ++i) {
    dx[i] = downsampled_pts[i].x;
    dy[i] = downsampled_pts[i].y;
    dz[i] = downsampled_pts[i].z;
  }
  PointCloudSoA downsampled_cloud = {dx.data(), dy.data(), dz.data(), n_down};

  // ── Stage 4: Build Search Index 1 (PointerOctree) ────────────────────────
  PointerOctree search;
  search.setInputCloud(downsampled_cloud);
  beginStage(4, "Build search index for downsampled cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  {
    RVPOINT_PROFILE_SCOPE("PointerOctree::build_downsampled");
    search.build();
  }
  stage_timings.push_back(
      {4, "Build search index for downsampled cloud",
       endStage(4, "Build search index for downsampled cloud", stage_start, progress_enabled), n_down});

  // ── Stage 5: Statistical Outlier Removal ─────────────────────────────────
  std::vector<PointXYZ> sor_pts(n_down);
  std::size_t n_sor = n_down;
  beginStage(5, scalar_mode ? "Statistical outlier removal (Scalar)" : "Statistical outlier removal", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  if (skip_sor) {
    sor_pts = downsampled_pts;
  } else {
    RVPOINT_PROFILE_SCOPE("SOR_pointer_octree_execution");
    n_sor = sor_pointer_octree_fast(downsampled_cloud, search, sor_pts.data(),
                                    kPipelineConfig.sor_mean_k,
                                    kPipelineConfig.sor_std_threshold,
                                    kPipelineConfig.sor_search_radius);
    sor_pts.resize(n_sor);
  }
  saveStagePoints(output_dir / "02_sor_filtered.pcd", sor_pts, "SOR", disable_disk);
  stage_timings.push_back(
      {5, scalar_mode ? "Statistical outlier removal (Scalar)" : "Statistical outlier removal",
       endStage(5, "Statistical outlier removal", stage_start, progress_enabled), n_sor});

  std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
  for (std::size_t i = 0; i < n_sor; ++i) {
    sx[i] = sor_pts[i].x;
    sy[i] = sor_pts[i].y;
    sz[i] = sor_pts[i].z;
  }
  PointCloudSoA sor_cloud = {sx.data(), sy.data(), sz.data(), n_sor};

  // ── Stage 6: Rebuild Search Index 2 (PointerOctree) ──────────────────────
  PointerOctree filtered_search;
  filtered_search.setInputCloud(sor_cloud);
  beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  {
    RVPOINT_PROFILE_SCOPE("PointerOctree::rebuild_filtered");
    filtered_search.build();
  }
  stage_timings.push_back(
      {6, "Rebuild search index for filtered cloud",
       endStage(6, "Rebuild search index for filtered cloud", stage_start, progress_enabled), n_sor});

  // ── Stage 7: Normal Estimation ───────────────────────────────────────────
  std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
  beginStage(7, "Normal estimation", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  {
    RVPOINT_PROFILE_SCOPE("normal_estimation_cardano");
    normal_estimation_cardano(sor_cloud, filtered_search, nx.data(), ny.data(), nz.data(),
                              kPipelineConfig.normal_k, kPipelineConfig.search_radius);
  }
  stage_timings.push_back(
      {7, "Normal estimation",
       endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

  // ── Stage 8: RANSAC Primitive Plane Fitting ──────────────────────────────
  float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<PointXYZ> inlier_pts(n_sor);
  std::vector<PointXYZ> outlier_pts(n_sor);
  std::size_t n_inliers = 0, n_outliers = 0;

  beginStage(8, scalar_mode ? "RANSAC primitive fitting (Scalar)" : "RANSAC primitive fitting", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  int ransac_inliers_count = 0;
  if (scalar_mode) {
    ransac_inliers_count =
        ransac_plane_sc(sor_pts.data(), n_sor, kPipelineConfig.ransac_distance_threshold,
                        kPipelineConfig.ransac_max_iterations, model);
    for (std::size_t i = 0; i < n_sor; ++i) {
      float dist = std::abs(model[0] * sor_pts[i].x + model[1] * sor_pts[i].y +
                            model[2] * sor_pts[i].z + model[3]);
      if (dist <= kPipelineConfig.ransac_distance_threshold) {
        inlier_pts[n_inliers++] = sor_pts[i];
      } else {
        outlier_pts[n_outliers++] = sor_pts[i];
      }
    }
  } else {
    RVPOINT_PROFILE_SCOPE("ransac_plane_rvv");
    ransac_inliers_count =
        ransac_plane_rvv(sor_cloud, kPipelineConfig.ransac_distance_threshold,
                         kPipelineConfig.ransac_max_iterations, model);
    if (ransac_inliers_count > 0) {
      RVPOINT_PROFILE_SCOPE("extract_plane_inliers_outliers_rvv");
      extract_plane_inliers_outliers_rvv(
          sor_cloud, model, kPipelineConfig.ransac_distance_threshold,
          inlier_pts.data(), outlier_pts.data(), n_inliers, n_outliers);
    }
  }
  if (ransac_inliers_count <= 0) {
    std::cerr << "RANSAC failed to fit a model." << std::endl;
    return 1;
  }
  inlier_pts.resize(n_inliers);
  outlier_pts.resize(n_outliers);

  saveStagePoints(output_dir / "04_ransac_inliers.pcd", inlier_pts, "RANSAC inliers", disable_disk);
  saveStagePoints(output_dir / "05_ground_plane_removed.pcd", outlier_pts, "Dominant plane removed", disable_disk);
  stage_timings.push_back(
      {8, scalar_mode ? "RANSAC primitive fitting (Scalar)" : "RANSAC primitive fitting",
       endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled), n_outliers});

  std::cout << "Final plane coefficients: [" << model[0] << ", " << model[1]
            << ", " << model[2] << ", " << model[3] << "]" << std::endl;

  // ── Stage 9: Euclidean Clustering ────────────────────────────────────────
  beginStage(9, "Euclidean clustering", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();

  std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
  for (std::size_t i = 0; i < n_outliers; ++i) {
    ox[i] = outlier_pts[i].x;
    oy[i] = outlier_pts[i].y;
    oz[i] = outlier_pts[i].z;
  }
  PointCloudSoA non_ground_cloud = {ox.data(), oy.data(), oz.data(), n_outliers};

  PointerOctree non_ground_search;
  non_ground_search.setInputCloud(non_ground_cloud);
  {
    RVPOINT_PROFILE_SCOPE("PointerOctree::build_non_ground");
    non_ground_search.build();
  }

  EuclideanClustering ec;
  ec.setInputCloud(non_ground_cloud);
  ec.setNeighborSearch(&non_ground_search);
  ec.setClusterTolerance(cluster_tolerance);
  ec.setMinClusterSize(min_cluster_size);
  ec.setMaxClusterSize(max_cluster_size);

  std::vector<ClusterIndices> clusters;
  {
    RVPOINT_PROFILE_SCOPE("EuclideanClustering::extract");
    clusters = ec.extract();
  }
  stage_timings.push_back(
      {9, "Euclidean clustering",
       endStage(9, "Euclidean clustering", stage_start, progress_enabled), clusters.size()});

  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // ── Stage 10: Export colored cluster cloud ───────────────────────────────
  beginStage(10, "Write cluster stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();

  std::size_t total_clustered_pts = 0;
  for (const auto &cls : clusters) total_clustered_pts += cls.indices.size();

  if (!disable_disk) {
    struct RGBColor { std::uint8_t r, g, b; };
    auto generateClusterColors = [](std::size_t count) {
      std::vector<RGBColor> colors;
      colors.reserve(count);
      const float golden_ratio = 0.618033988749895f;
      float hue = 0.35f;

      for (std::size_t i = 0; i < count; ++i) {
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

        colors.push_back({
          static_cast<std::uint8_t>((r_f + m) * 255.0f),
          static_cast<std::uint8_t>((g_f + m) * 255.0f),
          static_cast<std::uint8_t>((b_f + m) * 255.0f)
        });
      }
      return colors;
    };

    std::vector<RGBColor> cluster_colors = generateClusterColors(clusters.size());
    std::vector<PointXYZRGB> colored_cluster_pts;
    colored_cluster_pts.reserve(total_clustered_pts);

    for (std::size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
      const RGBColor &col = cluster_colors[c_idx];
      for (int pt_idx : clusters[c_idx].indices) {
        if (pt_idx >= 0 && static_cast<std::size_t>(pt_idx) < n_outliers) {
          colored_cluster_pts.push_back({
              outlier_pts[pt_idx].x,
              outlier_pts[pt_idx].y,
              outlier_pts[pt_idx].z,
              col.r,
              col.g,
              col.b
          });
        }
      }
    }

    const std::filesystem::path cluster_out_path = output_dir / "06_clusters.pcd";
    savePCDRGB(cluster_out_path.string(), colored_cluster_pts, true);
    std::cout << "Clusters: " << colored_cluster_pts.size() << " points ("
              << clusters.size() << " clusters) -> " << cluster_out_path.string()
              << std::endl;
  }

  stage_timings.push_back(
      {10, "Write cluster stage",
       endStage(10, "Write cluster stage", stage_start, progress_enabled), total_clustered_pts});

  const auto overall_end = std::chrono::high_resolution_clock::now();
  const double total_ms =
      std::chrono::duration<double, std::milli>(overall_end - overall_start)
          .count();

  if (progress_enabled) {
    printFinalBreakdown(stage_timings, total_ms);
    std::cout << "[progress] Pipeline complete in " << total_ms << " ms" << std::endl;
  }

  if (export_json || progress_enabled) {
    saveJSONMetrics(output_dir / "pipeline_metrics.json", stage_timings,
                    total_ms, voxel_leaf_size, skip_sor, cluster_tolerance);
  }
  return 0;
}
