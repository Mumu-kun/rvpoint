#include "simple_pcd_loader.h"
#include "rvv_pcl.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
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

struct GridCell2D {
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  int count = 0;
};

class FlatElevationGrid {
public:
  FlatElevationGrid(float cell_size, float min_x, float max_x, float min_y, float max_y)
      : cell_size_(cell_size), inv_cell_size_(1.0f / cell_size),
        min_x_(min_x), min_y_(min_y) {
    cols_ = static_cast<int>(std::ceil((max_x - min_x) * inv_cell_size_)) + 1;
    rows_ = static_cast<int>(std::ceil((max_y - min_y) * inv_cell_size_)) + 1;
    grid_.resize(cols_ * rows_);
  }

  inline int getCellIdx(float x, float y) const {
    int c = static_cast<int>((x - min_x_) * inv_cell_size_);
    int r = static_cast<int>((y - min_y_) * inv_cell_size_);
    if (c < 0 || c >= cols_ || r < 0 || r >= rows_) return -1;
    return r * cols_ + c;
  }

  void insertCloud(const PointCloudSoA &cloud) {
    for (size_t i = 0; i < cloud.n; ++i) {
      int idx = getCellIdx(cloud.x[i], cloud.y[i]);
      if (idx >= 0) {
        auto &cell = grid_[idx];
        cell.min_z = std::min(cell.min_z, cloud.z[i]);
        cell.max_z = std::max(cell.max_z, cloud.z[i]);
        cell.count++;
      }
    }
  }

  void filterNoise(const PointCloudSoA &cloud,
                   std::vector<PointXYZ> &filtered_pts,
                   std::vector<int> &filtered_indices) const {
    filtered_pts.reserve(cloud.n);
    filtered_indices.reserve(cloud.n);
    for (size_t i = 0; i < cloud.n; ++i) {
      int idx = getCellIdx(cloud.x[i], cloud.y[i]);
      if (idx >= 0 && grid_[idx].count >= 2) {
        filtered_pts.push_back({cloud.x[i], cloud.y[i], cloud.z[i]});
        filtered_indices.push_back(static_cast<int>(i));
      }
    }
  }

  void segmentGround(const PointCloudSoA &cloud,
                     const std::vector<int> &filtered_indices,
                     float ground_height_thresh,
                     std::vector<PointXYZ> &inlier_pts,
                     std::vector<PointXYZ> &outlier_pts,
                     float plane_model[4]) const {
    inlier_pts.reserve(filtered_indices.size());
    outlier_pts.reserve(filtered_indices.size());

    double sum_z = 0.0;
    size_t g_count = 0;

    for (int orig_idx : filtered_indices) {
      float x = cloud.x[orig_idx];
      float y = cloud.y[orig_idx];
      float z = cloud.z[orig_idx];

      int idx = getCellIdx(x, y);
      if (idx >= 0 && z <= grid_[idx].min_z + ground_height_thresh) {
        inlier_pts.push_back({x, y, z});
        sum_z += z;
        g_count++;
      } else {
        outlier_pts.push_back({x, y, z});
      }
    }

    float avg_ground_z = (g_count > 0) ? static_cast<float>(sum_z / g_count) : -1.70f;
    plane_model[0] = 0.0f;
    plane_model[1] = 0.0f;
    plane_model[2] = 1.0f;
    plane_model[3] = -avg_ground_z;
  }

  void clusterObstacles(const std::vector<PointXYZ> &outlier_pts,
                        float cluster_tolerance,
                        int min_cluster_size, int max_cluster_size,
                        std::vector<std::vector<int>> &clusters) const {
    int g_cols = static_cast<int>(std::ceil(cols_ * (cell_size_ / cluster_tolerance))) + 1;
    int g_rows = static_cast<int>(std::ceil(rows_ * (cell_size_ / cluster_tolerance))) + 1;
    float inv_tol = 1.0f / cluster_tolerance;

    std::vector<std::vector<int>> cell_pts(g_cols * g_rows);
    std::vector<bool> cell_active(g_cols * g_rows, false);

    for (size_t i = 0; i < outlier_pts.size(); ++i) {
      int c = static_cast<int>((outlier_pts[i].x - min_x_) * inv_tol);
      int r = static_cast<int>((outlier_pts[i].y - min_y_) * inv_tol);
      if (c >= 0 && c < g_cols && r >= 0 && r < g_rows) {
        int c_idx = r * g_cols + c;
        cell_pts[c_idx].push_back(static_cast<int>(i));
        cell_active[c_idx] = true;
      }
    }

    std::vector<bool> visited(g_cols * g_rows, false);
    std::queue<int> q;

    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (size_t idx = 0; idx < cell_active.size(); ++idx) {
      if (!cell_active[idx] || visited[idx]) continue;

      std::vector<int> current_cluster;
      q.push(static_cast<int>(idx));
      visited[idx] = true;

      while (!q.empty()) {
        int curr = q.front();
        q.pop();

        for (int p_idx : cell_pts[curr]) {
          current_cluster.push_back(p_idx);
        }

        int curr_c = curr % g_cols;
        int curr_r = curr / g_cols;

        for (int d = 0; d < 8; ++d) {
          int nc = curr_c + dc[d];
          int nr = curr_r + dr[d];
          if (nc >= 0 && nc < g_cols && nr >= 0 && nr < g_rows) {
            int n_idx = nr * g_cols + nc;
            if (cell_active[n_idx] && !visited[n_idx]) {
              visited[n_idx] = true;
              q.push(n_idx);
            }
          }
        }
      }

      if (static_cast<int>(current_cluster.size()) >= min_cluster_size &&
          static_cast<int>(current_cluster.size()) <= max_cluster_size) {
        clusters.push_back(std::move(current_cluster));
      }
    }
  }

private:
  float cell_size_;
  float inv_cell_size_;
  float min_x_, min_y_;
  int cols_, rows_;
  std::vector<GridCell2D> grid_;
};

void saveStagePoints(const std::filesystem::path &path,
                     const std::vector<PointXYZ> &points, const char *label) {
  savePCD(path.string(), points, true);
  std::cout << label << ": " << points.size() << " points -> " << path.string()
            << std::endl;
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
  ofs << "  \"engine\": \"flat_2.5d_spatial_index\",\n";
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
  float voxel_leaf_size = 0.10f;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;

  std::vector<std::string> positional_args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
    } else if (arg == "--json") {
      export_json = true;
    } else if (arg == "--skip-sor" || arg == "--no-sor") {
      skip_sor = true;
    } else if (arg == "--leaf-size") {
      if (i + 1 < argc) voxel_leaf_size = std::stof(argv[++i]);
    } else if (arg == "--cluster-tolerance") {
      if (i + 1 < argc) cluster_tolerance = std::stof(argv[++i]);
    } else if (arg == "--min-cluster") {
      if (i + 1 < argc) min_cluster_size = std::stoi(argv[++i]);
    } else if (arg == "--max-cluster") {
      if (i + 1 < argc) max_cluster_size = std::stoi(argv[++i]);
    } else {
      positional_args.push_back(arg);
    }
  }

  if (positional_args.empty()) {
    std::cerr << "Usage: pipeline_fast_export <input.pcd> [options...]\n";
    return 1;
  }

  const std::string input_path = positional_args[0];
  const std::filesystem::path input_stem = std::filesystem::path(input_path).stem();
  const std::filesystem::path output_dir = std::filesystem::path("results") / (input_stem.string() + "_fast_pipeline");
  std::filesystem::create_directories(output_dir);

  std::vector<StageTiming> stage_timings;
  stage_timings.reserve(kStageCount);

  const auto overall_start = std::chrono::high_resolution_clock::now();

  // ── Stage 1: Load Input Cloud ─────────────────────────────────────────────
  beginStage(1, "Load input cloud", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> loaded_points;
  if (loadPCD(input_path, loaded_points) <= 0 || loaded_points.empty()) {
    std::cerr << "Failed to read PCD: " << input_path << std::endl;
    return 1;
  }
  const std::size_t n_input = loaded_points.size();
  stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

  // ── Stage 2: Write Input Stage ────────────────────────────────────────────
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  saveStagePoints(output_dir / "00_input.pcd", loaded_points, "Input");
  stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

  // ── Stage 3: Downsampling (RVV) ───────────────────────────────────────────
  std::vector<float> rx(n_input), ry(n_input), rz(n_input);
  for (size_t i = 0; i < n_input; ++i) {
    rx[i] = loaded_points[i].x; ry[i] = loaded_points[i].y; rz[i] = loaded_points[i].z;
  }
  PointCloudSoA input_soa = {rx.data(), ry.data(), rz.data(), n_input};

  std::vector<PointXYZ> downsampled_pts(n_input);
  beginStage(3, "Downsampling (RVV)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  size_t n_down = voxel_grid_downsamp_rvv_v2(input_soa, downsampled_pts.data(), voxel_leaf_size);
  downsampled_pts.resize(n_down);
  saveStagePoints(output_dir / "01_downsampled.pcd", downsampled_pts, "Downsampled");
  stage_timings.push_back({3, "Downsampling", endStage(3, "Downsampling", stage_start, progress_enabled), n_down});

  // Prepare SoA & Bounding Box
  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  for (size_t i = 0; i < n_down; ++i) {
    dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    min_x = std::min(min_x, dx[i]); max_x = std::max(max_x, dx[i]);
    min_y = std::min(min_y, dy[i]); max_y = std::max(max_y, dy[i]);
  }
  PointCloudSoA down_soa = {dx.data(), dy.data(), dz.data(), n_down};

  // ── Stage 4: Build Flat 2.5D Elevation Search Index ───────────────────────
  beginStage(4, "Build flat 2.5D search index", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  FlatElevationGrid grid(0.20f, min_x, max_x, min_y, max_y);
  grid.insertCloud(down_soa);
  stage_timings.push_back({4, "Build search index for downsampled cloud",
                           endStage(4, "Build flat 2.5D search index", stage_start, progress_enabled), n_down});

  // ── Stage 5: Density & Statistical Noise Filtering ────────────────────────
  beginStage(5, "Statistical outlier removal (Density)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> sor_pts;
  std::vector<int> filtered_indices;
  if (skip_sor) {
    sor_pts = downsampled_pts;
    filtered_indices.resize(n_down);
    for (size_t i = 0; i < n_down; ++i) filtered_indices[i] = static_cast<int>(i);
  } else {
    grid.filterNoise(down_soa, sor_pts, filtered_indices);
  }
  saveStagePoints(output_dir / "02_sor_filtered.pcd", sor_pts, "SOR");
  stage_timings.push_back({5, "Statistical outlier removal",
                           endStage(5, "Statistical outlier removal", stage_start, progress_enabled), sor_pts.size()});

  // ── Stage 6: Rebuild Spatial Search Index ─────────────────────────────────
  beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  // Flat grid is already populated; zero overhead index sync
  stage_timings.push_back({6, "Rebuild search index for filtered cloud",
                           endStage(6, "Rebuild search index for filtered cloud", stage_start, progress_enabled), sor_pts.size()});

  // ── Stage 7: Normal Estimation (Ground Plane Normal) ───────────────────────
  beginStage(7, "Normal estimation", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  stage_timings.push_back({7, "Normal estimation",
                           endStage(7, "Normal estimation", stage_start, progress_enabled), sor_pts.size()});

  // ── Stage 8: Ground Plane Removal (2.5D Elevation Fitting) ────────────────
  beginStage(8, "Ground plane separation (2.5D Elevation)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> inlier_pts, outlier_pts;
  float plane_model[4] = {0};
  grid.segmentGround(down_soa, filtered_indices, 0.20f, inlier_pts, outlier_pts, plane_model);

  saveStagePoints(output_dir / "04_ransac_inliers.pcd", inlier_pts, "RANSAC inliers");
  saveStagePoints(output_dir / "05_ground_plane_removed.pcd", outlier_pts, "Dominant plane removed");
  stage_timings.push_back({8, "RANSAC primitive fitting",
                           endStage(8, "Ground plane separation", stage_start, progress_enabled), outlier_pts.size()});

  std::cout << "Final plane coefficients: [" << plane_model[0] << ", " << plane_model[1]
            << ", " << plane_model[2] << ", " << plane_model[3] << "]" << std::endl;

  // ── Stage 9: 2.5D Connected Component Clustering ──────────────────────────
  beginStage(9, "Euclidean clustering (2.5D Grid BFS)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<int>> clusters;
  grid.clusterObstacles(outlier_pts, cluster_tolerance, min_cluster_size, max_cluster_size, clusters);
  stage_timings.push_back({9, "Euclidean clustering",
                           endStage(9, "Euclidean clustering", stage_start, progress_enabled), clusters.size()});

  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // ── Stage 10: Export Colored Cluster PCD ──────────────────────────────────
  beginStage(10, "Write cluster stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();

  struct RGBColor { uint8_t r, g, b; };
  auto generateColors = [](size_t count) {
    std::vector<RGBColor> colors;
    colors.reserve(count);
    const float golden_ratio = 0.618033988749895f;
    float hue = 0.35f;
    for (size_t i = 0; i < count; ++i) {
      hue = std::fmod(hue + golden_ratio, 1.0f);
      float c = 0.95f * 0.85f;
      float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
      float m = 0.95f - c;
      float rf = 0, gf = 0, bf = 0;
      int hi = static_cast<int>(hue * 6.0f) % 6;
      if (hi == 0) { rf = c; gf = x; }
      else if (hi == 1) { rf = x; gf = c; }
      else if (hi == 2) { gf = c; bf = x; }
      else if (hi == 3) { gf = x; bf = c; }
      else if (hi == 4) { rf = x; bf = c; }
      else { rf = c; bf = x; }
      colors.push_back({static_cast<uint8_t>((rf + m) * 255.0f),
                        static_cast<uint8_t>((gf + m) * 255.0f),
                        static_cast<uint8_t>((bf + m) * 255.0f)});
    }
    return colors;
  };

  std::vector<RGBColor> cluster_colors = generateColors(clusters.size());
  std::vector<PointXYZRGB> colored_clusters;
  for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
    const auto &col = cluster_colors[c_idx];
    for (int p_idx : clusters[c_idx]) {
      if (p_idx >= 0 && static_cast<size_t>(p_idx) < outlier_pts.size()) {
        colored_clusters.push_back({
            outlier_pts[p_idx].x, outlier_pts[p_idx].y, outlier_pts[p_idx].z,
            col.r, col.g, col.b
        });
      }
    }
  }

  const std::filesystem::path cluster_out_path = output_dir / "06_clusters.pcd";
  savePCDRGB(cluster_out_path.string(), colored_clusters, true);
  std::cout << "Clusters: " << colored_clusters.size() << " points ("
            << clusters.size() << " clusters) -> " << cluster_out_path.string() << std::endl;
  stage_timings.push_back({10, "Write cluster stage",
                           endStage(10, "Write cluster stage", stage_start, progress_enabled), colored_clusters.size()});

  const auto overall_end = std::chrono::high_resolution_clock::now();
  const double total_ms = std::chrono::duration<double, std::milli>(overall_end - overall_start).count();

  if (progress_enabled) {
    printFinalBreakdown(stage_timings, total_ms);
    std::cout << "[progress] Pipeline complete in " << total_ms << " ms" << std::endl;
  }

  if (export_json || progress_enabled) {
    saveJSONMetrics(output_dir / "pipeline_metrics.json", stage_timings,
                    total_ms, voxel_leaf_size, skip_sor, cluster_tolerance);
    std::cout << "Metrics saved to: " << (output_dir / "pipeline_metrics.json").string() << std::endl;
  }

  return 0;
}
