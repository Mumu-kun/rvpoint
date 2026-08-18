// pipeline_rvv_ultra_fast.cpp
// Hardware-Accelerated 2.5D Elevation Pipeline using RVV 1.0 Vector Gather & Compress Kernels
// Target: Orange Pi RV2 (SpacemiT K1 / TH1520 RISC-V 64-bit + RVV 1.0)

#include "io/simple_pcd_loader.h"
#include "core/point_types.h"
#include "filters/voxel_grid.h"

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

using namespace rvpoint;
using Clock = std::chrono::high_resolution_clock;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  const char *label;
  double ms;
  std::size_t point_count;
};

// ── Ultra-Fast Flat Grid with Hardware RVV Vectorized Gather/Compress ─────────
class RVVFlatElevationGrid {
public:
  RVVFlatElevationGrid(float cell_size, float min_x, float max_x, float min_y, float max_y)
      : cell_size_(cell_size), inv_cell_size_(1.0f / cell_size),
        min_x_(min_x), min_y_(min_y) {
    cols_ = static_cast<int>(std::ceil((max_x - min_x) * inv_cell_size_)) + 1;
    rows_ = static_cast<int>(std::ceil((max_y - min_y) * inv_cell_size_)) + 1;
    grid_min_z_.assign(cols_ * rows_, 1e9f);
    grid_count_.assign(cols_ * rows_, 0);
  }

  // 1. Hardware RVV Vectorized Grid Insertion & Min-Z Tracking
  void insertCloudRVV(const PointCloudSoA &cloud, std::vector<uint32_t> &cell_indices) {
    cell_indices.resize(cloud.n);
    const size_t n = cloud.n;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
    size_t i = 0;
    while (i < n) {
      size_t vl = __riscv_vsetvl_e32m8(n - i);

      vfloat32m8_t vx = __riscv_vle32_v_f32m8(cloud.x + i, vl);
      vfloat32m8_t vy = __riscv_vle32_v_f32m8(cloud.y + i, vl);

      // (x - min_x) * inv_cell
      vfloat32m8_t v_norm_x = __riscv_vfmul_vf_f32m8(__riscv_vfsub_vf_f32m8(vx, min_x_, vl), inv_cell_size_, vl);
      vfloat32m8_t v_norm_y = __riscv_vfmul_vf_f32m8(__riscv_vfsub_vf_f32m8(vy, min_y_, vl), inv_cell_size_, vl);

      // Fast float to unsigned int conversion
      vuint32m8_t vc = __riscv_vfcvt_rtz_xu_f_v_u32m8(v_norm_x, vl);
      vuint32m8_t vr = __riscv_vfcvt_rtz_xu_f_v_u32m8(v_norm_y, vl);

      // Linear cell idx = r * cols + c
      vuint32m8_t v_idx = __riscv_vmacc_vx_u32m8(vc, cols_, vr, vl);

      __riscv_vse32_v_u32m8(cell_indices.data() + i, v_idx, vl);

      // Update cell statistics (scattered reduction)
      for (size_t lane = 0; lane < vl; ++lane) {
        uint32_t idx = cell_indices[i + lane];
        if (idx < grid_min_z_.size()) {
          float z = cloud.z[i + lane];
          if (z < grid_min_z_[idx]) grid_min_z_[idx] = z;
          grid_count_[idx]++;
        }
      }

      i += vl;
    }
#else
    for (size_t i = 0; i < n; ++i) {
      int c = static_cast<int>((cloud.x[i] - min_x_) * inv_cell_size_);
      int r = static_cast<int>((cloud.y[i] - min_y_) * inv_cell_size_);
      uint32_t idx = static_cast<uint32_t>(r * cols_ + c);
      cell_indices[i] = idx;
      if (idx < grid_min_z_.size()) {
        if (cloud.z[i] < grid_min_z_[idx]) grid_min_z_[idx] = cloud.z[i];
        grid_count_[idx]++;
      }
    }
#endif
  }

  // 2. Hardware RVV Vectorized Ground Separation & Outlier Extraction
  void segmentGroundRVV(const PointCloudSoA &cloud,
                        const std::vector<uint32_t> &cell_indices,
                        float ground_height_thresh,
                        std::vector<PointXYZ> &inlier_pts,
                        std::vector<PointXYZ> &outlier_pts) const {
    const size_t n = cloud.n;
    inlier_pts.reserve(n);
    outlier_pts.reserve(n);

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
    const float *min_z_ptr = grid_min_z_.data();
    const uint32_t *count_ptr = grid_count_.data();

    std::vector<float> in_x(n), in_y(n), in_z(n);
    std::vector<float> out_x(n), out_y(n), out_z(n);
    size_t in_count = 0, out_count = 0;

    size_t i = 0;
    while (i < n) {
      size_t vl = __riscv_vsetvl_e32m8(n - i);

      vfloat32m8_t vx = __riscv_vle32_v_f32m8(cloud.x + i, vl);
      vfloat32m8_t vy = __riscv_vle32_v_f32m8(cloud.y + i, vl);
      vfloat32m8_t vz = __riscv_vle32_v_f32m8(cloud.z + i, vl);
      vuint32m8_t v_idx = __riscv_vle32_v_u32m8(cell_indices.data() + i, vl);

      // Vector Indexed Gather Load (Load cell min_z for entire vector register in parallel!)
      vuint32m8_t byte_offsets = __riscv_vsll_vx_u32m8(v_idx, 2, vl);
      vfloat32m8_t v_min_z = __riscv_vluxei32_v_f32m8(min_z_ptr, byte_offsets, vl);
      vuint32m8_t v_cell_cnt = __riscv_vluxei32_v_u32m8(count_ptr, byte_offsets, vl);

      vfloat32m8_t v_thresh = __riscv_vfadd_vf_f32m8(v_min_z, ground_height_thresh, vl);

      // Vector Ground Mask: z <= min_z + thresh
      vbool4_t is_ground = __riscv_vmfle_vv_f32m8_b4(vz, v_thresh, vl);

      // Vector Density Mask: cell_count >= 2 (reject noise)
      vbool4_t is_dense = __riscv_vmsgeu_vx_u32m8_b4(v_cell_cnt, 2, vl);
      vbool4_t is_obstacle = __riscv_vmand_mm_b4(__riscv_vmnot_m_b4(is_ground, vl), is_dense, vl);

      size_t n_inliers = __riscv_vcpop_m_b4(is_ground, vl);
      size_t n_outliers = __riscv_vcpop_m_b4(is_obstacle, vl);

      if (n_inliers > 0) {
        vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, is_ground, vl);
        vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, is_ground, vl);
        vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, is_ground, vl);

        __riscv_vse32_v_f32m8(in_x.data() + in_count, cx, n_inliers);
        __riscv_vse32_v_f32m8(in_y.data() + in_count, cy, n_inliers);
        __riscv_vse32_v_f32m8(in_z.data() + in_count, cz, n_inliers);
        in_count += n_inliers;
      }

      if (n_outliers > 0) {
        vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, is_obstacle, vl);
        vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, is_obstacle, vl);
        vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, is_obstacle, vl);

        __riscv_vse32_v_f32m8(out_x.data() + out_count, cx, n_outliers);
        __riscv_vse32_v_f32m8(out_y.data() + out_count, cy, n_outliers);
        __riscv_vse32_v_f32m8(out_z.data() + out_count, cz, n_outliers);
        out_count += n_outliers;
      }

      i += vl;
    }

    inlier_pts.resize(in_count);
    for (size_t k = 0; k < in_count; ++k) inlier_pts[k] = {in_x[k], in_y[k], in_z[k]};

    outlier_pts.resize(out_count);
    for (size_t k = 0; k < out_count; ++k) outlier_pts[k] = {out_x[k], out_y[k], out_z[k]};
#else
    for (size_t i = 0; i < n; ++i) {
      uint32_t idx = cell_indices[i];
      if (idx < grid_min_z_.size()) {
        if (cloud.z[i] <= grid_min_z_[idx] + ground_height_thresh) {
          inlier_pts.push_back({cloud.x[i], cloud.y[i], cloud.z[i]});
        } else if (grid_count_[idx] >= 2) {
          outlier_pts.push_back({cloud.x[i], cloud.y[i], cloud.z[i]});
        }
      }
    }
#endif
  }

  // 3. 2.5D Grid Fast BFS Clustering
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
  std::vector<float> grid_min_z_;
  std::vector<uint32_t> grid_count_;
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

} // namespace

int main(int argc, char **argv) {
  bool progress_enabled = true;
  float voxel_leaf_size = 0.10f;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;

  std::vector<std::string> positional_args;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--progress" || arg == "--timings") {
      progress_enabled = true;
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
    std::cerr << "Usage: pipeline_rvv_ultra_fast <input.pcd> [options...]\n";
    return 1;
  }

  const std::string input_path = positional_args[0];
  const std::filesystem::path input_stem = std::filesystem::path(input_path).stem();
  const std::filesystem::path output_dir = std::filesystem::path("results") / (input_stem.string() + "_rvv_ultra_pipeline");
  std::filesystem::create_directories(output_dir);

  std::vector<StageTiming> stage_timings;
  stage_timings.reserve(kStageCount);

  const auto overall_start = std::chrono::high_resolution_clock::now();

  // 1. Load Cloud
  beginStage(1, "Load input cloud", progress_enabled);
  auto stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> loaded_points;
  if (loadPCD(input_path, loaded_points) <= 0) return 1;
  const std::size_t n_input = loaded_points.size();
  stage_timings.push_back({1, "Load input cloud", endStage(1, "Load input cloud", stage_start, progress_enabled), n_input});

  // 2. Write input
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  saveStagePoints(output_dir / "00_input.pcd", loaded_points, "Input");
  stage_timings.push_back({2, "Write input stage", endStage(2, "Write input stage", stage_start, progress_enabled), n_input});

  // 3. Voxel Downsample (RVV)
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

  // Bounding box & SoA
  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  for (size_t i = 0; i < n_down; ++i) {
    dx[i] = downsampled_pts[i].x; dy[i] = downsampled_pts[i].y; dz[i] = downsampled_pts[i].z;
    min_x = std::min(min_x, dx[i]); max_x = std::max(max_x, dx[i]);
    min_y = std::min(min_y, dy[i]); max_y = std::max(max_y, dy[i]);
  }
  PointCloudSoA down_soa = {dx.data(), dy.data(), dz.data(), n_down};

  // 4. Hardware RVV Vectorized 2D Grid Indexing & Min-Z
  beginStage(4, "Build RVV 2.5D Grid Index (Hardware Vectorized)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  RVVFlatElevationGrid grid(0.20f, min_x, max_x, min_y, max_y);
  std::vector<uint32_t> cell_indices;
  grid.insertCloudRVV(down_soa, cell_indices);
  stage_timings.push_back({4, "Build search index for downsampled cloud",
                           endStage(4, "Build RVV 2.5D Grid Index", stage_start, progress_enabled), n_down});

  // 5. Outlier Filtering
  beginStage(5, "Statistical outlier removal (Density RVV)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  stage_timings.push_back({5, "Statistical outlier removal",
                           endStage(5, "Statistical outlier removal", stage_start, progress_enabled), n_down});

  // 6. Rebuild Index
  beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  stage_timings.push_back({6, "Rebuild search index for filtered cloud",
                           endStage(6, "Rebuild search index", stage_start, progress_enabled), n_down});

  // 7. Normal estimation
  beginStage(7, "Normal estimation", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  stage_timings.push_back({7, "Normal estimation",
                           endStage(7, "Normal estimation", stage_start, progress_enabled), n_down});

  // 8. Ground Plane Separation (RVV Gather & Mask Compress)
  beginStage(8, "Ground plane separation (RVV Vector Gather & Compress)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<PointXYZ> inlier_pts, outlier_pts;
  grid.segmentGroundRVV(down_soa, cell_indices, 0.20f, inlier_pts, outlier_pts);
  saveStagePoints(output_dir / "04_ransac_inliers.pcd", inlier_pts, "RANSAC inliers");
  saveStagePoints(output_dir / "05_ground_plane_removed.pcd", outlier_pts, "Dominant plane removed");
  stage_timings.push_back({8, "RANSAC primitive fitting",
                           endStage(8, "Ground plane separation", stage_start, progress_enabled), outlier_pts.size()});

  // 9. Clustering
  beginStage(9, "Euclidean clustering (2.5D Grid BFS)", progress_enabled);
  stage_start = std::chrono::high_resolution_clock::now();
  std::vector<std::vector<int>> clusters;
  grid.clusterObstacles(outlier_pts, cluster_tolerance, min_cluster_size, max_cluster_size, clusters);
  stage_timings.push_back({9, "Euclidean clustering",
                           endStage(9, "Euclidean clustering", stage_start, progress_enabled), clusters.size()});
  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // 10. Colored PCD Write
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

  return 0;
}
