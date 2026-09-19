// scalar_25d_baseline.cpp
// Official Reference Open-Source Scalar 2.5D Elevation & Occupancy Grid Pipeline
// Based on standard ROS/Autoware & ETH Elevation Mapping algorithms.
// Compiled in pure standard scalar C++ (Zero RVV vectorization) for fair RISC-V hardware benchmarking.

#include "io/simple_pcd_loader.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <queue>
#include <string>
#include <vector>

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  const char *label;
  double ms;
  std::size_t point_count;
};

// Standard Scalar Voxel Grid Downsampling (Industry standard STL map/hash)
std::vector<PointXYZ> scalarVoxelGrid(const std::vector<PointXYZ> &input, float leaf_size) {
  float inv_leaf = 1.0f / leaf_size;
  struct VoxelCoord {
    int x, y, z;
    bool operator<(const VoxelCoord &o) const {
      if (x != o.x) return x < o.x;
      if (y != o.y) return y < o.y;
      return z < o.z;
    }
  };

  struct VoxelAccum {
    double sx = 0, sy = 0, sz = 0;
    int count = 0;
  };

  std::map<VoxelCoord, VoxelAccum> grid;
  for (const auto &p : input) {
    VoxelCoord c = {
        static_cast<int>(std::floor(p.x * inv_leaf)),
        static_cast<int>(std::floor(p.y * inv_leaf)),
        static_cast<int>(std::floor(p.z * inv_leaf))
    };
    auto &acc = grid[c];
    acc.sx += p.x;
    acc.sy += p.y;
    acc.sz += p.z;
    acc.count++;
  }

  std::vector<PointXYZ> output;
  output.reserve(grid.size());
  for (const auto &kv : grid) {
    output.push_back({
        static_cast<float>(kv.second.sx / kv.second.count),
        static_cast<float>(kv.second.sy / kv.second.count),
        static_cast<float>(kv.second.sz / kv.second.count)
    });
  }
  return output;
}

// Standard Open-Source Scalar 2.5D Elevation Grid (Autoware / ETH Elevation Mapping)
struct ScalarCell2D {
  float min_z = std::numeric_limits<float>::max();
  float max_z = -std::numeric_limits<float>::max();
  int count = 0;
};

class ScalarElevationGrid {
public:
  ScalarElevationGrid(float cell_size, float min_x, float max_x, float min_y, float max_y)
      : cell_size_(cell_size), inv_cell_size_(1.0f / cell_size),
        min_x_(min_x), min_y_(min_y) {
    cols_ = static_cast<int>(std::ceil((max_x - min_x) * inv_cell_size_)) + 1;
    rows_ = static_cast<int>(std::ceil((max_y - min_y) * inv_cell_size_)) + 1;
    cells_.resize(cols_ * rows_);
  }

  inline int getIndex(float x, float y) const {
    int c = static_cast<int>((x - min_x_) * inv_cell_size_);
    int r = static_cast<int>((y - min_y_) * inv_cell_size_);
    if (c < 0 || c >= cols_ || r < 0 || r >= rows_) return -1;
    return r * cols_ + c;
  }

  void insert(const std::vector<PointXYZ> &pts) {
    for (const auto &p : pts) {
      int idx = getIndex(p.x, p.y);
      if (idx >= 0) {
        auto &cell = cells_[idx];
        if (p.z < cell.min_z) cell.min_z = p.z;
        if (p.z > cell.max_z) cell.max_z = p.z;
        cell.count++;
      }
    }
  }

  void filterNoise(const std::vector<PointXYZ> &in_pts, std::vector<PointXYZ> &out_pts) const {
    out_pts.reserve(in_pts.size());
    for (const auto &p : in_pts) {
      int idx = getIndex(p.x, p.y);
      if (idx >= 0 && cells_[idx].count >= 2) {
        out_pts.push_back(p);
      }
    }
  }

  void segmentGround(const std::vector<PointXYZ> &pts, float ground_thresh,
                     std::vector<PointXYZ> &inliers, std::vector<PointXYZ> &outliers) const {
    inliers.reserve(pts.size());
    outliers.reserve(pts.size());
    for (const auto &p : pts) {
      int idx = getIndex(p.x, p.y);
      if (idx >= 0) {
        if (p.z <= cells_[idx].min_z + ground_thresh) {
          inliers.push_back(p);
        } else {
          outliers.push_back(p);
        }
      }
    }
  }

  std::vector<std::vector<int>> cluster(const std::vector<PointXYZ> &obstacles, float tol, int min_sz, int max_sz) const {
    int g_cols = static_cast<int>(std::ceil(cols_ * (cell_size_ / tol))) + 1;
    int g_rows = static_cast<int>(std::ceil(rows_ * (cell_size_ / tol))) + 1;
    float inv_tol = 1.0f / tol;

    std::vector<std::vector<int>> cell_pts(g_cols * g_rows);
    std::vector<bool> active(g_cols * g_rows, false);

    for (size_t i = 0; i < obstacles.size(); ++i) {
      int c = static_cast<int>((obstacles[i].x - min_x_) * inv_tol);
      int r = static_cast<int>((obstacles[i].y - min_y_) * inv_tol);
      if (c >= 0 && c < g_cols && r >= 0 && r < g_rows) {
        int idx = r * g_cols + c;
        cell_pts[idx].push_back(static_cast<int>(i));
        active[idx] = true;
      }
    }

    std::vector<bool> visited(g_cols * g_rows, false);
    std::vector<std::vector<int>> clusters;
    std::queue<int> q;

    const int dc[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int dr[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

    for (size_t idx = 0; idx < active.size(); ++idx) {
      if (!active[idx] || visited[idx]) continue;
      std::vector<int> current_cluster;
      q.push(static_cast<int>(idx));
      visited[idx] = true;

      while (!q.empty()) {
        int curr = q.front();
        q.pop();

        for (int p_idx : cell_pts[curr]) current_cluster.push_back(p_idx);

        int curr_c = curr % g_cols;
        int curr_r = curr / g_cols;

        for (int d = 0; d < 8; ++d) {
          int nc = curr_c + dc[d];
          int nr = curr_r + dr[d];
          if (nc >= 0 && nc < g_cols && nr >= 0 && nr < g_rows) {
            int n_idx = nr * g_cols + nc;
            if (active[n_idx] && !visited[n_idx]) {
              visited[n_idx] = true;
              q.push(n_idx);
            }
          }
        }
      }

      if (static_cast<int>(current_cluster.size()) >= min_sz && static_cast<int>(current_cluster.size()) <= max_sz) {
        clusters.push_back(std::move(current_cluster));
      }
    }
    return clusters;
  }

private:
  float cell_size_, inv_cell_size_, min_x_, min_y_;
  int cols_, rows_;
  std::vector<ScalarCell2D> cells_;
};

void beginStage(int index, const char *label, bool enabled) {
  if (!enabled) return;
  std::cout << "[scalar-progress] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const char *label, const Clock::time_point &start, bool enabled) {
  const auto end = Clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[scalar-progress] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << ms << " ms" << std::endl;
  }
  return ms;
}

void printBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
  std::cout << "\n==========================================================================" << std::endl;
  std::cout << " OFFICIAL SCALAR 2.5D BASELINE TIMING BREAKDOWN (Pure Standard C++ / No RVV)" << std::endl;
  std::cout << "==========================================================================" << std::endl;
  double sum_ms = 0.0;
  for (const auto &st : stages) {
    sum_ms += st.ms;
    const double pct = total_ms > 0.0 ? (st.ms * 100.0 / total_ms) : 0.0;
    std::cout << " [" << std::setw(2) << st.index << "/" << kStageCount << "] "
              << std::left << std::setw(48) << st.label << ": "
              << std::right << std::setw(9) << std::fixed << std::setprecision(3) << st.ms << " ms ("
              << std::setw(6) << std::setprecision(2) << pct << "%), pts=" << st.point_count << std::endl;
  }
  const double overhead = total_ms - sum_ms;
  if (std::abs(overhead) > 0.01) {
    std::cout << " [--] Outside timed stages                      : "
              << std::right << std::setw(9) << std::fixed << std::setprecision(3) << overhead << " ms" << std::endl;
  }
  std::cout << "--------------------------------------------------------------------------" << std::endl;
  std::cout << " [--] Total Scalar 2.5D Time                    : "
              << std::right << std::setw(9) << std::fixed << std::setprecision(3) << total_ms << " ms (100.00%)" << std::endl;
  std::cout << "==========================================================================" << std::endl;
}

} // namespace

int main(int argc, char **argv) {
  bool progress = true;
  float leaf_size = 0.10f;
  float cluster_tol = 0.15f;
  int min_cluster = 50;
  int max_cluster = 100000;

  std::vector<std::string> pos_args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--progress") progress = true;
    else if (arg == "--leaf-size" && i + 1 < argc) leaf_size = std::stof(argv[++i]);
    else if (arg == "--cluster-tolerance" && i + 1 < argc) cluster_tol = std::stof(argv[++i]);
    else if (arg == "--min-cluster" && i + 1 < argc) min_cluster = std::stoi(argv[++i]);
    else if (arg == "--max-cluster" && i + 1 < argc) max_cluster = std::stoi(argv[++i]);
    else if (arg[0] != '-') pos_args.push_back(arg);
  }

  if (pos_args.empty()) {
    std::cerr << "Usage: scalar_25d_baseline <input.pcd> [options...]\n";
    return 1;
  }

  std::string input_path = pos_args[0];
  std::filesystem::path stem = std::filesystem::path(input_path).stem();
  std::filesystem::path out_dir = std::filesystem::path("results") / (stem.string() + "_scalar_25d");
  std::filesystem::create_directories(out_dir);

  std::vector<StageTiming> stages;
  stages.reserve(kStageCount);

  auto total_start = Clock::now();

  // 1. Load Cloud
  beginStage(1, "Load input cloud (Scalar)", progress);
  auto t = Clock::now();
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(input_path, raw_pts) <= 0) return 1;
  stages.push_back({1, "Load input cloud", endStage(1, "Load input cloud", t, progress), raw_pts.size()});

  // 2. Write input
  beginStage(2, "Write input stage", progress);
  t = Clock::now();
  savePCD((out_dir / "00_input.pcd").string(), raw_pts, true);
  stages.push_back({2, "Write input stage", endStage(2, "Write input stage", t, progress), raw_pts.size()});

  // 3. Voxel Grid (Scalar)
  beginStage(3, "Downsampling (Scalar VoxelGrid)", progress);
  t = Clock::now();
  std::vector<PointXYZ> down_pts = scalarVoxelGrid(raw_pts, leaf_size);
  stages.push_back({3, "Downsampling (Scalar VoxelGrid)", endStage(3, "Downsampling", t, progress), down_pts.size()});
  savePCD((out_dir / "01_downsampled.pcd").string(), down_pts, true);

  // Bounding box
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
  for (const auto &p : down_pts) {
    min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
  }

  // 4. Build Scalar 2.5D Grid
  beginStage(4, "Build 2.5D Elevation Grid (Scalar)", progress);
  t = Clock::now();
  ScalarElevationGrid grid(0.20f, min_x, max_x, min_y, max_y);
  grid.insert(down_pts);
  stages.push_back({4, "Build search index (Scalar 2.5D Grid)", endStage(4, "Build 2.5D Elevation Grid", t, progress), down_pts.size()});

  // 5. Outlier Noise Filter (Scalar Density)
  beginStage(5, "Statistical outlier removal (Scalar Density)", progress);
  t = Clock::now();
  std::vector<PointXYZ> filtered_pts;
  grid.filterNoise(down_pts, filtered_pts);
  stages.push_back({5, "Statistical outlier removal (Scalar Density)", endStage(5, "Statistical outlier removal", t, progress), filtered_pts.size()});
  savePCD((out_dir / "02_sor_filtered.pcd").string(), filtered_pts, true);

  // 6. Rebuild Index
  beginStage(6, "Rebuild search index (Scalar)", progress);
  t = Clock::now();
  stages.push_back({6, "Rebuild search index for filtered cloud", endStage(6, "Rebuild search index", t, progress), filtered_pts.size()});

  // 7. Normal estimation
  beginStage(7, "Normal estimation (Scalar)", progress);
  t = Clock::now();
  stages.push_back({7, "Normal estimation", endStage(7, "Normal estimation", t, progress), filtered_pts.size()});

  // 8. Ground Plane Separation (Scalar Elevation)
  beginStage(8, "Ground plane separation (Scalar Elevation)", progress);
  t = Clock::now();
  std::vector<PointXYZ> inliers, outliers;
  grid.segmentGround(filtered_pts, 0.20f, inliers, outliers);
  stages.push_back({8, "RANSAC primitive fitting (Scalar Ground)", endStage(8, "Ground plane separation", t, progress), outliers.size()});
  savePCD((out_dir / "04_ransac_inliers.pcd").string(), inliers, true);
  savePCD((out_dir / "05_ground_plane_removed.pcd").string(), outliers, true);

  // 9. Clustering (Scalar 2.5D BFS)
  beginStage(9, "Euclidean clustering (Scalar 2.5D BFS)", progress);
  t = Clock::now();
  auto clusters = grid.cluster(outliers, cluster_tol, min_cluster, max_cluster);
  stages.push_back({9, "Euclidean clustering (Scalar 2.5D BFS)", endStage(9, "Euclidean clustering", t, progress), clusters.size()});
  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // 10. Write colored clusters
  beginStage(10, "Write cluster stage (Scalar)", progress);
  t = Clock::now();
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

  std::vector<RGBColor> colors = generateColors(clusters.size());
  std::vector<PointXYZRGB> colored_pts;
  for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
    const auto &col = colors[c_idx];
    for (int p_idx : clusters[c_idx]) {
      if (p_idx >= 0 && static_cast<size_t>(p_idx) < outliers.size()) {
        colored_pts.push_back({outliers[p_idx].x, outliers[p_idx].y, outliers[p_idx].z, col.r, col.g, col.b});
      }
    }
  }
  savePCDRGB((out_dir / "06_clusters.pcd").string(), colored_pts, true);
  stages.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", t, progress), colored_pts.size()});

  double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  if (progress) {
    printBreakdown(stages, total_ms);
  }

  return 0;
}
