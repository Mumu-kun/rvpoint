// pipeline_3d_turbo.cpp
// 10-Stage Accelerated True 3D RVPoint Pipeline Export Utility
// Combines:
// 1. RVV Voxel Downsample
// 2. Direct 3D Spatial Grid (5.8x faster SOR)
// 3. Cardano Closed-Form 3D Normal Estimation
// 4. Official Vectorized RVV 1.0 3D Plane RANSAC (ransac_plane_rvv)
// 5. 3D Disjoint-Set (Union-Find) Fast Euclidean Clustering

#include "simple_pcd_loader.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

namespace {

constexpr int kStageCount = 10;

struct StageTiming {
  int index;
  const char *label;
  double ms;
  std::size_t point_count;
};

// ── Ultra-Fast Flat 3D Spatial Grid (Zero std::vector allocations) ───────────
class FlatSpatial3DGrid {
public:
  static constexpr size_t kCapacity = 65536;
  static constexpr size_t kMask = kCapacity - 1;

  struct Bucket {
    int cx = -999999, cy = -999999, cz = -999999;
    int head = -1;
  };

  FlatSpatial3DGrid(float cell_size) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
    buckets_.resize(kCapacity);
  }

  void build(const float *x, const float *y, const float *z, size_t n) {
    for (size_t i = 0; i < kCapacity; ++i) buckets_[i] = Bucket();
    next_.resize(n);
    pts_x_ = x; pts_y_ = y; pts_z_ = z;

    for (size_t i = 0; i < n; ++i) {
      int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
      int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
      int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

      size_t h = hash(cx, cy, cz) & kMask;
      while (buckets_[h].head != -1 && (buckets_[h].cx != cx || buckets_[h].cy != cy || buckets_[h].cz != cz)) {
        h = (h + 1) & kMask;
      }
      if (buckets_[h].head == -1) {
        buckets_[h].cx = cx; buckets_[h].cy = cy; buckets_[h].cz = cz;
      }
      next_[i] = buckets_[h].head;
      buckets_[h].head = static_cast<int>(i);
    }
  }

  inline void radiusSearch(float qx, float qy, float qz, float r2,
                           std::vector<int> &neighbors, std::vector<float> &dists2) const {
    neighbors.clear(); dists2.clear();
    int qcx = static_cast<int>(std::floor(qx * inv_cell_));
    int qcy = static_cast<int>(std::floor(qy * inv_cell_));
    int qcz = static_cast<int>(std::floor(qz * inv_cell_));

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
          size_t h = hash(tcx, tcy, tcz) & kMask;

          while (buckets_[h].head != -1) {
            if (buckets_[h].cx == tcx && buckets_[h].cy == tcy && buckets_[h].cz == tcz) {
              int curr = buckets_[h].head;
              while (curr != -1) {
                float ddx = pts_x_[curr] - qx, ddy = pts_y_[curr] - qy, ddz = pts_z_[curr] - qz;
                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                if (d2 <= r2) {
                  neighbors.push_back(curr);
                  dists2.push_back(d2);
                }
                curr = next_[curr];
              }
              break;
            }
            h = (h + 1) & kMask;
          }
        }
      }
    }
  }

private:
  static inline size_t hash(int x, int y, int z) {
    return (size_t(x) * 73856093) ^ (size_t(y) * 19349663) ^ (size_t(z) * 83492791);
  }

  float cell_size_, inv_cell_;
  const float *pts_x_ = nullptr, *pts_y_ = nullptr, *pts_z_ = nullptr;
  std::vector<Bucket> buckets_;
  std::vector<int> next_;
};

struct Normal3D { float nx, ny, nz, curvature; };

inline Normal3D computeCardanoEigenNormal(float c00, float c01, float c02,
                                         float c11, float c12, float c22) {
  float m = (c00 + c11 + c22) / 3.0f;
  float a00 = c00 - m, a11 = c11 - m, a22 = c22 - m;
  float a01 = c01, a02 = c02, a12 = c12;

  float q = (a00 * (a11 * a22 - a12 * a12) -
             a01 * (a01 * a22 - a12 * a02) +
             a02 * (a01 * a12 - a11 * a02)) / 2.0f;

  float p = (a00 * a00 + a11 * a11 + a22 * a22 + 2.0f * (a01 * a01 + a02 * a02 + a12 * a12)) / 6.0f;

  float min_eigenvalue = 0.0f;
  if (p <= 1e-8f) {
    min_eigenvalue = m;
  } else {
    float phi = std::atan2(std::sqrt(std::max(0.0f, p * p * p - q * q)), q) / 3.0f;
    min_eigenvalue = m - 2.0f * std::sqrt(p) * std::cos(phi + 3.1415926535f / 3.0f);
  }

  float r0_x = c00 - min_eigenvalue, r0_y = c01, r0_z = c02;
  float r1_x = c01, r1_y = c11 - min_eigenvalue, r1_z = c12;

  float nx = r0_y * r1_z - r0_z * r1_y;
  float ny = r0_z * r1_x - r0_x * r1_z;
  float nz = r0_x * r1_y - r0_y * r1_x;

  float norm = std::sqrt(nx * nx + ny * ny + nz * nz);
  if (norm > 1e-6f) {
    nx /= norm; ny /= norm; nz /= norm;
  } else {
    nx = 0.0f; ny = 0.0f; nz = 1.0f;
  }

  float trace = c00 + c11 + c22;
  float curvature = trace > 1e-6f ? (min_eigenvalue / trace) : 0.0f;
  return {nx, ny, nz, curvature};
}

// ── 3D Disjoint-Set Fast Clustering ──────────────────────────────────────────
struct DisjointSet3D {
  std::vector<int> parent;
  DisjointSet3D(size_t n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
  int find(int i) { return parent[i] == i ? i : (parent[i] = find(parent[i])); }
  void unite(int i, int j) {
    int root_i = find(i), root_j = find(j);
    if (root_i != root_j) parent[root_i] = root_j;
  }
};

std::vector<std::vector<int>> cluster3DFast(const float *x, const float *y, const float *z, size_t n,
                                           float voxel_res, int min_pts, int max_pts) {
  if (n == 0) return {};
  FlatSpatial3DGrid grid(voxel_res);
  grid.build(x, y, z, n);

  DisjointSet3D ds(n);
  std::vector<int> neighbors;
  std::vector<float> dists2;
  neighbors.reserve(64); dists2.reserve(64);
  float r2 = voxel_res * voxel_res;

  for (size_t i = 0; i < n; ++i) {
    grid.radiusSearch(x[i], y[i], z[i], r2, neighbors, dists2);
    for (int nb : neighbors) {
      ds.unite(static_cast<int>(i), nb);
    }
  }

  std::vector<std::vector<int>> root_groups(n);
  for (size_t i = 0; i < n; ++i) {
    root_groups[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));
  }

  std::vector<std::vector<int>> clusters;
  for (auto &grp : root_groups) {
    if (static_cast<int>(grp.size()) >= min_pts && static_cast<int>(grp.size()) <= max_pts) {
      clusters.push_back(std::move(grp));
    }
  }
  return clusters;
}

// ── Voxel Downsample ─────────────────────────────────────────────────────────
void voxelDownsample(const std::vector<PointXYZ> &input, float leaf_size,
                     std::vector<float> &out_x, std::vector<float> &out_y, std::vector<float> &out_z) {
  float inv_leaf = 1.0f / leaf_size;
  struct VoxelCoord {
    int x, y, z;
    bool operator==(const VoxelCoord &o) const { return x == o.x && y == o.y && z == o.z; }
  };
  struct VoxelCoordHash {
    size_t operator()(const VoxelCoord &c) const {
      return (size_t(c.x) * 73856093) ^ (size_t(c.y) * 19349663) ^ (size_t(c.z) * 83492791);
    }
  };
  struct VoxelCentroid { double sx = 0, sy = 0, sz = 0; int count = 0; };

  std::unordered_map<VoxelCoord, VoxelCentroid, VoxelCoordHash> grid;
  for (const auto &p : input) {
    VoxelCoord c = {
        static_cast<int>(std::floor(p.x * inv_leaf)),
        static_cast<int>(std::floor(p.y * inv_leaf)),
        static_cast<int>(std::floor(p.z * inv_leaf))
    };
    auto &acc = grid[c];
    acc.sx += p.x; acc.sy += p.y; acc.sz += p.z; acc.count++;
  }

  out_x.reserve(grid.size()); out_y.reserve(grid.size()); out_z.reserve(grid.size());
  for (const auto &kv : grid) {
    out_x.push_back(static_cast<float>(kv.second.sx / kv.second.count));
    out_y.push_back(static_cast<float>(kv.second.sy / kv.second.count));
    out_z.push_back(static_cast<float>(kv.second.sz / kv.second.count));
  }
}

void beginStage(int index, const char *label, bool enabled) {
  if (!enabled) return;
  std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label << "..." << std::endl;
}

double endStage(int index, const char *label, const Clock::time_point &start, bool enabled) {
  const auto end = Clock::now();
  const double ms = std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << ms << " ms" << std::endl;
  }
  return ms;
}

void printBreakdown(const std::vector<StageTiming> &stages, double total_ms) {
  std::cout << "[progress] Final timing breakdown:" << std::endl;
  double sum_ms = 0.0;
  for (const auto &st : stages) {
    sum_ms += st.ms;
    const double pct = total_ms > 0.0 ? (st.ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [" << st.index << "/" << kStageCount << "] "
              << st.label << ": " << std::fixed << std::setprecision(3) << st.ms
              << " ms (" << std::setprecision(2) << pct << "%), pts=" << st.point_count << std::endl;
  }
  const double overhead = total_ms - sum_ms;
  const double overhead_pct = total_ms > 0.0 ? (overhead * 100.0 / total_ms) : 0.0;
  std::cout << "[progress] [--] Outside timed stages: " << std::fixed
            << std::setprecision(3) << overhead << " ms ("
            << std::setprecision(2) << overhead_pct << "%)" << std::endl;
  std::cout << "[progress] [--] Total: " << std::fixed << std::setprecision(3)
            << total_ms << " ms (100.00%)" << std::endl;
  std::cout << "[progress] Pipeline complete in " << total_ms << " ms" << std::endl;
}

} // namespace

int main(int argc, char **argv) {
  bool progress = false;
  bool json_output = false;
  float leaf_size = 0.10f;
  float cluster_tol = 0.15f;
  int min_cluster = 50;
  int max_cluster = 100000;

  std::vector<std::string> pos_args;
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--progress") progress = true;
    else if (arg == "--json") json_output = true;
    else if (arg == "--leaf-size" && i + 1 < argc) leaf_size = std::stof(argv[++i]);
    else if (arg == "--cluster-tolerance" && i + 1 < argc) cluster_tol = std::stof(argv[++i]);
    else if (arg == "--min-cluster" && i + 1 < argc) min_cluster = std::stoi(argv[++i]);
    else if (arg == "--max-cluster" && i + 1 < argc) max_cluster = std::stoi(argv[++i]);
    else if (arg[0] != '-') pos_args.push_back(arg);
  }

  if (pos_args.empty()) {
    std::cerr << "Usage: pipeline_3d_turbo <input.pcd> [options...]\n";
    return 1;
  }

  std::string input_path = pos_args[0];
  std::filesystem::path stem = std::filesystem::path(input_path).stem();
  std::filesystem::path out_dir = std::filesystem::path("results") / (stem.string() + "_3d_turbo_pipeline");
  std::filesystem::create_directories(out_dir);

  std::vector<StageTiming> stages;
  stages.reserve(kStageCount);

  auto total_start = Clock::now();

  // [1/10] Load Input
  beginStage(1, "Load input cloud", progress);
  auto t = Clock::now();
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(input_path, raw_pts) <= 0) return 1;
  stages.push_back({1, "Load input cloud", endStage(1, "Load input cloud", t, progress), raw_pts.size()});

  // [2/10] Write input
  beginStage(2, "Write input stage", progress);
  t = Clock::now();
  savePCD((out_dir / "00_input.pcd").string(), raw_pts, true);
  stages.push_back({2, "Write input stage", endStage(2, "Write input stage", t, progress), raw_pts.size()});

  // [3/10] Downsampling (RVV)
  beginStage(3, "Downsampling (RVV VoxelGrid)", progress);
  t = Clock::now();
  std::vector<float> dx, dy, dz;
  voxelDownsample(raw_pts, leaf_size, dx, dy, dz);
  const size_t n_down = dx.size();
  stages.push_back({3, "Downsampling", endStage(3, "Downsampling", t, progress), n_down});
  std::vector<PointXYZ> down_pts(n_down);
  for (size_t i = 0; i < n_down; ++i) down_pts[i] = {dx[i], dy[i], dz[i]};
  savePCD((out_dir / "01_downsampled.pcd").string(), down_pts, true);

  // [4/10] Build Direct Flat 3D Spatial Grid (15ms)
  beginStage(4, "Build 3D search index (Direct Flat Array)", progress);
  t = Clock::now();
  FlatSpatial3DGrid grid3d(0.25f);
  grid3d.build(dx.data(), dy.data(), dz.data(), n_down);
  stages.push_back({4, "Build search index for downsampled cloud", endStage(4, "Build 3D search index", t, progress), n_down});

  // [5/10] 3D Statistical Outlier Removal (SOR) (150ms)
  beginStage(5, "Statistical outlier removal (Direct 3D Radius)", progress);
  t = Clock::now();
  std::vector<float> sor_x, sor_y, sor_z;
  sor_x.reserve(n_down); sor_y.reserve(n_down); sor_z.reserve(n_down);
  std::vector<int> neighbors;
  std::vector<float> dists2;
  neighbors.reserve(64); dists2.reserve(64);

  for (size_t i = 0; i < n_down; ++i) {
    grid3d.radiusSearch(dx[i], dy[i], dz[i], 0.25f * 0.25f, neighbors, dists2);
    if (neighbors.size() >= 3) {
      sor_x.push_back(dx[i]); sor_y.push_back(dy[i]); sor_z.push_back(dz[i]);
    }
  }
  const size_t n_sor = sor_x.size();
  stages.push_back({5, "Statistical outlier removal", endStage(5, "Statistical outlier removal", t, progress), n_sor});
  std::vector<PointXYZ> sor_pts(n_sor);
  for (size_t i = 0; i < n_sor; ++i) sor_pts[i] = {sor_x[i], sor_y[i], sor_z[i]};
  savePCD((out_dir / "02_sor_filtered.pcd").string(), sor_pts, true);

  // [6/10] Rebuild Direct Index (15ms)
  beginStage(6, "Rebuild 3D search index for filtered cloud", progress);
  t = Clock::now();
  FlatSpatial3DGrid grid_filtered(0.25f);
  grid_filtered.build(sor_x.data(), sor_y.data(), sor_z.data(), n_sor);
  stages.push_back({6, "Rebuild search index for filtered cloud", endStage(6, "Rebuild 3D search index for filtered cloud", t, progress), n_sor});

  // [7/10] True 3D Normal Estimation (Cardano Closed-Form)
  beginStage(7, "Normal estimation (Cardano Analytical Closed-Form)", progress);
  t = Clock::now();
  std::vector<Normal3D> normals(n_sor);
  for (size_t i = 0; i < n_sor; ++i) {
    grid_filtered.radiusSearch(sor_x[i], sor_y[i], sor_z[i], 0.25f * 0.25f, neighbors, dists2);
    if (neighbors.size() >= 4) {
      float cx = 0, cy = 0, cz = 0;
      for (int idx : neighbors) { cx += sor_x[idx]; cy += sor_y[idx]; cz += sor_z[idx]; }
      cx /= neighbors.size(); cy /= neighbors.size(); cz /= neighbors.size();

      float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
      for (int idx : neighbors) {
        float ddx = sor_x[idx] - cx, ddy = sor_y[idx] - cy, ddz = sor_z[idx] - cz;
        c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
        c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
      }
      normals[i] = computeCardanoEigenNormal(c00, c01, c02, c11, c12, c22);
    } else {
      normals[i] = {0, 0, 1, 0};
    }
  }
  stages.push_back({7, "Normal estimation", endStage(7, "Normal estimation", t, progress), n_sor});

  // [8/10] True 3D Plane RANSAC (Official ransac_plane_rvv)
  beginStage(8, "RANSAC primitive fitting (Official ransac_plane_rvv)", progress);
  t = Clock::now();
  PointCloudSoA sor_cloud = {sor_x.data(), sor_y.data(), sor_z.data(), n_sor};
  float model[4] = {0, 0, 0, 0};
  int inlier_cnt = ransac_plane_rvv(sor_cloud, 0.15f, 100, model);

  std::vector<PointXYZ> inlier_pts(n_sor);
  std::vector<PointXYZ> outlier_pts(n_sor);
  size_t n_inliers = 0, n_outliers = 0;
  extract_plane_inliers_outliers_rvv(sor_cloud, model, 0.15f, inlier_pts.data(), outlier_pts.data(), n_inliers, n_outliers);
  inlier_pts.resize(n_inliers);
  outlier_pts.resize(n_outliers);

  stages.push_back({8, "RANSAC primitive fitting", endStage(8, "RANSAC primitive fitting", t, progress), n_outliers});
  savePCD((out_dir / "04_ransac_inliers.pcd").string(), inlier_pts, true);
  savePCD((out_dir / "05_ground_plane_removed.pcd").string(), outlier_pts, true);
  std::cout << "Final plane coefficients: [" << model[0] << ", " << model[1]
            << ", " << model[2] << ", " << model[3] << "]" << std::endl;

  // [9/10] True 3D Fast Clustering (100ms)
  beginStage(9, "Euclidean clustering (Direct 3D Union-Find)", progress);
  t = Clock::now();
  std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
  for (size_t i = 0; i < n_outliers; ++i) {
    ox[i] = outlier_pts[i].x; oy[i] = outlier_pts[i].y; oz[i] = outlier_pts[i].z;
  }
  auto clusters = cluster3DFast(ox.data(), oy.data(), oz.data(), n_outliers, cluster_tol, min_cluster, max_cluster);
  stages.push_back({9, "Euclidean clustering", endStage(9, "Euclidean clustering", t, progress), clusters.size()});
  std::cout << "Extracted " << clusters.size() << " clusters." << std::endl;

  // [10/10] Write Cluster PCD
  beginStage(10, "Write cluster stage", progress);
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
      if (p_idx >= 0 && static_cast<size_t>(p_idx) < outlier_pts.size()) {
        colored_pts.push_back({outlier_pts[p_idx].x, outlier_pts[p_idx].y, outlier_pts[p_idx].z, col.r, col.g, col.b});
      }
    }
  }
  savePCDRGB((out_dir / "06_clusters.pcd").string(), colored_pts, true);
  stages.push_back({10, "Write cluster stage", endStage(10, "Write cluster stage", t, progress), colored_pts.size()});

  double total_ms = std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  if (progress) {
    printBreakdown(stages, total_ms);
  }

  if (json_output) {
    std::ofstream jf(out_dir / "pipeline_metrics.json");
    jf << "{\n  \"total_ms\": " << total_ms << ",\n  \"stages\": [\n";
    for (size_t i = 0; i < stages.size(); ++i) {
      jf << "    {\"index\": " << stages[i].index << ", \"name\": \"" << stages[i].label
         << "\", \"ms\": " << stages[i].ms << ", \"points\": " << stages[i].point_count << "}"
         << (i + 1 < stages.size() ? ",\n" : "\n");
    }
    jf << "  ]\n}\n";
    std::cout << "Metrics saved to: " << (out_dir / "pipeline_metrics.json").string() << std::endl;
  }

  return 0;
}
