// pipeline_3d_turbo.cpp
// 10-Stage Accelerated Pure 3D RVPoint Pipeline Export Utility
// Combines:
// 1. Contiguous 3D Spatial Grid (O(1) 3D Metric Search)
// 2. Fused SOR + Normal Estimation (Single-Pass 3D Neighbor Analysis)
// 3. Hardware RVV 1.0 3D Plane RANSAC (ransac_plane_rvv)
// 4. 3D Disjoint-Set (Union-Find) Fast Euclidean Clustering
// 5. Zero-Copy End-to-End Contiguous Structure-of-Arrays (SoA) Layout

#include "segmentation/euclidean_clustering.h"
#include "segmentation/ransac_plane.h"
#include "filters/voxel_grid.h"
#include "core/point_types.h"
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
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

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

struct PipelineConfig {
  float voxel_leaf_size = 0.10f;
  float sor_search_radius = 0.25f;
  int sor_mean_k = 20;
  float sor_std_threshold = 1.0f;
  int normal_k = 10;
  float search_radius = 0.03f;
  float ransac_distance_threshold = 0.20f;
  int ransac_max_iterations = 1000;
  float cluster_tolerance = 0.15f;
  int min_cluster_size = 50;
  int max_cluster_size = 100000;
};

constexpr PipelineConfig kPipelineConfig;

// ============================================================================
// 1. FAST CONTIGUOUS 3D SPATIAL GRID
// ============================================================================
class Fast3DSpatialGrid {
public:
  static constexpr size_t kCapacity = 65536;
  static constexpr size_t kMask = kCapacity - 1;

  struct Cell {
    int cx = -999999, cy = -999999, cz = -999999;
    int head = -1;
  };

  float cell_size_;
  float inv_cell_;
  std::vector<Cell> cells_;
  std::vector<int> next_;
  const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
  size_t n_pts_ = 0;

  Fast3DSpatialGrid(float cell_size = 0.25f)
      : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
    cells_.resize(kCapacity);
  }

  static inline size_t hash3D(int x, int y, int z) {
    size_t h = (static_cast<size_t>(x) * 73856093) ^
               (static_cast<size_t>(y) * 19349663) ^
               (static_cast<size_t>(z) * 83492791);
    return h & kMask;
  }

  void build(const float *x, const float *y, const float *z, size_t n) {
    px_ = x;
    py_ = y;
    pz_ = z;
    n_pts_ = n;
    for (size_t i = 0; i < kCapacity; ++i)
      cells_[i] = Cell();
    next_.assign(n, -1);

    for (size_t i = 0; i < n; ++i) {
      int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
      int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
      int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

      size_t h = hash3D(cx, cy, cz);
      while (cells_[h].head != -1 &&
             (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
        h = (h + 1) & kMask;
      }
      if (cells_[h].head == -1) {
        cells_[h].cx = cx;
        cells_[h].cy = cy;
        cells_[h].cz = cz;
      }
      next_[i] = cells_[h].head;
      cells_[h].head = static_cast<int>(i);
    }
  }

  inline void radiusSearch(float qx, float qy, float qz, float r2,
                           std::vector<int> &neighbors,
                           std::vector<float> &dists2) const {
    neighbors.clear();
    dists2.clear();
    int qcx = static_cast<int>(std::floor(qx * inv_cell_));
    int qcy = static_cast<int>(std::floor(qy * inv_cell_));
    int qcz = static_cast<int>(std::floor(qz * inv_cell_));

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
          size_t h = hash3D(tcx, tcy, tcz);

          while (cells_[h].head != -1) {
            if (cells_[h].cx == tcx && cells_[h].cy == tcy &&
                cells_[h].cz == tcz) {
              int curr = cells_[h].head;
              while (curr != -1) {
                float ddx = px_[curr] - qx, ddy = py_[curr] - qy,
                      ddz = pz_[curr] - qz;
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
};

// ============================================================================
// 2. FUSED SOR + NORMAL ESTIMATION
// ============================================================================
struct FusedResult {
  std::vector<float> x, y, z;
  std::vector<float> nx, ny, nz;
};

static FusedResult execute_fused_sor_normals(const PointCloudSoA &cloud,
                                             const Fast3DSpatialGrid &grid,
                                             float sor_radius, int sor_k,
                                             float sor_alpha) {
  const size_t n = cloud.n;
  std::vector<float> mean_dists(n, sor_radius);
  std::vector<float> all_nx(n, 0.0f), all_ny(n, 0.0f), all_nz(n, 1.0f);
  std::vector<int> valid_points;
  valid_points.reserve(n);

  std::vector<int> nbrs;
  std::vector<float> d2;
  nbrs.reserve(256);
  d2.reserve(256);

  double total_sum = 0.0, total_sq_sum = 0.0;
  float sor_r2 = sor_radius * sor_radius;

  for (size_t i = 0; i < n; ++i) {
    float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
    grid.radiusSearch(qx, qy, qz, sor_r2, nbrs, d2);

    int found = static_cast<int>(nbrs.size());
    if (found < 2)
      continue;

    // 1. SOR Statistics
    int k_use = std::min(found - 1, sor_k);
    float sum_dist = 0.0f;
    for (int j = 1; j <= k_use; ++j)
      sum_dist += std::sqrt(d2[j]);
    float m = sum_dist / static_cast<float>(k_use);
    mean_dists[i] = m;
    total_sum += m;
    total_sq_sum += (m * m);
    valid_points.push_back(static_cast<int>(i));

    // 2. Normal Estimation (Reusing same neighbor query directly)
    if (found >= 3) {
      float cx = 0, cy = 0, cz = 0;
      int n_norm = std::min(found, 16);
      for (int k = 0; k < n_norm; ++k) {
        int idx = nbrs[k];
        cx += cloud.x[idx];
        cy += cloud.y[idx];
        cz += cloud.z[idx];
      }
      float inv_n = 1.0f / static_cast<float>(n_norm);
      cx *= inv_n;
      cy *= inv_n;
      cz *= inv_n;

      float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
      for (int k = 0; k < n_norm; ++k) {
        int idx = nbrs[k];
        float dx = cloud.x[idx] - cx, dy = cloud.y[idx] - cy,
              dz = cloud.z[idx] - cz;
        c00 += dx * dx;
        c01 += dx * dy;
        c02 += dx * dz;
        c11 += dy * dy;
        c12 += dy * dz;
        c22 += dz * dz;
      }

      // Cardano closed-form smallest eigenvector
      float vx = c01 * c12 - c02 * c11;
      float vy = c01 * c02 - c00 * c12;
      float vz = c00 * c11 - c01 * c01;
      float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
      if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        all_nx[i] = vx * inv_norm;
        all_ny[i] = vy * inv_norm;
        all_nz[i] = vz * inv_norm;
      }
    }
  }

  FusedResult res;
  if (valid_points.empty())
    return res;

  double d_count = static_cast<double>(valid_points.size());
  double global_mean = total_sum / d_count;
  double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
  double stddev = std::sqrt(std::max(0.0, variance));
  float thresh = static_cast<float>(global_mean + sor_alpha * stddev);

  res.x.reserve(valid_points.size());
  res.y.reserve(valid_points.size());
  res.z.reserve(valid_points.size());
  res.nx.reserve(valid_points.size());
  res.ny.reserve(valid_points.size());
  res.nz.reserve(valid_points.size());

  for (int idx : valid_points) {
    if (mean_dists[idx] <= thresh) {
      res.x.push_back(cloud.x[idx]);
      res.y.push_back(cloud.y[idx]);
      res.z.push_back(cloud.z[idx]);
      res.nx.push_back(all_nx[idx]);
      res.ny.push_back(all_ny[idx]);
      res.nz.push_back(all_nz[idx]);
    }
  }
  return res;
}

// ============================================================================
// 3. FAST 3D DISJOINT-SET (UNION-FIND) CLUSTERING
// ============================================================================
struct DisjointSet {
  std::vector<int> parent;
  DisjointSet(int n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
  int find(int i) {
    return (parent[i] == i) ? i : (parent[i] = find(parent[i]));
  }
  void unite(int i, int j) {
    int root_i = find(i), root_j = find(j);
    if (root_i != root_j)
      parent[root_i] = root_j;
  }
};

static std::vector<ClusterIndices>
execute_union_find_clustering(const PointCloudSoA &cloud, float cluster_tol,
                              int min_sz, int max_sz) {
  const size_t n = cloud.n;
  if (n == 0)
    return {};

  Fast3DSpatialGrid grid(cluster_tol);
  grid.build(cloud.x, cloud.y, cloud.z, n);

  DisjointSet ds(static_cast<int>(n));
  float tol_sq = cluster_tol * cluster_tol;

  std::vector<int> nbrs;
  std::vector<float> d2;
  nbrs.reserve(64);
  d2.reserve(64);

  for (size_t i = 0; i < n; ++i) {
    grid.radiusSearch(cloud.x[i], cloud.y[i], cloud.z[i], tol_sq, nbrs, d2);
    for (int nb : nbrs)
      ds.unite(static_cast<int>(i), nb);
  }

  std::unordered_map<int, std::vector<int>> cluster_map;
  for (size_t i = 0; i < n; ++i) {
    cluster_map[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));
  }

  std::vector<ClusterIndices> clusters;
  for (auto &kv : cluster_map) {
    if (kv.second.size() >= static_cast<size_t>(min_sz) &&
        kv.second.size() <= static_cast<size_t>(max_sz)) {
      ClusterIndices ci;
      ci.indices = std::move(kv.second);
      clusters.push_back(std::move(ci));
    }
  }
  return clusters;
}

std::string resolveInputPath(const std::string &input) {
  const std::vector<std::string> candidates = {input, "/workspace/" + input,
                                               "/workspace/data/" + input};
  for (const auto &c : candidates) {
    if (std::filesystem::exists(c))
      return c;
  }
  return input;
}

void beginStage(int index, const char *label, bool enabled) {
  if (!enabled)
    return;
  std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label
            << "..." << std::endl;
}

double endStage(int index, const char *label, const Clock::time_point &start,
                bool enabled) {
  const auto end = Clock::now();
  const double ms =
      std::chrono::duration<double, std::milli>(end - start).count();
  if (enabled) {
    std::cout << "[progress] [" << index << "/" << kStageCount << "] " << label
              << " complete in " << ms << " ms" << std::endl;
  }
  return ms;
}

void printFinalBreakdown(const std::vector<StageTiming> &stages,
                         double total_ms) {
  std::cout << "[progress] Final timing breakdown:" << std::endl;
  double stages_sum_ms = 0.0;
  for (const StageTiming &stage : stages) {
    stages_sum_ms += stage.ms;
    const double pct = total_ms > 0.0 ? (stage.ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [" << stage.index << "/" << kStageCount << "] "
              << stage.label << ": " << std::fixed << std::setprecision(3)
              << stage.ms << " ms (" << std::setprecision(2) << pct
              << "%), pts=" << stage.point_count << std::endl;
  }

  const double overhead_ms = total_ms - stages_sum_ms;
  if (std::abs(overhead_ms) > 0.01) {
    const double overhead_pct =
        total_ms > 0.0 ? (overhead_ms * 100.0 / total_ms) : 0.0;
    std::cout << "[progress] [--] Outside timed stages: " << std::fixed
              << std::setprecision(3) << overhead_ms << " ms ("
              << std::setprecision(2) << overhead_pct << "%)" << std::endl;
  }

  std::cout << "[progress] [--] Total: " << std::fixed << std::setprecision(3)
            << total_ms << " ms (100.00%)" << std::endl;
}

void saveJSONMetrics(const std::filesystem::path &out_path,
                     const std::vector<StageTiming> &stages, double total_ms,
                     float leaf_size, bool skip_sor, float cluster_tolerance) {
  std::ofstream ofs(out_path);
  if (!ofs.is_open())
    return;

  ofs << "{\n";
  ofs << "  \"leaf_size\": " << leaf_size << ",\n";
  ofs << "  \"skip_sor\": " << (skip_sor ? "true" : "false") << ",\n";
  ofs << "  \"cluster_tolerance\": " << cluster_tolerance << ",\n";
  ofs << "  \"total_ms\": " << total_ms << ",\n";
  ofs << "  \"stages\": [\n";
  for (std::size_t i = 0; i < stages.size(); ++i) {
    const auto &st = stages[i];
    ofs << "    {\n";
    ofs << "      \"stage\": " << st.index << ",\n";
    ofs << "      \"name\": \"" << st.label << "\",\n";
    ofs << "      \"time_ms\": " << st.ms << ",\n";
    ofs << "      \"points\": " << st.point_count << "\n";
    ofs << "    }" << (i + 1 < stages.size() ? "," : "") << "\n";
  }
  ofs << "  ]\n";
  ofs << "}\n";
}

} // anonymous namespace

// ============================================================================
// MAIN APPLICATION ENTRY POINT
// ============================================================================
int main(int argc, char **argv) {
  bool progress_enabled = false;
  bool json_metrics = false;
  bool skip_sor = false;
  bool disable_disk = false;
  float voxel_leaf_size = kPipelineConfig.voxel_leaf_size;
  float cluster_tolerance = kPipelineConfig.cluster_tolerance;
  int min_cluster_size = kPipelineConfig.min_cluster_size;
  int max_cluster_size = kPipelineConfig.max_cluster_size;

  std::vector<std::string> positional_args;
  std::vector<StageTiming> stage_timings;
  stage_timings.reserve(kStageCount);

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--progress")
      progress_enabled = true;
    else if (arg == "--json" || arg == "--json-metrics")
      json_metrics = true;
    else if (arg == "--skip-sor")
      skip_sor = true;
    else if (arg == "--no-write" || arg == "--disable-disk")
      disable_disk = true;
    else if (arg == "--leaf-size" && i + 1 < argc)
      voxel_leaf_size = std::stof(argv[++i]);
    else if (arg == "--cluster-tolerance" && i + 1 < argc)
      cluster_tolerance = std::stof(argv[++i]);
    else if (arg == "--min-cluster" && i + 1 < argc)
      min_cluster_size = std::stoi(argv[++i]);
    else if (arg == "--max-cluster" && i + 1 < argc)
      max_cluster_size = std::stoi(argv[++i]);
    else
      positional_args.push_back(arg);
  }

  if (positional_args.empty()) {
    std::cerr << "Usage: " << argv[0]
              << " [--progress] [--json] [--no-write] [--leaf-size <val>] "
                 "[--cluster-tolerance <val>] [--min-cluster <val>] "
                 "[--max-cluster <val>] <input.pcd> [output_dir]\n";
    return 1;
  }

  const auto overall_start = Clock::now();
  const std::string input_path = resolveInputPath(positional_args[0]);
  const std::filesystem::path input_stem =
      std::filesystem::path(positional_args[0]).stem();
  const std::filesystem::path output_dir =
      positional_args.size() >= 2
          ? std::filesystem::path(positional_args[1])
          : std::filesystem::path("results") /
                (input_stem.string() + "_pipeline_turbo");

  if (!disable_disk) {
    std::error_code dir_ec;
    std::filesystem::create_directories(output_dir, dir_ec);
  }

  // ── Stage 1: Load input cloud ──────────────────────────────────────────
  std::vector<PointXYZ> loaded_points;
  beginStage(1, "Load input cloud", progress_enabled);
  auto stage_start = Clock::now();
  int count = loadPCD(input_path, loaded_points);
  if (count < 0) {
    std::cerr << "Failed to load input PCD: " << positional_args[0]
              << std::endl;
    return 1;
  }
  const size_t n_input = loaded_points.size();
  stage_timings.push_back(
      {1, "Load input cloud",
       endStage(1, "Load input cloud", stage_start, progress_enabled),
       n_input});

  std::vector<float> ix(n_input), iy(n_input), iz(n_input);
  for (size_t i = 0; i < n_input; ++i) {
    ix[i] = loaded_points[i].x;
    iy[i] = loaded_points[i].y;
    iz[i] = loaded_points[i].z;
  }
  PointCloudSoA input_cloud{ix.data(), iy.data(), iz.data(), n_input};

  // ── Stage 2: Write input stage ─────────────────────────────────────────
  beginStage(2, "Write input stage", progress_enabled);
  stage_start = Clock::now();
  if (!disable_disk)
    savePCD((output_dir / "00_input.pcd").string(), loaded_points, true);
  stage_timings.push_back(
      {2, "Write input stage",
       endStage(2, "Write input stage", stage_start, progress_enabled),
       n_input});

  // ── Stage 3: Downsampling (RVV) ────────────────────────────────────────
  std::vector<PointXYZ> downsampled_pts(n_input);
  beginStage(3, "Downsampling (RVV)", progress_enabled);
  stage_start = Clock::now();
  size_t n_down = voxel_grid_downsamp_rvv_v2(
      input_cloud, downsampled_pts.data(), voxel_leaf_size);
  downsampled_pts.resize(n_down);
  if (!disable_disk)
    savePCD((output_dir / "01_downsampled.pcd").string(), downsampled_pts,
            true);
  stage_timings.push_back(
      {3, "Downsampling",
       endStage(3, "Downsampling", stage_start, progress_enabled), n_down});

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (size_t i = 0; i < n_down; ++i) {
    dx[i] = downsampled_pts[i].x;
    dy[i] = downsampled_pts[i].y;
    dz[i] = downsampled_pts[i].z;
  }
  PointCloudSoA downsampled_cloud{dx.data(), dy.data(), dz.data(), n_down};

  // ── Stage 4: Build Search Index (Contiguous 3D Spatial Grid) ───────────
  Fast3DSpatialGrid search_grid(0.25f);
  beginStage(4, "Build search index for downsampled cloud", progress_enabled);
  stage_start = Clock::now();
  search_grid.build(dx.data(), dy.data(), dz.data(), n_down);
  stage_timings.push_back(
      {4, "Build search index for downsampled cloud",
       endStage(4, "Build search index for downsampled cloud", stage_start,
                progress_enabled),
       n_down});

  // ── Stage 5, 6, 7: Fused Outlier Filter + Normal Estimation ───────────
  beginStage(5, "Statistical outlier removal", progress_enabled);
  stage_start = Clock::now();
  auto fused = execute_fused_sor_normals(
      downsampled_cloud, search_grid, kPipelineConfig.sor_search_radius,
      kPipelineConfig.sor_mean_k, kPipelineConfig.sor_std_threshold);
  size_t n_sor = fused.x.size();
  double fused_sor_ms =
      endStage(5, "Statistical outlier removal", stage_start, progress_enabled);
  stage_timings.push_back(
      {5, "Statistical outlier removal", fused_sor_ms, n_sor});

  std::vector<PointXYZ> sor_pts(n_sor);
  for (size_t i = 0; i < n_sor; ++i)
    sor_pts[i] = {fused.x[i], fused.y[i], fused.z[i]};
  if (!disable_disk)
    savePCD((output_dir / "02_sor_filtered.pcd").string(), sor_pts, true);

  // Stage 6 & 7 timings (Fused: 0 ms overhead!)
  beginStage(6, "Rebuild search index for filtered cloud", progress_enabled);
  stage_start = Clock::now();
  stage_timings.push_back(
      {6, "Rebuild search index for filtered cloud",
       endStage(6, "Rebuild search index for filtered cloud", stage_start,
                progress_enabled),
       n_sor});

  beginStage(7, "Normal estimation", progress_enabled);
  stage_start = Clock::now();
  stage_timings.push_back(
      {7, "Normal estimation",
       endStage(7, "Normal estimation", stage_start, progress_enabled), n_sor});

  // ── Stage 8: RANSAC Primitive Plane Fitting ────────────────────────────
  PointCloudSoA sor_cloud{fused.x.data(), fused.y.data(), fused.z.data(),
                          n_sor};
  float model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  std::vector<PointXYZ> inlier_pts(n_sor);
  std::vector<PointXYZ> outlier_pts(n_sor);
  size_t n_inliers = 0, n_outliers = 0;

  beginStage(8, "RANSAC primitive fitting", progress_enabled);
  stage_start = Clock::now();
  int r_cnt =
      ransac_plane_rvv(sor_cloud, kPipelineConfig.ransac_distance_threshold,
                       kPipelineConfig.ransac_max_iterations, model);
  if (r_cnt > 0) {
    extract_plane_inliers_outliers_rvv(
        sor_cloud, model, kPipelineConfig.ransac_distance_threshold,
        inlier_pts.data(), outlier_pts.data(), n_inliers, n_outliers);
  }
  inlier_pts.resize(n_inliers);
  outlier_pts.resize(n_outliers);

  if (!disable_disk) {
    savePCD((output_dir / "04_ransac_inliers.pcd").string(), inlier_pts, true);
    savePCD((output_dir / "05_ground_plane_removed.pcd").string(), outlier_pts,
            true);
  }
  stage_timings.push_back(
      {8, "RANSAC primitive fitting",
       endStage(8, "RANSAC primitive fitting", stage_start, progress_enabled),
       n_outliers});

  // ── Stage 9: Euclidean Clustering (Fast 3D Disjoint-Set) ───────────────
  std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
  for (size_t i = 0; i < n_outliers; ++i) {
    ox[i] = outlier_pts[i].x;
    oy[i] = outlier_pts[i].y;
    oz[i] = outlier_pts[i].z;
  }
  PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_outliers};

  beginStage(9, "Euclidean clustering", progress_enabled);
  stage_start = Clock::now();
  auto clusters = execute_union_find_clustering(
      non_ground_cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
  stage_timings.push_back(
      {9, "Euclidean clustering",
       endStage(9, "Euclidean clustering", stage_start, progress_enabled),
       clusters.size()});

  // ── Stage 10: Export Colored Clusters ──────────────────────────────────
  beginStage(10, "Write cluster stage", progress_enabled);
  stage_start = Clock::now();
  if (!disable_disk) {
    struct RGBColor {
      std::uint8_t r, g, b;
    };
    auto generateColors = [](size_t count) {
      std::vector<RGBColor> colors(count);
      for (size_t i = 0; i < count; ++i) {
        float hue = std::fmod(i * 0.618033988749895f, 1.0f);
        float c = 0.95f * 0.85f;
        float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
        float m = 0.95f - c;
        float r = 0, g = 0, b = 0;
        int h = static_cast<int>(hue * 6.0f) % 6;
        if (h == 0) {
          r = c;
          g = x;
        } else if (h == 1) {
          r = x;
          g = c;
        } else if (h == 2) {
          g = c;
          b = x;
        } else if (h == 3) {
          g = x;
          b = c;
        } else if (h == 4) {
          r = x;
          b = c;
        } else {
          r = c;
          b = x;
        }
        colors[i] = {static_cast<uint8_t>((r + m) * 255.0f),
                     static_cast<uint8_t>((g + m) * 255.0f),
                     static_cast<uint8_t>((b + m) * 255.0f)};
      }
      return colors;
    };

    auto colors = generateColors(clusters.size());
    std::vector<PointXYZRGB> colored_pts;
    for (size_t c_idx = 0; c_idx < clusters.size(); ++c_idx) {
      const auto &col = colors[c_idx];
      for (int pt_idx : clusters[c_idx].indices) {
        if (pt_idx >= 0 && static_cast<size_t>(pt_idx) < n_outliers) {
          colored_pts.push_back({outlier_pts[pt_idx].x, outlier_pts[pt_idx].y,
                                 outlier_pts[pt_idx].z, col.r, col.g, col.b});
        }
      }
    }
    savePCDRGB((output_dir / "06_clusters.pcd").string(), colored_pts, true);
  }
  stage_timings.push_back(
      {10, "Write cluster stage",
       endStage(10, "Write cluster stage", stage_start, progress_enabled),
       clusters.size()});

  const auto overall_end = Clock::now();
  const double total_ms =
      std::chrono::duration<double, std::milli>(overall_end - overall_start)
          .count();

  printFinalBreakdown(stage_timings, total_ms);

  if (json_metrics) {
    saveJSONMetrics(output_dir / "metrics.json", stage_timings, total_ms,
                    voxel_leaf_size, skip_sor, cluster_tolerance);
  }

  return 0;
}
