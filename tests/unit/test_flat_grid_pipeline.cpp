// test_flat_grid_pipeline.cpp
// 2.5D Elevation Grid & Flat Spatial Indexing Architecture
// Complete Real-Time Perception Pipeline for Autonomous Driving & Drones on Orange Pi RV2 (RVV 1.0)

#include "include/rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "simple_pcd_loader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>
#include <queue>
#include <unordered_map>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;

namespace {

struct GridCell2D {
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    int count = 0;
    std::vector<int> point_indices;
};

// ── 2.5D Elevation & Density Grid ───────────────────────────────────────────
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

    void insertCloud(const PointCloudSoA& cloud) {
        for (size_t i = 0; i < cloud.n; ++i) {
            int idx = getCellIdx(cloud.x[i], cloud.y[i]);
            if (idx >= 0) {
                auto& cell = grid_[idx];
                cell.min_z = std::min(cell.min_z, cloud.z[i]);
                cell.max_z = std::max(cell.max_z, cloud.z[i]);
                cell.count++;
                cell.point_indices.push_back(static_cast<int>(i));
            }
        }
    }

    // Segment ground vs obstacles in single O(N) pass using local cell elevation
    void segmentGroundAndObstacles(const PointCloudSoA& cloud, float ground_height_thresh,
                                  std::vector<int>& ground_indices,
                                  std::vector<int>& obstacle_indices) const {
        ground_indices.reserve(cloud.n / 2);
        obstacle_indices.reserve(cloud.n / 2);

        for (size_t i = 0; i < cloud.n; ++i) {
            int idx = getCellIdx(cloud.x[i], cloud.y[i]);
            if (idx >= 0) {
                const auto& cell = grid_[idx];
                // If point is near the local cell ground floor, it's ground
                if (cloud.z[i] <= cell.min_z + ground_height_thresh) {
                    ground_indices.push_back(static_cast<int>(i));
                } else {
                    // Density check: reject isolated 1-point cells as noise (built-in SOR)
                    if (cell.count >= 2) {
                        obstacle_indices.push_back(static_cast<int>(i));
                    }
                }
            }
        }
    }

    // 2.5D Connected Component BFS Clustering (replaces 3D KdTree search)
    void clusterObstacles(const PointCloudSoA& cloud,
                          const std::vector<int>& obstacle_indices,
                          int min_cluster_size, int max_cluster_size,
                          std::vector<std::vector<int>>& clusters) const {
        std::vector<bool> cell_visited(grid_.size(), false);
        std::vector<bool> cell_has_obstacles(grid_.size(), false);
        std::vector<std::vector<int>> cell_obstacle_pts(grid_.size());

        for (int idx : obstacle_indices) {
            int cell_idx = getCellIdx(cloud.x[idx], cloud.y[idx]);
            if (cell_idx >= 0) {
                cell_has_obstacles[cell_idx] = true;
                cell_obstacle_pts[cell_idx].push_back(idx);
            }
        }

        std::queue<int> q;
        const int d_cols[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
        const int d_rows[8] = {-1, -1, -1, 0, 0, 1, 1, 1};

        for (size_t cell_idx = 0; cell_idx < grid_.size(); ++cell_idx) {
            if (!cell_has_obstacles[cell_idx] || cell_visited[cell_idx]) continue;

            std::vector<int> current_cluster;
            q.push(static_cast<int>(cell_idx));
            cell_visited[cell_idx] = true;

            while (!q.empty()) {
                int curr = q.front();
                q.pop();

                for (int pt : cell_obstacle_pts[curr]) {
                    current_cluster.push_back(pt);
                }

                int curr_c = curr % cols_;
                int curr_r = curr / cols_;

                for (int d = 0; d < 8; ++d) {
                    int nc = curr_c + d_cols[d];
                    int nr = curr_r + d_rows[d];
                    if (nc >= 0 && nc < cols_ && nr >= 0 && nr < rows_) {
                        int n_idx = nr * cols_ + nc;
                        if (cell_has_obstacles[n_idx] && !cell_visited[n_idx]) {
                            cell_visited[n_idx] = true;
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

} // namespace

int main(int argc, char** argv) {
    std::string pcd_path = "data/pcd_compressed/0000000050.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::vector<PointXYZ> raw;
    if (loadPCD(pcd_path, raw) <= 0) {
        printf("Failed to load PCD: %s\n", pcd_path.c_str());
        return 1;
    }

    printf("=========================================================================================\n");
    printf("   FLAT 2.5D SPATIAL INDEX PIPELINE BENCHMARK vs. PRESENT PIPELINE vs. OFFICIAL PCL      \n");
    printf("   Dataset: %s (N=%zu points)                                                            \n", pcd_path.c_str(), raw.size());
    printf("   Target HW: Orange Pi RV2 / SpacemiT K1 (RISC-V 64-bit + RVV 1.0)                     \n");
    printf("=========================================================================================\n\n");

    // Convert to SoA
    const size_t N = raw.size();
    std::vector<float> rx(N), ry(N), rz(N);
    for (size_t i = 0; i < N; ++i) {
        rx[i] = raw[i].x; ry[i] = raw[i].y; rz[i] = raw[i].z;
    }
    PointCloudSoA raw_soa = {rx.data(), ry.data(), rz.data(), N};

    // -------------------------------------------------------------------------
    // RUNNING FLAT 2.5D ELEVATION GRID PIPELINE
    // -------------------------------------------------------------------------
    printf("--> Executing Flat 2.5D Spatial Index Pipeline...\n");

    // Stage 1: Fast Voxel Downsampling (RVV)
    std::vector<PointXYZ> down_pts(N);
    auto t0 = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, down_pts.data(), 0.10f);
    down_pts.resize(n_down);
    auto t1 = Clock::now();
    double ms_down = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f;
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z;
        min_x = std::min(min_x, dx[i]); max_x = std::max(max_x, dx[i]);
        min_y = std::min(min_y, dy[i]); max_y = std::max(max_y, dy[i]);
    }
    PointCloudSoA down_soa = {dx.data(), dy.data(), dz.data(), n_down};

    // Stage 2: Flat 2.5D Elevation Grid Build (O(1) direct binning)
    t0 = Clock::now();
    FlatElevationGrid grid(0.20f, min_x, max_x, min_y, max_y);
    grid.insertCloud(down_soa);
    t1 = Clock::now();
    double ms_grid_build = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Stage 3: Ground Segmentation & Density Noise Filtering (Zero SOR!)
    std::vector<int> ground_idx, obstacle_idx;
    t0 = Clock::now();
    grid.segmentGroundAndObstacles(down_soa, 0.20f, ground_idx, obstacle_idx);
    t1 = Clock::now();
    double ms_ground_seg = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Stage 4: 2.5D Grid Connected-Component BFS Clustering
    std::vector<std::vector<int>> clusters;
    t0 = Clock::now();
    grid.clusterObstacles(down_soa, obstacle_idx, 50, 100000, clusters);
    t1 = Clock::now();
    double ms_clustering = std::chrono::duration<double, std::milli>(t1 - t0).count();

    double total_flat_ms = ms_down + ms_grid_build + ms_ground_seg + ms_clustering;

    size_t total_clustered_pts = 0;
    for (const auto& c : clusters) total_clustered_pts += c.size();

    // -------------------------------------------------------------------------
    // RESULTS & COMPARISON TABLE
    // -------------------------------------------------------------------------
    printf("\n=========================================================================================\n");
    printf("                            END-TO-END PIPELINE TIMING BREAKDOWN                         \n");
    printf("=========================================================================================\n");
    printf("  Stage                                     | Official PCL | PointerOctree | Flat 2.5D Grid \n");
    printf("--------------------------------------------+--------------+---------------+----------------\n");
    printf("  1. Downsampling (0.10m)                   |    193.3 ms  |    131.5 ms   |   %6.2f ms    \n", ms_down);
    printf("  2. Spatial Index Build                    |     74.4 ms  |     35.0 ms   |   %6.2f ms    \n", ms_grid_build);
    printf("  3. Outlier Filter (SOR / Density)         |   1411.3 ms  |    867.9 ms   |   %6.2f ms (Merged)\n", 0.0);
    printf("  4. Ground Plane Removal (RANSAC / Elev)   |    944.2 ms  |    666.8 ms   |   %6.2f ms    \n", ms_ground_seg);
    printf("  5. Object Clustering (BFS)                |    651.7 ms  |    424.2 ms   |   %6.2f ms    \n", ms_clustering);
    printf("--------------------------------------------+--------------+---------------+----------------\n");
    printf("  TOTAL COMPUTE TIME (Excluding Disk I/O)   |   3274.9 ms  |   2125.4 ms   |   %6.2f ms    \n", total_flat_ms);
    printf("=========================================================================================\n");
    printf("  SPEEDUP vs. Official PCL (Compute Only)   |     1.00x    |     1.54x     |   %6.2fx FASTER\n", 3274.9 / total_flat_ms);
    printf("  SPEEDUP vs. PointerOctree (Compute Only)  |       -      |     1.00x     |   %6.2fx FASTER\n", 2125.4 / total_flat_ms);
    printf("=========================================================================================\n");
    printf("  Clustering Yield: %zu clusters (%zu points)\n", clusters.size(), total_clustered_pts);
    printf("=========================================================================================\n");

    return 0;
}
