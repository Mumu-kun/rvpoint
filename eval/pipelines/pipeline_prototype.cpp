// ============================================================================
// PROTOTYPE: Unified Register-File Pipeline & PipelineManager
// ============================================================================
// Deepened architecture adhering to:
//   - Candidate 1: Zero middleman structs (Stateful scratch co-located in closures)
//   - Candidate 2: Unified RegisterFile (Consolidates PipelineContext & ConfigStore)
//   - Candidate 3: Deep PipelineManager (Declarative Probes & Multi-Mode step())
//   - ADR-0010: Zero-Heap Hot-Path Invariant
//   - ADR-0011: Non-Virtual Deep Class Pattern
//   - ADR-0012: Slotted Register-File Pipeline & PipelineManager
//   - Kahn's Algorithm DAG Topological Dependency Resolution
//   - Generic Variadic Tagged Positional Binding with static_assert diagnostics
// ============================================================================

#include "include/rvpoint.h"
#include "search/fast_3d_spatial_grid.h"
#include "filters/voxel_grid.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "io/simple_pcd_loader.h"

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <unordered_map>
#include <typeindex>
#include <memory>
#include <functional>
#include <chrono>
#include <cassert>
#include <cmath>
#include <cstring>
#include <variant>
#include <queue>
#include <algorithm>

namespace rvpoint {
// Use production PipelineManager, RegisterFile, and TaggedBinding from librvpoint.a
using ClusterList = std::vector<std::vector<int>>;
} // namespace rvpoint

// ============================================================================
// Evaluation Entry Point
// ============================================================================
int main(int argc, char** argv) {
    std::cout << "==============================================================\n";
    std::cout << "    RVPOINT UNIFIED REGISTER-FILE PIPELINE PROTOTYPE          \n";
    std::cout << "==============================================================\n\n";

    // 1. Load or synthesize a point cloud
    rvpoint::OwnedPointCloud raw_cloud;
    std::string pcd_path = "data/01_table_scene_lms400.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::cout << "[Step 1] Loading input point cloud: " << pcd_path << "\n";
    std::vector<rvpoint::PointXYZ> loaded_pts;
    if (rvpoint::loadPCD(pcd_path, loaded_pts) && !loaded_pts.empty()) {
        std::cout << "  ✓ Loaded " << loaded_pts.size() << " points from PCD file.\n";
        raw_cloud.reserve(loaded_pts.size());
        for (const auto& p : loaded_pts) {
            raw_cloud.push_back(p.x, p.y, p.z);
        }
    } else {
        std::cout << "  (File not found or empty; generating synthetic tabletop scan of 30,000 points...)\n";
        raw_cloud.reserve(30000);
        for (size_t i = 0; i < 20000; ++i) {
            float px = (static_cast<float>(i % 200) - 100.0f) * 0.01f;
            float py = (static_cast<float>(i / 200) - 50.0f) * 0.01f;
            float pz = 0.0f + (static_cast<float>(i % 7) * 0.001f);
            raw_cloud.push_back(px, py, pz);
        }
        for (size_t i = 20000; i < 30000; ++i) {
            float offset_x = (i < 25000) ? 0.3f : -0.3f;
            float px = offset_x + (static_cast<float>((i - 20000) % 50) * 0.005f);
            float py = (static_cast<float>((i - 20000) / 50) * 0.005f);
            float pz = 0.15f + (static_cast<float>(i % 10) * 0.01f);
            raw_cloud.push_back(px, py, pz);
        }
    }
    std::cout << "  ✓ Input cloud ready: " << raw_cloud.n << " points\n\n";

    // 2. Setup Pipeline with unified DAG registration (ZERO middleman structs!)
    rvpoint::PipelineManager pm;
    std::cout << "[Step 2] Building Pipeline with Unified RegisterFile & Closure Scratch...\n";

    // Stage 1: Voxel Downsampling (Stage-Owned Scratch Workspace)
    pm.add_node("voxel_grid",
        rvpoint::in<rvpoint::OwnedPointCloud>("raw_cloud"),
        rvpoint::out<rvpoint::OwnedPointCloud>("downsampled_cloud"),
        rvpoint::param<float>("voxel_leaf_size", 0.05f)
    )
    .kernel([vg = rvpoint::VoxelGrid()](const rvpoint::OwnedPointCloud& in,
                                        rvpoint::OwnedPointCloud& out, float leaf) mutable
    {
        vg(in.as_soa(), out, leaf);
    });

    // Stage 2: RANSAC Ground Plane Split
    pm.add_node("ransac_plane_split",
        rvpoint::in<rvpoint::OwnedPointCloud>("downsampled_cloud"),
        rvpoint::out<rvpoint::PlaneModel>("ground_plane"),
        rvpoint::out<rvpoint::OwnedPointCloud>("ground_cloud"),
        rvpoint::out<rvpoint::OwnedPointCloud>("obstacle_cloud"),
        rvpoint::param<float>("ransac_dist_thresh", 0.02f)
    )
    .kernel([rp = rvpoint::RansacPlane()](const rvpoint::OwnedPointCloud& in, rvpoint::PlaneModel& plane,
                                          rvpoint::OwnedPointCloud& gnd, rvpoint::OwnedPointCloud& obs, float dist) mutable
    {
        plane.inliers = rp(in.as_soa(), plane, dist, 150);
        rp.extract(in.as_soa(), plane, dist, gnd, obs);
    });

    // Stage 3: Fast 3D Spatial Grid Build with dynamic reconfiguration
    pm.add_node("grid_build",
        rvpoint::in<rvpoint::OwnedPointCloud>("obstacle_cloud"),
        rvpoint::out<rvpoint::Fast3DSpatialGrid>("spatial_grid"),
        rvpoint::param<float>("grid_cell_size", 0.05f)
    )
    .on_reconfig<float>("grid_cell_size", [](rvpoint::RegisterFile& rf, float new_cell_size) {
        std::cout << "      --> [Grid Reconfig Hook] Reinitializing Fast3DSpatialGrid with cell_size = "
                  << new_cell_size << "\n";
        auto& grid = rf.get_mut<rvpoint::Fast3DSpatialGrid>(rf.get_id("spatial_grid"));
        grid = rvpoint::Fast3DSpatialGrid(new_cell_size, 32768);
    })
    .kernel([](const rvpoint::OwnedPointCloud& obs, rvpoint::Fast3DSpatialGrid& grid, float /*cell_size*/) {
        grid.build(obs.as_soa());
    });

    // Stage 4: Euclidean Clustering (BFS queues co-located inside closure)
    pm.add_node("euclidean_clustering",
        rvpoint::in<rvpoint::OwnedPointCloud>("obstacle_cloud"),
        rvpoint::in<rvpoint::Fast3DSpatialGrid>("spatial_grid"),
        rvpoint::out<rvpoint::ClusterList>("clusters"),
        rvpoint::param<float>("cluster_tolerance", 0.05f)
    )
    .kernel([
        visited   = std::vector<bool>{},
        queue     = std::vector<int>{},
        neighbors = std::vector<int>{},
        dists     = std::vector<float>{}
    ](const rvpoint::OwnedPointCloud& obs, const rvpoint::Fast3DSpatialGrid& grid,
      rvpoint::ClusterList& clusters, float tol) mutable
    {
        if (visited.size() < obs.n) visited.resize(obs.n);
        if (queue.capacity() < obs.n) queue.reserve(obs.n);
        if (neighbors.capacity() < 2048) neighbors.reserve(2048);
        if (dists.capacity() < 2048) dists.reserve(2048);

        std::fill(visited.begin(), visited.begin() + obs.n, false);
        clusters.clear();
        float tol_sq = tol * tol;

        for (std::size_t i = 0; i < obs.n; ++i) {
            if (visited[i]) continue;
            queue.clear();
            queue.push_back(static_cast<int>(i));
            visited[i] = true;

            std::vector<int> current_cluster;
            size_t head = 0;
            while (head < queue.size()) {
                int curr = queue[head++];
                current_cluster.push_back(curr);

                grid.radiusSearch(obs.x[curr], obs.y[curr], obs.z[curr], tol_sq, neighbors, dists);
                for (int nb : neighbors) {
                    if (!visited[nb]) {
                        visited[nb] = true;
                        queue.push_back(nb);
                    }
                }
            }
            if (current_cluster.size() >= 15) {
                clusters.push_back(std::move(current_cluster));
            }
        }
    });

    // 3. Declarative Probes
    pm.add_probe<rvpoint::OwnedPointCloud>("downsampled_cloud", [](const rvpoint::OwnedPointCloud& down) {
        std::cout << "    [Probe: Downsampled] Count = " << down.n << " points\n";
    });

    pm.add_probe<rvpoint::PlaneModel>("ground_plane", [](const rvpoint::PlaneModel& plane) {
        std::cout << "    [Probe: RANSAC] Model = [" << std::fixed << std::setprecision(3)
                  << plane.a << "x + " << plane.b << "y + " << plane.c << "z + " << plane.d << " = 0]\n";
    });

    pm.add_probe<rvpoint::OwnedPointCloud>("ground_cloud", [](const rvpoint::OwnedPointCloud& gnd) {
        std::cout << "    [Probe: Ground Cloud] Count = " << gnd.n << " points\n";
    });

    pm.add_probe<rvpoint::OwnedPointCloud>("obstacle_cloud", [](const rvpoint::OwnedPointCloud& obs) {
        std::cout << "    [Probe: Obstacle Cloud] Count = " << obs.n << " points\n";
    });

    pm.add_probe<rvpoint::Fast3DSpatialGrid>("spatial_grid", [](const rvpoint::Fast3DSpatialGrid& grid) {
        std::cout << "    [Probe: Grid] Capacity = " << grid.capacity_ << " slots\n";
    });

    pm.add_probe<rvpoint::ClusterList>("clusters", [](const rvpoint::ClusterList& clust) {
        std::cout << "    [Probe: Clustering] Extracted " << clust.size() << " discrete obstacle clusters!\n";
        for (size_t c = 0; c < std::min(clust.size(), size_t(3)); ++c) {
            std::cout << "      - Cluster #" << (c + 1) << ": " << clust[c].size() << " points\n";
        }
    });

    // 4. Initialize pipeline (resolves DAG order, slots, pre-binds pointers)
    pm.set_primary_input("raw_cloud");
    pm.initialize(65536);

    // 5. Frame 1 Execution (Baseline parameters via 1-line step())
    std::cout << "[Step 3] Executing Frame 1 (Baseline Parameters via pm.step())...\n";
    pm.step(raw_cloud.as_soa());
    pm.print_telemetry();

    // 6. Dynamic Parameter Reconfiguration
    std::cout << "[Step 4] Tweaking parameters on the fly via RegisterFile:\n";
    std::cout << "  * Setting 'grid_cell_size' = 0.08m (triggers dynamic reconfig hook!)\n";
    std::cout << "  * Setting 'ransac_dist_thresh' = 0.015m\n\n";

    pm.set_param("grid_cell_size", 0.08f);
    pm.set_param("ransac_dist_thresh", 0.015f);

    // 7. Frame 2 Execution (Dynamic parameters active via 1-line step())
    std::cout << "[Step 5] Executing Frame 2 (Dynamic Parameters Active via pm.step())...\n";
    pm.step(raw_cloud.as_soa());
    pm.print_telemetry();

    std::cout << "==============================================================\n";
    std::cout << "         PROTOTYPE EVALUATION COMPLETED SUCCESSFULLY          \n";
    std::cout << "==============================================================\n";
    return 0;
}
