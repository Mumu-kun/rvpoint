#include "kd_tree/kd_tree.h"
#include "pointer_octree/pointer_octree.h"
#include "include/simple_pcd_loader.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cstdarg>

using namespace rvv_pcl;

int main(int argc, char** argv) {
    std::string input_file = "table_scene_lms400.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }

    std::printf("============================================================\n");
    std::printf("  RVPoint Production Outdoor LiDAR Pipeline Parameter Sweep\n");
    std::printf("============================================================\n\n");

    // Load full resolution point cloud
    std::vector<PointXYZ> raw_points;
    int n = loadPCD(input_file, raw_points);
    if (n < 0) n = loadPCD("../" + input_file, raw_points);
    if (n < 0) n = loadPCD("data/" + input_file, raw_points);
    if (n < 0) n = loadPCD("../data/" + input_file, raw_points);
    if (n < 0) n = loadPCD("/workspace/data/" + input_file, raw_points);

    if (n < 0) {
        std::printf("[WARN] Could not load table_scene_lms400.pcd, trying bunny.pcd...\n");
        input_file = "bunny.pcd";
        n = loadPCD(input_file, raw_points);
        if (n < 0) n = loadPCD("data/" + input_file, raw_points);
        if (n < 0) n = loadPCD("../data/" + input_file, raw_points);
        if (n < 0) n = loadPCD("/workspace/data/" + input_file, raw_points);
    }

    if (n < 0) {
        std::printf("[FAIL] Could not load any point cloud file.\n");
        return 1;
    }
    std::printf("Loaded raw point cloud: %s (%d points).\n", input_file.c_str(), n);

    // Output file configuration
    std::ofstream report("/workspace/results/outdoor_sweep_report.txt");
    if (!report.is_open()) {
        std::printf("[FAIL] Could not open output report file.\n");
        return 1;
    }

    // Print headers to terminal and report file
    auto print_line = [&](const char* format, ...) {
        char buffer[1024];
        va_list args;
        va_start(args, format);
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        std::printf("%s", buffer);
        report << buffer;
    };

    print_line("# Production Outdoor LiDAR Pipeline Parameter Sweep Report\n\n");
    print_line("| Voxel (m) | Points | Leaf Cap | Radius (m) | Oct Scalar (ms) | Oct RVV (ms) | Oct Speedup | Kd Scalar (ms) | Kd RVV (ms) | Kd Speedup | Kd/Oct RVV Speedup |\n");
    print_line("| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |\n");

    // Hyperparameter Sweep Grids
    const float voxel_sizes[] = {0.05f, 0.10f, 0.15f, 0.20f, 0.30f};
    const int leaf_capacities[] = {16, 24, 32};
    const float search_radii[] = {0.08f, 0.15f, 0.25f, 0.40f, 0.60f};

    // Convert raw points to SoA for downsampling
    std::vector<float> rx(n), ry(n), rz(n);
    for (int i = 0; i < n; ++i) {
        rx[i] = raw_points[i].x;
        ry[i] = raw_points[i].y;
        rz[i] = raw_points[i].z;
    }
    PointCloudSoA raw_cloud = {rx.data(), ry.data(), rz.data(), (size_t)n};

    for (float voxel_size : voxel_sizes) {
        // 1. Perform Voxel Downsampling
        std::vector<PointXYZ> downsampled_pts(n);
        size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, downsampled_pts.data(), voxel_size);
        if (n_down == 0) continue;

        // Convert downsampled to SoA
        std::vector<float> dx(n_down), dy(n_down), dz(n_down);
        for (size_t i = 0; i < n_down; ++i) {
            dx[i] = downsampled_pts[i].x;
            dy[i] = downsampled_pts[i].y;
            dz[i] = downsampled_pts[i].z;
        }
        PointCloudSoA down_cloud = {dx.data(), dy.data(), dz.data(), n_down};

        for (int leaf_cap : leaf_capacities) {
            // Build Octree on downsampled cloud
            PointerOctree octree;
            octree.setInputCloud(down_cloud);
            octree.setMaxPointsPerLeaf(leaf_cap);
            octree.build();

            // Build K-d Tree on downsampled cloud
            KdTree kdtree;
            kdtree.setInputCloud(down_cloud);
            kdtree.setMaxPointsPerLeaf(leaf_cap);
            kdtree.build();

            for (float search_radius : search_radii) {
                // Buffers for queries
                std::vector<std::vector<int>> octree_sc_indices(n_down);
                std::vector<std::vector<float>> octree_sc_dists(n_down);
                std::vector<std::vector<int>> octree_rvv_indices(n_down);
                std::vector<std::vector<float>> octree_rvv_dists(n_down);
                std::vector<std::vector<int>> kd_sc_indices(n_down);
                std::vector<std::vector<float>> kd_sc_dists(n_down);
                std::vector<std::vector<int>> kd_rvv_indices(n_down);
                std::vector<std::vector<float>> kd_rvv_dists(n_down);

                // Warmup
                for (size_t i = 0; i < n_down; ++i) {
                    octree.radiusSearchScalar(downsampled_pts[i], search_radius, octree_sc_indices[i], octree_sc_dists[i]);
                    octree.radiusSearch(downsampled_pts[i], search_radius, octree_rvv_indices[i], octree_rvv_dists[i]);
                    kdtree.radiusSearchScalar(downsampled_pts[i], search_radius, kd_sc_indices[i], kd_sc_dists[i]);
                    kdtree.radiusSearch(downsampled_pts[i], search_radius, kd_rvv_indices[i], kd_rvv_dists[i]);
                }

                // Octree Scalar Benchmark
                auto start = std::chrono::high_resolution_clock::now();
                for (size_t i = 0; i < n_down; ++i) {
                    octree.radiusSearchScalar(downsampled_pts[i], search_radius, octree_sc_indices[i], octree_sc_dists[i]);
                }
                auto end = std::chrono::high_resolution_clock::now();
                double oct_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

                // Octree RVV Benchmark
                start = std::chrono::high_resolution_clock::now();
                for (size_t i = 0; i < n_down; ++i) {
                    octree.radiusSearch(downsampled_pts[i], search_radius, octree_rvv_indices[i], octree_rvv_dists[i]);
                }
                end = std::chrono::high_resolution_clock::now();
                double oct_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

                // K-d Tree Scalar Benchmark
                start = std::chrono::high_resolution_clock::now();
                for (size_t i = 0; i < n_down; ++i) {
                    kdtree.radiusSearchScalar(downsampled_pts[i], search_radius, kd_sc_indices[i], kd_sc_dists[i]);
                }
                end = std::chrono::high_resolution_clock::now();
                double kd_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

                // K-d Tree RVV Benchmark
                start = std::chrono::high_resolution_clock::now();
                for (size_t i = 0; i < n_down; ++i) {
                    kdtree.radiusSearch(downsampled_pts[i], search_radius, kd_rvv_indices[i], kd_rvv_dists[i]);
                }
                end = std::chrono::high_resolution_clock::now();
                double kd_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

                // Format and output results
                print_line("| %9.3f | %6zu | %8d | %10.3f | %15.2f | %12.2f | %10.2fx | %14.2f | %11.2f | %10.2fx | %18.2fx |\n",
                           voxel_size, n_down, leaf_cap, search_radius,
                           oct_sc_ms, oct_rvv_ms, oct_sc_ms / oct_rvv_ms,
                           kd_sc_ms, kd_rvv_ms, kd_sc_ms / kd_rvv_ms,
                           oct_rvv_ms / kd_rvv_ms);
            }
        }
    }

    report.close();
    std::printf("\n[SUCCESS] Outdoor hyperparameter sweep completed. Report written to: /workspace/results/outdoor_sweep_report.txt\n");
    return 0;
}
