// test_caravan_vs_pointer_octree.cpp
// Rigorous head-to-head evaluation: Standard PointerOctree vs CaravanPointerOctree (Batch RVV)
// Evaluated for embedded RISC-V targets (Orange Pi RV2 / SpacemiT K1 / TH1520)

#include "include/rvv_pcl.h"
#include "include/caravan_pointer_octree.h"
#include "pointer_octree/pointer_octree.h"
#include "simple_pcd_loader.h"

#include <chrono>
#include <cstdio>
#include <vector>

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;

int main(int argc, char** argv) {
    std::string pcd_path = "data/pcd_compressed/0000000020.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::vector<PointXYZ> raw;
    if (loadPCD(pcd_path, raw) <= 0) {
        printf("Failed to load PCD: %s\n", pcd_path.c_str());
        return 1;
    }

    // Downsample to realistic density (~49k points)
    std::vector<PointXYZ> down(raw.size());
    size_t n_down = voxel_grid_downsamp_sc(raw.data(), raw.size(), down.data(), 0.10f);
    down.resize(n_down);

    std::vector<float> xs(n_down), ys(n_down), zs(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        xs[i] = down[i].x; ys[i] = down[i].y; zs[i] = down[i].z;
    }
    PointCloudSoA cloud = {xs.data(), ys.data(), zs.data(), n_down};

    printf("=========================================================================================\n");
    printf("   HEAD-TO-HEAD: Present PointerOctree vs. CaravanPointerOctree (Batch Query RVV)       \n");
    printf("   Target Architecture: RISC-V 64-bit with RVV 1.0 (Orange Pi RV2 Simulation)           \n");
    printf("   Input Cloud: %zu points, Radius: 0.25 m                                               \n", n_down);
    printf("=========================================================================================\n\n");

    // =========================================================================
    // 1. Build Times
    // =========================================================================
    printf("--> [1/3] Index Build Benchmarks:\n");

    PointerOctree ptr_octree;
    ptr_octree.setInputCloud(cloud);
    ptr_octree.setMaxPointsPerLeaf(32);
    auto t0 = Clock::now();
    ptr_octree.build();
    auto t1 = Clock::now();
    double ptr_build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    CaravanPointerOctree caravan_octree;
    caravan_octree.setInputCloud(cloud);
    t0 = Clock::now();
    caravan_octree.build();
    t1 = Clock::now();
    double caravan_build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    printf("  • PointerOctree Build:        %8.2f ms\n", ptr_build_ms);
    printf("  • CaravanPointerOctree Build: %8.2f ms\n\n", caravan_build_ms);

    // =========================================================================
    // 2. All-Points Batch Radius Search (N=49,900 Queries)
    // =========================================================================
    printf("--> [2/3] Full Cloud Radius Search (%zu queries across all points):\n", n_down);

    // Method A: Present PointerOctree (Individual point queries)
    std::vector<int> ind;
    std::vector<float> dst;
    ind.reserve(256);
    dst.reserve(256);
    size_t total_found_ptr = 0;

    t0 = Clock::now();
    for (size_t q = 0; q < n_down; ++q) {
        PointXYZ query = {cloud.x[q], cloud.y[q], cloud.z[q]};
        total_found_ptr += ptr_octree.radiusSearch(query, 0.25f, ind, dst);
    }
    t1 = Clock::now();
    double ptr_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Method B: CaravanPointerOctree (Batched Morton-ordered RVV Tiles)
    std::vector<std::vector<int32_t>> caravan_results;
    t0 = Clock::now();
    caravan_octree.batchRadiusSearch(cloud, 0.25f, caravan_results);
    t1 = Clock::now();
    double caravan_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    size_t total_found_caravan = 0;
    for (const auto& r : caravan_results) total_found_caravan += r.size();

    printf("  • Present PointerOctree (Individual RVV): %8.2f ms (%zu neighbors found)\n", ptr_search_ms, total_found_ptr);
    printf("  • CaravanPointerOctree (Batch Tile RVV):  %8.2f ms (%zu neighbors found)\n", caravan_search_ms, total_found_caravan);
    printf("  • Speedup (Caravan vs PointerOctree):     %8.2fx\n\n", ptr_search_ms / caravan_search_ms);

    // =========================================================================
    // 3. Full Statistical Outlier Removal (SOR) Stage
    // =========================================================================
    printf("--> [3/3] End-to-End SOR Stage Benchmark (%zu points):\n", n_down);

    std::vector<PointXYZ> out_ptr(n_down);
    t0 = Clock::now();
    size_t n_ptr = sor_pointer_octree(cloud, ptr_octree, out_ptr.data(), 20, 1.0f, 0.25f);
    t1 = Clock::now();
    double ptr_sor_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::vector<PointXYZ> out_caravan(n_down);
    t0 = Clock::now();
    size_t n_caravan = caravan_octree.sorFilter(out_caravan.data(), 20, 1.0f, 0.25f);
    t1 = Clock::now();
    double caravan_sor_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    printf("  • Present PointerOctree SOR: %8.2f ms (%zu inliers)\n", ptr_sor_ms, n_ptr);
    printf("  • CaravanPointerOctree SOR:  %8.2f ms (%zu inliers)\n", caravan_sor_ms, n_caravan);
    printf("  • Speedup (Caravan vs PointerOctree): %8.2fx\n\n", ptr_sor_ms / caravan_sor_ms);

    printf("=========================================================================================\n");
    return 0;
}
