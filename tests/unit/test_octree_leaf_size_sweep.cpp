// test_octree_leaf_size_sweep.cpp
#include "include/rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "simple_pcd_loader.h"

#include <chrono>
#include <cstdio>
#include <vector>

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;

int main() {
    std::string pcd_path = "data/pcd_compressed/0000000020.pcd";
    std::vector<PointXYZ> raw;
    if (loadPCD(pcd_path, raw) <= 0) return 1;

    // Downsample to ~49k points
    std::vector<PointXYZ> down(raw.size());
    size_t n_down = voxel_grid_downsamp_sc(raw.data(), raw.size(), down.data(), 0.10f);
    down.resize(n_down);

    std::vector<float> xs(n_down), ys(n_down), zs(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        xs[i] = down[i].x; ys[i] = down[i].y; zs[i] = down[i].z;
    }
    PointCloudSoA cloud = {xs.data(), ys.data(), zs.data(), n_down};

    printf("=================================================================================\n");
    printf("   POINTER OCTREE LEAF SIZE SWEEP BENCHMARK (N=%zu, radius=0.25m, 5000 queries)  \n", n_down);
    printf("=================================================================================\n");
    printf(" Leaf Capacity | Depth | Build (ms) | RVV Search (ms) | Scalar Search (ms) | Speedup\n");
    printf("---------------+-------+------------+-----------------+--------------------+--------\n");

    const int leaf_capacities[] = {16, 32, 64, 128, 256, 512};
    const size_t n_queries = 5000;
    float search_radius = 0.25f;

    for (int cap : leaf_capacities) {
        PointerOctree tree;
        tree.setInputCloud(cloud);
        tree.setMaxPointsPerLeaf(cap);
        tree.setMaxDepth(8);

        auto t0 = Clock::now();
        tree.build();
        auto t1 = Clock::now();
        double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Measure RVV radiusSearch
        std::vector<int> ind;
        std::vector<float> dst;
        t0 = Clock::now();
        for (size_t q = 0; q < n_queries; ++q) {
            PointXYZ query = {cloud.x[q], cloud.y[q], cloud.z[q]};
            tree.radiusSearch(query, search_radius, ind, dst);
        }
        t1 = Clock::now();
        double rvv_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        // Measure Scalar radiusSearchScalar
        t0 = Clock::now();
        for (size_t q = 0; q < n_queries; ++q) {
            PointXYZ query = {cloud.x[q], cloud.y[q], cloud.z[q]};
            tree.radiusSearchScalar(query, search_radius, ind, dst);
        }
        t1 = Clock::now();
        double sc_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

        printf("      %4d     |   8   |  %8.2f  |    %8.2f     |     %8.2f       | %5.2fx\n",
               cap, build_ms, rvv_search_ms, sc_search_ms, sc_search_ms / rvv_search_ms);
    }
    printf("=================================================================================\n");
    return 0;
}
