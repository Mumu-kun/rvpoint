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

using namespace rvv_pcl;

struct InstructionTimer {
    uint64_t start;
    void reset() {
        asm volatile("" ::: "memory");
        asm volatile ("rdinstret %0" : "=r" (start));
        asm volatile("" ::: "memory");
    }
    uint64_t elapsed() {
        asm volatile("" ::: "memory");
        uint64_t end;
        asm volatile ("rdinstret %0" : "=r" (end));
        asm volatile("" ::: "memory");
        return end - start;
    }
};

int main(int argc, char** argv) {
    std::string input_file = "bunny.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }

    std::printf("============================================================\n");
    std::printf("  RVPoint 3D K-d Tree vs. Pointer Octree Real PCD Benchmark\n");
    std::printf("============================================================\n\n");

    std::printf("Loading point cloud: %s...\n", input_file.c_str());
    std::vector<PointXYZ> loaded_points;
    int n = loadPCD(input_file, loaded_points);
    if (n < 0) n = loadPCD("../" + input_file, loaded_points);
    if (n < 0) n = loadPCD("data/" + input_file, loaded_points);
    if (n < 0) n = loadPCD("../data/" + input_file, loaded_points);
    if (n < 0) n = loadPCD("/workspace/data/" + input_file, loaded_points);

    if (n < 0) {
        std::printf("[FAIL] Could not load point cloud file.\n");
        return 1;
    }
    std::printf("Loaded %d points.\n", n);

    // Convert to SoA
    std::vector<float> x(n), y(n), z(n);
    for (int i = 0; i < n; ++i) {
        x[i] = loaded_points[i].x;
        y[i] = loaded_points[i].y;
        z[i] = loaded_points[i].z;
    }
    PointCloudSoA cloud = {x.data(), y.data(), z.data(), (size_t)n};

    const float radius = 0.02f; 
    std::printf("Search Radius: %.3f\n\n", radius);

    const int Q = n;
    std::vector<PointXYZ> queries = loaded_points;

    const int leaf_sizes[] = {32, 64, 128};

    for (int leaf_size : leaf_sizes) {
        std::printf("============================================================\n");
        std::printf("  Testing Leaf Size Limit: %d\n", leaf_size);
        std::printf("============================================================\n");

        // 1. Build Octree
        PointerOctree octree;
        octree.setInputCloud(cloud);
        octree.setMaxPointsPerLeaf(leaf_size);
        octree.build();

        // 2. Build K-d Tree
        KdTree kdtree;
        kdtree.setInputCloud(cloud);
        kdtree.setMaxPointsPerLeaf(leaf_size);
        kdtree.build();

        // ─── Warmup Pass ───
        std::vector<std::vector<int>> octree_sc_indices(Q);
        std::vector<std::vector<float>> octree_sc_dists(Q);
        std::vector<std::vector<int>> octree_rvv_indices(Q);
        std::vector<std::vector<float>> octree_rvv_dists(Q);
        std::vector<std::vector<int>> kd_sc_indices(Q);
        std::vector<std::vector<float>> kd_sc_dists(Q);
        std::vector<std::vector<int>> kd_rvv_indices(Q);
        std::vector<std::vector<float>> kd_rvv_dists(Q);

        for (int i = 0; i < Q; ++i) {
            octree.radiusSearchScalar(queries[i], radius, octree_sc_indices[i], octree_sc_dists[i]);
            octree.radiusSearch(queries[i], radius, octree_rvv_indices[i], octree_rvv_dists[i]);
            kdtree.radiusSearchScalar(queries[i], radius, kd_sc_indices[i], kd_sc_dists[i]);
            kdtree.radiusSearch(queries[i], radius, kd_rvv_indices[i], kd_rvv_dists[i]);
        }

        // ─── Benchmark Pointer Octree (Scalar) ───
        InstructionTimer inst_timer;
        auto start = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        for (int i = 0; i < Q; ++i) {
            octree.radiusSearchScalar(queries[i], radius, octree_sc_indices[i], octree_sc_dists[i]);
        }
        uint64_t octree_sc_inst = inst_timer.elapsed();
        auto end = std::chrono::high_resolution_clock::now();
        double octree_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

        // ─── Benchmark Pointer Octree (RVV) ───
        start = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        for (int i = 0; i < Q; ++i) {
            octree.radiusSearch(queries[i], radius, octree_rvv_indices[i], octree_rvv_dists[i]);
        }
        uint64_t octree_rvv_inst = inst_timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double octree_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

        // ─── Benchmark K-d Tree (Scalar) ───
        start = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        for (int i = 0; i < Q; ++i) {
            kdtree.radiusSearchScalar(queries[i], radius, kd_sc_indices[i], kd_sc_dists[i]);
        }
        uint64_t kd_sc_inst = inst_timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double kd_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

        // ─── Benchmark K-d Tree (RVV) ───
        start = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        for (int i = 0; i < Q; ++i) {
            kdtree.radiusSearch(queries[i], radius, kd_rvv_indices[i], kd_rvv_dists[i]);
        }
        uint64_t kd_rvv_inst = inst_timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double kd_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

        // ─── Correctness Verification ───
        bool matches = true;
        size_t total_found = 0;
        for (int i = 0; i < Q; ++i) {
            if (kd_rvv_indices[i].size() != octree_rvv_indices[i].size()) {
                matches = false;
                break;
            }
            total_found += kd_rvv_indices[i].size();

            std::vector<int> kd_idx = kd_rvv_indices[i];
            std::vector<int> oct_idx = octree_rvv_indices[i];
            std::sort(kd_idx.begin(), kd_idx.end());
            std::sort(oct_idx.begin(), oct_idx.end());

            for (size_t k = 0; k < kd_idx.size(); ++k) {
                if (kd_idx[k] != oct_idx[k]) {
                    matches = false;
                    break;
                }
            }
            if (!matches) break;
        }

        if (matches) {
            std::printf("  [PASS] Correctness verified. Neighbors found: %zu\n", total_found);
        } else {
            std::printf("  [FAIL] K-d Tree output mismatch against Octree!\n");
        }

        std::printf("  OCTREE:\n");
        std::printf("    Scalar: %10lu inst | %6.2f ms\n", octree_sc_inst, octree_sc_ms);
        std::printf("    RVV:    %10lu inst | %6.2f ms | Speedup: %.2fx (time), %.2fx (inst)\n", 
                    octree_rvv_inst, octree_rvv_ms, octree_sc_ms / octree_rvv_ms, (double)octree_sc_inst / octree_rvv_inst);
        
        std::printf("  K-D TREE:\n");
        std::printf("    Scalar: %10lu inst | %6.2f ms\n", kd_sc_inst, kd_sc_ms);
        std::printf("    RVV:    %10lu inst | %6.2f ms | Speedup: %.2fx (time), %.2fx (inst)\n", 
                    kd_rvv_inst, kd_rvv_ms, kd_sc_ms / kd_rvv_ms, (double)kd_sc_inst / kd_rvv_inst);
        
        std::printf("  TRAVERSAL EFFICIENCY IMPROVEMENT (K-D vs Octree):\n");
        std::printf("    Scalar Comparison: %.2fx faster absolute time\n", octree_sc_ms / kd_sc_ms);
        std::printf("    RVV Comparison:    %.2fx faster absolute time\n", octree_rvv_ms / kd_rvv_ms);
    }

    std::printf("============================================================\n");
    return 0;
}
