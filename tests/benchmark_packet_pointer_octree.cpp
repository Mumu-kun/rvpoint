#include "packet_pointer_octree/packet_pointer_octree.h"
#include "include/simple_pcd_loader.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>

using namespace rvv_pcl;

// Timer for instruction counting (rdinstret)
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
    std::string input_file = "living_room.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }

    std::printf("============================================================\n");
    std::printf("  RVPoint Query-Parallel Packet Octree PCD Benchmark\n");
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

    // Standard search radius for feature estimation (e.g. 2cm for bunny)
    const float radius = 0.02f; 
    std::printf("Search Radius: %.3f\n\n", radius);

    // We will query for EVERY point in the cloud (Q = N) to estimate features
    std::vector<PointXYZ> queries = loaded_points;

    // Test a range of leaf sizes (from 8 to 128)
    const int leaf_sizes[] = {8, 16, 32, 64, 128, 256, 512};

    for (int leaf_size : leaf_sizes) {
        std::printf("------------------------------------------------------------\n");
        std::printf("  Testing Leaf Size Limit: %d\n", leaf_size);
        std::printf("------------------------------------------------------------\n");

        PacketPointerOctree octree;
        octree.setInputCloud(cloud);
        octree.setMaxPointsPerLeaf(leaf_size);
        octree.build();

        std::vector<std::vector<int>> scalar_indices;
        std::vector<std::vector<float>> scalar_dists;

        // Warmup Scalar
        octree.radiusSearchBatchScalar(queries, radius, scalar_indices, scalar_dists);

        InstructionTimer inst_timer;
        auto start_time_sc = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        octree.radiusSearchBatchScalar(queries, radius, scalar_indices, scalar_dists);
        uint64_t sc_instructions = inst_timer.elapsed();
        auto end_time_sc = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_sc = end_time_sc - start_time_sc;

        std::vector<std::vector<int>> rvv_indices;
        std::vector<std::vector<float>> rvv_dists;

        // Warmup RVV
        octree.radiusSearchBatch(queries, radius, rvv_indices, rvv_dists);

        auto start_time_rvv = std::chrono::high_resolution_clock::now();
        inst_timer.reset();
        octree.radiusSearchBatch(queries, radius, rvv_indices, rvv_dists);
        uint64_t rvv_instructions = inst_timer.elapsed();
        auto end_time_rvv = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_rvv = end_time_rvv - start_time_rvv;

        // Correctness Verification
        bool matches = true;
        size_t total_found_sc = 0;
        size_t total_found_rvv = 0;
        const int Q = (int)queries.size();
        int first_mismatch_idx = -1;

        for (int i = 0; i < Q; ++i) {
            total_found_sc += scalar_indices[i].size();
            total_found_rvv += rvv_indices[i].size();

            if (scalar_indices[i].size() != rvv_indices[i].size()) {
                if (first_mismatch_idx == -1) first_mismatch_idx = i;
                matches = false;
            } else {
                std::vector<int> sc_idx = scalar_indices[i];
                std::vector<int> rvv_idx = rvv_indices[i];
                std::sort(sc_idx.begin(), sc_idx.end());
                std::sort(rvv_idx.begin(), rvv_idx.end());

                for (size_t k = 0; k < sc_idx.size(); ++k) {
                    if (sc_idx[k] != rvv_idx[k]) {
                        if (first_mismatch_idx == -1) first_mismatch_idx = i;
                        matches = false;
                        break;
                    }
                }
            }
        }

        if (matches) {
            std::printf("  [PASS] Results match. Neighbors found: %zu\n", total_found_sc);
        } else {
            std::printf("  [FAIL] Results mismatch! Scalar found: %zu, RVV found: %zu. First mismatch at query %d.\n",
                        total_found_sc, total_found_rvv, first_mismatch_idx);
            if (first_mismatch_idx != -1) {
                std::printf("    Query %d: Scalar size %zu, RVV size %zu\n",
                            first_mismatch_idx, scalar_indices[first_mismatch_idx].size(),
                            rvv_indices[first_mismatch_idx].size());
            }
        }

        double inst_gain = (double)sc_instructions / rvv_instructions;
        double time_gain = elapsed_sc.count() / elapsed_rvv.count();

        std::printf("  Scalar Instructions: %lu | RVV Instructions: %lu | Speedup: %.2fx\n",
                    sc_instructions, rvv_instructions, inst_gain);
        std::printf("  Scalar Time: %.2f ms | RVV Time: %.2f ms | Speedup: %.2fx\n",
                    elapsed_sc.count(), elapsed_rvv.count(), time_gain);
    }

    std::printf("============================================================\n");
    return 0;
}
