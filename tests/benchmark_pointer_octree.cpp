#include "pointer_octree/pointer_octree.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
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

int main() {
    const int N = 50000;          // 50k points
    const float radius = 5.0f;    // Search radius
    const float max_val = 100.0f;

    std::printf("============================================================\n");
    std::printf("  RVPoint Pointer-Based Octree Benchmark: Scalar vs. RVV\n");
    std::printf("  Point Cloud Size: %d | Radius: %.3f\n", N, radius);
    std::printf("============================================================\n\n");

    std::printf("Generating synthetic data...\n");
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist(0.0f, max_val);

    std::vector<float> x(N), y(N), z(N);
    for (int i = 0; i < N; ++i) {
        x[i] = dist(gen);
        y[i] = dist(gen);
        z[i] = dist(gen);
    }
    PointCloudSoA cloud = {x.data(), y.data(), z.data(), (size_t)N};

    const int query_sizes[] = {100, 1000, 5000};
    const int leaf_sizes[] = {32, 64, 128, 256, 512};

    for (int Q : query_sizes) {
        std::printf("\n============================================================\n");
        std::printf("  Running with Q = %d queries\n", Q);
        std::printf("============================================================\n");

        std::printf("Generating %d query points...\n", Q);
        std::vector<PointXYZ> queries(Q);
        for (int i = 0; i < Q; ++i) {
            queries[i] = {dist(gen), dist(gen), dist(gen)};
        }

        for (int leaf_size : leaf_sizes) {
            std::printf("------------------------------------------------------------\n");
            std::printf("  Testing Leaf Size Limit: %d\n", leaf_size);
            std::printf("------------------------------------------------------------\n");

            PointerOctree octree;
            octree.setInputCloud(cloud);
            octree.setMaxPointsPerLeaf(leaf_size);
            octree.build();

            std::vector<std::vector<int>> scalar_indices(Q);
            std::vector<std::vector<float>> scalar_dists(Q);

            // Warmup
            for (int i = 0; i < Q; ++i) {
                octree.radiusSearchScalar(queries[i], radius, scalar_indices[i], scalar_dists[i]);
            }

            InstructionTimer inst_timer;
            auto start_time_sc = std::chrono::high_resolution_clock::now();
            inst_timer.reset();
            for (int i = 0; i < Q; ++i) {
                octree.radiusSearchScalar(queries[i], radius, scalar_indices[i], scalar_dists[i]);
            }
            uint64_t sc_instructions = inst_timer.elapsed();
            auto end_time_sc = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_sc = end_time_sc - start_time_sc;

            std::vector<std::vector<int>> rvv_indices(Q);
            std::vector<std::vector<float>> rvv_dists(Q);

            // Warmup
            for (int i = 0; i < Q; ++i) {
                octree.radiusSearch(queries[i], radius, rvv_indices[i], rvv_dists[i]);
            }

            auto start_time_rvv = std::chrono::high_resolution_clock::now();
            inst_timer.reset();
            for (int i = 0; i < Q; ++i) {
                octree.radiusSearch(queries[i], radius, rvv_indices[i], rvv_dists[i]);
            }
            uint64_t rvv_instructions = inst_timer.elapsed();
            auto end_time_rvv = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_rvv = end_time_rvv - start_time_rvv;

            // Correctness Verification
            bool matches = true;
            size_t total_found = 0;
            for (int i = 0; i < Q; ++i) {
                if (scalar_indices[i].size() != rvv_indices[i].size()) {
                    matches = false;
                    break;
                }
                total_found += scalar_indices[i].size();

                std::vector<int> sc_idx = scalar_indices[i];
                std::vector<int> rvv_idx = rvv_indices[i];
                std::sort(sc_idx.begin(), sc_idx.end());
                std::sort(rvv_idx.begin(), rvv_idx.end());

                for (size_t k = 0; k < sc_idx.size(); ++k) {
                    if (sc_idx[k] != rvv_idx[k]) {
                        matches = false;
                        break;
                    }
                }
                if (!matches) break;
            }

            if (matches) {
                std::printf("  [PASS] Results match. Neighbors: %zu\n", total_found);
            } else {
                std::printf("  [FAIL] Results mismatch!\n");
            }

            double inst_gain = (double)sc_instructions / rvv_instructions;
            double time_gain = elapsed_sc.count() / elapsed_rvv.count();

            std::printf("  Scalar Instructions: %lu | RVV Instructions: %lu | Speedup: %.2fx\n",
                        sc_instructions, rvv_instructions, inst_gain);
            std::printf("  Scalar Time: %.2f ms | RVV Time: %.2f ms | Speedup: %.2fx\n",
                    elapsed_sc.count(), elapsed_rvv.count(), time_gain);
        }
    }

    std::printf("============================================================\n");
    return 0;
}
