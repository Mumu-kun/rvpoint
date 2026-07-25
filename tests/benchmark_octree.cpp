#include "types.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>
#include <chrono>
#include <iostream>
#include <iomanip>

extern void cloud_alloc(struct Cloud* cloud, int N, float voxel_size);
extern void cloud_free(struct Cloud* cloud);
extern void build(struct Cloud* cloud, struct Node** nodes, int* node_count, int leaf_threshold);
extern void radiusSearchBatch_scalar(struct Cloud* cloud, struct QueryBatch* batch, struct Results* out);
extern "C" void radiusSearchBatch_rvv(struct Cloud* cloud, struct Node* nodes, int node_count, struct QueryBatch* batch, struct Results* out);
extern "C" void rvv_init(void);

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

static void free_results(struct Results* results) {
    std::free(results->result_flat);
    std::free(results->result_offsets);
    std::free(results->result_counts);
    results->result_flat = nullptr;
    results->result_offsets = nullptr;
    results->result_counts = nullptr;
    results->total_results = 0;
}

int main() {
    // RVV vector initialization
    rvv_init();

    const int N = 50000;      // 50k points
    const int Q = 500;        // 500 queries
    const float voxel_size = 0.01f;
    const float radius = voxel_size * 2.0f;

    std::printf("============================================================\n");
    std::printf("  RVPoint Octree Benchmark: Scalar vs. RVV\n");
    std::printf("  Point Cloud Size: %d | Queries: %d | Radius: %.3f\n", N, Q, radius);
    std::printf("============================================================\n\n");

    std::printf("Generating synthetic data...\n");
    std::mt19937 rng_cloud(42);
    std::uniform_real_distribution<float> uniform01(0.0f, 1.0f);
    std::mt19937 rng_query(99);

    struct Cloud cloud = {};
    cloud_alloc(&cloud, N, voxel_size);

    for (int i = 0; i < N; ++i) {
        cloud.x[i] = uniform01(rng_cloud);
        cloud.y[i] = uniform01(rng_cloud);
        cloud.z[i] = uniform01(rng_cloud);
    }

    std::printf("Building Morton Octree...\n");
    struct Node* nodes = nullptr;
    int node_count = 0;
    build(&cloud, &nodes, &node_count, 64);

    std::vector<float> qx(Q), qy(Q), qz(Q);
    for (int i = 0; i < Q; ++i) {
        qx[i] = uniform01(rng_query);
        qy[i] = uniform01(rng_query);
        qz[i] = uniform01(rng_query);
    }

    struct QueryBatch batch = {};
    batch.qx = qx.data();
    batch.qy = qy.data();
    batch.qz = qz.data();
    batch.Q = Q;
    batch.r = radius;

    struct Results scalar_res = {};
    struct Results rvv_res = {};

    InstructionTimer inst_timer;
    
    // --- Benchmark Scalar ---
    std::printf("Running Scalar Octree Batch Search...\n");
    
    // Warmup
    radiusSearchBatch_scalar(&cloud, &batch, &scalar_res);
    free_results(&scalar_res);

    auto start_time_sc = std::chrono::high_resolution_clock::now();
    inst_timer.reset();
    radiusSearchBatch_scalar(&cloud, &batch, &scalar_res);
    uint64_t sc_instructions = inst_timer.elapsed();
    auto end_time_sc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_sc = end_time_sc - start_time_sc;

    // --- Benchmark RVV ---
    std::printf("Running RVV Octree Batch Search...\n");

    // Warmup
    radiusSearchBatch_rvv(&cloud, nodes, node_count, &batch, &rvv_res);
    free_results(&rvv_res);

    auto start_time_rvv = std::chrono::high_resolution_clock::now();
    inst_timer.reset();
    radiusSearchBatch_rvv(&cloud, nodes, node_count, &batch, &rvv_res);
    uint64_t rvv_instructions = inst_timer.elapsed();
    auto end_time_rvv = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed_rvv = end_time_rvv - start_time_rvv;

    // Check correctness
    bool matches = true;
    if (scalar_res.total_results != rvv_res.total_results) {
        matches = false;
    } else {
        for (int q = 0; q < Q; ++q) {
            if (scalar_res.result_counts[q] != rvv_res.result_counts[q]) {
                matches = false;
                break;
            }
        }
    }

    if (!matches) {
        std::printf("[WARNING] Results mismatch between Scalar and RVV implementations!\n");
    } else {
        std::printf("[SUCCESS] Results match. Total found indices: %d\n\n", scalar_res.total_results);
    }

    // --- Output Results ---
    double inst_gain = (double)sc_instructions / rvv_instructions;
    double time_gain = elapsed_sc.count() / elapsed_rvv.count();

    std::printf("============================================================\n");
    std::printf("  BENCHMARK RESULTS\n");
    std::printf("============================================================\n");
    std::printf("Implementation | Instructions (rdinstret) | Emulation Time (ms)\n");
    std::printf("------------------------------------------------------------\n");
    std::printf("Scalar         | %-24lu | %.2f ms\n", sc_instructions, elapsed_sc.count());
    std::printf("RVV            | %-24lu | %.2f ms\n", rvv_instructions, elapsed_rvv.count());
    std::printf("------------------------------------------------------------\n");
    std::printf("Speedup / Gain | %-24.2fx | %.2fx\n", inst_gain, time_gain);
    std::printf("============================================================\n");

    free_results(&scalar_res);
    free_results(&rvv_res);
    cloud_free(&cloud);
    std::free(nodes);

    return 0;
}
