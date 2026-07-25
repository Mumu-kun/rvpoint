#include "kd_tree/kd_tree.h"
#include "pointer_octree/pointer_octree.h"
#include "include/simple_pcd_loader.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <chrono>
#include <random>
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

void generate_synthetic_data(size_t N, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, std::vector<PointXYZ>& points) {
    x.resize(N); y.resize(N); z.resize(N); points.resize(N);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.0f, 100.0f);
    for (size_t i = 0; i < N; ++i) {
        x[i] = dist(rng);
        y[i] = dist(rng);
        z[i] = dist(rng);
        points[i] = {x[i], y[i], z[i]};
    }
}

void run_test_set(const PointCloudSoA& cloud, const std::vector<PointXYZ>& queries, float radius, const std::string& label) {
    std::printf("\n============================================================\n");
    std::printf("  Dataset: %s | N = %zu | Q = %zu | Radius = %.3f\n", label.c_str(), cloud.n, queries.size(), radius);
    std::printf("============================================================\n");

    const int leaf_sizes[] = {32, 64, 128, 256};
    const size_t Q = queries.size();

    for (int leaf_size : leaf_sizes) {
        std::printf("\n------------------------------------------------------------\n");
        std::printf("  Leaf Size Limit: %d\n", leaf_size);
        std::printf("------------------------------------------------------------\n");

        PointerOctree octree;
        octree.setInputCloud(cloud);
        octree.setMaxPointsPerLeaf(leaf_size);
        octree.build();

        KdTree kdtree;
        kdtree.setInputCloud(cloud);
        kdtree.setMaxPointsPerLeaf(leaf_size);
        kdtree.build();

        std::vector<std::vector<int>> octree_sc_indices(Q);
        std::vector<std::vector<float>> octree_sc_dists(Q);
        std::vector<std::vector<int>> octree_rvv_indices(Q);
        std::vector<std::vector<float>> octree_rvv_dists(Q);
        std::vector<std::vector<int>> kd_sc_indices(Q);
        std::vector<std::vector<float>> kd_sc_dists(Q);
        std::vector<std::vector<int>> kd_rvv_indices(Q);
        std::vector<std::vector<float>> kd_rvv_dists(Q);

        for (size_t i = 0; i < Q; ++i) {
            octree.radiusSearchScalar(queries[i], radius, octree_sc_indices[i], octree_sc_dists[i]);
            octree.radiusSearch(queries[i], radius, octree_rvv_indices[i], octree_rvv_dists[i]);
            kdtree.radiusSearchScalar(queries[i], radius, kd_sc_indices[i], kd_sc_dists[i]);
            kdtree.radiusSearch(queries[i], radius, kd_rvv_indices[i], kd_rvv_dists[i]);
        }

        InstructionTimer timer;
        auto start = std::chrono::high_resolution_clock::now();
        timer.reset();
        for (size_t i = 0; i < Q; ++i) {
            octree.radiusSearchScalar(queries[i], radius, octree_sc_indices[i], octree_sc_dists[i]);
        }
        uint64_t oct_sc_inst = timer.elapsed();
        auto end = std::chrono::high_resolution_clock::now();
        double oct_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        timer.reset();
        for (size_t i = 0; i < Q; ++i) {
            octree.radiusSearch(queries[i], radius, octree_rvv_indices[i], octree_rvv_dists[i]);
        }
        uint64_t oct_rvv_inst = timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double oct_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        timer.reset();
        for (size_t i = 0; i < Q; ++i) {
            kdtree.radiusSearchScalar(queries[i], radius, kd_sc_indices[i], kd_sc_dists[i]);
        }
        uint64_t kd_sc_inst = timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double kd_sc_ms = std::chrono::duration<double, std::milli>(end - start).count();

        start = std::chrono::high_resolution_clock::now();
        timer.reset();
        for (size_t i = 0; i < Q; ++i) {
            kdtree.radiusSearch(queries[i], radius, kd_rvv_indices[i], kd_rvv_dists[i]);
        }
        uint64_t kd_rvv_inst = timer.elapsed();
        end = std::chrono::high_resolution_clock::now();
        double kd_rvv_ms = std::chrono::duration<double, std::milli>(end - start).count();

        bool pass = true;
        size_t total_found = 0;
        for (size_t i = 0; i < Q; ++i) {
            if (kd_rvv_indices[i].size() != octree_rvv_indices[i].size()) {
                pass = false;
                break;
            }
            total_found += kd_rvv_indices[i].size();

            std::vector<int> oct_idx = octree_rvv_indices[i];
            std::vector<int> kd_idx = kd_rvv_indices[i];
            std::sort(oct_idx.begin(), oct_idx.end());
            std::sort(kd_idx.begin(), kd_idx.end());

            for (size_t k = 0; k < oct_idx.size(); ++k) {
                if (oct_idx[k] != kd_idx[k]) {
                    pass = false;
                    break;
                }
            }
            if (!pass) break;
        }

        if (pass) {
            std::printf("  [PASS] Correctness verified. Neighbors: %zu\n", total_found);
        } else {
            std::printf("  [FAIL] Result Mismatch!\n");
        }

        std::printf("  Octree:\n");
        std::printf("    Scalar: %12lu inst | %7.2f ms\n", oct_sc_inst, oct_sc_ms);
        std::printf("    RVV:    %12lu inst | %7.2f ms | Speedup: %.2fx (time), %.2fx (inst)\n", 
                    oct_rvv_inst, oct_rvv_ms, oct_sc_ms / oct_rvv_ms, (double)oct_sc_inst / oct_rvv_inst);

        std::printf("  K-d Tree:\n");
        std::printf("    Scalar: %12lu inst | %7.2f ms\n", kd_sc_inst, kd_sc_ms);
        std::printf("    RVV:    %12lu inst | %7.2f ms | Speedup: %.2fx (time), %.2fx (inst)\n", 
                    kd_rvv_inst, kd_rvv_ms, kd_sc_ms / kd_rvv_ms, (double)kd_sc_inst / kd_rvv_inst);

        std::printf("  Traversal Efficiency (K-d vs. Octree):\n");
        std::printf("    Scalar: %.2fx faster absolute time\n", oct_sc_ms / kd_sc_ms);
        std::printf("    RVV:    %.2fx faster absolute time\n", oct_rvv_ms / kd_rvv_ms);
    }
}

int main(int argc, char** argv) {
    std::string pcd_file = "bunny.pcd";
    if (argc > 1) {
        pcd_file = argv[1];
    }

    std::printf("============================================================\n");
    std::printf("  RVPoint 3D K-d Tree Comprehensive Sweep Benchmark\n");
    std::printf("============================================================\n");

    std::vector<PointXYZ> loaded_points;
    int n = loadPCD(pcd_file, loaded_points);
    if (n < 0) n = loadPCD("../" + pcd_file, loaded_points);
    if (n < 0) n = loadPCD("data/" + pcd_file, loaded_points);
    if (n < 0) n = loadPCD("../data/" + pcd_file, loaded_points);
    if (n < 0) n = loadPCD("/workspace/data/" + pcd_file, loaded_points);

    if (n >= 0) {
        std::vector<float> rx(n), ry(n), rz(n);
        for (int i = 0; i < n; ++i) {
            rx[i] = loaded_points[i].x;
            ry[i] = loaded_points[i].y;
            rz[i] = loaded_points[i].z;
        }
        PointCloudSoA real_cloud = {rx.data(), ry.data(), rz.data(), (size_t)n};
        run_test_set(real_cloud, loaded_points, 0.02f, "LiDAR Bunny PCD");
    } else {
        std::printf("[WARN] Could not load bunny.pcd, skipping PCD stage.\n");
    }

    {
        std::vector<float> sx, sy, sz;
        std::vector<PointXYZ> spoints;
        generate_synthetic_data(10000, sx, sy, sz, spoints);
        PointCloudSoA synth_cloud = {sx.data(), sy.data(), sz.data(), 10000};
        std::vector<PointXYZ> queries(spoints.begin(), spoints.begin() + 1000);
        run_test_set(synth_cloud, queries, 5.0f, "Synthetic 10K Cloud");
    }

    {
        std::vector<float> lx, ly, lz;
        std::vector<PointXYZ> lpoints;
        generate_synthetic_data(50000, lx, ly, lz, lpoints);
        PointCloudSoA synth_cloud = {lx.data(), ly.data(), lz.data(), 50000};
        std::vector<PointXYZ> queries(lpoints.begin(), lpoints.begin() + 1000);
        run_test_set(synth_cloud, queries, 5.0f, "Synthetic 50K Cloud");
    }

    std::printf("\n============================================================\n");
    return 0;
}
