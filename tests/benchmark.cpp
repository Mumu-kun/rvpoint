#include "../src/include/rvv_pcl.h"
#include <vector>
#include <random>
#include <iostream>
#include <chrono>
#include <iomanip>
#include <string>
#include <cstdlib>

using namespace rvv_pcl;

// ============================================================================
// Configuration
// ============================================================================
// Global dataset size for easy adjustment (Small for Spike, Large for QEMU)
// You can change this single value.
const size_t GLOBAL_N = 100; 

// ============================================================================
// Utilities
// ============================================================================
static inline uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

struct Timer {
    uint64_t start;
    
    void reset() { start = read_cycles(); }
    
    uint64_t elapsed_cycles() {
        uint64_t end = read_cycles();
        return end - start;
    }
};

void print_row(const std::string& name, uint64_t t_sc, uint64_t t_rvv) {
    double speedup = (double)t_sc / (double)t_rvv;
    std::cout << std::left << std::setw(20) << name 
              << "| " << std::setw(12) << t_sc 
              << "| " << std::setw(12) << t_rvv 
              << "| " << std::fixed << std::setprecision(2) << speedup << "x" << std::endl;
}

void log_start(const std::string& name) {
    std::cout << "[RUNNING] " << name << " (N=" << GLOBAL_N << ")..." << std::endl;
}

// ============================================================================
// Benchmarks
// ============================================================================

void run_test(const std::string& algo, const std::string& mode) {
    // Voxel Grid
    if (algo == "voxel") {
        size_t N = GLOBAL_N;
        std::vector<float> x(N), y(N), z(N);
        std::vector<PointXYZ> input_aos(N);
        std::srand(42);
        for(size_t i=0; i<N; ++i) {
            x[i] = (float)(std::rand() % 1000) / 10.0f;
            y[i] = (float)(std::rand() % 1000) / 10.0f;
            z[i] = (float)(std::rand() % 1000) / 10.0f;
            input_aos[i] = {x[i], y[i], z[i]};
        }
        PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
        std::vector<PointXYZ> out(N);
        float leaf = 0.5f;

        if (mode == "sc") {
            voxel_grid_downsamp_sc(input_aos.data(), N, out.data(), leaf);
        } else if (mode == "rvv") {
            voxel_grid_downsamp_rvv(input_soa, out.data(), leaf);
        } else if (mode == "setup") {
            return;
        }
    }
    // SOR
    else if (algo == "sor") {
        size_t N = GLOBAL_N;
        std::vector<float> x(N), y(N), z(N);
        std::vector<PointXYZ> input_aos(N);
        for(size_t i=0; i<N; ++i) {
            x[i] = (float)(std::rand() % 100);
            y[i] = (float)(std::rand() % 100);
            z[i] = (float)(std::rand() % 100);
            input_aos[i] = {x[i], y[i], z[i]};
        }
        PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
        std::vector<PointXYZ> out(N);
        int k=10; float alpha=1.0;

        if (mode == "sc") {
            sor_sc(input_aos.data(), N, out.data(), k, alpha);
        } else if (mode == "rvv") {
            sor_rvv(input_soa, out.data(), k, alpha);
        } else if (mode == "setup") {
            return;
        }
    }
    // Normal Estimation
    else if (algo == "normal") {
        size_t N = GLOBAL_N;
        std::vector<float> x(N), y(N), z(N);
        std::vector<PointXYZ> input_aos(N);
        for(size_t i=0; i<N; ++i) {
            x[i] = (float)(std::rand() % 100);
            y[i] = (float)(std::rand() % 100);
            z[i] = (float)(std::rand() % 100);
            input_aos[i] = {x[i], y[i], z[i]};
        }
        PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
        std::vector<float> nx(N), ny(N), nz(N);
        int k=10;

        if (mode == "sc") {
            normal_estimation_sc(input_aos.data(), N, nx.data(), ny.data(), nz.data(), k, 2.0f);
        } else if (mode == "rvv") {
            Octree octree;
            octree.setInputCloud(input_soa);
            octree.build();
            normal_estimation_rvv(input_soa, octree, nx.data(), ny.data(), nz.data(), k, 2.0f);
        } else if (mode == "setup") {
            return;
        }
    }
    // Radius Search
    else if (algo == "radius") {
        size_t N = GLOBAL_N; 
        std::vector<float> x(N), y(N), z(N);
        std::vector<PointXYZ> input_aos(N);
        for(size_t i=0; i<N; ++i) {
            x[i] = (float)(std::rand() % 100);
            y[i] = (float)(std::rand() % 100);
            z[i] = (float)(std::rand() % 100);
            input_aos[i] = {x[i], y[i], z[i]};
        }
        PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
        
        std::vector<int> idx(100);
        std::vector<float> dists(100);
        PointXYZ query = {50, 50, 50};
        float r = 10.0f;

        if (mode == "sc") {
            // Run once for instruction count (looping 100 times earlier was for timing stability)
            // Just run 1 iteration effectively
             radius_search_sc(input_aos.data(), N, query, r, idx.data(), dists.data(), 100);
        } else if (mode == "rvv") {
             radius_search_rvv(input_soa, query, r, idx.data(), dists.data(), 100);
        } else if (mode == "setup") {
             return;
        }
    }
    // RANSAC
    else if (algo == "ransac") {
        size_t N = GLOBAL_N; 
        std::vector<float> x(N), y(N), z(N);
        std::vector<PointXYZ> input_aos(N);
        for(size_t i=0; i<N; ++i) {
            x[i] = (float)(std::rand() % 100);
            y[i] = (float)(std::rand() % 100);
            z[i] = (float)(std::rand() % 100);
            input_aos[i] = {x[i], y[i], z[i]};
        }
        PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
        float model[4];
        
        if (mode == "sc") {
            ransac_plane_sc(input_aos.data(), N, 0.1f, 500, model);
        } else if (mode == "rvv") {
            ransac_plane_rvv(input_soa, 0.1f, 500, model);
        } else if (mode == "setup") {
            return;
        }
    }
}

int main(int argc, char** argv) {
    if (argc < 3) {
        // Fallback to original behavior if no args (optional, or just exit)
        std::cout << "Usage: benchmark <algo> <sc|rvv>" << std::endl;
        return 1;
    }
    
    std::string algo = argv[1];
    std::string mode = argv[2];
    
    run_test(algo, mode);

    return 0;
}
