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
// Utilities
// ============================================================================
struct Timer {
    std::chrono::high_resolution_clock::time_point start;
    
    void reset() { start = std::chrono::high_resolution_clock::now(); }
    
    double elapsed_ms() {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(end - start).count();
    }
};

void print_row(const std::string& name, double t_sc, double t_rvv) {
    std::cout << std::left << std::setw(20) << name 
              << "| " << std::setw(12) << t_sc 
              << "| " << std::setw(12) << t_rvv 
              << "| " << std::fixed << std::setprecision(2) << (t_sc / t_rvv) << "x" << std::endl;
}

// ============================================================================
// Benchmarks
// ============================================================================

void bench_voxel() {
    size_t N = 10000;
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

    Timer t;
    t.reset();
    voxel_grid_downsamp_sc(input_aos.data(), N, out.data(), leaf);
    double t_sc = t.elapsed_ms();

    t.reset();
    voxel_grid_downsamp_rvv(input_soa, out.data(), leaf);
    double t_rvv = t.elapsed_ms();

    print_row("Voxel Grid", t_sc, t_rvv);
}

void bench_sor() {
    size_t N = 2000; // O(N^2)
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

    Timer t;
    t.reset();
    sor_sc(input_aos.data(), N, out.data(), k, alpha);
    double t_sc = t.elapsed_ms();

    t.reset();
    sor_rvv(input_soa, out.data(), k, alpha);
    double t_rvv = t.elapsed_ms();

    print_row("SOR", t_sc, t_rvv);
}

void bench_normal() {
    size_t N = 2000; // O(N^2)
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

    Timer t;
    t.reset();
    normal_estimation_sc(input_aos.data(), N, nx.data(), ny.data(), nz.data(), k);
    double t_sc = t.elapsed_ms();

    t.reset();
    normal_estimation_rvv(input_soa, nx.data(), ny.data(), nz.data(), k);
    double t_rvv = t.elapsed_ms();

    print_row("Normal Est.", t_sc, t_rvv);
}

void bench_radius() {
    size_t N = 4000; 
    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    for(size_t i=0; i<N; ++i) {
        x[i] = (float)(std::rand() % 100);
        y[i] = (float)(std::rand() % 100);
        z[i] = (float)(std::rand() % 100);
        input_aos[i] = {x[i], y[i], z[i]};
    }
    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};
    
    // Perform 100 queries to average out
    std::vector<int> idx(100);
    std::vector<float> dists(100);
    PointXYZ query = {50, 50, 50};
    float r = 10.0f;

    Timer t;
    t.reset();
    for(int i=0; i<100; ++i)
        radius_search_sc(input_aos.data(), N, query, r, idx.data(), dists.data(), 100);
    double t_sc = t.elapsed_ms();

    t.reset();
    for(int i=0; i<100; ++i)
        radius_search_rvv(input_soa, query, r, idx.data(), dists.data(), 100);
    double t_rvv = t.elapsed_ms();

    print_row("Radius Search", t_sc, t_rvv);
}

void bench_ransac() {
    size_t N = 5000; 
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
    
    Timer t;
    t.reset();
    ransac_plane_sc(input_aos.data(), N, 0.1f, 500, model);
    double t_sc = t.elapsed_ms();

    t.reset();
    ransac_plane_rvv(input_soa, 0.1f, 500, model);
    double t_rvv = t.elapsed_ms();

    print_row("RANSAC", t_sc, t_rvv);
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << "  RVPoint Benchmark (QEMU Emulation)" << std::endl;
    std::cout << "  *Times are in milliseconds (lower is better)*" << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << std::left << std::setw(20) << "Algorithm" 
              << "| " << std::setw(12) << "Scalar(ms)" 
              << "| " << std::setw(12) << "RVV(ms)" 
              << "| " << "Speedup" << std::endl;
    std::cout << "------------------------------------------------------------" << std::endl;

    bench_voxel();
    bench_sor();
    bench_normal();
    bench_radius();
    bench_ransac();
    
    std::cout << "============================================================" << std::endl;
    return 0;
}
