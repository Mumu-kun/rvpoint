#include "../src/include/rvv_pcl.h"
#include <vector>
#include <random>
#include <iostream>
#include <iomanip>
#include <string>
#include <cstdlib>
#include <functional>

using namespace rvv_pcl;

// Timer measuring Instructions (instret) - Accurate for QEMU instruction count
struct Timer {
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

struct BenchmarkData {
    std::string name;
    uint64_t sc_inst;
    uint64_t rvv_inst;
};

// Global config
const size_t N = 1024; // Standard benchmark size

// Data generation helper
void generate_data(size_t n, std::vector<float>& x, std::vector<float>& y, std::vector<float>& z, 
                   std::vector<PointXYZ>& aos, PointCloudSoA& soa) {
    std::srand(42); 
    x.resize(n); y.resize(n); z.resize(n); aos.resize(n);
    for(size_t i=0; i<n; ++i) {
        x[i] = (float)(std::rand() % 1000) / 10.0f;
        y[i] = (float)(std::rand() % 1000) / 10.0f;
        z[i] = (float)(std::rand() % 1000) / 10.0f;
        aos[i] = {x[i], y[i], z[i]};
    }
    soa = {x.data(), y.data(), z.data(), n};
}

BenchmarkData bench_voxel() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    std::vector<PointXYZ> out(N);
    float leaf = 0.5f;

    Timer t;
    t.reset();
    voxel_grid_downsamp_sc(aos.data(), N, out.data(), leaf);
    uint64_t sc = t.elapsed();

    t.reset();
    voxel_grid_downsamp_rvv(soa, out.data(), leaf);
    uint64_t rvv = t.elapsed();

    return {"VoxelGrid", sc, rvv};
}

BenchmarkData bench_voxel_v2() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    std::vector<PointXYZ> out(N);
    float leaf = 0.5f;

    Timer t;
    t.reset();
    voxel_grid_downsamp_sc(aos.data(), N, out.data(), leaf);
    uint64_t sc = t.elapsed();

    t.reset();
    voxel_grid_downsamp_rvv_v2(soa, out.data(), leaf);
    uint64_t rvv = t.elapsed();

    return {"VoxelGrid_v2", sc, rvv};
}

BenchmarkData bench_sor() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    std::vector<PointXYZ> out(N);
    int k=10; float alpha=1.0;

    Timer t;
    t.reset();
    sor_sc(aos.data(), N, out.data(), k, alpha);
    uint64_t sc = t.elapsed();

    t.reset();
    sor_rvv(soa, out.data(), k, alpha);
    uint64_t rvv = t.elapsed();

    return {"SOR", sc, rvv};
}

BenchmarkData bench_normal() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    std::vector<float> nx(N), ny(N), nz(N);
    int k=10; float r=2.0f;

    Timer t;
    t.reset();
    normal_estimation_sc(aos.data(), N, nx.data(), ny.data(), nz.data(), k, r);
    uint64_t sc = t.elapsed();

    // For RVV, pre-build octree (excluded from timer to measure kernel efficiency)
    Octree octree;
    octree.setInputCloud(soa);
    octree.build();
    
    t.reset();
    normal_estimation_rvv(soa, octree, nx.data(), ny.data(), nz.data(), k, r);
    uint64_t rvv = t.elapsed();

    return {"NormalEst", sc, rvv};
}

BenchmarkData bench_radius() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    std::vector<int> idx(100);
    std::vector<float> dists(100);
    PointXYZ query = {50, 50, 50};
    float r = 10.0f;

    Timer t;
    t.reset();
    radius_search_sc(aos.data(), N, query, r, idx.data(), dists.data(), 100);
    uint64_t sc = t.elapsed();

    t.reset();
    radius_search_rvv(soa, query, r, idx.data(), dists.data(), 100);
    uint64_t rvv = t.elapsed();

    return {"Radius", sc, rvv};
}

BenchmarkData bench_ransac() {
    std::vector<float> x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA soa;
    generate_data(N, x, y, z, aos, soa);
    float model[4];
    
    Timer t;
    t.reset();
    ransac_plane_sc(aos.data(), N, 0.1f, 500, model);
    uint64_t sc = t.elapsed();

    t.reset();
    ransac_plane_rvv(soa, 0.1f, 500, model);
    uint64_t rvv = t.elapsed();

    return {"RANSAC", sc, rvv};
}



#include <fstream>
#include <sstream>
#include <ctime>
#include <filesystem>

// ... (existing includes and Timer struct remains the same)

// Helper for dual output (terminal + file)
struct DualStream {
    std::ofstream& file;
    
    template<typename T>
    DualStream& operator<<(const T& data) {
        std::cout << data;
        file << data;
        return *this;
    }
    
    // Support manipulators like std::endl
    DualStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::cout << manip;
        file << manip;
        return *this;
    }
};

int main(int argc, char** argv) {
    // 1. Setup timestamped file
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    
    // Ensure results dir exists
    system("mkdir -p results"); 
    
    std::string filename = "results/benchmark_report_" + ss.str() + ".txt";
    std::ofstream outfile(filename);
    
    if (!outfile.is_open()) {
        std::cerr << "Error: Could not open output file " << filename << std::endl;
        return 1;
    }
    
    DualStream out = {outfile};
    out << "Saving report to: " << filename << "\n\n";

    // If specific algo requested, keep flexibility (but default to all)
    std::vector<BenchmarkData> results;
    
    // Silence stdout regarding running status
    // std::cout << "Running benchmarks (N=" << N << ")..." << std::endl;
    
    results.push_back(bench_voxel());
    results.push_back(bench_voxel_v2());
    results.push_back(bench_sor());
    results.push_back(bench_normal());
    results.push_back(bench_radius());
    results.push_back(bench_ransac());

    // Print Table in User's Requested Format to BOTH stdout and file
    out << "============================================================\n";
    out << "  RVPoint Benchmark (Instruction Counts) - QEMU Trace       \n";
    out << "  *Measured by counting QEMU TB executions (-one-insn-per-tb)*\n";
    out << "============================================================\n";
    out << "Algorithm            | Scalar(ins)  | RVV(ins)     | Ratio\n";
    out << "------------------------------------------------------------\n";
    
    for (const auto& r : results) {
        // Just lowercase the name to match example (VoxelGrid -> voxel)
        std::string name_lower = r.name;
        // Manual mapping for exact match
        if (name_lower == "VoxelGrid") name_lower = "voxel";
        else if (name_lower == "VoxelGrid_v2") name_lower = "voxel_v2";
        else if (name_lower == "SOR") name_lower = "sor";
        else if (name_lower == "NormalEst") name_lower = "normal";
        else if (name_lower == "Radius") name_lower = "radius";
        else if (name_lower == "RANSAC") name_lower = "ransac";

        double ratio = (double)r.sc_inst / (double)r.rvv_inst;
        
        // Format buffer for clean alignment in dual stream
        std::stringstream line_ss;
        line_ss << std::left << std::setw(21) << name_lower 
                << "| " << std::left << std::setw(13) << r.sc_inst 
                << "| " << std::left << std::setw(13) << r.rvv_inst 
                << "| " << std::fixed << std::setprecision(2) << ratio << "x\n";
        
        out << line_ss.str();
    }
    out << "============================================================\n";
    
    outfile.close();
    return 0;
}


