#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <ctime>

using namespace rvv_pcl;

// Simple timer utility
class Timer {
public:
    void start() { t_start = std::chrono::high_resolution_clock::now(); }
    double elapsedMs() {
        auto t_end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double, std::milli>(t_end - t_start).count();
    }
private:
    std::chrono::high_resolution_clock::time_point t_start;
};

int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "  Spatial Hash vs Octree Comparison" << std::endl;
    std::cout << "========================================" << std::endl;

    std::string input_file = "bunny.pcd";
    if (argc > 1) input_file = argv[1];
    
    // 1. Load Cloud
    std::cout << "\n[Step 1] Loading " << input_file << "..." << std::endl;
    std::vector<PointXYZ> loaded_points;
    int n = loadPCD(input_file, loaded_points);
    if (n < 0) n = loadPCD("../" + input_file, loaded_points);
    if (n < 0) n = loadPCD("data/" + input_file, loaded_points);
    if (n < 0) n = loadPCD("../data/" + input_file, loaded_points);
    if (n < 0) n = loadPCD("/workspace/data/" + input_file, loaded_points);
    
    if (n < 0) {
        std::cerr << "[FAIL] Could not load " << input_file << std::endl;
        return 1;
    }
    std::cout << "Loaded " << n << " points." << std::endl;

    // Convert to SoA
    std::vector<float> x(n), y(n), z(n);
    for(int i=0; i<n; ++i) {
        x[i] = loaded_points[i].x;
        y[i] = loaded_points[i].y;
        z[i] = loaded_points[i].z;
    }
    PointCloudSoA cloud_soa = {x.data(), y.data(), z.data(), (size_t)n};

    // 2. Voxel Grid Downsampling
    std::cout << "\n[Step 2] Voxel Grid Downsampling (Leaf=0.01)..." << std::endl;
    std::vector<PointXYZ> filtered_points(n);
    size_t n_filtered = voxel_grid_downsamp_rvv(cloud_soa, filtered_points.data(), 0.01f);
    std::cout << "Filtered count: " << n_filtered << " (Original: " << n << ")" << std::endl;

    // Prepare filtered SoA
    std::vector<float> fx(n_filtered), fy(n_filtered), fz(n_filtered);
    for(size_t i=0; i<n_filtered; ++i) {
        fx[i] = filtered_points[i].x;
        fy[i] = filtered_points[i].y;
        fz[i] = filtered_points[i].z;
    }
    PointCloudSoA filtered_soa = {fx.data(), fy.data(), fz.data(), n_filtered};

    // ========================================================================
    // OCTREE BUILD + SEARCH
    // ========================================================================
    std::cout << "\n[Method 1: OCTREE]" << std::endl;
    
    Timer timer;
    timer.start();
    Octree octree;
    octree.setInputCloud(filtered_soa);
    octree.build();
    double octree_build_time = timer.elapsedMs();
    std::cout << "  Build Time: " << octree_build_time << " ms" << std::endl;

    // Test MULTIPLE query points for better statistics
    float radius = 0.05f;
    std::vector<size_t> test_indices = {
        n_filtered / 4,
        n_filtered / 2,
        3 * n_filtered / 4,
        n_filtered / 10,
        9 * n_filtered / 10
    };
    
    double total_oct_search_time = 0;
    size_t total_oct_found = 0;
    std::vector<int> oct_indices;
    std::vector<float> oct_dists;
    
    for (size_t test_idx : test_indices) {
        PointXYZ query = filtered_points[test_idx];
        oct_indices.clear();
        oct_dists.clear();
        
        timer.start();
        size_t found = octree.radiusSearch(query, radius, oct_indices, oct_dists);
        total_oct_search_time += timer.elapsedMs();
        total_oct_found += found;
    }
    
    double avg_oct_search = total_oct_search_time / test_indices.size();
    double avg_oct_neighbors = (double)total_oct_found / test_indices.size();
    
    std::cout << "  Avg Search Time: " << avg_oct_search << " ms" << std::endl;
    std::cout << "  Avg Neighbors Found: " << avg_oct_neighbors << std::endl;

    // ========================================================================
    // SPATIAL HASH BUILD + SEARCH
    // ========================================================================
    std::cout << "\n[Method 2: SPATIAL HASH]" << std::endl;
    
    // Determine optimal cell size (typically radius or slightly larger)
    float cell_size = radius * 1.5f;
    
    timer.start();
    SpatialHash spatial_hash;
    spatial_hash.setInputCloud(filtered_soa, cell_size);
    spatial_hash.build();
    double hash_build_time = timer.elapsedMs();
    std::cout << "  Build Time: " << hash_build_time << " ms" << std::endl;

    double total_hash_search_time = 0;
    size_t total_hash_found = 0;
    std::vector<int> hash_indices;
    std::vector<float> hash_dists;
    
    for (size_t test_idx : test_indices) {
        PointXYZ query = filtered_points[test_idx];
        hash_indices.clear();
        hash_dists.clear();
        
        timer.start();
        size_t found = spatial_hash.radiusSearch(query, radius, hash_indices, hash_dists);
        total_hash_search_time += timer.elapsedMs();
        total_hash_found += found;
    }
    
    double avg_hash_search = total_hash_search_time / test_indices.size();
    double avg_hash_neighbors = (double)total_hash_found / test_indices.size();
    
    std::cout << "  Avg Search Time: " << avg_hash_search << " ms" << std::endl;
    std::cout << "  Avg Neighbors Found: " << avg_hash_neighbors << std::endl;

    // ========================================================================
    // COMPARISON
    // ========================================================================
    std::cout << "\n[COMPARISON]" << std::endl;
    std::cout << "  Build Time Speedup: " << (octree_build_time / hash_build_time) << "x" << std::endl;
    std::cout << "  Search Time Speedup: " << (avg_oct_search / avg_hash_search) << "x" << std::endl;
    
    // Check accuracy (should find same neighbors on average)
    bool accuracy_match = (total_oct_found == total_hash_found);
    std::cout << "  Total Neighbors Match: " << (accuracy_match ? "YES" : "NO") << std::endl;
    std::cout << "    Octree total: " << total_oct_found << ", Hash total: " << total_hash_found << std::endl;

    std::cout << "\n[SUCCESS] Comparison Complete!" << std::endl;
    return 0;
}
