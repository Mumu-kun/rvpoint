#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <ctime>

using namespace rvv_pcl;

// Mimics https://pcl.readthedocs.io/projects/tutorials/en/master/walkthrough.html
int main(int argc, char** argv) {
    std::cout << "========================================" << std::endl;
    std::cout << "   RISC-V PCL Pipeline Walkthrough      " << std::endl;
    std::cout << "========================================" << std::endl;

    std::string input_file = "bunny.pcd";
    if (argc > 1) input_file = argv[1];
    
    std::string base_name = input_file;
    size_t last_slash = base_name.find_last_of("/\\");
    if (last_slash != std::string::npos) base_name = base_name.substr(last_slash + 1);
    std::string stem = base_name.substr(0, base_name.find_last_of('.'));
    
    // Generate timestamp for serialization
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_now = std::localtime(&time_t_now);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_now);
    
    // Output to results/ directory
    // If running from bin/, we need ../results/
    std::string output_dir = "results/";
    std::ifstream check_res("results");
    if (!check_res.good()) output_dir = "../results/";
    
    std::string output_file = output_dir + stem + "_" + timestamp + "_voxelized.pcd";

    // 1. Load Cloud
    std::cout << "\n[Step 1] Loading " << input_file << "..." << std::endl;
    std::vector<PointXYZ> loaded_points;
    // Try current directory first, then data/, then absolute
    int n = loadPCD(input_file, loaded_points);
    if (n < 0) n = loadPCD("../" + input_file, loaded_points); // Check parent (e.g. if in bin/)
    if (n < 0) n = loadPCD("data/" + input_file, loaded_points);
    if (n < 0) n = loadPCD("../data/" + input_file, loaded_points); 
    if (n < 0) n = loadPCD("/workspace/data/" + input_file, loaded_points);
    
    if (n < 0) {
        std::cerr << "[FAIL] Could not load " << input_file << std::endl;
        return 1;
    }
    std::cout << "Loaded " << n << " points." << std::endl;

    // Convert to SoA for processing
    std::vector<float> x(n), y(n), z(n);
    for(int i=0; i<n; ++i) {
        x[i] = loaded_points[i].x;
        y[i] = loaded_points[i].y;
        z[i] = loaded_points[i].z;
    }
    PointCloudSoA cloud_soa = {x.data(), y.data(), z.data(), (size_t)n};


    // 2. Voxel Grid Downsampling
    std::cout << "\n[Step 2] Voxel Grid Downsampling (Leaf=0.01)..." << std::endl;
    std::vector<PointXYZ> filtered_points(n); // Alloc max
    // Leaf 0.01 is fine for bunny (size ~0.15)
    size_t n_filtered = voxel_grid_downsamp_rvv(cloud_soa, filtered_points.data(), 0.01f);
    std::cout << "Filtered count: " << n_filtered << " (Original: " << n << ")" << std::endl;

    // Save Voxelized Cloud
    std::vector<PointXYZ> final_points;
    for(size_t i=0; i<n_filtered; ++i) final_points.push_back(filtered_points[i]);
    savePCD(output_file, final_points);


    // 3. Build Octree (Explicit Step)
    std::cout << "\n[Step 3] Building Octree..." << std::endl;
    // We need SoA for the filtered cloud
    std::vector<float> fx(n_filtered), fy(n_filtered), fz(n_filtered);
    for(size_t i=0; i<n_filtered; ++i) {
        fx[i] = filtered_points[i].x;
        fy[i] = filtered_points[i].y;
        fz[i] = filtered_points[i].z;
    }
    PointCloudSoA filtered_soa = {fx.data(), fy.data(), fz.data(), n_filtered};
    
    Octree octree;
    octree.setInputCloud(filtered_soa);
    octree.build();
    std::cout << "Octree built successfully." << std::endl;

    // 4. Normal Estimation using Octree
    std::cout << "\n[Step 4] Estimating Normals (K=10) with ViewPoint(0,0,0)..." << std::endl;
    std::vector<float> nx(n_filtered), ny(n_filtered), nz(n_filtered);
    
    // Viewpoint at origin (0,0,0) - simulating scanner position
    normal_estimation_rvv(filtered_soa, octree, nx.data(), ny.data(), nz.data(), 10, 0.03f, 0.0f, 0.0f, 0.0f);
    
    // Check index 0
    float vp_dx = 0 - fx[0];
    float vp_dy = 0 - fy[0];
    float vp_dz = 0 - fz[0];
    float dot = nx[0]*vp_dx + ny[0]*vp_dy + nz[0]*vp_dz;
    std::cout << "Point[0] Normal Dot with ViewVec: " << dot << std::endl;
    
    if (dot >= -1e-5) { 
        std::cout << "[PASS] Normal orientation correct (aligned with line of sight)." << std::endl;
    } else {
        std::cerr << "[FAIL] Normal points away from viewpoint!" << std::endl;
    }
    
    // 5. Verify Radius Search (Sanity Check)
    std::cout << "\n[Step 5] Octree Radius Search Verification..." << std::endl;
    size_t mid_idx = n_filtered / 2;
    PointXYZ query = filtered_points[mid_idx];
    float radius = 0.05f; 
    
    std::vector<int> indices;
    std::vector<float> dists;
    std::size_t found = octree.radiusSearch(query, radius, indices, dists);
    
    std::cout << "Neighbors found within r=" << radius << ": " << found << std::endl;
    
    // Sanity check: Should find at least itself (dist=0)
    bool found_self = false;
    for(float d : dists) {
        if(d < 1e-9) found_self = true;
    }

    
    if (found > 0 && found_self) {
        std::cout << "[PASS] Search returned valid results." << std::endl;
    } else {
        std::cerr << "[FAIL] Search failed or did not find self." << std::endl;
        return 1;
    }

    // 5. Visualization (Automatic)
    // Assumes script is in ../scripts/ relative to build dir, or /workspace/scripts/
    std::string png_file = output_file.substr(0, output_file.find_last_of('.')) + ".png";
    // Try relative path first
    std::string script_path = "../scripts/visualize_result.py";
    // If running from root, it might be just scripts/
    std::ifstream check_s("scripts/visualize_result.py");
    if (check_s.good()) script_path = "scripts/visualize_result.py";
    // Fallback to absolute if needed? simpler to just try one.
    
    std::string cmd = "python3 " + script_path + " " + output_file + " " + png_file;
    std::cout << "\n[Step 5] Generating Visualization..." << std::endl;
    std::cout << "Executing: " << cmd << std::endl;
    
    int ret = std::system(cmd.c_str());
    if (ret == 0) {
        std::cout << "[SUCCESS] Visualization saved to " << png_file << std::endl;
    } else {
        std::cerr << "[WARN] Visualization script failed (Python/Matplotlib missing or path error?)." << std::endl;
        std::cerr << "       Try running manually: " << cmd << std::endl;
    }

    std::cout << "\n[SUCCESS] Custom Pipeline Walkthrough Complete!" << std::endl;
    return 0;
}
