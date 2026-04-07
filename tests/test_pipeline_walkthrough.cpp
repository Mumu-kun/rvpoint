#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;

// Mimics
// https://pcl.readthedocs.io/projects/tutorials/en/master/walkthrough.html
int main(int argc, char **argv) {
  std::cout << "========================================" << std::endl;
  std::cout << "   RISC-V PCL Pipeline Walkthrough      " << std::endl;
  std::cout << "========================================" << std::endl;

  std::string input_file = "bunny.pcd";//as we will be using this frequently for testing
  if (argc > 1)
    input_file = argv[1];

  std::string base_name = input_file;
  size_t last_slash = base_name.find_last_of("/\\");
  if (last_slash != std::string::npos)
    base_name = base_name.substr(last_slash + 1);
  std::string stem = base_name.substr(0, base_name.find_last_of('.'));

  // Generate timestamp for serialization
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm *tm_now = std::localtime(&time_t_now);
  char timestamp[32];
  std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", tm_now);

  // Output to results/ directory
  // If running from bin/, we need ../results/
  std::string output_dir = "results/";
  std::ifstream check_res("results");
  if (!check_res.good())
    output_dir = "../results/";

  std::string output_file =
      output_dir + stem + "_" + timestamp + "_voxelized.pcd";

  // 1. Load Cloud
  std::cout << "\n[Step 1] Loading " << input_file << "..." << std::endl;
  std::vector<PointXYZ> loaded_points;
  // Try current directory first, then data/, then absolute
  int n = loadPCD(input_file, loaded_points);
  if (n < 0)
    n = loadPCD("../" + input_file,
                loaded_points); // Check parent (e.g. if in bin/)
  if (n < 0)
    n = loadPCD("data/" + input_file, loaded_points);
  if (n < 0)
    n = loadPCD("../data/" + input_file, loaded_points);
  if (n < 0)
    n = loadPCD("/workspace/data/" + input_file, loaded_points);

  if (n < 0) {
    std::cerr << "[FAIL] Could not load " << input_file << std::endl;
    return 1;
  }
  std::cout << "Loaded " << n << " points." << std::endl;

  // Convert to SoA for processing
  std::vector<float> x(n), y(n), z(n);
  for (int i = 0; i < n; ++i) {
    x[i] = loaded_points[i].x;
    y[i] = loaded_points[i].y;
    z[i] = loaded_points[i].z;
  }

  // 1.5 Add Synthetic Ground Plane
  std::cout << "\n[Step 1.5] Adding Synthetic Ground Plane..." << std::endl;

  // Find min Z of bunny to place ground plane below it
  float min_z = *std::min_element(z.begin(), z.end());
  float min_x = *std::min_element(x.begin(), x.end());
  float max_x = *std::max_element(x.begin(), x.end());
  float min_y = *std::min_element(y.begin(), y.end());
  float max_y = *std::max_element(y.begin(), y.end());

  float ground_z = min_z - 0.005f;  // Slightly below bunny
  const size_t N_GROUND = 1000;

  // Generate ground plane points within bunny's XY extent
  std::mt19937 gen(42);
  std::uniform_real_distribution<float> dist_x(min_x - 0.02f, max_x + 0.02f);
  std::uniform_real_distribution<float> dist_y(min_y - 0.02f, max_y + 0.02f);

  size_t original_n = n;
  x.reserve(n + N_GROUND);
  y.reserve(n + N_GROUND);
  z.reserve(n + N_GROUND);

  for (size_t i = 0; i < N_GROUND; ++i) {
    x.push_back(dist_x(gen));
    y.push_back(dist_y(gen));
    z.push_back(ground_z);
  }
  n = x.size();

  std::cout << "Added " << N_GROUND << " ground plane points at Z=" << ground_z << std::endl;
  std::cout << "Total points: " << n << " (Bunny: " << original_n << " + Ground: " << N_GROUND << ")" << std::endl;

  PointCloudSoA cloud_soa = {x.data(), y.data(), z.data(), (size_t)n};

  // 2. Voxel Grid Downsampling
  std::cout << "\n[Step 2] Voxel Grid Downsampling (Leaf=0.01)..." << std::endl;
  std::vector<PointXYZ> filtered_points(n); // Alloc max
  // Leaf 0.01 is fine for bunny (size ~0.15)
  size_t n_filtered =
      voxel_grid_downsamp_rvv_v2(cloud_soa, filtered_points.data(), 0.01f);
  std::cout << "Filtered count: " << n_filtered << " (Original: " << n << ")"
            << std::endl;

  // 2.5 RANSAC Plane Segmentation
  std::cout << "\n[Step 2.5] RANSAC Plane Segmentation..." << std::endl;

  // Convert filtered to SoA for RANSAC
  std::vector<float> vx(n_filtered), vy(n_filtered), vz(n_filtered);
  for (size_t i = 0; i < n_filtered; ++i) {
    vx[i] = filtered_points[i].x;
    vy[i] = filtered_points[i].y;
    vz[i] = filtered_points[i].z;
  }
  PointCloudSoA voxel_soa = {vx.data(), vy.data(), vz.data(), n_filtered};

  float plane_model[4];
  float ransac_thresh = 0.005f;  // Distance threshold for inliers
  int ransac_iters = 1000;

  int n_plane_inliers = ransac_plane_rvv(voxel_soa, ransac_thresh, ransac_iters, plane_model);

  std::cout << "RANSAC found " << n_plane_inliers << " plane inliers" << std::endl;
  std::cout << "Plane model: " << plane_model[0] << "x + " << plane_model[1] << "y + "
            << plane_model[2] << "z + " << plane_model[3] << " = 0" << std::endl;

  // Extract inliers (ground plane) and outliers (bunny)
  std::vector<PointXYZ> plane_pts(n_filtered);
  std::vector<PointXYZ> object_pts(n_filtered);
  std::size_t actual_inliers, actual_outliers;

  extract_plane_inliers_outliers_rvv(voxel_soa, plane_model, ransac_thresh,
                                      plane_pts.data(), object_pts.data(),
                                      actual_inliers, actual_outliers);

  std::cout << "Extracted: " << actual_inliers << " plane points (ground), "
            << actual_outliers << " object points (bunny)" << std::endl;

  // Save ground plane for visualization
  std::string ground_file = output_dir + stem + "_" + timestamp + "_ground.pcd";
  std::vector<PointXYZ> ground_vec(plane_pts.begin(), plane_pts.begin() + actual_inliers);
  savePCD(ground_file, ground_vec);
  std::cout << "Saved ground plane to: " << ground_file << std::endl;

  // Continue processing with object points (bunny without ground)
  std::vector<PointXYZ> final_points;
  for (size_t i = 0; i < actual_outliers; ++i)
    final_points.push_back(object_pts[i]);

  // Save Voxelized Cloud (bunny without ground)
  savePCD(output_file, final_points);

  // Update n_filtered to reflect the object (non-plane) points for subsequent steps
  n_filtered = actual_outliers;
  filtered_points.resize(n_filtered);
  for (size_t i = 0; i < n_filtered; ++i) {
    filtered_points[i] = object_pts[i];
  }

  std::cout << "Continuing pipeline with " << n_filtered << " object points (ground removed)." << std::endl;

  // 3. Statistical Outlier Removal (SOR)
  std::cout << "\n[Step 3] Statistical Outlier Removal (K=50, Std=1.0)..."
            << std::endl;
  // We need SoA for the filtered cloud
  std::vector<float> fx(n_filtered), fy(n_filtered), fz(n_filtered);
  for (size_t i = 0; i < n_filtered; ++i) {
    fx[i] = filtered_points[i].x;
    fy[i] = filtered_points[i].y;
    fz[i] = filtered_points[i].z;
  }
  PointCloudSoA filtered_soa = {fx.data(), fy.data(), fz.data(), n_filtered};

  std::vector<PointXYZ> sor_points(n_filtered); // Alloc max

  // Using K=50, std_mul=1.0 as standard defaults
  // Note: for bunny.pcd which is very clean, this might not remove much,
  // but ensures the pipeline works.
  size_t n_sor = sor_rvv(filtered_soa, sor_points.data(), 50, 1.0f);
  std::cout << "SOR Filtered count: " << n_sor << " (Original: " << n_filtered
            << ")" << std::endl;

  // 4. Build Octree (Explicit Step)
  std::cout << "\n[Step 4] Building Octree..." << std::endl;
  // We need SoA for the SOR-filtered cloud
  std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
  for (size_t i = 0; i < n_sor; ++i) {
    sx[i] = sor_points[i].x;
    sy[i] = sor_points[i].y;
    sz[i] = sor_points[i].z;
  }
  PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

  Octree octree;
  octree.setInputCloud(sor_soa);
  octree.build();
  std::cout << "Octree built successfully." << std::endl;

  // 5. Normal Estimation using Octree
  std::cout << "\n[Step 5] Estimating Normals (K=10) with ViewPoint(0,0,0)..."
            << std::endl;
  std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);

  // Viewpoint at origin (0,0,0) - simulating scanner position
  normal_estimation_rvv(sor_soa, octree, nx.data(), ny.data(), nz.data(), 10,
                        0.03f, 0.0f, 0.0f, 0.0f);

  // Check index 0
  float vp_dx = 0 - sx[0];
  float vp_dy = 0 - sy[0];
  float vp_dz = 0 - sz[0];
  float dot = nx[0] * vp_dx + ny[0] * vp_dy + nz[0] * vp_dz;
  std::cout << "Point[0] Normal Dot with ViewVec: " << dot << std::endl;

  if (dot >= -1e-5) {
    std::cout
        << "[PASS] Normal orientation correct (aligned with line of sight)."
        << std::endl;
  } else {
    std::cerr << "[FAIL] Normal points away from viewpoint!" << std::endl;
  }

  // 6. Verify Radius Search (Sanity Check)
  std::cout << "\n[Step 6] Octree Radius Search Verification..." << std::endl;
  size_t mid_idx = n_sor / 2;
  PointXYZ query = sor_points[mid_idx];
  float radius = 0.05f;

  std::vector<int> indices;
  std::vector<float> dists;
  std::size_t found = octree.radiusSearch(query, radius, indices, dists);

  std::cout << "Neighbors found within r=" << radius << ": " << found
            << std::endl;

  // Sanity check: Should find at least itself (dist=0)
  bool found_self = false;
  for (float d : dists) {
    if (d < 1e-9)
      found_self = true;
  }

  if (found > 0 && found_self) {
    std::cout << "[PASS] Search returned valid results." << std::endl;
  } else {
    std::cerr << "[FAIL] Search failed or did not find self." << std::endl;
    return 1;
  }

  // 7. Visualization (Automatic)
  // Assumes script is in ../scripts/ relative to build dir, or
  // /workspace/scripts/
  std::string png_file =
      output_file.substr(0, output_file.find_last_of('.')) + ".png";
  // Try relative path first
  std::string script_path = "../scripts/visualize_result.py";
  // If running from root, it might be just scripts/
  std::ifstream check_s("scripts/visualize_result.py");
  if (check_s.good())
    script_path = "scripts/visualize_result.py";
  // Fallback to absolute if needed? simpler to just try one.

  std::string cmd =
      "python3 " + script_path + " " + output_file + " " + png_file;
  std::cout << "\n[Step 7] Generating Visualization..." << std::endl;
  std::cout << "Executing: " << cmd << std::endl;

  int ret = std::system(cmd.c_str());
  if (ret == 0) {
    std::cout << "[SUCCESS] Visualization saved to " << png_file << std::endl;
  } else {
    std::cerr << "[WARN] Visualization script failed (Python/Matplotlib "
                 "missing or path error?)."
              << std::endl;
    std::cerr << "       Try running manually: " << cmd << std::endl;
  }

  std::cout << "\n[SUCCESS] Custom Pipeline Walkthrough Complete!" << std::endl;
  return 0;
}
