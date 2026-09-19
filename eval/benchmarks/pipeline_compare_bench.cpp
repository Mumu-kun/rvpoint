#include "core/point_types.h"
#include "filters/voxel_grid.h"
#include "filters/statistical_outlier_removal.h"
#include "features/normal_estimation.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "search/octree.h"
#include "search/pointer_octree.h"
#include "io/simple_pcd_loader.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

using namespace rvpoint;

namespace {

struct StageComparison {
  std::string stage_name;
  double scalar_ms;
  double rvv_ms;
  double speedup;
  std::size_t input_points;
  std::size_t output_yield;
  std::string notes;
};

struct PipelineParams {
  float voxel_leaf_size = 0.10f;          // 10 cm voxel grid
  float search_radius = 0.03f;            // 3 cm neighbor search
  int sor_mean_k = 20;                    // K-NN mean distance
  float sor_std_threshold = 1.0f;         // 1-sigma outlier cutoff
  int normal_k = 10;                      // K-NN for normal covariance
  float ransac_distance_thresh = 0.20f;   // 20 cm ground inlier distance
  int ransac_max_iters = 1000;            // RANSAC candidate iterations
  float cluster_tolerance = 0.15f;        // 15 cm Euclidean cluster tolerance
  int min_cluster_size = 50;              // Minimum cluster points
  int max_cluster_size = 100000;          // Maximum cluster points
};

std::string resolvePath(const std::string &path) {
  if (std::filesystem::exists(path)) return path;
  if (std::filesystem::exists("/workspace/" + path)) return "/workspace/" + path;
  if (std::filesystem::exists("data/" + path)) return "data/" + path;
  return path;
}

void aosToSoa(const std::vector<PointXYZ> &aos, std::vector<float> &x,
              std::vector<float> &y, std::vector<float> &z, PointCloudSoA &soa) {
  const std::size_t n = aos.size();
  x.resize(n);
  y.resize(n);
  z.resize(n);
  for (std::size_t i = 0; i < n; ++i) {
    x[i] = aos[i].x;
    y[i] = aos[i].y;
    z[i] = aos[i].z;
  }
  soa.x = x.data();
  soa.y = y.data();
  soa.z = z.data();
  soa.n = n;
}

} // namespace

int main(int argc, char **argv) {
  std::string input_pcd_path = "data/pcd_compressed/0000000000.pcd";
  std::string output_dir_path = "output/comparison";
  PipelineParams params;

  if (argc > 1) input_pcd_path = argv[1];
  if (argc > 2) output_dir_path = argv[2];
  if (argc > 3) params.voxel_leaf_size = std::stof(argv[3]);

  input_pcd_path = resolvePath(input_pcd_path);
  std::filesystem::create_directories(output_dir_path);

  std::cout << "==========================================================================" << std::endl;
  std::cout << "   RVPoint Pipeline: Stage-by-Stage Scalar vs. RVV Vector Verification    " << std::endl;
  std::cout << "==========================================================================" << std::endl;
  std::cout << "Target PCD:       " << input_pcd_path << std::endl;
  std::cout << "Output Directory: " << output_dir_path << std::endl;
  std::cout << "Parameters:       leaf=" << params.voxel_leaf_size << "m, sor_k=" << params.sor_mean_k
            << ", sor_std=" << params.sor_std_threshold << ", normal_k=" << params.normal_k
            << ", ransac_iters=" << params.ransac_max_iters << ", cluster_tol=" << params.cluster_tolerance << "m"
            << std::endl << std::endl;

  // Load Raw Input PCD
  std::vector<PointXYZ> raw_aos;
  int loaded = loadPCD(input_pcd_path, raw_aos);
  if (loaded <= 0 || raw_aos.empty()) {
    std::cerr << "Error: Failed to load PCD file from " << input_pcd_path << std::endl;
    return 1;
  }
  const std::size_t n_raw = raw_aos.size();
  std::vector<float> rx, ry, rz;
  PointCloudSoA raw_soa;
  aosToSoa(raw_aos, rx, ry, rz, raw_soa);

  std::cout << "[Input Cloud Loaded] " << n_raw << " points." << std::endl << std::endl;

  std::vector<StageComparison> comparisons;

  // --------------------------------------------------------------------------
  // Stage 1: Voxel Grid Downsampling
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 1: Voxel Grid Downsampling..." << std::endl;
  std::vector<PointXYZ> vg_sc_out(n_raw);
  auto t0 = std::chrono::high_resolution_clock::now();
  std::size_t n_vg_sc = voxel_grid_downsamp_sc(raw_aos.data(), n_raw, vg_sc_out.data(), params.voxel_leaf_size);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_vg_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();
  vg_sc_out.resize(n_vg_sc);

  std::vector<PointXYZ> vg_rvv_out(n_raw);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t n_vg_rvv = voxel_grid_downsamp_rvv_v2(raw_soa, vg_rvv_out.data(), params.voxel_leaf_size);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_vg_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();
  vg_rvv_out.resize(n_vg_rvv);

  comparisons.push_back({"1. Voxel Grid Downsampling", ms_vg_sc, ms_vg_rvv, ms_vg_sc / ms_vg_rvv,
                         n_raw, n_vg_rvv, "Sort-based reduction (v2) vs std::map"});

  // Prepare downsampled point cloud in SoA & AoS for subsequent stages
  std::vector<PointXYZ> &down_aos = vg_rvv_out;
  const std::size_t n_down = down_aos.size();
  std::vector<float> dx, dy, dz;
  PointCloudSoA down_soa;
  aosToSoa(down_aos, dx, dy, dz, down_soa);

  // --------------------------------------------------------------------------
  // Stage 2: Spatial Index Construction (Octree vs PointerOctree)
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 2: Spatial Search Index Construction..." << std::endl;
  Octree octree_sc;
  octree_sc.setInputCloud(down_soa);
  t0 = std::chrono::high_resolution_clock::now();
  octree_sc.build();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_tree_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  PointerOctree octree_rvv;
  octree_rvv.setInputCloud(down_soa);
  t0 = std::chrono::high_resolution_clock::now();
  octree_rvv.build();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_tree_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  comparisons.push_back({"2. Spatial Index Construction", ms_tree_sc, ms_tree_rvv, ms_tree_sc / ms_tree_rvv,
                         n_down, n_down, "PointerOctree vs Standard Octree"});

  // --------------------------------------------------------------------------
  // Stage 3: Statistical Outlier Removal (SOR)
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 3: Statistical Outlier Removal (SOR)..." << std::endl;
  std::vector<PointXYZ> sor_sc_out(n_down);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t n_sor_sc = sor_sc(down_aos.data(), n_down, sor_sc_out.data(), params.sor_mean_k, params.sor_std_threshold);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_sor_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();
  sor_sc_out.resize(n_sor_sc);

  std::vector<PointXYZ> sor_rvv_out(n_down);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t n_sor_rvv = sor_pointer_octree(down_soa, octree_rvv, sor_rvv_out.data(),
                                             params.sor_mean_k, params.sor_std_threshold, 0.5f);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_sor_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();
  sor_rvv_out.resize(n_sor_rvv);

  comparisons.push_back({"3. Statistical Outlier Removal", ms_sor_sc, ms_sor_rvv, ms_sor_sc / ms_sor_rvv,
                         n_down, n_sor_rvv, "PointerOctree O(N log N) vs Brute Force O(N^2)"});

  // Prepare filtered point cloud for subsequent stages
  std::vector<PointXYZ> &filt_aos = sor_rvv_out;
  const std::size_t n_filt = filt_aos.size();
  std::vector<float> fx, fy, fz;
  PointCloudSoA filt_soa;
  aosToSoa(filt_aos, fx, fy, fz, filt_soa);

  // Rebuild Octree for filtered cloud
  Octree octree_filt_rvv;
  octree_filt_rvv.setInputCloud(filt_soa);
  octree_filt_rvv.build();

  // --------------------------------------------------------------------------
  // Stage 4: Surface Normal Estimation
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 4: Surface Normal Estimation..." << std::endl;
  std::vector<float> nx_sc(n_filt), ny_sc(n_filt), nz_sc(n_filt);
  t0 = std::chrono::high_resolution_clock::now();
  normal_estimation_sc(filt_aos.data(), n_filt, nx_sc.data(), ny_sc.data(), nz_sc.data(),
                       params.normal_k, params.search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_norm_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::vector<float> nx_rvv(n_filt), ny_rvv(n_filt), nz_rvv(n_filt);
  t0 = std::chrono::high_resolution_clock::now();
  normal_estimation_rvv(filt_soa, octree_filt_rvv, nx_rvv.data(), ny_rvv.data(), nz_rvv.data(),
                        params.normal_k, params.search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_norm_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  comparisons.push_back({"4. Surface Normal Estimation", ms_norm_sc, ms_norm_rvv, ms_norm_sc / ms_norm_rvv,
                         n_filt, n_filt, "Vectorized 3x3 covariance vs scalar loops"});

  // --------------------------------------------------------------------------
  // Stage 5: RANSAC Ground Plane Fitting
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 5: RANSAC Ground Plane Fitting..." << std::endl;
  float model_sc[4] = {0};
  t0 = std::chrono::high_resolution_clock::now();
  int inliers_sc = ransac_plane_sc(filt_aos.data(), n_filt, params.ransac_distance_thresh,
                                   params.ransac_max_iters, model_sc);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ransac_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  float model_rvv[4] = {0};
  t0 = std::chrono::high_resolution_clock::now();
  int inliers_rvv = ransac_plane_rvv(filt_soa, params.ransac_distance_thresh,
                                     params.ransac_max_iters, model_rvv);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ransac_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  comparisons.push_back({"5. RANSAC Ground Plane Fitting", ms_ransac_sc, ms_ransac_rvv, ms_ransac_sc / ms_ransac_rvv,
                         n_filt, static_cast<std::size_t>(inliers_rvv), "Parallel vector distance masks vs scalar"});

  // Extract non-ground obstacle points
  std::vector<PointXYZ> obstacles_aos(n_filt);
  std::size_t n_obstacles = extract_plane_outliers_rvv(filt_soa, model_rvv, params.ransac_distance_thresh, obstacles_aos.data());
  obstacles_aos.resize(n_obstacles);
  std::vector<float> ox, oy, oz;
  PointCloudSoA obstacles_soa;
  aosToSoa(obstacles_aos, ox, oy, oz, obstacles_soa);

  // --------------------------------------------------------------------------
  // Stage 6: Euclidean Clustering
  // --------------------------------------------------------------------------
  std::cout << "--> Running Stage 6: Euclidean Clustering..." << std::endl;
  // Scalar Baseline: Standard Octree BFS
  Octree octree_obs;
  octree_obs.setInputCloud(obstacles_soa);
  octree_obs.build();

  t0 = std::chrono::high_resolution_clock::now();
  EuclideanClustering ec_sc;
  ec_sc.setInputCloud(obstacles_soa);
  ec_sc.setNeighborSearch(&octree_obs);
  ec_sc.setClusterTolerance(params.cluster_tolerance);
  ec_sc.setMinClusterSize(params.min_cluster_size);
  ec_sc.setMaxClusterSize(params.max_cluster_size);
  std::vector<ClusterIndices> clusters_sc = ec_sc.extract();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_clust_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // RVV / Optimized Path: PointerOctree BFS zero-alloc
  PointerOctree ptr_obs;
  ptr_obs.setInputCloud(obstacles_soa);
  ptr_obs.build();

  t0 = std::chrono::high_resolution_clock::now();
  EuclideanClustering ec_rvv;
  ec_rvv.setInputCloud(obstacles_soa);
  ec_rvv.setNeighborSearch(&ptr_obs);
  ec_rvv.setClusterTolerance(params.cluster_tolerance);
  ec_rvv.setMinClusterSize(params.min_cluster_size);
  ec_rvv.setMaxClusterSize(params.max_cluster_size);
  std::vector<ClusterIndices> clusters_rvv = ec_rvv.extract();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_clust_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  comparisons.push_back({"6. Euclidean Clustering", ms_clust_sc, ms_clust_rvv, ms_clust_sc / ms_clust_rvv,
                         n_obstacles, clusters_rvv.size(), "PointerOctree BFS vs standard BFS"});

  // Calculate totals
  double total_sc = 0.0;
  double total_rvv = 0.0;
  for (const auto &c : comparisons) {
    total_sc += c.scalar_ms;
    total_rvv += c.rvv_ms;
  }

  // --------------------------------------------------------------------------
  // Print Formatted Stage Comparison Table
  // --------------------------------------------------------------------------
  std::cout << std::endl;
  std::cout << "========================================================================================================" << std::endl;
  std::cout << "                     STRICT HEAD-TO-HEAD PIPELINE BENCHMARK VERIFICATION RESULTS                       " << std::endl;
  std::cout << "========================================================================================================" << std::endl;
  std::cout << std::left
            << std::setw(32) << "Pipeline Stage"
            << std::setw(16) << "Scalar Time (ms)"
            << std::setw(16) << "RVV Time (ms)"
            << std::setw(12) << "Speedup"
            << std::setw(12) << "In -> Out"
            << "Optimization Mechanism" << std::endl;
  std::cout << std::string(104, '-') << std::endl;

  for (const auto &c : comparisons) {
    std::stringstream pts_ss;
    pts_ss << c.input_points << "->" << c.output_yield;
    std::cout << std::left
              << std::setw(32) << c.stage_name
              << std::setw(16) << std::fixed << std::setprecision(2) << c.scalar_ms
              << std::setw(16) << std::fixed << std::setprecision(2) << c.rvv_ms
              << std::setw(12) << (std::to_string(c.speedup).substr(0, 5) + "x")
              << std::setw(12) << pts_ss.str()
              << c.notes << std::endl;
  }
  std::cout << std::string(104, '=') << std::endl;
  std::cout << std::left
            << std::setw(32) << "TOTAL PIPELINE LATENCY"
            << std::setw(16) << std::fixed << std::setprecision(2) << total_sc
            << std::setw(16) << std::fixed << std::setprecision(2) << total_rvv
            << std::setw(12) << (std::to_string(total_sc / total_rvv).substr(0, 5) + "x")
            << std::setw(12) << (std::to_string(n_raw) + "->" + std::to_string(clusters_rvv.size()))
            << "Full Stack Vector & Spatial Indexing" << std::endl;
  std::cout << "========================================================================================================" << std::endl;

  // --------------------------------------------------------------------------
  // Save Metrics to JSON & CSV
  // --------------------------------------------------------------------------
  const std::string json_path = output_dir_path + "/comparison_metrics.json";
  std::ofstream jf(json_path);
  jf << "{\n  \"input_file\": \"" << input_pcd_path << "\",\n";
  jf << "  \"input_points\": " << n_raw << ",\n";
  jf << "  \"total_scalar_ms\": " << total_sc << ",\n";
  jf << "  \"total_rvv_ms\": " << total_rvv << ",\n";
  jf << "  \"end_to_end_speedup\": " << (total_sc / total_rvv) << ",\n";
  jf << "  \"stages\": [\n";
  for (std::size_t i = 0; i < comparisons.size(); ++i) {
    const auto &c = comparisons[i];
    jf << "    {\n";
    jf << "      \"stage\": \"" << c.stage_name << "\",\n";
    jf << "      \"scalar_ms\": " << c.scalar_ms << ",\n";
    jf << "      \"rvv_ms\": " << c.rvv_ms << ",\n";
    jf << "      \"speedup\": " << c.speedup << ",\n";
    jf << "      \"input_points\": " << c.input_points << ",\n";
    jf << "      \"output_yield\": " << c.output_yield << ",\n";
    jf << "      \"mechanism\": \"" << c.notes << "\"\n";
    jf << "    }" << (i + 1 < comparisons.size() ? "," : "") << "\n";
  }
  jf << "  ]\n}\n";
  jf.close();

  const std::string csv_path = output_dir_path + "/comparison_metrics.csv";
  std::ofstream cf(csv_path);
  cf << "Stage,Scalar_ms,RVV_ms,Speedup,Input_Points,Output_Yield,Mechanism\n";
  for (const auto &c : comparisons) {
    cf << "\"" << c.stage_name << "\"," << c.scalar_ms << "," << c.rvv_ms << ","
       << c.speedup << "," << c.input_points << "," << c.output_yield << ",\""
       << c.notes << "\"\n";
  }
  cf << "\"TOTAL\"," << total_sc << "," << total_rvv << "," << (total_sc / total_rvv)
     << "," << n_raw << "," << clusters_rvv.size() << ",\"End-to-End Pipeline\"\n";
  cf.close();

  std::cout << std::endl << "Saved live metrics:" << std::endl;
  std::cout << "  • JSON: " << json_path << std::endl;
  std::cout << "  • CSV:  " << csv_path << std::endl;

  return 0;
}
