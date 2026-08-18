#include "core/point_types.h"
#include "core/profiler.h"
#include "filters/voxel_grid.h"
#include "filters/statistical_outlier_removal.h"
#include "features/normal_estimation.h"
#include "segmentation/ransac_plane.h"
#include "segmentation/euclidean_clustering.h"
#include "search/octree.h"
#include "search/spatial_hashing.h"
#include "search/pointer_octree.h"
#include "search/caravan_radius_search.h"
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

struct BenchmarkResult {
  std::string category;
  std::string implementation;
  double time_ms;
  std::size_t input_count;
  std::size_t output_count;
  double speedup_vs_baseline;
};

void runVoxelGridAblation(const PointCloudSoA &soa_in,
                         const std::vector<PointXYZ> &aos_in, float leaf_size,
                         std::vector<BenchmarkResult> &results) {
  std::cout << "\n=== [Ablation A: Voxel Grid Downsampling] ===" << std::endl;

  std::vector<PointXYZ> out_v2(soa_in.n);
  auto t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_v2 = voxel_grid_downsamp_rvv_v2(soa_in, out_v2.data(), leaf_size);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_v2 = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::vector<PointXYZ> out_sc(aos_in.size());
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_sc = voxel_grid_downsamp_sc(aos_in.data(), aos_in.size(), out_sc.data(), leaf_size);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::cout << "  Scalar std::map (v0):  " << std::fixed << std::setprecision(3)
            << ms_sc << " ms (" << count_sc << " pts)" << std::endl;
  std::cout << "  RVV Sort-based (v2):   " << std::fixed << std::setprecision(3)
            << ms_v2 << " ms (" << count_v2 << " pts) -> "
            << std::setprecision(2) << (ms_sc / ms_v2) << "x speedup" << std::endl;

  results.push_back({"VoxelGridDownsampling", "Scalar_map", ms_sc, soa_in.n, count_sc, 1.0});
  results.push_back({"VoxelGridDownsampling", "RVV_sort_v2", ms_v2, soa_in.n, count_v2, ms_sc / ms_v2});
}

void runNeighborSearchAblation(const PointCloudSoA &soa,
                              const std::vector<PointXYZ> &aos,
                              float radius,
                              std::vector<BenchmarkResult> &results) {
  std::cout << "\n=== [Ablation B: Spatial Neighbor Search] ===" << std::endl;
  if (soa.n == 0) return;

  const PointXYZ query = aos[soa.n / 2]; // Sample point as query
  constexpr int max_nn = 100;
  std::vector<int> indices(max_nn);
  std::vector<float> dists(max_nn);

  // 1. Global RVV Scan
  auto t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_rvv = radius_search_rvv(soa, query, radius, indices.data(), dists.data(), max_nn);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 2. Standard Octree Search
  Octree octree;
  octree.setInputCloud(soa);
  t0 = std::chrono::high_resolution_clock::now();
  octree.build();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_tree_build = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::vector<int> tree_indices;
  std::vector<float> tree_dists;
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_tree = octree.radiusSearch(query, radius, tree_indices, tree_dists, max_nn);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_tree_search = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 3. Pointer Octree Search (RVV & Scalar)
  PointerOctree ptr_octree;
  ptr_octree.setInputCloud(soa);
  t0 = std::chrono::high_resolution_clock::now();
  ptr_octree.build();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ptr_build = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::vector<int> ptr_indices_rvv, ptr_indices_sc;
  std::vector<float> ptr_dists_rvv, ptr_dists_sc;
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_ptr_rvv = ptr_octree.radiusSearch(query, radius, ptr_indices_rvv, ptr_dists_rvv);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ptr_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_ptr_sc = ptr_octree.radiusSearchScalar(query, radius, ptr_indices_sc, ptr_dists_sc);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ptr_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 4. Spatial Hash Search
  SpatialHash hash;
  hash.setInputCloud(soa, radius);
  t0 = std::chrono::high_resolution_clock::now();
  hash.build();
  t1 = std::chrono::high_resolution_clock::now();
  double ms_hash_build = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::vector<int> hash_indices;
  std::vector<float> hash_dists;
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_hash = hash.radiusSearch(query, radius, hash_indices, hash_dists, max_nn);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_hash_search = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::cout << "  Global RVV Scan:           " << std::fixed << std::setprecision(3)
            << ms_rvv << " ms (" << count_rvv << " nbrs)" << std::endl;
  std::cout << "  Standard Octree Query:     " << std::fixed << std::setprecision(3)
            << ms_tree_search << " ms (build: " << ms_tree_build << " ms)" << std::endl;
  std::cout << "  Pointer Octree (RVV):      " << std::fixed << std::setprecision(3)
            << ms_ptr_rvv << " ms (build: " << ms_ptr_build << " ms)" << std::endl;
  std::cout << "  Pointer Octree (Scalar):   " << std::fixed << std::setprecision(3)
            << ms_ptr_sc << " ms" << std::endl;
  std::cout << "  Spatial Hash Grid Query:   " << std::fixed << std::setprecision(3)
            << ms_hash_search << " ms (build: " << ms_hash_build << " ms)" << std::endl;

  results.push_back({"NeighborSearch", "Global_RVV_Scan", ms_rvv, soa.n, count_rvv, 1.0});
  results.push_back({"NeighborSearch", "Standard_Octree", ms_tree_search, soa.n, count_tree, ms_rvv / (ms_tree_search + 1e-6)});
  results.push_back({"NeighborSearch", "Pointer_Octree_RVV", ms_ptr_rvv, soa.n, count_ptr_rvv, ms_rvv / (ms_ptr_rvv + 1e-6)});
  results.push_back({"NeighborSearch", "Pointer_Octree_Scalar", ms_ptr_sc, soa.n, count_ptr_sc, ms_rvv / (ms_ptr_sc + 1e-6)});
  results.push_back({"NeighborSearch", "SpatialHash_Grid", ms_hash_search, soa.n, count_hash, ms_rvv / (ms_hash_search + 1e-6)});
}

void runSORAblation(const PointCloudSoA &soa,
                    const std::vector<PointXYZ> &aos,
                    int k, float alpha,
                    std::vector<BenchmarkResult> &results) {
  std::cout << "\n=== [Ablation C: Statistical Outlier Removal (SOR) Variants] ===" << std::endl;

  std::size_t test_n = std::min(soa.n, static_cast<std::size_t>(5000));
  PointCloudSoA sub_soa = {soa.x, soa.y, soa.z, test_n};
  float search_radius = 0.5f;

  // 1. RVV Brute-Force O(N^2)
  std::vector<PointXYZ> out_rvv(test_n);
  auto t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_rvv = sor_rvv(sub_soa, out_rvv.data(), k, alpha);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 2. Scalar Brute-Force O(N^2)
  std::vector<PointXYZ> out_sc(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_sc = sor_sc(aos.data(), test_n, out_sc.data(), k, alpha);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 3. Octree-Accelerated SOR O(N log N)
  Octree octree;
  octree.setInputCloud(sub_soa);
  octree.build();
  std::vector<PointXYZ> out_tree(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_tree = sor_octree(sub_soa, octree, out_tree.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_tree = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 4. PointerOctree-Accelerated SOR O(N log N)
  PointerOctree ptr_octree;
  ptr_octree.setInputCloud(sub_soa);
  ptr_octree.build();
  std::vector<PointXYZ> out_ptr(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_ptr = sor_pointer_octree(sub_soa, ptr_octree, out_ptr.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ptr = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 5. SpatialHash-Accelerated SOR O(N)
  SpatialHash hash;
  hash.setInputCloud(sub_soa, search_radius);
  hash.build();
  std::vector<PointXYZ> out_hash(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_hash = sor_spatial_hash(sub_soa, hash, out_hash.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_hash = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 6. Caravan Query-Pack SOR
  std::vector<PointXYZ> out_caravan(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_caravan = sor_grid_caravan(sub_soa, out_caravan.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_caravan = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::cout << "  Scalar Brute-force O(N^2):    " << std::fixed << std::setprecision(3)
            << ms_sc << " ms (" << count_sc << " inliers)" << std::endl;
  std::cout << "  RVV Brute-force O(N^2):       " << std::fixed << std::setprecision(3)
            << ms_rvv << " ms (" << count_rvv << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_rvv) << "x speedup" << std::endl;
  std::cout << "  Caravan Query-Pack SOR:       " << std::fixed << std::setprecision(3)
            << ms_caravan << " ms (" << count_caravan << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_caravan) << "x speedup vs scalar" << std::endl;
  std::cout << "  Standard Octree SOR:          " << std::fixed << std::setprecision(3)
            << ms_tree << " ms (" << count_tree << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_tree) << "x speedup vs scalar" << std::endl;
  std::cout << "  Pointer Octree SOR:           " << std::fixed << std::setprecision(3)
            << ms_ptr << " ms (" << count_ptr << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_ptr) << "x speedup vs scalar" << std::endl;
  std::cout << "  Spatial Hash Grid SOR:        " << std::fixed << std::setprecision(3)
            << ms_hash << " ms (" << count_hash << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_hash) << "x speedup vs scalar" << std::endl;

  results.push_back({"StatisticalOutlierRemoval", "Scalar_BruteForce", ms_sc, test_n, count_sc, 1.0});
  results.push_back({"StatisticalOutlierRemoval", "RVV_BruteForce", ms_rvv, test_n, count_rvv, ms_sc / ms_rvv});
  results.push_back({"StatisticalOutlierRemoval", "Caravan_QueryPack_SOR", ms_caravan, test_n, count_caravan, ms_sc / (ms_caravan + 1e-6)});
  results.push_back({"StatisticalOutlierRemoval", "Standard_Octree_SOR", ms_tree, test_n, count_tree, ms_sc / (ms_tree + 1e-6)});
  results.push_back({"StatisticalOutlierRemoval", "Pointer_Octree_SOR", ms_ptr, test_n, count_ptr, ms_sc / (ms_ptr + 1e-6)});
  results.push_back({"StatisticalOutlierRemoval", "SpatialHash_Grid_SOR", ms_hash, test_n, count_hash, ms_sc / (ms_hash + 1e-6)});
}

void runRANSACAblation(const PointCloudSoA &soa,
                       const std::vector<PointXYZ> &aos,
                       float dist_thresh,
                       std::vector<BenchmarkResult> &results) {
  std::cout << "\n=== [Ablation D: RANSAC Plane Fitting Iterations] ===" << std::endl;

  const std::vector<int> iter_counts = {100, 250, 500, 1000};
  for (int iters : iter_counts) {
    float model_rvv[4] = {0};
    auto t0 = std::chrono::high_resolution_clock::now();
    int inliers_rvv = ransac_plane_rvv(soa, dist_thresh, iters, model_rvv);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

    float model_sc[4] = {0};
    t0 = std::chrono::high_resolution_clock::now();
    int inliers_sc = ransac_plane_sc(aos.data(), aos.size(), dist_thresh, iters, model_sc);
    t1 = std::chrono::high_resolution_clock::now();
    double ms_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "  Iters " << std::setw(4) << iters << " | Scalar: " << std::fixed
              << std::setprecision(2) << ms_sc << " ms | RVV: " << ms_rvv
              << " ms (" << std::setprecision(2) << (ms_sc / ms_rvv) << "x speedup)" << std::endl;

    std::ostringstream ss_sc, ss_rvv;
    ss_sc << "Scalar_iter_" << iters;
    ss_rvv << "RVV_iter_" << iters;
    results.push_back({"RANSACPlaneFitting", ss_sc.str(), ms_sc, soa.n, static_cast<std::size_t>(inliers_sc), 1.0});
    results.push_back({"RANSACPlaneFitting", ss_rvv.str(), ms_rvv, soa.n, static_cast<std::size_t>(inliers_rvv), ms_sc / ms_rvv});
  }
}

void saveJSONResults(const std::filesystem::path &out_path,
                     const std::vector<BenchmarkResult> &results) {
  std::ofstream ofs(out_path);
  if (!ofs.is_open()) return;

  ofs << "{\n  \"ablation_benchmarks\": [\n";
  for (std::size_t i = 0; i < results.size(); ++i) {
    const auto &r = results[i];
    ofs << "    {\n";
    ofs << "      \"category\": \"" << r.category << "\",\n";
    ofs << "      \"implementation\": \"" << r.implementation << "\",\n";
    ofs << "      \"time_ms\": " << r.time_ms << ",\n";
    ofs << "      \"input_count\": " << r.input_count << ",\n";
    ofs << "      \"output_count\": " << r.output_count << ",\n";
    ofs << "      \"speedup\": " << r.speedup_vs_baseline << "\n";
    ofs << "    }" << (i + 1 < results.size() ? "," : "") << "\n";
  }
  ofs << "  ]\n}\n";
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input.pcd> [output_dir]" << std::endl;
    return 1;
  }

  const std::string input_file = argv[1];
  const std::filesystem::path output_dir = (argc >= 3) ? std::filesystem::path(argv[2]) : std::filesystem::path("output/ablation_results");
  std::filesystem::create_directories(output_dir);

  std::vector<PointXYZ> loaded_pts;
  std::cout << "Loading input PCD: " << input_file << std::endl;
  if (loadPCD(input_file, loaded_pts) < 0) {
    std::cerr << "Failed to load " << input_file << std::endl;
    return 1;
  }

  // Downsample to reasonable test density for micro-benchmarks (~15k points)
  std::size_t n_raw = loaded_pts.size();
  std::vector<float> rx(n_raw), ry(n_raw), rz(n_raw);
  for (std::size_t i = 0; i < n_raw; ++i) {
    rx[i] = loaded_pts[i].x;
    ry[i] = loaded_pts[i].y;
    rz[i] = loaded_pts[i].z;
  }
  PointCloudSoA raw_soa = {rx.data(), ry.data(), rz.data(), n_raw};

  std::vector<PointXYZ> down_pts(n_raw);
  std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_soa, down_pts.data(), 0.20f);
  down_pts.resize(n_down);

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (std::size_t i = 0; i < n_down; ++i) {
    dx[i] = down_pts[i].x;
    dy[i] = down_pts[i].y;
    dz[i] = down_pts[i].z;
  }
  PointCloudSoA test_soa = {dx.data(), dy.data(), dz.data(), n_down};

  std::cout << "Input cloud: " << n_raw << " points -> Test Cloud: " << n_down << " points" << std::endl;

  std::vector<BenchmarkResult> results;
  runVoxelGridAblation(raw_soa, loaded_pts, 0.20f, results);
  runNeighborSearchAblation(test_soa, down_pts, 0.05f, results);
  runSORAblation(test_soa, down_pts, 20, 1.0f, results);
  runRANSACAblation(test_soa, down_pts, 0.20f, results);

  saveJSONResults(output_dir / "ablation_metrics.json", results);
  std::cout << "\nSaved ablation metrics to: " << (output_dir / "ablation_metrics.json").string() << std::endl;

  return 0;
}
