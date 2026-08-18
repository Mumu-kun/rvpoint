// neighbor_search_sor_bench.cpp
// Comparative Benchmark Suite for Plain Neighbor Search & All 4 Caravan SOR Strategies

#include "caravan_pointer_octree.h"
#include "caravan_radius_search.h"
#include "pointer_octree/pointer_octree.h"
#include "profiler.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

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

using namespace rvv_pcl;

namespace {

struct BenchResult {
  std::string category;
  std::string algorithm;
  double time_ms;
  std::size_t input_count;
  std::size_t output_count;
  double speedup_vs_baseline;
};

void runPlainSearchBenchmark(const PointCloudSoA &soa,
                             const std::vector<PointXYZ> &aos,
                             float radius,
                             std::vector<BenchResult> &results) {
  std::cout << "\n==========================================================================" << std::endl;
  std::cout << " PART 1: Plain Spatial Neighbor Search Benchmarks (N=" << soa.n << ")" << std::endl;
  std::cout << "==========================================================================" << std::endl;
  if (soa.n == 0) return;

  const PointXYZ query = aos[soa.n / 2];
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

  // 5. Caravan Single Query Search
  CaravanRadiusSearch caravan;
  caravan.setInputCloud(soa);
  std::vector<int32_t> caravan_indices;
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_caravan = caravan.radiusSearch(query, radius, caravan_indices);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_caravan = std::chrono::duration<double, std::milli>(t1 - t0).count();

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
  std::cout << "  Caravan Radius Search:     " << std::fixed << std::setprecision(3)
            << ms_caravan << " ms (" << count_caravan << " nbrs)" << std::endl;

  results.push_back({"PlainSearch", "Global_RVV_Scan", ms_rvv, soa.n, count_rvv, 1.0});
  results.push_back({"PlainSearch", "Standard_Octree", ms_tree_search, soa.n, count_tree, ms_rvv / (ms_tree_search + 1e-6)});
  results.push_back({"PlainSearch", "Pointer_Octree_RVV", ms_ptr_rvv, soa.n, count_ptr_rvv, ms_rvv / (ms_ptr_rvv + 1e-6)});
  results.push_back({"PlainSearch", "Pointer_Octree_Scalar", ms_ptr_sc, soa.n, count_ptr_sc, ms_rvv / (ms_ptr_sc + 1e-6)});
  results.push_back({"PlainSearch", "SpatialHash_Grid", ms_hash_search, soa.n, count_hash, ms_rvv / (ms_hash_search + 1e-6)});
  results.push_back({"PlainSearch", "Caravan_SingleQuery", ms_caravan, soa.n, count_caravan, ms_rvv / (ms_caravan + 1e-6)});
}

void runSORBenchmark(const PointCloudSoA &soa,
                     const std::vector<PointXYZ> &aos,
                     int k, float alpha,
                     std::vector<BenchResult> &results) {
  std::cout << "\n==========================================================================" << std::endl;
  std::cout << " PART 2: Statistical Outlier Removal (SOR) Benchmarks across 4 Strategies" << std::endl;
  std::cout << "==========================================================================" << std::endl;

  std::size_t test_n = std::min(soa.n, static_cast<std::size_t>(18542));
  PointCloudSoA sub_soa = {soa.x, soa.y, soa.z, test_n};
  float search_radius = 0.5f;

  // 1. Scalar Brute-Force O(N^2)
  std::vector<PointXYZ> out_sc(test_n);
  auto t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_sc = sor_sc(aos.data(), test_n, out_sc.data(), k, alpha);
  auto t1 = std::chrono::high_resolution_clock::now();
  double ms_sc = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 2. RVV Brute-Force O(N^2)
  std::vector<PointXYZ> out_rvv(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_rvv = sor_rvv(sub_soa, out_rvv.data(), k, alpha);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_rvv = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 3. Pointer Octree SOR O(N log N)
  PointerOctree ptr_octree;
  ptr_octree.setInputCloud(sub_soa);
  ptr_octree.build();
  std::vector<PointXYZ> out_ptr(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_ptr = sor_pointer_octree(sub_soa, ptr_octree, out_ptr.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_ptr = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 4. Strategy 1: Grid-Caravan Bounding Box Pruned SOR
  std::vector<PointXYZ> out_strat1(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_strat1 = sor_grid_caravan(sub_soa, out_strat1.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_strat1 = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // 5. Caravan-PointerOctree Hybrid SOR
  CaravanPointerOctree hybrid_octree;
  hybrid_octree.setInputCloud(sub_soa);
  hybrid_octree.build();
  std::vector<PointXYZ> out_hybrid(test_n);
  t0 = std::chrono::high_resolution_clock::now();
  std::size_t count_hybrid = hybrid_octree.sorFilter(out_hybrid.data(), k, alpha, search_radius);
  t1 = std::chrono::high_resolution_clock::now();
  double ms_hybrid = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::cout << "  Scalar Brute-force O(N^2):             " << std::fixed << std::setprecision(3)
            << ms_sc << " ms (" << count_sc << " inliers)" << std::endl;
  std::cout << "  RVV Brute-force O(N^2):                " << std::fixed << std::setprecision(3)
            << ms_rvv << " ms (" << count_rvv << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_rvv) << "x speedup" << std::endl;
  std::cout << "  Pointer Octree SOR (Baseline):         " << std::fixed << std::setprecision(3)
            << ms_ptr << " ms (" << count_ptr << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_ptr) << "x speedup vs scalar" << std::endl;
  std::cout << "  Caravan-PointerOctree Hybrid SOR:      " << std::fixed << std::setprecision(3)
            << ms_hybrid << " ms (" << count_hybrid << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_hybrid) << "x speedup vs scalar" << std::endl;
  std::cout << "  Strategy 1 (Grid-Caravan AABB Pruned):  " << std::fixed << std::setprecision(3)
            << ms_strat1 << " ms (" << count_strat1 << " inliers) -> "
            << std::setprecision(2) << (ms_sc / ms_strat1) << "x speedup vs scalar" << std::endl;

  results.push_back({"SOR", "Scalar_BruteForce", ms_sc, test_n, count_sc, 1.0});
  results.push_back({"SOR", "RVV_BruteForce", ms_rvv, test_n, count_rvv, ms_sc / ms_rvv});
  results.push_back({"SOR", "Pointer_Octree_SOR", ms_ptr, test_n, count_ptr, ms_sc / (ms_ptr + 1e-6)});
  results.push_back({"SOR", "Caravan_PointerOctree_Hybrid", ms_hybrid, test_n, count_hybrid, ms_sc / (ms_hybrid + 1e-6)});
  results.push_back({"SOR", "Strategy1_Grid_Caravan", ms_strat1, test_n, count_strat1, ms_sc / (ms_strat1 + 1e-6)});
}

void saveJSONResults(const std::vector<BenchResult> &results, const std::string &out_path) {
  std::ofstream fs(out_path);
  if (!fs.is_open()) return;

  fs << "[\n";
  for (size_t i = 0; i < results.size(); ++i) {
    const auto &r = results[i];
    fs << "  {\n"
       << "    \"category\": \"" << r.category << "\",\n"
       << "    \"algorithm\": \"" << r.algorithm << "\",\n"
       << "    \"time_ms\": " << r.time_ms << ",\n"
       << "    \"input_count\": " << r.input_count << ",\n"
       << "    \"output_count\": " << r.output_count << ",\n"
       << "    \"speedup_vs_baseline\": " << r.speedup_vs_baseline << "\n"
       << "  }" << (i + 1 < results.size() ? "," : "") << "\n";
  }
  fs << "]\n";
  std::cout << "\nSaved benchmark metrics to: " << out_path << std::endl;
}

} // namespace

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <input_pcd> [output_dir]" << std::endl;
    return 1;
  }

  std::string input_pcd = argv[1];
  std::string output_dir = (argc >= 3) ? argv[2] : "output/neighbor_search_results";

  std::filesystem::create_directories(output_dir);

  std::cout << "==========================================================================" << std::endl;
  std::cout << " NEIGHBOR SEARCH & 4 CARAVAN STRATEGIES COMPARATIVE BENCHMARK SUITE" << std::endl;
  std::cout << "==========================================================================" << std::endl;

  PointCloudSoA soa;
  std::vector<PointXYZ> aos;
  int num_points = loadPCD(input_pcd, aos);
  if (num_points <= 0) {
    std::cerr << "Failed to load input PCD: " << input_pcd << std::endl;
    return 1;
  }

  std::vector<float> orig_x(num_points), orig_y(num_points), orig_z(num_points);
  for (size_t i = 0; i < static_cast<size_t>(num_points); ++i) {
    orig_x[i] = aos[i].x;
    orig_y[i] = aos[i].y;
    orig_z[i] = aos[i].z;
  }
  soa.x = orig_x.data();
  soa.y = orig_y.data();
  soa.z = orig_z.data();
  soa.n = num_points;

  std::vector<PointXYZ> downsampled_pts(soa.n);
  std::size_t downsampled_n = voxel_grid_downsamp_rvv_v2(soa, downsampled_pts.data(), 0.2f);
  PointCloudSoA test_soa;
  std::vector<float> tx(downsampled_n), ty(downsampled_n), tz(downsampled_n);
  for (size_t i = 0; i < downsampled_n; ++i) {
    tx[i] = downsampled_pts[i].x;
    ty[i] = downsampled_pts[i].y;
    tz[i] = downsampled_pts[i].z;
  }
  test_soa.x = tx.data();
  test_soa.y = ty.data();
  test_soa.z = tz.data();
  test_soa.n = downsampled_n;

  std::vector<BenchResult> results;
  runPlainSearchBenchmark(test_soa, downsampled_pts, 0.5f, results);
  runSORBenchmark(test_soa, downsampled_pts, 20, 1.0f, results);

  saveJSONResults(results, output_dir + "/neighbor_sor_benchmark.json");
  return 0;
}
