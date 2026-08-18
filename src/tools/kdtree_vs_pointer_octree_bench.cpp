#include "core/point_types.h"
#include "search/pointer_octree.h"
#include "io/simple_pcd_loader.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/octree.h>

using namespace rvpoint;

int main(int argc, char **argv) {
  std::string pcd_path = "data/pcd_compressed/0000000000.pcd";
  if (argc > 1) {
    pcd_path = argv[1];
  }

  std::cout << "==========================================================================" << std::endl;
  std::cout << " HEAD-TO-HEAD BENCHMARK: Official PCL KdTree vs. RVPoint PointerOctree   " << std::endl;
  std::cout << "==========================================================================" << std::endl;
  std::cout << "Target PCD: " << pcd_path << std::endl;

  // 1. Load Data
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(pcd_path, raw_pts) <= 0 || raw_pts.empty()) {
    std::cerr << "Failed to load PCD: " << pcd_path << std::endl;
    return 1;
  }
  const std::size_t N = raw_pts.size();
  std::cout << "Input Points (N): " << N << std::endl << std::endl;

  // Prepare SoA for PointerOctree
  std::vector<float> xs(N), ys(N), zs(N);
  for (std::size_t i = 0; i < N; ++i) {
    xs[i] = raw_pts[i].x;
    ys[i] = raw_pts[i].y;
    zs[i] = raw_pts[i].z;
  }
  PointCloudSoA soa_cloud = {xs.data(), ys.data(), zs.data(), N};

  // Prepare PCL PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl_cloud->points.resize(N);
  for (std::size_t i = 0; i < N; ++i) {
    pcl_cloud->points[i].x = raw_pts[i].x;
    pcl_cloud->points[i].y = raw_pts[i].y;
    pcl_cloud->points[i].z = raw_pts[i].z;
  }

  // --------------------------------------------------------------------------
  // 1. Index Build Time Comparison
  // --------------------------------------------------------------------------
  std::cout << "--> [1/3] Benchmarking Index Construction Time..." << std::endl;

  // PCL KdTree (FLANN)
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
  auto t0 = std::chrono::high_resolution_clock::now();
  kdtree->setInputCloud(pcl_cloud);
  auto t1 = std::chrono::high_resolution_clock::now();
  double kdtree_build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  // RVPoint PointerOctree
  PointerOctree ptr_octree;
  ptr_octree.setInputCloud(soa_cloud);
  t0 = std::chrono::high_resolution_clock::now();
  ptr_octree.build();
  t1 = std::chrono::high_resolution_clock::now();
  double octree_build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  std::cout << "  • Official PCL KdTree Build:      " << std::setw(8) << std::fixed << std::setprecision(3) << kdtree_build_ms << " ms" << std::endl;
  std::cout << "  • RVPoint PointerOctree Build:    " << std::setw(8) << std::fixed << std::setprecision(3) << octree_build_ms << " ms"
            << "  (Speedup: " << std::fixed << std::setprecision(2) << (kdtree_build_ms / octree_build_ms) << "x)" << std::endl << std::endl;

  // --------------------------------------------------------------------------
  // 2. Batch Radius Search Comparison
  // --------------------------------------------------------------------------
  const std::vector<float> test_radii = {0.05f, 0.15f, 0.30f};
  const std::size_t num_queries = std::min<std::size_t>(1000, N);
  std::cout << "--> [2/3] Benchmarking Batch Radius Search (" << num_queries << " query points)..." << std::endl;

  for (float r : test_radii) {
    std::cout << "\n  --- Radius = " << r << " m ---" << std::endl;

    // PCL KdTree Radius Search
    std::size_t total_found_kd = 0;
    std::vector<int> kd_indices;
    std::vector<float> kd_dists;
    t0 = std::chrono::high_resolution_clock::now();
    for (std::size_t q = 0; q < num_queries; ++q) {
      kd_indices.clear();
      kd_dists.clear();
      kdtree->radiusSearch(pcl_cloud->points[q], r, kd_indices, kd_dists);
      total_found_kd += kd_indices.size();
    }
    t1 = std::chrono::high_resolution_clock::now();
    double kdtree_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // PointerOctree (Scalar) Search
    std::size_t total_found_sc = 0;
    std::vector<int> sc_indices;
    std::vector<float> sc_dists;
    t0 = std::chrono::high_resolution_clock::now();
    for (std::size_t q = 0; q < num_queries; ++q) {
      sc_indices.clear();
      sc_dists.clear();
      PointXYZ q_pt = raw_pts[q];
      ptr_octree.radiusSearchScalar(q_pt, r, sc_indices, sc_dists);
      total_found_sc += sc_indices.size();
    }
    t1 = std::chrono::high_resolution_clock::now();
    double octree_sc_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // PointerOctree (RVV Vectorized) Search
    std::size_t total_found_rvv = 0;
    std::vector<int> rvv_indices;
    std::vector<float> rvv_dists;
    t0 = std::chrono::high_resolution_clock::now();
    for (std::size_t q = 0; q < num_queries; ++q) {
      rvv_indices.clear();
      rvv_dists.clear();
      PointXYZ q_pt = raw_pts[q];
      ptr_octree.radiusSearch(q_pt, r, rvv_indices, rvv_dists);
      total_found_rvv += rvv_indices.size();
    }
    t1 = std::chrono::high_resolution_clock::now();
    double octree_rvv_search_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "    • PCL KdTree (FLANN):           " << std::setw(8) << std::fixed << std::setprecision(3) << kdtree_search_ms
              << " ms  (avg " << std::fixed << std::setprecision(1) << (static_cast<double>(total_found_kd) / num_queries) << " neighbors)" << std::endl;
    std::cout << "    • PointerOctree (Scalar):       " << std::setw(8) << std::fixed << std::setprecision(3) << octree_sc_search_ms
              << " ms  (avg " << std::fixed << std::setprecision(1) << (static_cast<double>(total_found_sc) / num_queries) << " neighbors)"
              << "  vs KdTree: " << std::fixed << std::setprecision(2) << (kdtree_search_ms / octree_sc_search_ms) << "x" << std::endl;
    std::cout << "    • PointerOctree (RVV Vector):   " << std::setw(8) << std::fixed << std::setprecision(3) << octree_rvv_search_ms
              << " ms  (avg " << std::fixed << std::setprecision(1) << (static_cast<double>(total_found_rvv) / num_queries) << " neighbors)"
              << "  vs KdTree: " << std::fixed << std::setprecision(2) << (kdtree_search_ms / octree_rvv_search_ms) << "x"
              << "  (Vector Speedup: " << std::fixed << std::setprecision(2) << (octree_sc_search_ms / octree_rvv_search_ms) << "x)" << std::endl;
  }

  // --------------------------------------------------------------------------
  // 3. Summary Breakdown Table
  // --------------------------------------------------------------------------
  std::cout << "\n==========================================================================================" << std::endl;
  std::cout << "                                SUMMARY COMPARISON RESULTS                                " << std::endl;
  std::cout << "==========================================================================================" << std::endl;
  std::cout << std::left
            << std::setw(30) << "Operation"
            << std::setw(22) << "Official PCL KdTree"
            << std::setw(22) << "RVPoint PointerOctree"
            << std::setw(16) << "Winner / Speedup" << std::endl;
  std::cout << std::string(90, '-') << std::endl;

  std::cout << std::left
            << std::setw(30) << "Index Construction (Build)"
            << std::setw(22) << (std::to_string(kdtree_build_ms).substr(0, 7) + " ms")
            << std::setw(22) << (std::to_string(octree_build_ms).substr(0, 7) + " ms")
            << (std::to_string(kdtree_build_ms / octree_build_ms).substr(0, 5) + "x (Octree)") << std::endl;

  std::cout << "==========================================================================================" << std::endl;
  return 0;
}
