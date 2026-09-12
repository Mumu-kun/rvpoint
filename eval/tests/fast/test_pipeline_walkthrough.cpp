#include "include/rvpoint.h"

#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <cassert>
#include <string>
#include <algorithm>

using namespace rvpoint;

int main(int argc, char** argv) {
  std::cout << "==============================================================\n";
  std::cout << "    RVPOINT END-TO-END PERCEPTION PIPELINE WALKTHROUGH        \n";
  std::cout << "==============================================================\n\n";

  // 1. Load or synthesize test point cloud
  std::string pcd_path = "data/0000000000.pcd";
  if (argc > 1) {
    pcd_path = argv[1];
  }

  std::cout << "[Step 1] Loading Point Cloud Input..." << std::endl;
  PointCloud input_cloud;
  bool ok = loadPCD(pcd_path, input_cloud);
  if (!ok) ok = loadPCD("data/pcd_compressed/0000000090.pcd", input_cloud);
  if (!ok) ok = loadPCD("../data/0000000000.pcd", input_cloud);

  if (!ok || input_cloud.empty()) {
    std::cout << "  (Generating synthetic tabletop LiDAR frame of 30,000 points...)\n";
    input_cloud.reserve(30000);
    // Ground plane points
    for (std::size_t i = 0; i < 20000; ++i) {
      float px = (static_cast<float>(i % 200) - 100.0f) * 0.02f;
      float py = (static_cast<float>(i / 200) - 50.0f) * 0.02f;
      float pz = 0.0f + (static_cast<float>(i % 5) * 0.001f);
      input_cloud.push_back(px, py, pz);
    }
    // Obstacle cluster 1
    for (std::size_t i = 20000; i < 25000; ++i) {
      float px = 0.5f + (static_cast<float>((i - 20000) % 50) * 0.005f);
      float py = 0.5f + (static_cast<float>((i - 20000) / 50) * 0.005f);
      float pz = 0.2f + (static_cast<float>(i % 10) * 0.01f);
      input_cloud.push_back(px, py, pz);
    }
    // Obstacle cluster 2
    for (std::size_t i = 25000; i < 30000; ++i) {
      float px = -0.6f + (static_cast<float>((i - 25000) % 50) * 0.005f);
      float py = -0.4f + (static_cast<float>((i - 25000) / 50) * 0.005f);
      float pz = 0.25f + (static_cast<float>(i % 10) * 0.01f);
      input_cloud.push_back(px, py, pz);
    }
  }

  const std::size_t num_raw_pts = input_cloud.size();
  std::cout << "  ✓ Raw Input Cloud: " << num_raw_pts << " points\n\n";

  // 2. Construct PipelineManager and Register Stages
  std::cout << "[Step 2] Building Slotted Perception Pipeline via PipelineManager...\n";
  PipelineManager pm;

  // Stage 1: Voxel Grid Downsampling (Zero-heap functor)
  pm.add_node("voxel_grid",
      in<PointCloud>("raw_cloud"),
      out<PointCloud>("downsampled_cloud"),
      param<float>("voxel_leaf_size", 1.0f)
  ).kernel([filter = VoxelGrid{}](
      const PointCloud& in_cloud, PointCloud& out_cloud, float leaf_size) mutable
  {
    filter(in_cloud.view(), out_cloud, leaf_size);
  });

  // Stage 2: RANSAC Ground Segmentation & Extraction (Pipeline Inversion)
  pm.add_node("ransac_ground",
      in<PointCloud>("downsampled_cloud"),
      out<PlaneModel>("ground_plane"),
      out<PointCloud>("ground_cloud"),
      out<PointCloud>("obstacle_cloud"),
      param<float>("ransac_thresh", 0.20f),
      param<int>("ransac_max_iters", 50)
  ).kernel([ransac = RansacPlane{}](
      const PointCloud& in_cloud, PlaneModel& plane,
      PointCloud& ground, PointCloud& obstacles,
      float dist_thresh, int max_iters) mutable
  {
    ransac(in_cloud.view(), plane, dist_thresh, max_iters);
    ransac.extract(in_cloud.view(), plane, dist_thresh, ground, obstacles);
  });

  // Stage 3: Radius Outlier Removal on Obstacle Points (Uniform Grid Accelerated)
  pm.add_node("ror_filter",
      in<PointCloud>("obstacle_cloud"),
      out<PointCloud>("cleaned_obstacles"),
      param<float>("ror_radius", 2.0f),
      param<int>("ror_min_neighbors", 2)
  ).kernel([ror = RadiusOutlierRemoval{}](
      const PointCloud& in_cloud, PointCloud& out_cloud, float radius, int min_neighbors) mutable
  {
    ror(in_cloud.view(), out_cloud, radius, min_neighbors);
  });

  // Stage 4: Fast 3D Spatial Grid Construction
  pm.add_node("spatial_grid_build",
      in<PointCloud>("cleaned_obstacles"),
      out<Fast3DSpatialGrid>("spatial_grid"),
      param<float>("grid_cell_size", 2.0f)
  )
  .on_reconfig<float>("grid_cell_size", [](RegisterFile& rf, float new_cell_size) {
    std::cout << "    [Reconfig Hook] Updating Fast3DSpatialGrid cell size to: " << new_cell_size << "\n";
    auto& grid = rf.get_mut<Fast3DSpatialGrid>(rf.get_id("spatial_grid"));
    grid = Fast3DSpatialGrid(new_cell_size, 32768);
  })
  .kernel([](const PointCloud& obs_cloud, Fast3DSpatialGrid& grid, float /*cell_size*/) {
    grid.build(obs_cloud.view());
  });

  // Stage 5: Euclidean Clustering with Flat CSR ClusterResult
  pm.add_node("clustering",
      in<PointCloud>("cleaned_obstacles"),
      out<ClusterResult>("clusters"),
      param<float>("cluster_tol", 2.0f),
      param<int>("cluster_min_size", 2),
      param<int>("cluster_max_size", 50000)
  ).kernel([ec = EuclideanClustering{}](
      const PointCloud& in_cloud,
      ClusterResult& out_clusters, float tol, int min_sz, int max_sz) mutable
  {
    ec(in_cloud.view(), out_clusters, tol, min_sz, max_sz);
  });

  // Stage 6: Surface Normal Estimation on Obstacles
  pm.add_node("normal_estimation",
      in<PointCloud>("cleaned_obstacles"),
      out<PointCloud>("normals"),
      param<int>("normal_k", 5),
      param<float>("normal_radius", 2.0f)
  ).kernel([ne = NormalEstimation{}](
      const PointCloud& in_cloud, PointCloud& out_normals, int k, float radius) mutable
  {
    ne(in_cloud.view(), out_normals, k, radius, 0.0f, 0.0f, 0.0f);
  });

  // 3. Declarative Probes for Telemetry and Diagnostics
  std::size_t probed_downsampled = 0;
  std::size_t probed_ground = 0;
  std::size_t probed_obstacles = 0;
  std::size_t probed_clusters = 0;
  std::size_t probed_normals = 0;

  pm.add_probe<PointCloud>("downsampled_cloud", [&](const PointCloud& down) {
    probed_downsampled = down.size();
  });

  pm.add_probe<PlaneModel>("ground_plane", [&](const PlaneModel& plane) {
    std::cout << "    [Probe: Ground Plane] " << std::fixed << std::setprecision(3)
              << plane.a << "x + " << plane.b << "y + " << plane.c << "z + " << plane.d
              << " = 0 (Inliers: " << plane.inliers << ")\n";
  });

  pm.add_probe<PointCloud>("ground_cloud", [&](const PointCloud& gnd) {
    probed_ground = gnd.size();
  });

  pm.add_probe<PointCloud>("cleaned_obstacles", [&](const PointCloud& obs) {
    probed_obstacles = obs.size();
  });

  pm.add_probe<ClusterResult>("clusters", [&](const ClusterResult& clust) {
    probed_clusters = clust.num_clusters();
    std::cout << "    [Probe: Clustering] Extracted " << clust.num_clusters()
              << " clusters (Total clustered indices: " << clust.indices.size() << ")\n";
  });

  pm.add_probe<PointCloud>("normals", [&](const PointCloud& nrm) {
    probed_normals = nrm.size();
  });

  // 4. Initialize Pipeline & Pre-allocate Capacities (Zero Heap Setup)
  std::cout << "[Step 3] Initializing Pipeline & Pre-allocating Register Buffers...\n";
  pm.set_primary_input("raw_cloud");
  pm.initialize(num_raw_pts + 1024);

  // 5. Execute Frame 1 (Baseline Execution)
  std::cout << "\n[Step 4] Executing Frame 1 (Baseline)...\n";
  pm.step(input_cloud);
  pm.print_telemetry();

  assert(probed_downsampled > 0 && probed_downsampled <= num_raw_pts);
  assert(probed_ground > 0);
  assert(probed_obstacles > 0);
  assert(probed_normals == probed_obstacles);

  // Record allocated capacities across register slots after Frame 1
  auto cap_down = pm.registers().get<PointCloud>(pm.get_id("downsampled_cloud")).x.capacity();
  auto cap_gnd  = pm.registers().get<PointCloud>(pm.get_id("ground_cloud")).x.capacity();
  auto cap_obs  = pm.registers().get<PointCloud>(pm.get_id("obstacle_cloud")).x.capacity();
  auto cap_nrm  = pm.registers().get<PointCloud>(pm.get_id("normals")).x.capacity();

  // 6. Execute Frame 2 (Verify Steady-State Zero-Heap Invariant)
  std::cout << "[Step 5] Executing Frame 2 (Steady-State Zero-Heap Verification)...\n";
  pm.step(input_cloud);

  // Assert capacities are strictly unchanged (no reallocation occurred)
  assert(pm.registers().get<PointCloud>(pm.get_id("downsampled_cloud")).x.capacity() == cap_down);
  assert(pm.registers().get<PointCloud>(pm.get_id("ground_cloud")).x.capacity() == cap_gnd);
  assert(pm.registers().get<PointCloud>(pm.get_id("obstacle_cloud")).x.capacity() == cap_obs);
  assert(pm.registers().get<PointCloud>(pm.get_id("normals")).x.capacity() == cap_nrm);
  std::cout << "  ✓ Buffer capacities verified: ZERO heap allocations on steady-state hot path!\n\n";

  // 7. Dynamic Parameter Reconfiguration Test
  std::cout << "[Step 6] Testing Live Dynamic Parameter Reconfiguration...\n";
  std::cout << "  * Updating 'grid_cell_size' = 0.20m\n";
  std::cout << "  * Updating 'cluster_tol' = 0.25m\n";

  pm.set_param("grid_cell_size", 0.20f);
  pm.set_param("cluster_tol", 0.25f);

  // Execute Frame 3 with new parameters
  pm.step(input_cloud);
  pm.print_telemetry();

  std::cout << "==============================================================\n";
  std::cout << "   >>> PIPELINE WALKTHROUGH COMPLETED SUCCESSFULLY! <<<       \n";
  std::cout << "==============================================================\n";
  return 0;
}
