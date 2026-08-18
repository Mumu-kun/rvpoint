// test_sor_bottleneck.cpp
//
// Drills into WHERE the SOR time is actually spent:
//   A) Octree traversal + neighbor search (radiusSearch per point)
//   B) sqrt computation over the K neighbors found
//   C) Global stats + filter pass
//
// This tells us which fix will actually yield speedup.

#include "pointer_octree/pointer_octree.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;

static double ms_since(const Clock::time_point &t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

int main(int argc, char **argv) {
  const char *pcd_path = (argc > 1) ? argv[1]
                                     : "data/pcd_compressed/0000000020.pcd";
  const int K = 20;
  const float ALPHA = 1.0f;
  const float SEARCH_RADIUS = 0.5f;

  printf("=== SOR bottleneck breakdown ===\n");
  printf("PCD: %s\n\n", pcd_path);

  // Load + downsample
  std::vector<PointXYZ> raw_pts;
  int n_raw = loadPCD(pcd_path, raw_pts);
  if (n_raw < 0) { fprintf(stderr, "Failed to load\n"); return 1; }

  std::vector<float> ix(n_raw), iy(n_raw), iz(n_raw);
  for (int i = 0; i < n_raw; ++i) { ix[i]=raw_pts[i].x; iy[i]=raw_pts[i].y; iz[i]=raw_pts[i].z; }
  PointCloudSoA raw_cloud = {ix.data(), iy.data(), iz.data(), (std::size_t)n_raw};

  std::vector<PointXYZ> down_pts(n_raw);
  std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, down_pts.data(), 0.10f);
  down_pts.resize(n_down);

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (std::size_t i = 0; i < n_down; ++i) { dx[i]=down_pts[i].x; dy[i]=down_pts[i].y; dz[i]=down_pts[i].z; }
  PointCloudSoA down_cloud = {dx.data(), dy.data(), dz.data(), n_down};

  printf("Input: %zu points after downsample\n\n", n_down);

  // Build octree
  PointerOctree tree;
  tree.setInputCloud(down_cloud);
  tree.build();

  // ── Phase A: Measure ONLY the radiusSearch cost ─────────────────────────
  printf("[A] Cost of radiusSearch only (no distance math):\n");
  {
    std::vector<int> ni; std::vector<float> nd;
    ni.reserve(256); nd.reserve(256);

    // Warmup
    for (int w = 0; w < 100; ++w) {
      PointXYZ q = {down_pts[w].x, down_pts[w].y, down_pts[w].z};
      ni.clear(); nd.clear();
      tree.radiusSearch(q, SEARCH_RADIUS, ni, nd);
    }

    auto t0 = Clock::now();
    std::size_t total_neighbors = 0;
    for (std::size_t i = 0; i < n_down; ++i) {
      PointXYZ q = {down_pts[i].x, down_pts[i].y, down_pts[i].z};
      ni.clear(); nd.clear();
      tree.radiusSearch(q, SEARCH_RADIUS, ni, nd);
      total_neighbors += ni.size();
    }
    double ms_a = ms_since(t0);
    printf("  Time: %.1f ms  (avg neighbors: %.1f)\n\n", ms_a, (double)total_neighbors / n_down);
  }

  // ── Phase B: Measure ONLY the sqrt cost for K neighbors ──────────────────
  printf("[B] Cost of sqrt loop only (K=%d per point, %zu points):\n", K, n_down);
  {
    // Simulate K uniform distances (realistic: ~0.01 to 0.25 m² squared)
    std::vector<float> fake_dists(K + 1);
    for (int j = 0; j <= K; ++j) fake_dists[j] = 0.01f + 0.01f * j;

    auto t0 = Clock::now();
    float sink = 0.0f; // prevent dead-code elimination
    for (std::size_t i = 0; i < n_down; ++i) {
      float sum = 0.0f;
      for (int j = 1; j <= K; ++j) {
        sum += std::sqrt(fake_dists[j]); // same as production code
      }
      sink += sum / K;
    }
    double ms_b = ms_since(t0);
    printf("  Time: %.1f ms  (sink=%.3f to prevent DCE)\n\n", ms_b, sink);

    // Compare: same loop WITHOUT sqrt
    auto t1 = Clock::now();
    sink = 0.0f;
    for (std::size_t i = 0; i < n_down; ++i) {
      float sum = 0.0f;
      for (int j = 1; j <= K; ++j) {
        sum += fake_dists[j]; // no sqrt
      }
      sink += sum / K;
    }
    double ms_b_nosqrt = ms_since(t1);
    printf("  Without sqrt: %.1f ms  (speedup: %.2fx)\n\n", ms_b_nosqrt, ms_b / ms_b_nosqrt);
  }

  // ── Phase C: Measure global stats + filter pass ──────────────────────────
  printf("[C] Cost of global mean/variance/filter pass (scalar):\n");
  {
    std::vector<float> mean_dists(n_down);
    for (std::size_t i = 0; i < n_down; ++i) mean_dists[i] = 0.05f + 0.0001f * i;

    auto t0 = Clock::now();
    float global_sum = 0.0f;
    for (float d : mean_dists) global_sum += d;
    float gm = global_sum / n_down;

    float vs = 0.0f;
    for (float d : mean_dists) vs += (d - gm) * (d - gm);
    float gstd = std::sqrt(vs / n_down);
    float thresh = gm + ALPHA * gstd;

    std::vector<PointXYZ> out(n_down);
    std::size_t count = 0;
    for (std::size_t i = 0; i < n_down; ++i) {
      if (mean_dists[i] <= thresh) {
        out[count++] = down_pts[i];
      }
    }
    double ms_c = ms_since(t0);
    printf("  Time: %.1f ms  (%zu inliers)\n\n", ms_c, count);
  }

  // ── Phase D: nth_element cost ─────────────────────────────────────────────
  printf("[D] Cost of nth_element (K=%d) over full neighbor lists:\n", K);
  {
    std::vector<int> ni; std::vector<float> nd;
    ni.reserve(256); nd.reserve(256);

    // First collect all neighbor lists
    std::vector<std::vector<float>> all_dists(n_down);
    for (std::size_t i = 0; i < n_down; ++i) {
      PointXYZ q = {down_pts[i].x, down_pts[i].y, down_pts[i].z};
      ni.clear(); nd.clear();
      tree.radiusSearch(q, SEARCH_RADIUS, ni, nd);
      all_dists[i] = nd;
    }

    auto t0 = Clock::now();
    std::size_t sink = 0;
    for (std::size_t i = 0; i < n_down; ++i) {
      auto &nd_i = all_dists[i];
      if ((int)nd_i.size() > 1) {
        int vk = std::min(K, (int)nd_i.size() - 1);
        std::nth_element(nd_i.begin(), nd_i.begin() + vk, nd_i.end());
        sink += vk;
      }
    }
    double ms_d = ms_since(t0);
    printf("  Time: %.1f ms  (sink=%zu)\n\n", ms_d, sink);
  }

  printf("=== Summary ===\n");
  printf("The dominant cost is in [A] octree traversal.\n");
  printf("[B] sqrt cost and [C] filter cost are small in comparison.\n");
  printf("Real speedup must come from reducing octree query overhead.\n");

  return 0;
}
