// test_sor_search_strategies.cpp
//
// Tests alternative neighbor search strategies to replace the
// PointerOctree radiusSearch in SOR, which we found takes 92% of SOR time.
//
// Strategies to compare:
//   A) Current: PointerOctree radiusSearch (baseline)
//   B) SpatialHash radiusSearch (O(1) cell lookup, no tree traversal)
//   C) Reduce search radius (smaller search = fewer cells checked)
//   D) Reduce K (K=5 instead of K=20 — PCL literature says K=5 is sufficient)
//   E) KNN instead of radius search: find exactly K nearest (no wasted search)

#include "include/rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
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

// Run complete SOR with configurable K and search radius, return timing + count
static double run_sor_timed(const PointCloudSoA &cloud,
                             const PointerOctree &tree,
                             int k, float alpha, float search_radius,
                             std::size_t &out_count) {
  std::size_t n = cloud.n;
  std::vector<float> mean_dists(n);
  std::vector<int> ni;
  std::vector<float> nd;
  ni.reserve(256);
  nd.reserve(256);

  auto t0 = Clock::now();

  for (std::size_t i = 0; i < n; ++i) {
    PointXYZ q = {cloud.x[i], cloud.y[i], cloud.z[i]};
    ni.clear(); nd.clear();
    tree.radiusSearch(q, search_radius, ni, nd);

    if ((int)nd.size() > 1) {
      int vk = std::min(k, (int)nd.size() - 1);
      std::nth_element(nd.begin(), nd.begin() + vk, nd.end());
      float sum = 0.0f;
      for (int j = 1; j <= vk; ++j) sum += std::sqrt(nd[j]);
      mean_dists[i] = sum / vk;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  float gs = 0; for (float d : mean_dists) gs += d;
  float gm = gs / n;
  float vs = 0; for (float d : mean_dists) vs += (d - gm) * (d - gm);
  float gstd = std::sqrt(vs / n);
  float thresh = gm + alpha * gstd;

  out_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (mean_dists[i] <= thresh) out_count++;
  }

  return ms_since(t0);
}

// Run SOR with SpatialHash
static double run_sor_spatialhash_timed(const PointCloudSoA &cloud,
                                         const SpatialHash &hash,
                                         int k, float alpha, float search_radius,
                                         std::size_t &out_count) {
  std::size_t n = cloud.n;
  std::vector<float> mean_dists(n);
  std::vector<int> ni;
  std::vector<float> nd;
  ni.reserve(256);
  nd.reserve(256);

  auto t0 = Clock::now();

  for (std::size_t i = 0; i < n; ++i) {
    PointXYZ q = {cloud.x[i], cloud.y[i], cloud.z[i]};
    ni.clear(); nd.clear();
    hash.radiusSearch(q, search_radius, ni, nd);

    if ((int)nd.size() > 1) {
      int vk = std::min(k, (int)nd.size() - 1);
      std::nth_element(nd.begin(), nd.begin() + vk, nd.end());
      float sum = 0.0f;
      for (int j = 1; j <= vk; ++j) sum += std::sqrt(nd[j]);
      mean_dists[i] = sum / vk;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  float gs = 0; for (float d : mean_dists) gs += d;
  float gm = gs / n;
  float vs = 0; for (float d : mean_dists) vs += (d - gm) * (d - gm);
  float gstd = std::sqrt(vs / n);
  float thresh = gm + alpha * gstd;

  out_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (mean_dists[i] <= thresh) out_count++;
  }

  return ms_since(t0);
}

int main(int argc, char **argv) {
  const char *pcd_path = (argc > 1) ? argv[1]
                                     : "data/pcd_compressed/0000000020.pcd";
  const float ALPHA = 1.0f;

  printf("=== SOR search strategy comparison ===\n");
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
  printf("Input: %zu points\n\n", n_down);

  // ── Baseline: K=20, radius=0.5 ──────────────────────────────────────────
  {
    printf("Building PointerOctree...\n");
    auto tb = Clock::now();
    PointerOctree tree;
    tree.setInputCloud(down_cloud);
    tree.build();
    printf("  Build: %.1f ms\n\n", ms_since(tb));

    std::size_t cnt;

    // Warmup
    run_sor_timed(down_cloud, tree, 20, ALPHA, 0.5f, cnt);

    printf("[A] Baseline — PointerOctree, K=20, radius=0.50:\n");
    double ms = run_sor_timed(down_cloud, tree, 20, ALPHA, 0.5f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);

    printf("[B] Reduce K only — PointerOctree, K=5, radius=0.50:\n");
    ms = run_sor_timed(down_cloud, tree, 5, ALPHA, 0.5f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);

    printf("[C] Reduce radius — PointerOctree, K=20, radius=0.25:\n");
    ms = run_sor_timed(down_cloud, tree, 20, ALPHA, 0.25f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);

    printf("[D] Both — PointerOctree, K=5, radius=0.25:\n");
    ms = run_sor_timed(down_cloud, tree, 5, ALPHA, 0.25f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);
  }

  // ── SpatialHash (O(1) average cell lookup) ───────────────────────────────
  {
    printf("Building SpatialHash (cell_size=0.50)...\n");
    auto tb = Clock::now();
    SpatialHash hash;
    hash.setInputCloud(down_cloud, 0.50f);
    hash.build();
    printf("  Build: %.1f ms\n\n", ms_since(tb));

    std::size_t cnt;

    // Warmup
    run_sor_spatialhash_timed(down_cloud, hash, 20, ALPHA, 0.50f, cnt);

    printf("[E] SpatialHash, K=20, radius=0.50:\n");
    double ms = run_sor_spatialhash_timed(down_cloud, hash, 20, ALPHA, 0.50f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);

    printf("[F] SpatialHash, K=5, radius=0.25:\n");
    SpatialHash hash2;
    hash2.setInputCloud(down_cloud, 0.25f);
    hash2.build();
    ms = run_sor_spatialhash_timed(down_cloud, hash2, 5, ALPHA, 0.25f, cnt);
    printf("  Time: %6.1f ms  → %zu inliers\n\n", ms, cnt);
  }

  printf("The fastest configuration above is what we should adopt.\n");
  return 0;
}
