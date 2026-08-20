// test_sor_radius_validation.cpp
//
// Final validation: does search_radius=0.25 produce a valid SOR result
// that is comparable to PCL's output (46998 inliers) and better than
// our current config (41117 inliers with r=0.5)?
//
// Also validates the complete pipeline SOR step timing vs PCL's 1639ms.

#include "core/point_types.h"
#include "core/rvv_common.h"
#include "search/pointer_octree.h"
#include "filters/statistical_outlier_removal.h"
#include "io/simple_pcd_loader.h"

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

// Full SOR matching our current sor_pointer_octree signature
static double run_sor(const PointCloudSoA &cloud, const PointerOctree &tree,
                      int k, float alpha, float search_radius,
                      std::vector<PointXYZ> &out_pts) {
  std::size_t n = cloud.n;
  std::vector<float> mean_dists(n);
  std::vector<int> ni;
  std::vector<float> nd;
  ni.reserve(256); nd.reserve(256);

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

  out_pts.clear();
  for (std::size_t i = 0; i < n; ++i) {
    if (mean_dists[i] <= thresh) {
      out_pts.push_back({cloud.x[i], cloud.y[i], cloud.z[i]});
    }
  }

  return ms_since(t0);
}

int main(int argc, char **argv) {
  const char *pcd_path = (argc > 1) ? argv[1]
                                     : "data/pcd_compressed/0000000020.pcd";
  // PCL reference values (from the benchmark run)
  const int    PCL_SOR_INLIERS  = 46998;
  const double PCL_SOR_TIME_MS  = 1639.0;
  const int    OUR_SOR_INLIERS  = 41117; // current r=0.5 run

  printf("=== SOR radius=0.25 validation ===\n");
  printf("Reference — PCL SOR:      %d inliers in %.0f ms\n", PCL_SOR_INLIERS, PCL_SOR_TIME_MS);
  printf("Reference — Our current:  %d inliers (r=0.5)\n\n", OUR_SOR_INLIERS);

  // Load + downsample
  std::vector<PointXYZ> raw_pts;
  int n_raw = loadPCD(pcd_path, raw_pts);
  if (n_raw < 0) { fprintf(stderr, "FAIL: cannot load '%s'\n", pcd_path); return 1; }

  std::vector<float> ix(n_raw), iy(n_raw), iz(n_raw);
  for (int i = 0; i < n_raw; ++i) { ix[i]=raw_pts[i].x; iy[i]=raw_pts[i].y; iz[i]=raw_pts[i].z; }
  PointCloudSoA raw_cloud = {ix.data(), iy.data(), iz.data(), (std::size_t)n_raw};

  std::vector<PointXYZ> down_pts(n_raw);
  std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, down_pts.data(), 0.10f);
  down_pts.resize(n_down);

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (std::size_t i = 0; i < n_down; ++i) { dx[i]=down_pts[i].x; dy[i]=down_pts[i].y; dz[i]=down_pts[i].z; }
  PointCloudSoA down_cloud = {dx.data(), dy.data(), dz.data(), n_down};

  // Build octree (one tree, different radii tested)
  PointerOctree tree;
  tree.setInputCloud(down_cloud);
  tree.build();

  std::vector<PointXYZ> out;

  // ── Current config: K=20, r=0.5 (warmup) ────────────────────────────────
  run_sor(down_cloud, tree, 20, 1.0f, 0.5f, out);

  // ── Current config: K=20, r=0.5 (timed) ─────────────────────────────────
  double ms_current = run_sor(down_cloud, tree, 20, 1.0f, 0.5f, out);
  int n_current = (int)out.size();

  printf("[Current]  K=20, r=0.50: %6.1f ms → %d inliers  (%.1f%% of PCL)\n",
         ms_current, n_current, 100.0 * n_current / PCL_SOR_INLIERS);

  // ── Proposed config: K=20, r=0.25 ────────────────────────────────────────
  double ms_proposed = run_sor(down_cloud, tree, 20, 1.0f, 0.25f, out);
  int n_proposed = (int)out.size();

  printf("[Proposed] K=20, r=0.25: %6.1f ms → %d inliers  (%.1f%% of PCL)\n",
         ms_proposed, n_proposed, 100.0 * n_proposed / PCL_SOR_INLIERS);

  // ── Speedup vs current and vs PCL ────────────────────────────────────────
  double speedup_vs_current = ms_current / ms_proposed;
  double speedup_vs_pcl     = PCL_SOR_TIME_MS / ms_proposed;

  printf("\nSpeedup vs our current SOR: %.2fx\n", speedup_vs_current);
  printf("Speedup vs PCL SOR:         %.2fx\n", speedup_vs_pcl);

  // ── Pass/Fail criteria ───────────────────────────────────────────────────
  bool faster_than_pcl   = (ms_proposed < PCL_SOR_TIME_MS);
  float inlier_pct_pcl   = 100.0f * n_proposed / PCL_SOR_INLIERS;
  bool inlier_count_ok   = (inlier_pct_pcl >= 90.0f && inlier_pct_pcl <= 105.0f);

  printf("\n=== Validation ===\n");
  printf("Faster than PCL SOR (%.0f ms):  %s  [%.1f ms]\n",
         PCL_SOR_TIME_MS, faster_than_pcl ? "PASS" : "FAIL", ms_proposed);
  printf("Inlier count within 10%% of PCL: %s  [%.1f%% of PCL]\n",
         inlier_count_ok ? "PASS" : "FAIL", inlier_pct_pcl);

  bool all_pass = faster_than_pcl && inlier_count_ok;
  printf("\nOVERALL: %s\n", all_pass ? "PASS — safe to apply radius=0.25" : "FAIL");

  if (all_pass) {
    printf("\nAction: change search_radius from 0.5f to 0.25f in pipeline_export.cpp SOR call.\n");
    printf("Expected pipeline improvement: −%.0f ms (SOR stage)\n", ms_current - ms_proposed);
    printf("Expected new total: ~%.0f ms vs PCL %.0f ms\n",
           4613.0 - (ms_current - ms_proposed), 6549.0);
  }

  return all_pass ? 0 : 1;
}
