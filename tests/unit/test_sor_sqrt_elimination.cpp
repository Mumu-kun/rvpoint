// test_sor_sqrt_elimination.cpp
//
// Validates the core hypothesis for Plan Fix #1:
//   "Eliminating sqrt() from SOR neighbor distance accumulation
//    produces equivalent outlier classification with lower cost."
//
// What this tests:
//   1. Both sqrt-based and sq-based SOR on the same cloud + octree
//   2. Compare: which points are classified as outliers?
//   3. Report: agreement rate, timing, point count delta
//
// Pass/Fail criteria:
//   - Point count agreement: > 95% of the same points removed
//   - Timing: sq-based version is faster (no sqrt hot loop)

#include "pointer_octree/pointer_octree.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;

// ── Helper: milliseconds since a time_point ──────────────────────────────────
static double ms_since(const Clock::time_point &t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ── SOR variant A: current implementation (uses sqrt per neighbor) ───────────
//
// Mirrors sor_pointer_octree() in statistical_outlier_removal.cpp exactly.
static std::size_t sor_sqrt(const PointCloudSoA &in, const PointerOctree &tree,
                            PointXYZ *out, int k, float alpha,
                            float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n);

  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;
  nbr_indices.reserve(256);
  nbr_dists.reserve(256);

  for (std::size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    nbr_indices.clear();
    nbr_dists.clear();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

    if ((int)nbr_dists.size() > 1) {
      int valid_k = std::min(k, (int)nbr_dists.size() - 1);
      std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k,
                       nbr_dists.end());
      float sum = 0.0f;
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(nbr_dists[j]); // ← sqrt here (current code)
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
  }

  // Global stats
  float global_sum = 0.0f;
  for (float d : mean_dists) global_sum += d;
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
  float global_std = std::sqrt(variance_sum / in.n);
  float thresh = global_mean + alpha * global_std;

  std::size_t count = 0;
  for (std::size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
  return count;
}

// ── SOR variant B: proposed implementation (squared distances, no sqrt) ───────
//
// Key change: nbr_dists from radiusSearch are already squared distances.
// We accumulate squared distances, compute global stats on squared values,
// and compare squared mean-dist to a squared threshold.
// The relative ordering of points by "average neighbor closeness" is preserved.
static std::size_t sor_nosqrt(const PointCloudSoA &in,
                               const PointerOctree &tree, PointXYZ *out, int k,
                               float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists_sq(in.n); // squared distances throughout

  std::vector<int> nbr_indices;
  std::vector<float> nbr_dists;
  nbr_indices.reserve(256);
  nbr_dists.reserve(256);

  float search_radius_sq = search_radius * search_radius;

  for (std::size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    nbr_indices.clear();
    nbr_dists.clear();
    tree.radiusSearch(query, search_radius, nbr_indices, nbr_dists);

    if ((int)nbr_dists.size() > 1) {
      int valid_k = std::min(k, (int)nbr_dists.size() - 1);
      std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k,
                       nbr_dists.end());
      float sum_sq = 0.0f;
      for (int j = 1; j <= valid_k; ++j) {
        sum_sq += nbr_dists[j]; // ← nbr_dists[j] is already d²; no sqrt
      }
      mean_dists_sq[i] = sum_sq / valid_k;
    } else {
      mean_dists_sq[i] = search_radius_sq; // fallback in squared units
    }
  }

  // Global stats (in squared-distance space)
  float global_sum = 0.0f;
  for (float d : mean_dists_sq) global_sum += d;
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists_sq) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
  float global_std = std::sqrt(variance_sum / in.n); // std of squared dists
  float thresh = global_mean + alpha * global_std;   // threshold in d² space

  std::size_t count = 0;
  for (std::size_t i = 0; i < in.n; ++i) {
    if (mean_dists_sq[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
  return count;
}

// ── Classification agreement check ───────────────────────────────────────────
// Computes per-point inlier/outlier flags for both variants,
// returns the agreement rate (0..1).
static float compute_agreement(const PointCloudSoA &in,
                                const PointerOctree &tree, int k, float alpha,
                                float search_radius) {
  std::size_t n = in.n;

  // Compute per-point flags for both variants
  std::vector<bool> is_inlier_sqrt(n, false);
  std::vector<bool> is_inlier_nosqrt(n, false);

  // ── Variant A: sqrt ─────────────────────────────────────────────────────
  {
    std::vector<float> mean_dists(n);
    std::vector<int> ni;
    std::vector<float> nd;
    ni.reserve(256);
    nd.reserve(256);

    for (std::size_t i = 0; i < n; ++i) {
      PointXYZ q = {in.x[i], in.y[i], in.z[i]};
      ni.clear(); nd.clear();
      tree.radiusSearch(q, search_radius, ni, nd);
      if ((int)nd.size() > 1) {
        int vk = std::min(k, (int)nd.size() - 1);
        std::nth_element(nd.begin(), nd.begin() + vk, nd.end());
        float sum = 0;
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
    for (std::size_t i = 0; i < n; ++i) is_inlier_sqrt[i] = (mean_dists[i] <= thresh);
  }

  // ── Variant B: no sqrt ──────────────────────────────────────────────────
  {
    float sr2 = search_radius * search_radius;
    std::vector<float> mean_dists_sq(n);
    std::vector<int> ni;
    std::vector<float> nd;
    ni.reserve(256);
    nd.reserve(256);

    for (std::size_t i = 0; i < n; ++i) {
      PointXYZ q = {in.x[i], in.y[i], in.z[i]};
      ni.clear(); nd.clear();
      tree.radiusSearch(q, search_radius, ni, nd);
      if ((int)nd.size() > 1) {
        int vk = std::min(k, (int)nd.size() - 1);
        std::nth_element(nd.begin(), nd.begin() + vk, nd.end());
        float sum = 0;
        for (int j = 1; j <= vk; ++j) sum += nd[j];
        mean_dists_sq[i] = sum / vk;
      } else {
        mean_dists_sq[i] = sr2;
      }
    }
    float gs = 0; for (float d : mean_dists_sq) gs += d;
    float gm = gs / n;
    float vs = 0; for (float d : mean_dists_sq) vs += (d - gm) * (d - gm);
    float gstd = std::sqrt(vs / n);
    float thresh = gm + alpha * gstd;
    for (std::size_t i = 0; i < n; ++i) is_inlier_nosqrt[i] = (mean_dists_sq[i] <= thresh);
  }

  // Agreement: fraction of points classified the same way
  std::size_t agree = 0;
  for (std::size_t i = 0; i < n; ++i) {
    if (is_inlier_sqrt[i] == is_inlier_nosqrt[i]) agree++;
  }
  return static_cast<float>(agree) / static_cast<float>(n);
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char **argv) {
  const char *pcd_path = (argc > 1) ? argv[1]
                                     : "data/pcd_compressed/0000000020.pcd";
  const int K = 20;
  const float ALPHA = 1.0f;
  const float SEARCH_RADIUS = 0.5f;

  printf("=== SOR sqrt-elimination validation test ===\n");
  printf("PCD:           %s\n", pcd_path);
  printf("K neighbors:   %d\n", K);
  printf("Alpha:         %.1f\n", ALPHA);
  printf("Search radius: %.2f m\n\n", SEARCH_RADIUS);

  // ── Load ──────────────────────────────────────────────────────────────────
  std::vector<PointXYZ> raw_pts;
  int n_raw = loadPCD(pcd_path, raw_pts);
  if (n_raw < 0) {
    fprintf(stderr, "FAIL: cannot load '%s'\n", pcd_path);
    return 1;
  }

  // ── Downsample (same params as pipeline) ─────────────────────────────────
  std::vector<float> ix(n_raw), iy(n_raw), iz(n_raw);
  for (int i = 0; i < n_raw; ++i) {
    ix[i] = raw_pts[i].x;
    iy[i] = raw_pts[i].y;
    iz[i] = raw_pts[i].z;
  }
  PointCloudSoA raw_cloud = {ix.data(), iy.data(), iz.data(), (std::size_t)n_raw};

  std::vector<PointXYZ> down_pts(n_raw);
  std::size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, down_pts.data(), 0.10f);
  down_pts.resize(n_down);

  std::vector<float> dx(n_down), dy(n_down), dz(n_down);
  for (std::size_t i = 0; i < n_down; ++i) {
    dx[i] = down_pts[i].x;
    dy[i] = down_pts[i].y;
    dz[i] = down_pts[i].z;
  }
  PointCloudSoA down_cloud = {dx.data(), dy.data(), dz.data(), n_down};

  printf("Loaded %d points, downsampled to %zu points.\n\n", n_raw, n_down);

  // ── Build PointerOctree ───────────────────────────────────────────────────
  printf("Building PointerOctree...\n");
  auto t0 = Clock::now();
  PointerOctree tree;
  tree.setInputCloud(down_cloud);
  tree.build();
  printf("  Build: %.1f ms\n\n", ms_since(t0));

  // ── Classification agreement ──────────────────────────────────────────────
  printf("[TEST 1] Classification agreement (same points inlier/outlier?)\n");
  float agreement = compute_agreement(down_cloud, tree, K, ALPHA, SEARCH_RADIUS);
  printf("  Agreement: %.2f%%\n", agreement * 100.0f);
  bool agreement_ok = (agreement >= 0.95f);
  printf("  %s (threshold: >= 95%%)\n\n", agreement_ok ? "PASS" : "FAIL");

  // ── Timing comparison ─────────────────────────────────────────────────────
  printf("[TEST 2] Timing: sqrt vs no-sqrt SOR\n");

  std::vector<PointXYZ> out_sqrt(n_down), out_nosqrt(n_down);

  // Warm-up run (avoid cold-start penalty on first timed run)
  sor_sqrt(down_cloud, tree, out_sqrt.data(), K, ALPHA, SEARCH_RADIUS);

  // Timed: with sqrt
  t0 = Clock::now();
  std::size_t n_sqrt = sor_sqrt(down_cloud, tree, out_sqrt.data(), K, ALPHA, SEARCH_RADIUS);
  double ms_sqrt = ms_since(t0);

  // Warm-up
  sor_nosqrt(down_cloud, tree, out_nosqrt.data(), K, ALPHA, SEARCH_RADIUS);

  // Timed: without sqrt
  t0 = Clock::now();
  std::size_t n_nosqrt = sor_nosqrt(down_cloud, tree, out_nosqrt.data(), K, ALPHA, SEARCH_RADIUS);
  double ms_nosqrt = ms_since(t0);

  printf("  SOR with    sqrt: %6.1f ms → %zu inliers\n", ms_sqrt, n_sqrt);
  printf("  SOR without sqrt: %6.1f ms → %zu inliers\n", ms_nosqrt, n_nosqrt);

  double speedup = ms_sqrt / ms_nosqrt;
  bool faster = (ms_nosqrt < ms_sqrt);
  printf("  Speedup: %.2fx — %s\n\n", speedup, faster ? "PASS (faster)" : "FAIL (not faster)");

  // ── Point count delta ─────────────────────────────────────────────────────
  printf("[TEST 3] Point count delta (inlier counts close?)\n");
  long delta = (long)n_sqrt - (long)n_nosqrt;
  float delta_pct = 100.0f * std::abs((float)delta) / (float)n_down;
  bool count_ok = (delta_pct <= 5.0f); // within 5% of downsampled cloud
  printf("  sqrt  inliers: %zu\n", n_sqrt);
  printf("  nosqrt inliers: %zu\n", n_nosqrt);
  printf("  Delta: %+ld pts (%.2f%% of cloud) — %s (threshold: <= 5%%)\n\n",
         delta, delta_pct, count_ok ? "PASS" : "FAIL");

  // ── Summary ───────────────────────────────────────────────────────────────
  bool all_pass = agreement_ok && count_ok;
  printf("==============================================\n");
  printf(" OVERALL: %s\n", all_pass ? "PASS — safe to eliminate sqrt in SOR" : "FAIL — sqrt cannot be safely eliminated");
  printf("  Agreement: %.2f%%\n", agreement * 100.0f);
  printf("  Count delta: %.2f%%\n", delta_pct);
  printf("  Speedup: %.2fx\n", speedup);
  printf("==============================================\n");

  return all_pass ? 0 : 1;
}
