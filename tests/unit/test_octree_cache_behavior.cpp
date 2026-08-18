// test_octree_cache_behavior.cpp
//
// Isolates whether the PointerOctree bottleneck is:
//   A) Tree traversal (pointer chasing → cache misses, inherently serial)
//   B) Leaf-level distance computation (vectorized with RVV → real HW benefit)
//
// On QEMU there is no cache simulation — all memory accesses cost the same.
// On real HW, pointer chasing causes L1/L2 misses (4-40 cycles each).
// This test measures the SPLIT between traversal cost and leaf-compute cost.

#include "include/rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "simple_pcd_loader.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>
#include <riscv_vector.h>

using namespace rvv_pcl;
using Clock = std::chrono::high_resolution_clock;
static double ms_since(const Clock::time_point &t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}

// ── Pure leaf distance computation (RVV) — NO tree traversal ─────────────────
// Simulates the leaf check cost if we had a perfect index that jumped directly
// to the right leaves, with zero traversal overhead.
// We do this by scanning a flat contiguous array — maximum RVV throughput.
static double pure_leaf_rvv(const float *x, const float *y, const float *z,
                              std::size_t n, float qx, float qy, float qz, float r2) {
  auto t = Clock::now();
  std::size_t count = 0;
  std::size_t i = 0;
  while (i < n) {
    std::size_t vl = __riscv_vsetvl_e32m8(n - i);
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(x + i, vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(y + i, vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(z + i, vl);
    vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
    vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
    vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
    vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
    vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
    count += __riscv_vcpop_m_b4(mask, vl);
    i += vl;
  }
  (void)count;
  return ms_since(t);
}

// ── Pure leaf distance computation (Scalar) — same arithmetic, no vector ──────
static double pure_leaf_scalar(const float *x, const float *y, const float *z,
                                std::size_t n, float qx, float qy, float qz, float r2) {
  auto t = Clock::now();
  std::size_t count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    float dx = x[i]-qx, dy = y[i]-qy, dz = z[i]-qz;
    float d2 = dx*dx + dy*dy + dz*dz;
    if (d2 <= r2) count++;
  }
  (void)count;
  return ms_since(t);
}

// ── Pointer-chasing traversal only (visits nodes, does no distance math) ──────
// Counts leaf visits without computing distances — isolates the tree walk cost.
static double count_nodes_visited(const PointerOctreeNode *root,
                                   const PointXYZ &query, float r2) {
  // box-sphere overlap (same as in pointer_octree.cpp)
  auto box_overlaps = [](const PointerOctreeNode *node, const PointXYZ &q, float r2) {
    float cx = std::max(node->min_x, std::min(q.x, node->max_x));
    float cy = std::max(node->min_y, std::min(q.y, node->max_y));
    float cz = std::max(node->min_z, std::min(q.z, node->max_z));
    float dx = q.x-cx, dy = q.y-cy, dz = q.z-cz;
    return dx*dx + dy*dy + dz*dz <= r2;
  };

  const PointerOctreeNode *stack[128];
  int sp = 0;
  stack[sp++] = root;
  std::size_t leaf_pts = 0;
  std::size_t nodes_visited = 0;

  while (sp > 0) {
    const PointerOctreeNode *curr = stack[--sp];
    if (!box_overlaps(curr, query, r2)) continue;
    nodes_visited++;
    if (curr->is_leaf) {
      leaf_pts += curr->leaf_x.size();
    } else {
      for (int i = 7; i >= 0; --i)
        if (curr->children[i]) stack[sp++] = curr->children[i];
    }
  }
  return (double)nodes_visited;
}

int main(int argc, char **argv) {
  const char *pcd_path = (argc > 1) ? argv[1] : "data/pcd_compressed/0000000020.pcd";
  const float RADIUS = 0.25f;

  printf("=== PointerOctree: traversal vs leaf compute cost split ===\n");
  printf("Radius: %.2f m\n\n", RADIUS);

  std::vector<PointXYZ> raw;
  int n_raw = loadPCD(pcd_path, raw);
  if (n_raw < 0) { fprintf(stderr, "load failed\n"); return 1; }

  std::vector<float> ix(n_raw), iy(n_raw), iz(n_raw);
  for (int i = 0; i < n_raw; ++i) { ix[i]=raw[i].x; iy[i]=raw[i].y; iz[i]=raw[i].z; }
  PointCloudSoA raw_cloud = {ix.data(), iy.data(), iz.data(), (std::size_t)n_raw};

  std::vector<PointXYZ> down(n_raw);
  std::size_t n = voxel_grid_downsamp_rvv_v2(raw_cloud, down.data(), 0.10f);
  down.resize(n);

  std::vector<float> dx(n), dy(n), dz(n);
  for (std::size_t i = 0; i < n; ++i) { dx[i]=down[i].x; dy[i]=down[i].y; dz[i]=down[i].z; }
  PointCloudSoA cloud = {dx.data(), dy.data(), dz.data(), n};

  PointerOctree tree;
  tree.setInputCloud(cloud);
  tree.build();
  const PointerOctreeNode *root = tree.getRoot();

  float r2 = RADIUS * RADIUS;
  printf("Cloud: %zu pts, leaf_size=64, tree depth<=8\n\n", n);

  // ── Measure full radiusSearch (traversal + leaf compute) ─────────────────
  printf("[A] Full radiusSearch (traversal + leaf RVV checks):\n");
  std::vector<int> ni; std::vector<float> nd;
  ni.reserve(128); nd.reserve(128);
  {
    // warmup
    for (int w = 0; w < 200; ++w) {
      ni.clear(); nd.clear();
      PointXYZ q = {down[w].x, down[w].y, down[w].z};
      tree.radiusSearch(q, RADIUS, ni, nd);
    }
    auto t0 = Clock::now();
    std::size_t total_nbrs = 0;
    for (std::size_t i = 0; i < n; ++i) {
      ni.clear(); nd.clear();
      PointXYZ q = {down[i].x, down[i].y, down[i].z};
      tree.radiusSearch(q, RADIUS, ni, nd);
      total_nbrs += ni.size();
    }
    double ms_full = ms_since(t0);
    printf("  Time: %.1f ms  (avg neighbors: %.1f)\n\n", ms_full, (double)total_nbrs / n);
  }

  // ── Measure ONLY tree traversal (pointer chasing), no leaf math ───────────
  printf("[B] Tree traversal only (pointer chasing, box-sphere test):\n");
  {
    auto t0 = Clock::now();
    double total_nodes = 0;
    for (std::size_t i = 0; i < n; ++i) {
      PointXYZ q = {down[i].x, down[i].y, down[i].z};
      total_nodes += count_nodes_visited(root, q, r2);
    }
    double ms_trav = ms_since(t0);
    printf("  Time: %.1f ms  (avg nodes per query: %.1f)\n\n",
           ms_trav, total_nodes / n);
  }

  // ── Measure ONLY flat-array leaf distance computation (no tree) ───────────
  // Best-case: if we had a perfect flat index, leaf checks with RVV would take:
  printf("[C] Flat-array scan — same leaf math, zero traversal (SCALAR):\n");
  {
    // Run 10x to simulate running it n=49900 times with small subsets (avg 14 nbrs)
    // Simulate avg 14 neighbors in 64-point leaf = 1 leaf per query
    auto t0 = Clock::now();
    for (std::size_t i = 0; i < n; ++i) {
      pure_leaf_scalar(dx.data(), dy.data(), dz.data(), 64,
                       down[i].x, down[i].y, down[i].z, r2);
    }
    double ms_flat_sc = ms_since(t0);
    printf("  Time: %.1f ms  (n queries × 64-pt leaf, scalar)\n\n", ms_flat_sc);
  }

  printf("[D] Flat-array scan — same leaf math, zero traversal (RVV):\n");
  {
    auto t0 = Clock::now();
    for (std::size_t i = 0; i < n; ++i) {
      pure_leaf_rvv(dx.data(), dy.data(), dz.data(), 64,
                    down[i].x, down[i].y, down[i].z, r2);
    }
    double ms_flat_rv = ms_since(t0);
    printf("  Time: %.1f ms  (n queries × 64-pt leaf, RVV)\n\n", ms_flat_rv);
  }

  printf("=== Analysis ===\n");
  printf("B/A ratio shows what fraction of radiusSearch is tree traversal.\n");
  printf("If B ≈ A → traversal dominates → RVV leaf checks cannot help much.\n");
  printf("If B << A → leaf compute dominates → RVV leaf checks are the win.\n\n");
  printf("C vs D ratio = raw RVV speedup for leaf distance compute on QEMU.\n");
  printf("On real HW (VLEN=128): expect this ratio to be ~4x larger.\n");
  printf("But traversal cost (B) does NOT benefit from RVV — it's pointer chasing.\n");
  printf("On real HW, pointer chasing cache misses may also increase traversal cost.\n");

  return 0;
}
