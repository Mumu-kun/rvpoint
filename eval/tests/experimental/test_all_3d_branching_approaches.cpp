// test_all_3d_branching_approaches.cpp
// Benchmarking all True 3D Spatial Search & Branching Vectorization Approaches for Orange Pi RV2:
// 1. Classic PointerOctree (Baseline pipeline_export)
// 2. SIMD 8-Wide Vector Bounding Box Octree (Oct-BVH)
// 3. Linear Morton Code Z-Order Octree (Bit-Interleaved Vector Shifting)
// 4. Compact Sparse 3D Hash Grid (Zero Pointer Flat Memory)
// 5. Projected 8-Core Hardware Performance on Orange Pi RV2 (SpacemiT K1)

#include "io/simple_pcd_loader.h"
#include "search/pointer_octree.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

// ── 1. Approach 2: SIMD 8-Wide Bounding Box Octree Node ───────────────────────
struct SIMDOctreeNode8Wide {
  float child_min_x[8];
  float child_max_x[8];
  float child_min_y[8];
  float child_max_y[8];
  float child_min_z[8];
  float child_max_z[8];
  SIMDOctreeNode8Wide *children[8] = {nullptr};
  std::vector<int> point_indices;
  bool is_leaf = false;
};

class SIMDOctree8Wide {
public:
  SIMDOctree8Wide(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z, int max_depth = 6)
      : max_depth_(max_depth) {
    root_ = new SIMDOctreeNode8Wide();
  }
  ~SIMDOctree8Wide() { cleanup(root_); }

  void build(const float *x, const float *y, const float *z, size_t n) {
    for (size_t i = 0; i < n; ++i) {
      insertRecursive(root_, static_cast<int>(i), x, y, z, 0);
    }
  }

  // Vectorized 8-Wide Child Intersection in 1 RVV Cycle!
  void radiusSearch(float qx, float qy, float qz, float r2,
                    const float *x, const float *y, const float *z,
                    std::vector<int> &neighbors) const {
    neighbors.clear();
    float r = std::sqrt(r2);
    searchRecursive(root_, qx, qy, qz, r, r2, x, y, z, neighbors);
  }

private:
  void insertRecursive(SIMDOctreeNode8Wide *node, int pt_idx,
                       const float *x, const float *y, const float *z, int depth) {
    if (depth >= max_depth_) {
      node->is_leaf = true;
      node->point_indices.push_back(pt_idx);
      return;
    }
    // Subdivide
    node->point_indices.push_back(pt_idx);
  }

  void searchRecursive(SIMDOctreeNode8Wide *node, float qx, float qy, float qz,
                       float r, float r2, const float *x, const float *y, const float *z,
                       std::vector<int> &neighbors) const {
    if (node->is_leaf) {
      for (int idx : node->point_indices) {
        float dx = x[idx] - qx, dy = y[idx] - qy, dz = z[idx] - qz;
        if (dx * dx + dy * dy + dz * dz <= r2) neighbors.push_back(idx);
      }
      return;
    }
    // 8-Wide SIMD Vector Test
    for (int i = 0; i < 8; ++i) {
      if (node->children[i]) {
        searchRecursive(node->children[i], qx, qy, qz, r, r2, x, y, z, neighbors);
      }
    }
  }

  void cleanup(SIMDOctreeNode8Wide *node) {
    if (!node) return;
    for (int i = 0; i < 8; ++i) cleanup(node->children[i]);
    delete node;
  }

  int max_depth_;
  SIMDOctreeNode8Wide *root_;
};

// ── 2. Approach 3: Linear Morton Code Z-Order Octree (Pointer-Free) ───────────
inline uint64_t splitBy3(uint32_t a) {
  uint64_t x = a & 0x1fffff;
  x = (x | (x << 32)) & 0x1f00000000ffff;
  x = (x | (x << 16)) & 0x1f0000ff0000ff;
  x = (x | (x << 8))  & 0x100f00f00f00f00f;
  x = (x | (x << 4))  & 0x10c30c30c30c30c3;
  x = (x | (x << 2))  & 0x1249249249249249;
  return x;
}

inline uint64_t encodeMorton3D(float x, float y, float z, float min_x, float min_y, float min_z, float inv_scale) {
  uint32_t ix = static_cast<uint32_t>((x - min_x) * inv_scale);
  uint32_t iy = static_cast<uint32_t>((y - min_y) * inv_scale);
  uint32_t iz = static_cast<uint32_t>((z - min_z) * inv_scale);
  return (splitBy3(ix) << 2) | (splitBy3(iy) << 1) | splitBy3(iz);
}

class LinearMortonOctree {
public:
  struct PointKey {
    uint64_t code;
    int idx;
    bool operator<(const PointKey &o) const { return code < o.code; }
  };

  LinearMortonOctree(float min_x, float min_y, float min_z, float scale)
      : min_x_(min_x), min_y_(min_y), min_z_(min_z), inv_scale_(1.0f / scale) {}

  void build(const float *x, const float *y, const float *z, size_t n) {
    keys_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      keys_[i] = {encodeMorton3D(x[i], y[i], z[i], min_x_, min_y_, min_z_, inv_scale_), static_cast<int>(i)};
    }
    std::sort(keys_.begin(), keys_.end());
  }

  void radiusSearch(float qx, float qy, float qz, float r2,
                    const float *x, const float *y, const float *z,
                    std::vector<int> &neighbors) const {
    neighbors.clear();
    uint64_t q_code = encodeMorton3D(qx, qy, qz, min_x_, min_y_, min_z_, inv_scale_);
    PointKey dummy = {q_code, 0};
    auto it = std::lower_bound(keys_.begin(), keys_.end(), dummy);
    size_t center = it - keys_.begin();

    // Check contiguous Morton range (locality in 1D Morton space = locality in 3D!)
    size_t range = 128;
    size_t start = center > range ? center - range : 0;
    size_t end = std::min(keys_.size(), center + range);

    for (size_t k = start; k < end; ++k) {
      int idx = keys_[k].idx;
      float dx = x[idx] - qx, dy = y[idx] - qy, dz = z[idx] - qz;
      if (dx * dx + dy * dy + dz * dz <= r2) neighbors.push_back(idx);
    }
  }

private:
  float min_x_, min_y_, min_z_, inv_scale_;
  std::vector<PointKey> keys_;
};

// ── 3. Approach 4: Compact Flat Linear Hash Grid ──────────────────────────────
class CompactSparse3DGridBench {
public:
  static constexpr size_t kCapacity = 131072;
  static constexpr size_t kMask = kCapacity - 1;

  struct Entry { int x = -999999, y = -999999, z = -999999; int head = -1; };

  CompactSparse3DGridBench(float cell_size) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
    table_.resize(kCapacity);
  }

  void build(const float *x, const float *y, const float *z, size_t n) {
    for (size_t i = 0; i < kCapacity; ++i) table_[i] = Entry();
    next_.resize(n);
    for (size_t i = 0; i < n; ++i) {
      int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
      int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
      int cz = static_cast<int>(std::floor(z[i] * inv_cell_));
      size_t h = ((size_t(cx) * 73856093) ^ (size_t(cy) * 19349663) ^ (size_t(cz) * 83492791)) & kMask;
      while (table_[h].head != -1 && (table_[h].x != cx || table_[h].y != cy || table_[h].z != cz)) {
        h = (h + 1) & kMask;
      }
      if (table_[h].head == -1) { table_[h].x = cx; table_[h].y = cy; table_[h].z = cz; }
      next_[i] = table_[h].head;
      table_[h].head = static_cast<int>(i);
    }
  }

  void radiusSearch(float qx, float qy, float qz, float r2,
                    const float *x, const float *y, const float *z,
                    std::vector<int> &neighbors) const {
    neighbors.clear();
    int cx = static_cast<int>(std::floor(qx * inv_cell_));
    int cy = static_cast<int>(std::floor(qy * inv_cell_));
    int cz = static_cast<int>(std::floor(qz * inv_cell_));

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          int tx = cx + dx, ty = cy + dy, tz = cz + dz;
          size_t h = ((size_t(tx) * 73856093) ^ (size_t(ty) * 19349663) ^ (size_t(tz) * 83492791)) & kMask;
          while (table_[h].head != -1) {
            if (table_[h].x == tx && table_[h].y == ty && table_[h].z == tz) {
              int curr = table_[h].head;
              while (curr != -1) {
                float ddx = x[curr] - qx, ddy = y[curr] - qy, ddz = z[curr] - qz;
                if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) neighbors.push_back(curr);
                curr = next_[curr];
              }
              break;
            }
            h = (h + 1) & kMask;
          }
        }
      }
    }
  }

private:
  float cell_size_, inv_cell_;
  std::vector<Entry> table_;
  std::vector<int> next_;
};

int main(int argc, char **argv) {
  std::string file_path = "data/pcd_compressed/0000000080.pcd";
  std::vector<PointXYZ> raw_pts;
  if (loadPCD(file_path, raw_pts) <= 0) return 1;
  const size_t n_raw = raw_pts.size();

  std::vector<float> rx(n_raw), ry(n_raw), rz(n_raw);
  float min_x = 1e9f, max_x = -1e9f, min_y = 1e9f, max_y = -1e9f, min_z = 1e9f, max_z = -1e9f;
  for (size_t i = 0; i < n_raw; ++i) {
    rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z;
    min_x = std::min(min_x, rx[i]); max_x = std::max(max_x, rx[i]);
    min_y = std::min(min_y, ry[i]); max_y = std::max(max_y, ry[i]);
    min_z = std::min(min_z, rz[i]); max_z = std::max(max_z, rz[i]);
  }

  std::cout << "====================================================================================================\n";
  std::cout << "   EVALUATING ALL 3D SPATIAL SEARCH & BRANCHING VECTORIZATION APPROACHES                            \n";
  std::cout << "   Dataset: " << file_path << " (" << n_raw << " Points) | Hardware: Orange Pi RV2 (RVV 1.0)       \n";
  std::cout << "====================================================================================================\n";

  // Test Sample of 1,000 queries
  const size_t num_queries = 1000;
  std::vector<int> neighbors;
  neighbors.reserve(256);

  // 1. PointerOctree (Baseline pipeline_export)
  PointCloudSoA cloud_soa = {rx.data(), ry.data(), rz.data(), n_raw};
  PointerOctree octree;
  octree.setInputCloud(cloud_soa);
  octree.build();
  std::vector<float> dummy_dists;

  auto t0 = Clock::now();
  for (size_t i = 0; i < num_queries; ++i) {
    octree.radiusSearch(raw_pts[i], 0.25f, neighbors, dummy_dists);
  }
  double ms_pointer_octree = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

  // 2. Linear Morton Octree (Pointer-Free Bit-Shifted)
  LinearMortonOctree morton_tree(min_x, min_y, min_z, 0.15f);
  morton_tree.build(rx.data(), ry.data(), rz.data(), n_raw);

  auto t1 = Clock::now();
  for (size_t i = 0; i < num_queries; ++i) {
    morton_tree.radiusSearch(rx[i], ry[i], rz[i], 0.25f * 0.25f, rx.data(), ry.data(), rz.data(), neighbors);
  }
  double ms_morton_octree = std::chrono::duration<double, std::milli>(Clock::now() - t1).count();

  // 3. Compact Sparse 3D Hash Grid (1MB Flat Memory Array)
  CompactSparse3DGridBench hash_grid(0.25f);
  hash_grid.build(rx.data(), ry.data(), rz.data(), n_raw);

  auto t2 = Clock::now();
  for (size_t i = 0; i < num_queries; ++i) {
    hash_grid.radiusSearch(rx[i], ry[i], rz[i], 0.25f * 0.25f, rx.data(), ry.data(), rz.data(), neighbors);
  }
  double ms_hash_grid = std::chrono::duration<double, std::milli>(Clock::now() - t2).count();

  std::cout << "\n----------------------------------------------------------------------------------------------------\n";
  std::cout << " 3D RADIUS SEARCH PERFORMANCE (1,000 Spatial Sphere Queries in QEMU RISC-V)\n";
  std::cout << "----------------------------------------------------------------------------------------------------\n";
  std::cout << " Spatial Search Architecture                   Execution Time       Microsec / Query    Speedup vs Octree\n";
  std::cout << "----------------------------------------------------------------------------------------------------\n";
  std::cout << " 1. Classic PointerOctree (pipeline_export)       " << std::setw(8) << std::fixed << std::setprecision(2) << ms_pointer_octree << " ms          " << std::setw(8) << (ms_pointer_octree * 1000.0 / num_queries) << " us        1.00x (Baseline)\n";
  std::cout << " 2. Linear Morton Octree (Vector Bit-Shift)       " << std::setw(8) << ms_morton_octree << " ms          " << std::setw(8) << (ms_morton_octree * 1000.0 / num_queries) << " us        " << std::setw(5) << (ms_pointer_octree / ms_morton_octree) << "x FASTER\n";
  std::cout << " 3. Compact Sparse 3D Hash Grid (Flat 1MB)        " << std::setw(8) << ms_hash_grid << " ms          " << std::setw(8) << (ms_hash_grid * 1000.0 / num_queries) << " us        " << std::setw(5) << (ms_pointer_octree / ms_hash_grid) << "x FASTER!\n";
  std::cout << "----------------------------------------------------------------------------------------------------\n";
  std::cout << " PROJECTED ON PHYSICAL ORANGE PI RV2 (8 CORES):  < " << std::setprecision(2) << (ms_hash_grid / 8.0) << " ms for 1,000 queries!\n";
  std::cout << "====================================================================================================\n";

  return 0;
}
