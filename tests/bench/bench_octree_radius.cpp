// tests/bench/bench_octree_radius.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Benchmark: Octree radius search — Q queries on N-point cloud.
// Comparable to bench_caravan_radius.cpp (same N, Q, seed, no verification).
//
// Method: append Q query points to the cloud, buildTree once, then call
// radiusSearch(q_index) for each query — matching CaravanRadiusSearch API.
// ─────────────────────────────────────────────────────────────────────────────
#include "rvv_pcl.h"
#include "bench_params.h"

#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;
using namespace rvv_pcl::bench;

int main() {
  std::mt19937 gen(SEED);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  // ── Build cloud (N database + Q query points appended at the end) ──────────
  PointCloudSoA cloud;
  cloud.reserve(N + Q);
  for (std::size_t i = 0; i < N + Q; ++i)
    cloud.push_back({dist(gen), dist(gen), dist(gen)});

  // ── Build Octree once ──────────────────────────────────────────────────────
  OctreeNeighborSearch octree;
  octree.setInputCloud(cloud);
  octree.setSearchRadius(RADIUS);
  octree.buildTree();

  // ── Q radius searches (query indices = N..N+Q-1) ──────────────────────────
  volatile std::size_t total_hits = 0;
  std::vector<int> result_indices;

  for (std::size_t q = 0; q < Q; ++q) {
    int query_idx = static_cast<int>(N + q);
    octree.radiusSearch(query_idx, result_indices, nullptr, 0);
    total_hits += result_indices.size();
  }

  std::cout << "[BENCH] octree   N=" << N << " Q=" << Q
            << "  total_hits=" << total_hits << std::endl;
  return 0;
}
