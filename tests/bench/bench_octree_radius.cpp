// tests/bench/bench_octree_radius.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Benchmark: Octree radius search — Q queries on N-point cloud.
// Comparable to bench_caravan_radius.cpp and bench_scalar_radius.cpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "core/point_types.h"
#include "search/octree.h"
#include "bench_params.h"

#include <iostream>
#include <random>
#include <vector>

using namespace rvpoint;
using namespace rvpoint::bench;

int main() {
  std::mt19937 gen(SEED);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  // ── Build database cloud (N points) in SoA vectors ────────────────────────
  std::vector<float> cx(N), cy(N), cz(N);
  for (std::size_t i = 0; i < N; ++i) {
    cx[i] = dist(gen);
    cy[i] = dist(gen);
    cz[i] = dist(gen);
  }
  PointCloudSoA cloud = {cx.data(), cy.data(), cz.data(), N};

  // ── Build query points (Q points, same seed continuation) ─────────────────
  std::vector<PointXYZ> queries(Q);
  for (std::size_t i = 0; i < Q; ++i) {
    queries[i] = {dist(gen), dist(gen), dist(gen)};
  }

  // ── Build Octree ──────────────────────────────────────────────────────────
  Octree octree;
  octree.setInputCloud(cloud);
  octree.build();

  // ── Run Q radius searches ─────────────────────────────────────────────────
  volatile std::size_t total_hits = 0;
  std::vector<int> result_indices;
  std::vector<float> result_dists;

  for (std::size_t q = 0; q < Q; ++q) {
    octree.radiusSearch(queries[q], RADIUS, result_indices, result_dists);
    total_hits += result_indices.size();
  }

  std::cout << "[BENCH] octree   N=" << N << " Q=" << Q
            << "  total_hits=" << total_hits << std::endl;
  return 0;
}
