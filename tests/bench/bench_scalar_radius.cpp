// tests/bench/bench_scalar_radius.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Benchmark: Scalar brute-force radius search — Q queries on N-point cloud.
// No SIMD, no tree. Pure O(N·Q) linear scan in scalar C++.
// Comparable to bench_octree_radius.cpp and bench_caravan_radius.cpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "core/point_types.h"
#include "bench_params.h"

#include <iostream>
#include <random>
#include <vector>

using namespace rvpoint;
using namespace rvpoint::bench;

int main() {
  std::mt19937 gen(SEED);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  // ── Build database cloud (N points) ───────────────────────────────────────
  std::vector<float> cx(N), cy(N), cz(N);
  for (std::size_t i = 0; i < N; ++i) {
    cx[i] = dist(gen);
    cy[i] = dist(gen);
    cz[i] = dist(gen);
  }
  PointCloudSoA cloud = {cx.data(), cy.data(), cz.data(), N};

  // ── Build query points (Q, same seed continuation) ────────────────────────
  std::vector<PointXYZ> queries(Q);
  for (std::size_t i = 0; i < Q; ++i) {
    queries[i] = {dist(gen), dist(gen), dist(gen)};
  }

  // ── Scalar brute-force: no preprocessing ─────────────────────────────────
  const float r2 = RADIUS * RADIUS;

  const float *px = cloud.x;
  const float *py = cloud.y;
  const float *pz = cloud.z;

  volatile std::size_t total_hits = 0;
  std::vector<int32_t> result_indices;
  result_indices.reserve(256);

  for (std::size_t q = 0; q < Q; ++q) {
    const float qx = queries[q].x;
    const float qy = queries[q].y;
    const float qz = queries[q].z;

    result_indices.clear();
    for (std::size_t p = 0; p < N; ++p) {
      const float dx = px[p] - qx;
      const float dy = py[p] - qy;
      const float dz = pz[p] - qz;
      if (dx * dx + dy * dy + dz * dz <= r2) {
        result_indices.push_back(static_cast<int32_t>(p));
      }
    }
    total_hits += result_indices.size();
  }

  std::cout << "[BENCH] scalar   N=" << N << " Q=" << Q
            << "  total_hits=" << total_hits << std::endl;
  return 0;
}
