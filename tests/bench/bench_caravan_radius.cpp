// tests/bench/bench_caravan_radius.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Benchmark: Caravan RVV batch radius search — Q queries on N-point cloud.
// Comparable to bench_octree_radius.cpp and bench_scalar_radius.cpp.
// ─────────────────────────────────────────────────────────────────────────────
#include "core/point_types.h"
#include "search/caravan_radius_search.h"
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

  // ── Build query cloud (Q points, same seed continuation) ──────────────────
  std::vector<float> qx(Q), qy(Q), qz(Q);
  for (std::size_t i = 0; i < Q; ++i) {
    qx[i] = dist(gen);
    qy[i] = dist(gen);
    qz[i] = dist(gen);
  }
  PointCloudSoA queries = {qx.data(), qy.data(), qz.data(), Q};

  // ── Caravan batch radius search ───────────────────────────────────────────
  CaravanRadiusSearch caravan;
  caravan.setInputCloud(cloud);

  std::vector<std::vector<int32_t>> results;
  caravan.batchRadiusSearch(queries, RADIUS, results);

  // Sink results so the compiler cannot eliminate the calls
  volatile std::size_t total_hits = 0;
  for (std::size_t q = 0; q < Q; ++q) {
    total_hits += results[q].size();
  }

  std::cout << "[BENCH] caravan  N=" << N << " Q=" << Q
            << "  total_hits=" << total_hits << std::endl;
  return 0;
}
