// tests/bench/bench_caravan_radius.cpp
// ─────────────────────────────────────────────────────────────────────────────
// Benchmark: Caravan RVV batch radius search — Q queries on N-point cloud.
// Comparable to bench_octree_radius.cpp (same N, Q, seed, no verification).
//
// Method: N database points in cloud, Q query points in a separate query
// cloud. batchRadiusSearch runs all Q queries in one vectorized pass.
// ─────────────────────────────────────────────────────────────────────────────
#include "rvv_pcl.h"
#include "caravan_radius_search.h"
#include "bench_params.h"

#include <iostream>
#include <random>
#include <vector>

using namespace rvv_pcl;
using namespace rvv_pcl::bench;

int main() {
  std::mt19937 gen(SEED);
  std::normal_distribution<float> dist(0.0f, 2.0f);

  // ── Build database cloud (N points) ───────────────────────────────────────
  auto cloud = std::make_shared<PointCloudSoA>();
  cloud->reserve(N);
  for (std::size_t i = 0; i < N; ++i)
    cloud->push_back({dist(gen), dist(gen), dist(gen)});

  // ── Build query cloud (Q points, same seed continuation as octree bench) ──
  PointCloudSoA queries;
  queries.reserve(Q);
  for (std::size_t i = 0; i < Q; ++i)
    queries.push_back({dist(gen), dist(gen), dist(gen)});

  // ── No index-build step for Caravan (flat scan, no preprocessing) ─────────
  CaravanRadiusSearch caravan;
  caravan.setInputCloud(cloud);
  caravan.setSearchRadius(RADIUS);

  // ── Run all Q queries as one batch ────────────────────────────────────────
  std::vector<std::vector<int32_t>> results;
  caravan.batchRadiusSearch(queries, RADIUS, results);

  // Sink results so the compiler cannot eliminate the calls
  volatile std::size_t total_hits = 0;
  for (std::size_t q = 0; q < Q; ++q)
    total_hits += results[q].size();

  std::cout << "[BENCH] caravan  N=" << N << " Q=" << Q
            << "  total_hits=" << total_hits << std::endl;
  return 0;
}
