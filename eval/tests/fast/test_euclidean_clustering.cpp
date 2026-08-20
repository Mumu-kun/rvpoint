// test_euclidean_clustering.cpp
//
// Unit test for EuclideanClustering.
// Run via:  ./scripts/run.sh euclidean_clustering
//
// Test cases
// ──────────
// 1. Empty cloud  → no clusters produced, no crash.
// 2. Single-point cloud  → one cluster of size 1 (with minClusterSize=1).
// 3. Two well-separated blobs  → exactly 2 clusters, each containing the
//    correct indices verified against a naive brute-force BFS.
// 4. Min/max size filter  → blobs outside the size window are dropped.
// 5. All points in one tight ball  → single cluster containing all N points.
// 6. Result indices are sorted ascending per cluster.

#include "euclidean_clustering.h"
#include "include/rvpoint.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <numeric>
#include <queue>
#include <random>
#include <set>
#include <vector>

using namespace rvpoint;

// ─── helpers ─────────────────────────────────────────────────────────────────

struct SoABuffer {
    std::vector<float> x, y, z;
    PointCloudSoA cloud;
};

static SoABuffer make_cloud(const std::vector<PointXYZ> &pts) {
    SoABuffer buf;
    buf.x.resize(pts.size());
    buf.y.resize(pts.size());
    buf.z.resize(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
        buf.x[i] = pts[i].x;
        buf.y[i] = pts[i].y;
        buf.z[i] = pts[i].z;
    }
    buf.cloud = {buf.x.data(), buf.y.data(), buf.z.data(), pts.size()};
    return buf;
}

// Naive brute-force BFS clustering (reference implementation for correctness
// cross-check).  Returns sorted per-cluster index sets.
static std::vector<std::set<int>> naive_bfs(
    const PointCloudSoA &cloud, float tol, int min_sz, int max_sz
) {
    const std::size_t  n      = cloud.n;
    const float        tol_sq = tol * tol;
    std::vector<bool>  visited(n, false);
    std::vector<std::set<int>> clusters;

    for (std::size_t seed = 0; seed < n; ++seed) {
        if (visited[seed]) continue;
        std::set<int> cluster;
        std::queue<int> q;
        visited[seed] = true;
        q.push(static_cast<int>(seed));
        while (!q.empty()) {
            const int cur = q.front(); q.pop();
            cluster.insert(cur);
            const PointXYZ pc = {cloud.x[cur], cloud.y[cur], cloud.z[cur]};
            for (std::size_t j = 0; j < n; ++j) {
                if (visited[j]) continue;
                const PointXYZ pj = {cloud.x[j], cloud.y[j], cloud.z[j]};
                const float dx = pj.x - pc.x;
                const float dy = pj.y - pc.y;
                const float dz = pj.z - pc.z;
                if (dx*dx + dy*dy + dz*dz <= tol_sq) {
                    visited[j] = true;
                    q.push(static_cast<int>(j));
                }
            }
        }
        const int sz = static_cast<int>(cluster.size());
        if (sz >= min_sz && sz <= max_sz) clusters.push_back(std::move(cluster));
    }
    return clusters;
}

// Convert ClusterIndices result to a vector of sorted sets for comparison.
static std::vector<std::set<int>> to_sets(const std::vector<ClusterIndices> &cs) {
    std::vector<std::set<int>> out;
    out.reserve(cs.size());
    for (const auto &c : cs) out.emplace_back(c.indices.begin(), c.indices.end());
    return out;
}

// ─── test helpers ─────────────────────────────────────────────────────────────

static int g_passed = 0;
static int g_failed = 0;

#define CHECK(cond, msg)                                                   \
    do {                                                                   \
        if (!(cond)) {                                                     \
            std::cerr << "[FAIL] " << msg << "\n";                        \
            ++g_failed;                                                    \
        } else {                                                           \
            std::cout << "[PASS] " << msg << "\n";                        \
            ++g_passed;                                                    \
        }                                                                  \
    } while (false)

// ─── test cases ───────────────────────────────────────────────────────────────

static void test_empty_cloud() {
    PointCloudSoA cloud{}; // empty
    EuclideanClustering ec;
    ec.setInputCloud(cloud);
    ec.setClusterTolerance(0.1f);
    ec.setMinClusterSize(1);
    const auto clusters = ec.extract();
    CHECK(clusters.empty(), "Empty cloud → no clusters");
}

static void test_single_point() {
    auto buf = make_cloud({{1.0f, 2.0f, 3.0f}});
    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(0.5f);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(100);
    const auto clusters = ec.extract();
    CHECK(clusters.size() == 1, "Single point → 1 cluster");
    CHECK(!clusters.empty() && clusters[0].indices.size() == 1,
          "Single-point cluster has exactly 1 index");
}

static void test_two_separated_blobs() {
    // Build two blobs of 50 points each, clearly separated in X.
    constexpr int BLOB_SIZE = 50;
    constexpr float SPREAD  = 0.04f;  // tight: all neighbours within 0.1 m
    constexpr float GAP     = 5.0f;   // blobs 5 m apart

    std::mt19937 gen(7);
    std::uniform_real_distribution<float> jitter(-SPREAD, SPREAD);

    std::vector<PointXYZ> pts;
    pts.reserve(BLOB_SIZE * 2);
    for (int i = 0; i < BLOB_SIZE; ++i)
        pts.push_back({jitter(gen), jitter(gen), jitter(gen)});          // blob A near origin
    for (int i = 0; i < BLOB_SIZE; ++i)
        pts.push_back({GAP + jitter(gen), jitter(gen), jitter(gen)});    // blob B far in X

    auto buf = make_cloud(pts);

    constexpr float TOL = 0.15f;
    constexpr int   MIN = 1;
    constexpr int   MAX = 10000;

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(TOL);
    ec.setMinClusterSize(MIN);
    ec.setMaxClusterSize(MAX);
    const auto clusters = ec.extract();

    // Reference
    const auto ref = naive_bfs(buf.cloud, TOL, MIN, MAX);

    CHECK(clusters.size() == 2, "Two-blob cloud → exactly 2 clusters");
    CHECK(clusters.size() == ref.size(),
          "Two-blob: cluster count matches naive BFS");

    // Sort both result sets by their minimum index so we can compare pairwise.
    auto imp_sets = to_sets(clusters);
    std::sort(imp_sets.begin(), imp_sets.end(),
              [](const std::set<int> &a, const std::set<int> &b) {
                  return *a.begin() < *b.begin();
              });
    auto ref_sets = ref;
    std::sort(ref_sets.begin(), ref_sets.end(),
              [](const std::set<int> &a, const std::set<int> &b) {
                  return *a.begin() < *b.begin();
              });

    bool match = (imp_sets == ref_sets);
    CHECK(match, "Two-blob: cluster membership matches naive BFS exactly");

    // Each blob should have exactly BLOB_SIZE members
    if (!clusters.empty()) {
        const bool blob_a_ok = (clusters[0].indices.size() == static_cast<std::size_t>(BLOB_SIZE) ||
                                clusters[1].indices.size() == static_cast<std::size_t>(BLOB_SIZE));
        CHECK(blob_a_ok, "Each blob cluster contains exactly BLOB_SIZE points");
    }
}

static void test_size_filter_min() {
    // Three blobs: sizes 5, 20, 5. Ask for min=10 → only the blob of 20 survives.
    constexpr float SPREAD = 0.04f;
    constexpr float GAP    = 5.0f;

    std::mt19937 gen(99);
    std::uniform_real_distribution<float> jitter(-SPREAD, SPREAD);

    std::vector<PointXYZ> pts;
    // blob A: 5 points at X=0
    for (int i = 0; i < 5; ++i) pts.push_back({jitter(gen), jitter(gen), jitter(gen)});
    // blob B: 20 points at X=GAP
    for (int i = 0; i < 20; ++i) pts.push_back({GAP + jitter(gen), jitter(gen), jitter(gen)});
    // blob C: 5 points at X=2*GAP
    for (int i = 0; i < 5; ++i) pts.push_back({2*GAP + jitter(gen), jitter(gen), jitter(gen)});

    auto buf = make_cloud(pts);

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(0.15f);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(100);
    const auto clusters = ec.extract();

    CHECK(clusters.size() == 1, "Size filter (min=10): only the 20-point blob survives");
    if (!clusters.empty()) {
        CHECK(clusters[0].indices.size() == 20,
              "Size filter: surviving cluster has 20 points");
    }
}

static void test_size_filter_max() {
    // Two blobs: sizes 30 and 5. Ask for max=10 → only the small blob survives.
    constexpr float SPREAD = 0.04f;
    constexpr float GAP    = 5.0f;

    std::mt19937 gen(123);
    std::uniform_real_distribution<float> jitter(-SPREAD, SPREAD);

    std::vector<PointXYZ> pts;
    for (int i = 0; i < 30; ++i) pts.push_back({jitter(gen), jitter(gen), jitter(gen)});
    for (int i = 0; i < 5;  ++i) pts.push_back({GAP + jitter(gen), jitter(gen), jitter(gen)});

    auto buf = make_cloud(pts);

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(0.15f);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(10);
    const auto clusters = ec.extract();

    CHECK(clusters.size() == 1, "Size filter (max=10): only the 5-point blob survives");
    if (!clusters.empty()) {
        CHECK(clusters[0].indices.size() == 5,
              "Size filter: surviving cluster has 5 points");
    }
}

static void test_all_one_cluster() {
    // 200 points drawn from a tight Gaussian: all should end up in one cluster.
    constexpr std::size_t N = 200;
    std::mt19937 gen(42);
    std::normal_distribution<float> dist(0.0f, 0.02f);

    std::vector<PointXYZ> pts;
    pts.reserve(N);
    for (std::size_t i = 0; i < N; ++i)
        pts.push_back({dist(gen), dist(gen), dist(gen)});

    auto buf = make_cloud(pts);

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(0.2f);  // generous tolerance
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(10000);
    const auto clusters = ec.extract();

    CHECK(clusters.size() == 1, "Tight ball → exactly 1 cluster");
    if (!clusters.empty()) {
        CHECK(clusters[0].indices.size() == N,
              "Tight ball: single cluster contains all N points");
    }
}

static void test_indices_sorted() {
    // Verify that each returned cluster's indices are in ascending order.
    constexpr int BLOB_SIZE = 30;
    constexpr float SPREAD  = 0.04f;
    constexpr float GAP     = 5.0f;

    std::mt19937 gen(3);
    std::uniform_real_distribution<float> jitter(-SPREAD, SPREAD);

    std::vector<PointXYZ> pts;
    for (int i = 0; i < BLOB_SIZE; ++i) pts.push_back({jitter(gen), jitter(gen), jitter(gen)});
    for (int i = 0; i < BLOB_SIZE; ++i) pts.push_back({GAP + jitter(gen), jitter(gen), jitter(gen)});

    auto buf = make_cloud(pts);

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(0.15f);
    ec.setMinClusterSize(1);
    ec.setMaxClusterSize(10000);
    const auto clusters = ec.extract();

    bool all_sorted = true;
    for (const auto &c : clusters) {
        if (!std::is_sorted(c.indices.begin(), c.indices.end())) {
            all_sorted = false;
            break;
        }
    }
    CHECK(all_sorted, "All cluster index lists are sorted ascending");
}

static void test_tolerance_boundary() {
    // Two points exactly at distance D apart.  Test with tol just below and
    // just above D to confirm boundary behaviour.
    const float D = 1.0f;
    auto buf = make_cloud({{0.0f, 0.0f, 0.0f}, {D, 0.0f, 0.0f}});

    // tol < D → 2 separate clusters
    {
        EuclideanClustering ec;
        ec.setInputCloud(buf.cloud);
        ec.setClusterTolerance(D * 0.999f);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(100);
        const auto clusters = ec.extract();
        CHECK(clusters.size() == 2, "Tol just below D → 2 separate clusters");
    }

    // tol >= D → 1 merged cluster  (d2 == r2 is in-range, check uses <=)
    {
        EuclideanClustering ec;
        ec.setInputCloud(buf.cloud);
        ec.setClusterTolerance(D);
        ec.setMinClusterSize(1);
        ec.setMaxClusterSize(100);
        const auto clusters = ec.extract();
        CHECK(clusters.size() == 1, "Tol == D → 1 merged cluster (boundary inclusive)");
    }
}

static void test_against_naive_random() {
    // Random small cloud: exhaustively compare output against naive BFS.
    constexpr std::size_t N = 300;
    constexpr float TOL = 0.3f;
    constexpr int   MIN = 1;
    constexpr int   MAX = 1000;

    std::mt19937 gen(55);
    std::uniform_real_distribution<float> pos(-1.0f, 1.0f);

    std::vector<PointXYZ> pts;
    pts.reserve(N);
    for (std::size_t i = 0; i < N; ++i)
        pts.push_back({pos(gen), pos(gen), pos(gen)});

    auto buf = make_cloud(pts);

    EuclideanClustering ec;
    ec.setInputCloud(buf.cloud);
    ec.setClusterTolerance(TOL);
    ec.setMinClusterSize(MIN);
    ec.setMaxClusterSize(MAX);
    const auto clusters = ec.extract();

    const auto ref = naive_bfs(buf.cloud, TOL, MIN, MAX);

    // Build canonical sets and sort by min element for comparison
    auto imp_sets = to_sets(clusters);
    auto ref_sets = ref;
    auto by_min = [](const std::set<int> &a, const std::set<int> &b) {
        return *a.begin() < *b.begin();
    };
    std::sort(imp_sets.begin(), imp_sets.end(), by_min);
    std::sort(ref_sets.begin(), ref_sets.end(), by_min);

    CHECK(imp_sets.size() == ref_sets.size(),
          "Random cloud: cluster count matches naive BFS");
    CHECK(imp_sets == ref_sets,
          "Random cloud: cluster membership matches naive BFS exactly");
}

// ─── main ─────────────────────────────────────────────────────────────────────

int main() {
    std::cout << "=== EuclideanClustering Unit Tests ===\n\n" << std::flush;

    test_empty_cloud(); std::cout << std::flush;
    test_single_point(); std::cout << std::flush;
    test_two_separated_blobs(); std::cout << std::flush;
    test_size_filter_min(); std::cout << std::flush;
    test_size_filter_max(); std::cout << std::flush;
    test_all_one_cluster(); std::cout << std::flush;
    test_indices_sorted(); std::cout << std::flush;
    test_tolerance_boundary(); std::cout << std::flush;
    test_against_naive_random(); std::cout << std::flush;

    std::cout << "\n------------------------------------------\n";
    std::cout << "Results: " << g_passed << " passed, " << g_failed << " failed.\n" << std::flush;

    return (g_failed == 0) ? 0 : 1;
}
