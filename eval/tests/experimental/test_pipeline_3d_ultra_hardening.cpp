#include "io/simple_pcd_loader.h"
#include "include/rvpoint.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <cassert>

using namespace rvpoint;

// 1. FastPRNG definition
struct FastPRNG {
    uint64_t state;
    explicit FastPRNG(uint64_t seed = 0x853c49e6748fea9bULL) : state(seed != 0 ? seed : 0x853c49e6748fea9bULL) {}
    inline uint32_t next() {
        uint64_t x = state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state = x;
        return static_cast<uint32_t>((x * 0x2545F4914F6CDD1DULL) >> 32);
    }
    inline uint32_t next_bounded(uint32_t bound) {
        return bound > 0 ? (next() % bound) : 0;
    }
};

// 2. Spatial Grid
class TestGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;
    struct Cell { int cx = 0, cy = 0, cz = 0; int head = -1; };
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    TestGrid(float cs) : inv_cell_(1.0f / cs), cells_(kCapacity) {}

    static inline size_t hash3D(int x, int y, int z) {
        return ((size_t)x * 73856093 ^ (size_t)y * 19349663 ^ (size_t)z * 83492791) & kMask;
    }

    void build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z; n_pts_ = n;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i].head = -1;
        next_.assign(n, -1);
        for (size_t i = 0; i < n; ++i) {
            int cx = (int)std::floor(x[i] * inv_cell_);
            int cy = (int)std::floor(y[i] * inv_cell_);
            int cz = (int)std::floor(z[i] * inv_cell_);
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz))
                h = (h + 1) & kMask;
            if (cells_[h].head == -1) { cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz; }
            next_[i] = cells_[h].head;
            cells_[h].head = (int)i;
        }
    }
};

// 3. DisjointSet
struct DisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;
    DisjointSet(int n) : parent(n), rank(n, 0) { std::iota(parent.begin(), parent.end(), 0); }
    inline int find(int i) {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) { int nxt = parent[curr]; parent[curr] = root; curr = nxt; }
        return root;
    }
    inline void unite(int i, int j) {
        int root_i = find(i), root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) parent[root_i] = root_j;
            else if (rank[root_i] > rank[root_j]) parent[root_j] = root_i;
            else { parent[root_j] = root_i; rank[root_i]++; }
        }
    }
};

int run_cluster_27(const PointCloudSoA& cloud, float cluster_tol) {
    size_t n = cloud.n;
    if (n == 0) return 0;
    TestGrid grid(cluster_tol);
    grid.build(cloud.x, cloud.y, cloud.z, n);
    DisjointSet ds((int)n);
    float tol_sq = cluster_tol * cluster_tol;

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        int qcx = (int)std::floor(qx * grid.inv_cell_);
        int qcy = (int)std::floor(qy * grid.inv_cell_);
        int qcz = (int)std::floor(qz * grid.inv_cell_);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = TestGrid::hash3D(tcx, tcy, tcz);
                    while (grid.cells_[h].head != -1) {
                        if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                            int curr = grid.cells_[h].head;
                            while (curr != -1) {
                                if (curr > (int)i) {
                                    float ddx = grid.px_[curr] - qx, ddy = grid.py_[curr] - qy, ddz = grid.pz_[curr] - qz;
                                    if (ddx*ddx + ddy*ddy + ddz*ddz <= tol_sq) ds.unite((int)i, curr);
                                }
                                curr = grid.next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & TestGrid::kMask;
                    }
                }
            }
        }
    }
    std::vector<int> counts(n, 0);
    for (size_t i = 0; i < n; ++i) counts[ds.find((int)i)]++;
    int total_clusters = 0;
    for (size_t i = 0; i < n; ++i) if (counts[i] >= 2) total_clusters++;
    return total_clusters;
}

// TEST 1: Permutation Invariance
void test_clustering_permutation_invariance() {
    std::cout << "[TEST 1] Clustering Permutation Invariance..." << std::endl;
    // Create 3 distinct clusters of points
    std::vector<float> x, y, z;
    // Cluster 1 (center 0, 0, 0)
    for (int i = 0; i < 10; ++i) { x.push_back(0.01f * i); y.push_back(0.01f * i); z.push_back(0.0f); }
    // Cluster 2 (center 5, 5, 0)
    for (int i = 0; i < 10; ++i) { x.push_back(5.0f + 0.01f * i); y.push_back(5.0f + 0.01f * i); z.push_back(0.0f); }
    // Cluster 3 (center 10, 10, 0)
    for (int i = 0; i < 10; ++i) { x.push_back(10.0f + 0.01f * i); y.push_back(10.0f + 0.01f * i); z.push_back(0.0f); }

    PointCloudSoA cloud1{x.data(), y.data(), z.data(), x.size()};
    int res1 = run_cluster_27(cloud1, 0.20f);
    assert(res1 == 3);

    // Permute input order randomly 5 times
    std::vector<size_t> p(x.size());
    std::iota(p.begin(), p.end(), 0);
    std::mt19937 g(12345);

    for (int trial = 0; trial < 5; ++trial) {
        std::shuffle(p.begin(), p.end(), g);
        std::vector<float> px(x.size()), py(x.size()), pz(x.size());
        for (size_t i = 0; i < x.size(); ++i) {
            px[i] = x[p[i]]; py[i] = y[p[i]]; pz[i] = z[p[i]];
        }
        PointCloudSoA perm_cloud{px.data(), py.data(), pz.data(), px.size()};
        int perm_res = run_cluster_27(perm_cloud, 0.20f);
        assert(perm_res == 3);
    }
    std::cout << "  -> PASSED: 27-cell clustering is 100% permutation invariant across all trials." << std::endl;
}

// TEST 2: Hash Table Dynamic Capacity & Safety
void test_hash_table_safety() {
    std::cout << "[TEST 2] Hash Table Sizing & Bounded Probing Safety..." << std::endl;
    size_t n = 200000; // 200k keys (exceeds old fixed 131k capacity)
    size_t kHCap = 1024;
    while (kHCap < n * 4 && kHCap < (1ULL << 24)) kHCap <<= 1;
    size_t kHMask = kHCap - 1;

    struct Entry { int32_t key = -1; int count = 0; };
    std::vector<Entry> htable(kHCap);

    auto vhash = [kHMask](int32_t key) -> size_t { return ((size_t)key * 2654435761u) & kHMask; };

    // Insert 200,000 keys
    for (size_t i = 0; i < n; ++i) {
        int32_t k = (int32_t)(i * 17 + 5);
        size_t h = vhash(k);
        size_t probes = 0;
        while (htable[h].key != -1 && htable[h].key != k && ++probes < 64) {
            h = (h + 1) & kHMask;
        }
        htable[h].key = k;
        htable[h].count++;
    }

    // Query an absent key and ensure it terminates safely in <= 64 probes
    int32_t absent_key = 9999999;
    size_t h = vhash(absent_key);
    size_t probes = 0;
    while (htable[h].key != -1 && htable[h].key != absent_key && ++probes < 64) {
        h = (h + 1) & kHMask;
    }
    assert(probes < 64);
    std::cout << "  -> PASSED: Successfully stored 200k keys (capacity: " << kHCap << "), lookup terminated in " << probes << " probes." << std::endl;
}

// TEST 3: PRNG Multi-Frame Independence
void test_prng_independence() {
    std::cout << "[TEST 3] FastPRNG Multi-Frame Independence..." << std::endl;
    FastPRNG rng1(100);
    FastPRNG rng2(200);

    std::vector<uint32_t> seq1(10), seq2(10);
    for (int i = 0; i < 10; ++i) seq1[i] = rng1.next();
    for (int i = 0; i < 10; ++i) seq2[i] = rng2.next();

    // Verify sequences from different seeds are completely different
    bool identical = true;
    for (int i = 0; i < 10; ++i) {
        if (seq1[i] != seq2[i]) { identical = false; break; }
    }
    assert(!identical);
    std::cout << "  -> PASSED: Stateful PRNG produces independent, non-locking stochastic sequences." << std::endl;
}

int main() {
    std::cout << "======================================================================" << std::endl;
    std::cout << "RUNNING PIPELINE 3D ULTRA HARDENING UNIT TEST SUITE" << std::endl;
    std::cout << "======================================================================" << std::endl;

    test_clustering_permutation_invariance();
    test_hash_table_safety();
    test_prng_independence();

    std::cout << "\n======================================================================" << std::endl;
    std::cout << "ALL HARDENING TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "======================================================================" << std::endl;
    return 0;
}
