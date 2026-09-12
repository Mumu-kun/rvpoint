#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

struct PointCloudSoA {
    const float* x;
    const float* y;
    const float* z;
    size_t n;
};

// Test 1: Verify the 14-halfspace + curr > i Clustering Completeness Bug
void verify_clustering_bug() {
    std::cout << "--- Verifying Issue 1: Clustering Completeness Bug ---" << std::endl;
    
    // Create 2 points that are within cluster_tol (0.15m) but in adjacent cells
    // Cell size = 0.15m
    // Point A: x = 0.14, y = 0, z = 0 (Cell 0: [0.0, 0.15))
    // Point B: x = 0.16, y = 0, z = 0 (Cell 1: [0.15, 0.30))
    // Distance = 0.02m <= 0.15m -> MUST be in the SAME cluster!
    
    // Order in array: Point B first (index 0), Point A second (index 1)
    // Point 0 is in Cell 1 (higher cell)
    // Point 1 is in Cell 0 (lower cell)
    std::vector<float> x = {0.16f, 0.14f};
    std::vector<float> y = {0.00f, 0.00f};
    std::vector<float> z = {0.00f, 0.00f};
    PointCloudSoA cloud{x.data(), y.data(), z.data(), 2};
    
    float cluster_tol = 0.15f;
    float tol_sq = cluster_tol * cluster_tol;
    
    // Simulate the exact code from pipeline_3d_ultra.cpp lines 683-722:
    static constexpr int kHalfOffsets[14][3] = {
        {0, 0, 0}, {1, 0, 0},
        {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
        {-1, 0, 1},  {0, 0, 1},  {1, 0, 1},
        {-1, 1, 1},  {0, 1, 1},  {1, 1, 1}
    };
    
    // Build grid manually
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;
    struct Cell { int cx = -999999, cy = -999999, cz = -999999; int head = -1; };
    std::vector<Cell> cells(kCapacity);
    std::vector<int> next(2, -1);
    float inv_cell = 1.0f / cluster_tol;
    
    auto hash3D = [](int cx, int cy, int cz) {
        return ((size_t)cx * 73856093 ^ (size_t)cy * 19349663 ^ (size_t)cz * 83492791) & kMask;
    };
    
    for (size_t i = 0; i < 2; ++i) {
        int cx = (int)std::floor(x[i] * inv_cell);
        int cy = (int)std::floor(y[i] * inv_cell);
        int cz = (int)std::floor(z[i] * inv_cell);
        size_t h = hash3D(cx, cy, cz);
        while (cells[h].head != -1 && (cells[h].cx != cx || cells[h].cy != cy || cells[h].cz != cz))
            h = (h + 1) & kMask;
        cells[h].cx = cx; cells[h].cy = cy; cells[h].cz = cz;
        next[i] = cells[h].head;
        cells[h].head = (int)i;
    }
    
    // Check comparisons performed
    bool points_compared = false;
    for (size_t i = 0; i < 2; ++i) {
        float qx = x[i], qy = y[i], qz = z[i];
        int qcx = (int)std::floor(qx * inv_cell);
        int qcy = (int)std::floor(qy * inv_cell);
        int qcz = (int)std::floor(qz * inv_cell);
        
        for (int o = 0; o < 14; ++o) {
            int tcx = qcx + kHalfOffsets[o][0];
            int tcy = qcy + kHalfOffsets[o][1];
            int tcz = qcz + kHalfOffsets[o][2];
            size_t h = hash3D(tcx, tcy, tcz);
            while (cells[h].head != -1) {
                if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                    int curr = cells[h].head;
                    while (curr != -1) {
                        if (curr > (int)i) {
                            points_compared = true;
                        }
                        curr = next[curr];
                    }
                    break;
                }
                h = (h + 1) & kMask;
            }
        }
    }
    
    std::cout << "  Distance between points: 0.02m (threshold: 0.15m)" << std::endl;
    std::cout << "  Were the 2 points compared?: " << (points_compared ? "YES" : "NO (BUG CONFIRMED!)") << std::endl;
    if (!points_compared) {
        std::cout << "  -> RESULT: Point 0 and Point 1 are NEVER united into the same cluster due to forward halfspace + curr > i indexing interaction!\n" << std::endl;
    }
}

// Test 2: Verify Hash Table Infinite Loop / Full Table Hang
void verify_hash_table_infinite_loop() {
    std::cout << "--- Verifying Issue 2: Hash Table Linear Probing Infinite Loop on Capacity Exhaustion ---" << std::endl;
    static constexpr size_t kHCap = 128; // Scale down to 128 for instant demonstration
    static constexpr size_t kHMask = kHCap - 1;
    struct VoxelEntry { int32_t key = -1; int count = 0; };
    std::vector<VoxelEntry> htable(kHCap);

    auto vhash = [](int32_t key) -> size_t { return ((static_cast<size_t>(key) * 2654435761u)) & kHMask; };

    // Insert 128 unique keys (filling the table 100%)
    for (int i = 0; i < 128; ++i) {
        int32_t k = i * 100;
        size_t h = vhash(k);
        while (htable[h].key != -1 && htable[h].key != k) h = (h + 1) & kHMask;
        htable[h].key = k;
        htable[h].count++;
    }

    std::cout << "  Table is now 100% full (128/128 elements)." << std::endl;
    std::cout << "  Attempting to look up a non-existent key (999999) with probe safety check..." << std::endl;

    int32_t query_key = 999999;
    size_t h = vhash(query_key);
    size_t probe_count = 0;
    while (htable[h].key != -1 && htable[h].key != query_key) {
        h = (h + 1) & kHMask;
        probe_count++;
        if (probe_count > kHCap * 2) {
            std::cout << "  -> BUG CONFIRMED: Probe entered an infinite loop (probed " << probe_count << " times without terminating)!\n" << std::endl;
            return;
        }
    }
}

// Test 3: Verify Sentinel Collision in Fast3DSpatialGrid
void verify_sentinel_collision() {
    std::cout << "--- Verifying Issue 3: Sentinel Collision in Fast3DSpatialGrid ---" << std::endl;
    struct Cell { int cx = -999999, cy = -999999, cz = -999999; int head = -1; };
    Cell default_cell;
    
    // A point at UTM coordinate x = -249999.75m with cell_size 0.25m
    float x = -249999.75f, inv_cell = 1.0f / 0.25f;
    int cx = (int)std::floor(x * inv_cell);
    int cy = -999999;
    int cz = -999999;

    bool matches_empty = (default_cell.cx == cx && default_cell.cy == cy && default_cell.cz == cz);
    std::cout << "  Cell coordinate: (" << cx << ", " << cy << ", " << cz << ")" << std::endl;
    std::cout << "  Does valid cell match uninitialized empty cell sentinel?: " << (matches_empty ? "YES (BUG CONFIRMED!)" : "NO") << std::endl;
    if (matches_empty) {
        std::cout << "  -> RESULT: Valid coordinates equal to -999999 collide with empty cell representation!\n" << std::endl;
    }
}

int main() {
    verify_clustering_bug();
    verify_hash_table_infinite_loop();
    verify_sentinel_collision();
    return 0;
}
