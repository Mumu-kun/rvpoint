#include "core/point_types.h"
#include "search/radius_search.h"
#include "search/fast_3d_spatial_grid.h"
#include "search/caravan_radius_search.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <set>

using namespace rvpoint;

int main() {
    const size_t N = 1000;
    const int MAX_NN = 100;
    const float RADIUS = 2.0f;
    const PointXYZ query = {0, 0, 0};

    PointCloud cloud(N);
    std::mt19937 gen(42);
    std::normal_distribution<float> dist(0.0f, 2.0f);

    std::vector<PointXYZ> input_aos(N);
    for (size_t i = 0; i < N; ++i) {
        float px = dist(gen);
        float py = dist(gen);
        float pz = dist(gen);
        cloud.push_back(px, py, pz);
        input_aos[i] = {px, py, pz};
    }

    PointCloudView input_soa = cloud.view();

    // 1. Scalar vs RVV free functions
    std::vector<int> idx_sc(MAX_NN);
    std::vector<float> dists_sc(MAX_NN);
    size_t count_sc = radius_search_sc(input_aos.data(), N, query, RADIUS,
                                       idx_sc.data(), dists_sc.data(), MAX_NN);

    std::vector<int> idx_rvv(MAX_NN);
    std::vector<float> dists_rvv(MAX_NN);
    size_t count_rvv = radius_search_rvv(input_soa, query, RADIUS,
                                         idx_rvv.data(), dists_rvv.data(), MAX_NN);

    std::cout << "[1] Scalar Count: " << count_sc << " | RVV Count: " << count_rvv << std::endl;
    if (count_sc != count_rvv) {
        std::cerr << "[FAIL] Scalar vs RVV counts differ!" << std::endl;
        return 1;
    }

    // 2. Fast3DSpatialGrid Verification
    Fast3DSpatialGrid grid(RADIUS, N);
    if (!grid.build(input_soa)) {
        std::cerr << "[FAIL] Fast3DSpatialGrid build failed!" << std::endl;
        return 1;
    }
    std::vector<int> grid_nbs;
    std::vector<float> grid_dists2;
    grid.radiusSearch(query.x, query.y, query.z, RADIUS * RADIUS, grid_nbs, grid_dists2);
    std::cout << "[2] Fast3DSpatialGrid found " << grid_nbs.size() << " neighbors." << std::endl;
    if (grid_nbs.empty()) {
        std::cerr << "[FAIL] Fast3DSpatialGrid returned empty results!" << std::endl;
        return 1;
    }

    // Test countNeighbors fastpath
    int fast_count = grid.countNeighbors(query.x, query.y, query.z, RADIUS * RADIUS, 10);
    if (fast_count < 10 && fast_count != static_cast<int>(grid_nbs.size())) {
        std::cerr << "[FAIL] countNeighbors returned inconsistent count: " << fast_count << std::endl;
        return 1;
    }

    // 3. CaravanRadiusSearch Verification with NeighborQueryResult
    CaravanRadiusSearch caravan;
    caravan.setInputCloud(input_soa);
    PointCloud queries(2);
    queries.push_back(0.0f, 0.0f, 0.0f);
    queries.push_back(1.0f, 1.0f, 1.0f);

    NeighborQueryResult caravan_res;
    caravan.batchRadiusSearch(queries.view(), RADIUS, caravan_res);
    std::cout << "[3] CaravanRadiusSearch processed " << caravan_res.num_queries()
              << " queries, total neighbors: " << caravan_res.indices.size() << std::endl;
    if (caravan_res.num_queries() != 2) {
        std::cerr << "[FAIL] Caravan query count mismatch!" << std::endl;
        return 1;
    }

    std::cout << "[PASS] Radius Search & Spatial Grid Verification Successful!" << std::endl;
    return 0;
}
