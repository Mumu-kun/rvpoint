#include "core/point_types.h"
#include "search/radius_search.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <set>

using namespace rvv_pcl;

int main() {
    const size_t N = 1000;
    const int MAX_NN = 100;
    const float RADIUS = 2.0f;
    const PointXYZ query = {0,0,0};

    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    
    std::mt19937 gen(42);
    std::normal_distribution<float> dist(0.0f, 2.0f);

    for(size_t i=0; i<N; ++i) {
        x[i] = dist(gen);
        y[i] = dist(gen);
        z[i] = dist(gen);
        input_aos[i] = {x[i], y[i], z[i]};
    }

    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};

    std::vector<int> idx_sc(MAX_NN);
    std::vector<float> dists_sc(MAX_NN);
    size_t count_sc = radius_search_sc(input_aos.data(), N, query, RADIUS, 
                                       idx_sc.data(), dists_sc.data(), MAX_NN);

    std::vector<int> idx_rvv(MAX_NN);
    std::vector<float> dists_rvv(MAX_NN);
    size_t count_rvv = radius_search_rvv(input_soa, query, RADIUS, 
                                         idx_rvv.data(), dists_rvv.data(), MAX_NN);

    std::cout << "Scalar Count: " << count_sc << std::endl;
    std::cout << "RVV Count:    " << count_rvv << std::endl;

    if (count_sc != count_rvv) {
        std::cerr << "[FAIL] Counts differ!" << std::endl;
        return 1;
    }

    std::set<int> set_sc, set_rvv;
    for(size_t i=0; i<count_sc; ++i) set_sc.insert(idx_sc[i]);
    for(size_t i=0; i<count_rvv; ++i) set_rvv.insert(idx_rvv[i]);

    if(set_sc != set_rvv) {
        std::cerr << "[FAIL] Index sets differ!" << std::endl;
        return 1;
    }

    std::cout << "[PASS] Radius Search Verification Successful!" << std::endl;
    return 0;
}
