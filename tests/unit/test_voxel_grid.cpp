#include "rvv_pcl.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace rvv_pcl;

bool are_points_close(const PointXYZ& a, const PointXYZ& b, float eps = 1e-4) {
    return std::abs(a.x - b.x) < eps &&
           std::abs(a.y - b.y) < eps &&
           std::abs(a.z - b.z) < eps;
}

int main() {
    const size_t N = 1000;
    const float LEAF = 0.5f;

    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist(0.0f, 10.0f);

    for(size_t i=0; i<N; ++i) {
        x[i] = dist(gen);
        y[i] = dist(gen);
        z[i] = dist(gen);
        input_aos[i] = {x[i], y[i], z[i]};
    }

    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};

    std::vector<PointXYZ> out_sc(N);
    size_t count_sc = voxel_grid_downsamp_sc(input_aos.data(), N, out_sc.data(), LEAF);

    std::vector<PointXYZ> out_rvv(N);
    size_t count_rvv = voxel_grid_downsamp_rvv(input_soa, out_rvv.data(), LEAF);

    std::vector<PointXYZ> out_rvv_v2(N);
    size_t count_rvv_v2 = voxel_grid_downsamp_rvv_v2(input_soa, out_rvv_v2.data(), LEAF);

    std::cout << "Scalar Count:  " << count_sc << std::endl;
    std::cout << "RVV Count:     " << count_rvv << std::endl;
    std::cout << "RVV v2 Count:  " << count_rvv_v2 << std::endl;

    if (count_sc != count_rvv) {
        std::cerr << "[FAIL] Scalar vs RVV counts differ!" << std::endl;
        return 1;
    }
    if (count_sc != count_rvv_v2) {
        std::cerr << "[FAIL] Scalar vs RVV v2 counts differ!" << std::endl;
        return 1;
    }

    auto sort_fn = [](const PointXYZ& a, const PointXYZ& b) {
        if(a.x != b.x) return a.x < b.x;
        if(a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    std::sort(out_sc.begin(), out_sc.begin()+count_sc, sort_fn);
    std::sort(out_rvv.begin(), out_rvv.begin()+count_rvv, sort_fn);
    std::sort(out_rvv_v2.begin(), out_rvv_v2.begin()+count_rvv_v2, sort_fn);

    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv[i])) {
            std::cerr << "[FAIL] SC vs RVV mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Scalar vs RVV (hybrid) match." << std::endl;

    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv_v2[i])) {
            std::cerr << "[FAIL] SC vs RVV v2 mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Scalar vs RVV v2 (fully vectorized) match." << std::endl;

    return 0;
}
