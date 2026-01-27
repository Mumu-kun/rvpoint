#include "../src/include/rvv_pcl.h"
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

    // 1. Generate Data
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

    // 2. Run Scalar
    std::vector<PointXYZ> out_sc(N); // Oversized buffer
    size_t count_sc = voxel_grid_downsamp_sc(input_aos.data(), N, out_sc.data(), LEAF);

    // 3. Run RVV (hybrid)
    std::vector<PointXYZ> out_rvv(N);
    size_t count_rvv = voxel_grid_downsamp_rvv(input_soa, out_rvv.data(), LEAF);

    // 4. Run RVV v2 (fully vectorized, sort-based)
    std::vector<PointXYZ> out_rvv_v2(N);
    size_t count_rvv_v2 = voxel_grid_downsamp_rvv_v2(input_soa, out_rvv_v2.data(), LEAF);

    // 5. Verify counts
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

    // Sort outputs to compare content
    auto sort_fn = [](const PointXYZ& a, const PointXYZ& b) {
        if(a.x != b.x) return a.x < b.x;
        if(a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    std::sort(out_sc.begin(), out_sc.begin()+count_sc, sort_fn);
    std::sort(out_rvv.begin(), out_rvv.begin()+count_rvv, sort_fn);
    std::sort(out_rvv_v2.begin(), out_rvv_v2.begin()+count_rvv_v2, sort_fn);

    // Verify RVV hybrid vs scalar
    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv[i])) {
            std::cerr << "[FAIL] SC vs RVV mismatch at index " << i << std::endl;
            std::cerr << "SC: " << out_sc[i].x << " " << out_sc[i].y << " " << out_sc[i].z << std::endl;
            std::cerr << "RVV: " << out_rvv[i].x << " " << out_rvv[i].y << " " << out_rvv[i].z << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Scalar vs RVV (hybrid) match." << std::endl;

    // Verify RVV v2 vs scalar
    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv_v2[i])) {
            std::cerr << "[FAIL] SC vs RVV v2 mismatch at index " << i << std::endl;
            std::cerr << "SC: " << out_sc[i].x << " " << out_sc[i].y << " " << out_sc[i].z << std::endl;
            std::cerr << "RVV v2: " << out_rvv_v2[i].x << " " << out_rvv_v2[i].y << " " << out_rvv_v2[i].z << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Scalar vs RVV v2 (fully vectorized) match." << std::endl;

    // 6. Test with negative coordinates (verify floor correctness via bbox subtraction)
    std::cout << "\n--- Negative Coordinate Test ---" << std::endl;
    const size_t N_NEG = 500;
    std::vector<float> xn(N_NEG), yn(N_NEG), zn(N_NEG);
    std::vector<PointXYZ> input_neg_aos(N_NEG);
    std::uniform_real_distribution<float> neg_dist(-10.0f, 10.0f);

    for(size_t i=0; i<N_NEG; ++i) {
        xn[i] = neg_dist(gen);
        yn[i] = neg_dist(gen);
        zn[i] = neg_dist(gen);
        input_neg_aos[i] = {xn[i], yn[i], zn[i]};
    }
    PointCloudSoA neg_soa = {xn.data(), yn.data(), zn.data(), N_NEG};

    std::vector<PointXYZ> out_neg_sc(N_NEG), out_neg_v2(N_NEG);
    size_t cnt_neg_sc = voxel_grid_downsamp_sc(input_neg_aos.data(), N_NEG, out_neg_sc.data(), LEAF);
    size_t cnt_neg_v2 = voxel_grid_downsamp_rvv_v2(neg_soa, out_neg_v2.data(), LEAF);

    std::cout << "Neg SC Count:  " << cnt_neg_sc << std::endl;
    std::cout << "Neg v2 Count:  " << cnt_neg_v2 << std::endl;

    if (cnt_neg_sc != cnt_neg_v2) {
        std::cerr << "[FAIL] Negative coord counts differ!" << std::endl;
        return 1;
    }

    std::sort(out_neg_sc.begin(), out_neg_sc.begin()+cnt_neg_sc, sort_fn);
    std::sort(out_neg_v2.begin(), out_neg_v2.begin()+cnt_neg_v2, sort_fn);

    for(size_t i=0; i<cnt_neg_sc; ++i) {
        if (!are_points_close(out_neg_sc[i], out_neg_v2[i])) {
            std::cerr << "[FAIL] Negative coord mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Negative coordinate test passed." << std::endl;

    std::cout << "\n[PASS] All Voxel Grid Tests Successful!" << std::endl;
    return 0;
}
