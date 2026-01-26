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

    // 3. Run RVV
    std::vector<PointXYZ> out_rvv(N);
    size_t count_rvv = voxel_grid_downsamp_rvv(input_soa, out_rvv.data(), LEAF);
    
    // Print VLEN for confirmation (read CSR)
    size_t vlenb;
    asm volatile("csrr %0, vlenb" : "=r"(vlenb));
    // Use printf for bare metal safety if cout is flaky, though cout works in semihosting usually
    printf("DEBUG: VLENB = %zu bytes (%zu bits)\n", vlenb, vlenb*8);
    printf("DEBUG: Effective Vector Width (m8) = %zu floats per instruction\n", (vlenb/4)*8);


    // 4. Verify
    std::cout << "Scalar Count: " << count_sc << std::endl;
    std::cout << "RVV Count:    " << count_rvv << std::endl;

    if (count_sc != count_rvv) {
        std::cerr << "[FAIL] Counts differ!" << std::endl;
        return 1;
    }

    // Sort outputs to compare content (order might differ due to map traversal vs parallel reduction if we had one,
    // but here both use std::map under the hood, so order should match given same keys)
    // Actually map order is deterministic by key (vx, vy, vz).
    // Let's sort just in case floating point noise caused slight key diffs (unlikely with identical floor logic).
    auto sort_fn = [](const PointXYZ& a, const PointXYZ& b) {
        if(a.x != b.x) return a.x < b.x;
        if(a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    std::sort(out_sc.begin(), out_sc.begin()+count_sc, sort_fn);
    std::sort(out_rvv.begin(), out_rvv.begin()+count_rvv, sort_fn);

    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv[i])) {
            std::cerr << "[FAIL] Data mismatch at index " << i << std::endl;
            std::cerr << "SC: " << out_sc[i].x << " " << out_sc[i].y << " " << out_sc[i].z << std::endl;
            std::cerr << "RVV: " << out_rvv[i].x << " " << out_rvv[i].y << " " << out_rvv[i].z << std::endl;
            return 1;
        }
    }

    std::cout << "[PASS] Voxel Grid Verification Successful!" << std::endl;
    return 0;
}
