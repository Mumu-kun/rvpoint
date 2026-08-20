#include "core/point_types.h"
#include "filters/statistical_outlier_removal.h"
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
    const int K = 10;
    const float ALPHA = 1.0f;

    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    
    std::mt19937 gen(42);
    std::normal_distribution<float> cluster_dist(0.0f, 1.0f);
    std::uniform_real_distribution<float> outlier_dist(-20.0f, 20.0f);

    for(size_t i=0; i<N; ++i) {
        if (i < N*0.9) {
            x[i] = cluster_dist(gen);
            y[i] = cluster_dist(gen);
            z[i] = cluster_dist(gen);
        } else {
            x[i] = outlier_dist(gen);
            y[i] = outlier_dist(gen);
            z[i] = outlier_dist(gen);
        }
        input_aos[i] = {x[i], y[i], z[i]};
    }

    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};

    std::vector<PointXYZ> out_sc(N); 
    size_t count_sc = sor_sc(input_aos.data(), N, out_sc.data(), K, ALPHA);

    std::vector<PointXYZ> out_rvv(N);
    size_t count_rvv = sor_rvv(input_soa, out_rvv.data(), K, ALPHA);

    std::cout << "Scalar Count: " << count_sc << std::endl;
    std::cout << "RVV Count:    " << count_rvv << std::endl;

    if (count_sc != count_rvv) {
        std::cerr << "[FAIL] Counts differ!" << std::endl;
        return 1;
    }

    for(size_t i=0; i<count_sc; ++i) {
        if (!are_points_close(out_sc[i], out_rvv[i])) {
            std::cerr << "[FAIL] Data mismatch at index " << i << std::endl;
            return 1;
        }
    }

    std::cout << "[PASS] SOR Verification Successful!" << std::endl;
    return 0;
}
