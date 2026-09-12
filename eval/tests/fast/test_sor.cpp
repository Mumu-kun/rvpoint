#include "core/point_types.h"
#include "filters/statistical_outlier_removal.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace rvpoint;

static bool are_points_close(float ax, float ay, float az, float bx, float by, float bz, float eps = 1e-4f) {
    return std::abs(ax - bx) < eps &&
           std::abs(ay - by) < eps &&
           std::abs(az - bz) < eps;
}

int main() {
    std::cout << "=== Running test_sor ===" << std::endl;
    const size_t N = 1000;
    const int K = 10;
    const float ALPHA = 1.0f;

    PointCloud cloud;
    cloud.reserve(N);

    std::mt19937 gen(42);
    std::normal_distribution<float> cluster_dist(0.0f, 1.0f);
    std::uniform_real_distribution<float> outlier_dist(-20.0f, 20.0f);

    for (size_t i = 0; i < N; ++i) {
        if (i < N * 0.9) {
            cloud.push_back(cluster_dist(gen), cluster_dist(gen), cluster_dist(gen));
        } else {
            cloud.push_back(outlier_dist(gen), outlier_dist(gen), outlier_dist(gen));
        }
    }

    // 1. Test class StatisticalOutlierRemoval with RVV and Scalar backends
    StatisticalOutlierRemoval sor_rvv(K, ALPHA, Backend::RVV);
    StatisticalOutlierRemoval sor_scalar(K, ALPHA, Backend::Scalar);

    sor_rvv.reserve(N);
    sor_scalar.reserve(N);

    PointCloud out_rvv;
    PointCloud out_scalar;

    size_t count_rvv = sor_rvv(cloud.view(), out_rvv);
    size_t count_scalar = sor_scalar(cloud.view(), out_scalar);

    std::cout << "StatisticalOutlierRemoval RVV Count:    " << count_rvv << " (out.size()=" << out_rvv.size() << ")" << std::endl;
    std::cout << "StatisticalOutlierRemoval Scalar Count: " << count_scalar << " (out.size()=" << out_scalar.size() << ")" << std::endl;

    if (count_rvv != count_scalar || out_rvv.size() != count_rvv || out_scalar.size() != count_scalar) {
        std::cerr << "[FAIL] SOR RVV vs Scalar count mismatch!" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < count_rvv; ++i) {
        if (!are_points_close(out_rvv.x[i], out_rvv.y[i], out_rvv.z[i],
                              out_scalar.x[i], out_scalar.y[i], out_scalar.z[i])) {
            std::cerr << "[FAIL] SOR data mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] StatisticalOutlierRemoval class RVV vs Scalar parity verified." << std::endl;

    // 2. Multi-frame stability check
    for (int frame = 0; frame < 5; ++frame) {
        size_t c = sor_rvv(cloud.view(), out_rvv);
        if (c != count_rvv) {
            std::cerr << "[FAIL] Multi-frame SOR execution inconsistency!" << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Multi-frame zero-reallocation stability verified." << std::endl;

    // 3. Test AoS export filter method
    std::vector<PointXYZ> leg_out(N);
    size_t leg_cnt = sor_scalar.filter(cloud.view(), leg_out.data(), K, ALPHA);
    if (leg_cnt != count_rvv) {
        std::cerr << "[FAIL] AoS export sor_scalar count mismatch: " << leg_cnt << " vs " << count_rvv << std::endl;
        return 1;
    }
    std::cout << "[PASS] StatisticalOutlierRemoval AoS export verified." << std::endl;

    std::cout << "All StatisticalOutlierRemoval tests PASS." << std::endl;
    return 0;
}
