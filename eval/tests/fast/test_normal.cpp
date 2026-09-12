#include "core/point_types.h"
#include "features/normal_estimation.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace rvpoint;

int main() {
    std::cout << "=== Running test_normal ===" << std::endl;
    const size_t N = 1000;
    const int K = 10;

    PointCloud cloud;
    cloud.reserve(N);

    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist_surf(-10.0f, 10.0f);
    std::normal_distribution<float> dist_noise(0.0f, 0.01f);

    for (size_t i = 0; i < N; ++i) {
        cloud.push_back(dist_surf(gen), dist_surf(gen), dist_noise(gen));
    }

    // 1. Test class NormalEstimation with RVV and Scalar backends
    NormalEstimation ne_rvv(K, 2.0f, 0, 0, 0, Backend::RVV);
    NormalEstimation ne_sc(K, 2.0f, 0, 0, 0, Backend::Scalar);

    ne_rvv.reserve(N);
    ne_sc.reserve(N);

    PointCloud normals_rvv;
    PointCloud normals_sc;

    ne_rvv(cloud.view(), normals_rvv);
    ne_sc(cloud.view(), normals_sc);

    if (normals_rvv.size() != N || normals_sc.size() != N) {
        std::cerr << "[FAIL] Output normal cloud size (" << normals_rvv.size() << ") != input cloud size (" << N << ")" << std::endl;
        return 1;
    }

    int pass_count = 0;
    for (size_t i = 0; i < N; ++i) {
        float dot_sc = std::abs(normals_sc.z[i]);
        float dot_rvv = std::abs(normals_rvv.z[i]);
        float impl_dot = normals_sc.x[i] * normals_rvv.x[i] +
                         normals_sc.y[i] * normals_rvv.y[i] +
                         normals_sc.z[i] * normals_rvv.z[i];

        if (dot_sc > 0.9f && dot_rvv > 0.9f && std::abs(impl_dot) > 0.9f) {
            pass_count++;
        }
    }

    std::cout << "Points with correct normal: " << pass_count << "/" << N << std::endl;

    if (pass_count < static_cast<int>(N * 0.95f)) {
        std::cerr << "[FAIL] Too many incorrect normals!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] NormalEstimation class RVV vs Scalar parity verified." << std::endl;

    // 2. Multi-frame zero-reallocation stability check
    for (int frame = 0; frame < 3; ++frame) {
        ne_rvv(cloud.view(), normals_rvv);
        if (normals_rvv.size() != N) {
            std::cerr << "[FAIL] Multi-frame NormalEstimation failed!" << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Multi-frame execution stability verified." << std::endl;

    std::cout << "All NormalEstimation tests PASS." << std::endl;
    return 0;
}
