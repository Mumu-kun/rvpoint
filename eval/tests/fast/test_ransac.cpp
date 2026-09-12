#include "core/point_types.h"
#include "segmentation/ransac_plane.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <cstdlib>

using namespace rvpoint;

int main() {
    std::cout << "=== Running test_ransac ===" << std::endl;
    const size_t N_INLIERS = 500;
    const size_t N_OUTLIERS = 500;
    const size_t N = N_INLIERS + N_OUTLIERS;
    const float THRESH = 0.1f;
    const int MAX_ITERS = 500;

    PointCloud cloud;
    cloud.reserve(N);

    std::mt19937 gen(42);
    std::uniform_real_distribution<float> range_xy(-10.0f, 10.0f);
    std::normal_distribution<float> range_z(10.0f, 0.01f);
    std::uniform_real_distribution<float> outliers(-20.0f, 20.0f);

    for (size_t i = 0; i < N; ++i) {
        if (i < N_INLIERS) {
            cloud.push_back(range_xy(gen), range_xy(gen), range_z(gen));
        } else {
            cloud.push_back(outliers(gen), outliers(gen), outliers(gen));
        }
    }

    // 1. Test class RansacPlane Functor with PlaneModel and Dual-Path Parity
    RansacPlane rp_rvv(THRESH, MAX_ITERS, Backend::RVV);
    RansacPlane rp_sc(THRESH, MAX_ITERS, Backend::Scalar);
    rp_rvv.set_seed(12345);
    rp_sc.set_seed(12345);

    PlaneModel model_rvv;
    PlaneModel model_sc;

    int inliers_rvv = rp_rvv(cloud.view(), model_rvv);
    int inliers_sc = rp_sc(cloud.view(), model_sc);

    std::cout << "RansacPlane RVV:    inliers=" << inliers_rvv
              << " [" << model_rvv.a << ", " << model_rvv.b << ", " << model_rvv.c << ", " << model_rvv.d << "]" << std::endl;
    std::cout << "RansacPlane Scalar: inliers=" << inliers_sc
              << " [" << model_sc.a << ", " << model_sc.b << ", " << model_sc.c << ", " << model_sc.d << "]" << std::endl;

    if (inliers_rvv < 450 || inliers_sc < 450) {
        std::cerr << "[FAIL] Too few inliers found!" << std::endl;
        return 1;
    }

    if (inliers_rvv != inliers_sc) {
        std::cerr << "[FAIL] RVV vs Scalar inlier count mismatch!" << std::endl;
        return 1;
    }

    if (std::abs(model_rvv.c) < 0.9f || std::abs(model_rvv.d) < 9.0f) {
        std::cerr << "[FAIL] RVV plane model orientation incorrect!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Dual-path RansacPlane parity verified." << std::endl;

    // 2. Test SoA extraction
    PointCloud inliers_cloud;
    PointCloud outliers_cloud;
    rp_rvv.extract(cloud.view(), model_rvv, inliers_cloud, outliers_cloud);

    std::cout << "Extracted inliers: " << inliers_cloud.size() << ", obstacles: " << outliers_cloud.size() << std::endl;

    if (inliers_cloud.size() != (size_t)inliers_rvv) {
        std::cerr << "[FAIL] Extracted inliers count (" << inliers_cloud.size() << ") != model.inliers (" << inliers_rvv << ")" << std::endl;
        return 1;
    }

    if (inliers_cloud.size() + outliers_cloud.size() != N) {
        std::cerr << "[FAIL] Extracted clouds sum (" << (inliers_cloud.size() + outliers_cloud.size()) << ") != total points (" << N << ")" << std::endl;
        return 1;
    }

    // Verify distance properties
    for (size_t i = 0; i < inliers_cloud.size(); ++i) {
        float d = model_rvv.distance(inliers_cloud.x[i], inliers_cloud.y[i], inliers_cloud.z[i]);
        if (d > THRESH + 1e-4f) {
            std::cerr << "[FAIL] Inlier at index " << i << " distance (" << d << ") exceeds threshold!" << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Inlier / outlier separation verified." << std::endl;

    // 3. Test AoS extraction methods
    std::vector<PointXYZ> aos_inliers(N);
    std::vector<PointXYZ> aos_outliers(N);
    float plane_coeffs[4] = {model_rvv.a, model_rvv.b, model_rvv.c, model_rvv.d};
    size_t aos_n_inliers = 0, aos_n_outliers = 0;
    rp_rvv.extract_inliers_outliers(cloud.view(), plane_coeffs, THRESH,
                                    aos_inliers.data(), aos_outliers.data(),
                                    aos_n_inliers, aos_n_outliers);
    if (aos_n_inliers != inliers_cloud.size() || aos_n_outliers != outliers_cloud.size()) {
        std::cerr << "[FAIL] AoS extraction count mismatch: " << aos_n_inliers << " vs " << inliers_cloud.size() << std::endl;
        return 1;
    }
    std::cout << "[PASS] AoS extraction methods verified." << std::endl;

    // 4. Test Ground Normal Prior Filtering
    RansacPlane rp_prior(THRESH, MAX_ITERS, Backend::RVV);
    rp_prior.set_seed(12345);
    rp_prior.set_ground_normal_prior(0.0f, 0.0f, 1.0f, 0.8f);
    PlaneModel model_prior;
    int inliers_prior = rp_prior(cloud.view(), model_prior);
    if (inliers_prior < 450 || std::abs(model_prior.c) < 0.8f) {
        std::cerr << "[FAIL] Ground normal prior failed to fit horizontal plane!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Ground normal prior filter verified (c=" << model_prior.c << ")." << std::endl;

    // 5. Test Covariance Normal Refinement
    RansacPlane rp_refine(THRESH, MAX_ITERS, Backend::RVV);
    rp_refine.set_seed(12345);
    rp_refine.set_use_covariance_refinement(true);
    PlaneModel model_refined;
    int inliers_refined = rp_refine(cloud.view(), model_refined);
    if (inliers_refined < 450 || std::abs(model_refined.c) < 0.99f) {
        std::cerr << "[FAIL] Covariance refinement failed!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Inlier covariance refinement verified (c=" << model_refined.c << ")." << std::endl;

    std::cout << "All RansacPlane tests PASS." << std::endl;
    return 0;
}
