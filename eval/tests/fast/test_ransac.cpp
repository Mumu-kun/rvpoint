#include "core/point_types.h"
#include "segmentation/ransac_plane.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <cstdlib>

using namespace rvpoint;

int main() {
    const size_t N_INLIERS = 500;
    const size_t N_OUTLIERS = 500;
    const size_t N = N_INLIERS + N_OUTLIERS;
    const float THRESH = 0.1f;
    const int MAX_ITERS = 1000;
    
    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> range_xy(-10, 10);
    std::normal_distribution<float> range_z(10.0f, 0.01f);
    std::uniform_real_distribution<float> outliers(-20, 20);

    for(size_t i=0; i<N; ++i) {
        if(i < N_INLIERS) {
            x[i] = range_xy(gen);
            y[i] = range_xy(gen);
            z[i] = range_z(gen);
        } else {
            x[i] = outliers(gen);
            y[i] = outliers(gen);
            z[i] = outliers(gen);
        }
        input_aos[i] = {x[i], y[i], z[i]};
    }

    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};

    float model_sc[4];
    int count_sc = ransac_plane_sc(input_aos.data(), N, THRESH, MAX_ITERS, model_sc);

    float model_rvv[4];
    int count_rvv = ransac_plane_rvv(input_soa, THRESH, MAX_ITERS, model_rvv);

    std::cout << "Scalar Count: " << count_sc << " Model: " 
              << model_sc[0] << " " << model_sc[1] << " " << model_sc[2] << " " << model_sc[3] << std::endl;
              
    std::cout << "RVV Count:    " << count_rvv << " Model: " 
              << model_rvv[0] << " " << model_rvv[1] << " " << model_rvv[2] << " " << model_rvv[3] << std::endl;

    bool sc_valid = (count_sc >= 450);
    bool rvv_valid = (count_rvv >= 450);
    
    bool sc_model_ok = (std::abs(model_sc[2]) > 0.9 && std::abs(model_sc[3]) >= 9.0);
    bool rvv_model_ok = (std::abs(model_rvv[2]) > 0.9 && std::abs(model_rvv[3]) >= 9.0);

    if (sc_valid && rvv_valid && sc_model_ok && rvv_model_ok) {
        std::cout << "[PASS] RANSAC Verification Successful!" << std::endl;
        return 0;
    } else {
        std::cerr << "[FAIL] RANSAC Failed!" << std::endl;
        return 1;
    }
}
