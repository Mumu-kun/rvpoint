#include "rvv_pcl.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace rvv_pcl;

int main() {
    const size_t N = 1000;
    const int K = 10;
    
    std::vector<float> x(N), y(N), z(N);
    std::vector<PointXYZ> input_aos(N);
    
    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist_surf(-10.0f, 10.0f);
    std::normal_distribution<float> dist_noise(0.0f, 0.01f);

    for(size_t i=0; i<N; ++i) {
        x[i] = dist_surf(gen);
        y[i] = dist_surf(gen);
        z[i] = dist_noise(gen);
        input_aos[i] = {x[i], y[i], z[i]};
    }

    PointCloudSoA input_soa = {x.data(), y.data(), z.data(), N};

    std::vector<float> nx_sc(N), ny_sc(N), nz_sc(N);
    normal_estimation_sc(input_aos.data(), N, nx_sc.data(), ny_sc.data(), nz_sc.data(), K, 2.0f);

    std::vector<float> nx_rvv(N), ny_rvv(N), nz_rvv(N);
    
    Octree octree;
    octree.setInputCloud(input_soa);
    octree.build();
    
    normal_estimation_rvv(input_soa, octree, nx_rvv.data(), ny_rvv.data(), nz_rvv.data(), K, 2.0f);

    int pass_count = 0;
    for(size_t i=0; i<N; ++i) {
        float dot_sc = std::abs(nz_sc[i]);
        float dot_rvv = std::abs(nz_rvv[i]);
        float impl_dot = nx_sc[i]*nx_rvv[i] + ny_sc[i]*ny_rvv[i] + nz_sc[i]*nz_rvv[i];
        
        if(dot_sc > 0.9 && dot_rvv > 0.9 && std::abs(impl_dot) > 0.9) {
            pass_count++;
        }
    }
    
    std::cout << "Points with correct normal: " << pass_count << "/" << N << std::endl;
    
    if (pass_count < N * 0.95) {
        std::cerr << "[FAIL] Too many incorrect normals!" << std::endl;
        return 1;
    }

    std::cout << "[PASS] Normal Estimation Verification Successful!" << std::endl;
    return 0;
}
