#include "rvv_pcl.h"
#include <vector>
#include <iostream>
#include <random>
#include <algorithm>
#include <chrono>

using namespace rvv_pcl;

int main() {
    std::cout << "[TEST] Starting Octree Verification..." << std::endl;

    size_t N = 10000;
    std::vector<float> x(N), y(N), z(N);
    
    std::mt19937 gen(1234);
    std::uniform_real_distribution<float> dist(0.0f, 100.0f);

    for(size_t i=0; i<N; ++i) {
        x[i] = dist(gen);
        y[i] = dist(gen);
        z[i] = dist(gen);
    }
    
    PointCloudSoA cloud = {x.data(), y.data(), z.data(), N};

    std::cout << "Building Octree..." << std::endl;
    Octree octree;
    octree.setInputCloud(cloud);
    
    auto start = std::chrono::high_resolution_clock::now();
    octree.build();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> diff = end - start;
    std::cout << "Octree Build Time: " << diff.count() << " s" << std::endl;

    int num_queries = 100;
    std::uniform_real_distribution<float> query_dist(0.0f, 100.0f);
    float radius = 5.0f;
    float r2 = radius * radius;
    int mismatches = 0;

    std::cout << "Verifying " << num_queries << " random queries (Radius=" << radius << ")..." << std::endl;

    for (int q=0; q<num_queries; ++q) {
        PointXYZ query = {query_dist(gen), query_dist(gen), query_dist(gen)};

        std::vector<int> idx_oct;
        std::vector<float> dist_oct;
        octree.radiusSearch(query, radius, idx_oct, dist_oct);

        std::vector<int> idx_bf;
        for (size_t i=0; i<N; ++i) {
            float dx = x[i] - query.x;
            float dy = y[i] - query.y;
            float dz = z[i] - query.z;
            float d2 = dx*dx + dy*dy + dz*dz;
            if (d2 <= r2) idx_bf.push_back(i);
        }

        std::sort(idx_oct.begin(), idx_oct.end());
        std::sort(idx_bf.begin(), idx_bf.end());

        if (idx_oct.size() != idx_bf.size()) {
            std::cerr << "Mismatch size! Quad " << q << ": Oct=" << idx_oct.size() << " BF=" << idx_bf.size() << std::endl;
            mismatches++;
        } else {
            for(size_t k=0; k<idx_oct.size(); ++k) {
                if (idx_oct[k] != idx_bf[k]) {
                    std::cerr << "Mismatch Content!" << std::endl;
                    mismatches++;
                    break;
                }
            }
        }
    }

    if (mismatches == 0) {
        std::cout << "[PASS] Octree Radius Search Verified." << std::endl;
        return 0;
    } else {
        std::cerr << "[FAIL] Found " << mismatches << " mismatches." << std::endl;
        return 1;
    }
}
