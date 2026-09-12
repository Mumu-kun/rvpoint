#include "features/fused_filter_normals.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

using namespace rvpoint;

int main() {
    std::cout << "=== Running test_fused_filter_normals ===" << std::endl;

    // 1. Create a synthetic horizontal plane of 100 points + 5 isolated outliers
    PointCloud cloud;
    cloud.reserve(105);

    // Plane at z = 5.0f
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            cloud.push_back(x * 0.05f, y * 0.05f, 5.0f);
        }
    }

    // 5 isolated outliers
    cloud.push_back(100.0f, 100.0f, 100.0f);
    cloud.push_back(-50.0f, -50.0f, -50.0f);
    cloud.push_back(20.0f, 30.0f, 40.0f);
    cloud.push_back(-20.0f, 30.0f, -40.0f);
    cloud.push_back(50.0f, -10.0f, 30.0f);

    std::cout << "Total input points: " << cloud.size() << std::endl;

    // 2. Instantiate and reserve FusedFilterNormals
    FusedFilterNormals ffn(0.15f, 4, true);
    ffn.reserve(200);

    PointCloud filtered;
    PointCloud normals;

    std::size_t kept = ffn(cloud.view(), filtered, &normals);
    std::cout << "Kept points: " << kept << ", normals count: " << normals.size() << std::endl;

    if (kept != 100) {
        std::cerr << "[FAIL] Expected 100 inlier points, got " << kept << std::endl;
        return 1;
    }

    if (filtered.size() != 100 || normals.size() != 100) {
        std::cerr << "[FAIL] Output cloud size mismatch!" << std::endl;
        return 1;
    }

    // 3. Verify normal orientation: should be perpendicular to z = 5 plane (i.e. |nz| ~ 1)
    int valid_normal_count = 0;
    for (size_t i = 0; i < normals.size(); ++i) {
        float nz = std::abs(normals.z[i]);
        if (nz > 0.90f) {
            valid_normal_count++;
        }
    }

    std::cout << "Valid planar normals (|nz| > 0.9): " << valid_normal_count << " / 100" << std::endl;
    if (valid_normal_count < 90) {
        std::cerr << "[FAIL] Too many normals failed orientation check!" << std::endl;
        return 1;
    }

    // 4. Multi-frame zero-heap stability test
    for (int frame = 0; frame < 5; ++frame) {
        std::size_t k = ffn(cloud.view(), filtered, &normals);
        if (k != 100) {
            std::cerr << "[FAIL] Multi-frame execution stability failed on frame " << frame << std::endl;
            return 1;
        }
    }

    std::cout << "[PASS] FusedFilterNormals verified successfully." << std::endl;
    return 0;
}

