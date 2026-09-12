#include <iostream>
#include <vector>
#include "include/rvpoint.h"
#include "simple_pcd_loader.h"

int main() {
    std::vector<std::string> candidates = {
        "data/0000000000.pcd",
        "data/pcd_compressed/0000000090.pcd",
        "data/region_growing_rgb_tutorial.pcd",
        "0000000000.pcd",
        "bunny.pcd",
        "../data/0000000000.pcd"
    };
    
    // 1. Test PointCloud SoA loader
    rvpoint::PointCloud cloud;
    int count_soa = -1;
    std::string loaded_path;
    for (const auto &path : candidates) {
        if (rvpoint::loadPCD(path, cloud)) {
            count_soa = static_cast<int>(cloud.size());
            loaded_path = path;
            break;
        }
    }

    if (count_soa <= 0) {
        std::cerr << "[FAIL] Failed to load PCD directly into PointCloud" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Successfully loaded " << count_soa << " points into PointCloud SoA from " << loaded_path << std::endl;
    std::cout << "  Point 0 (SoA): (" << cloud.x[0] << ", " << cloud.y[0] << ", " << cloud.z[0] << ")" << std::endl;

    // 2. Test legacy PointXYZ vector loader
    std::vector<rvpoint::PointXYZ> points;
    bool ok_aos = rvpoint::loadPCD(loaded_path, points);
    if (!ok_aos || points.size() != cloud.size()) {
        std::cerr << "[FAIL] PointXYZ loader count mismatch: " << points.size() << " vs " << cloud.size() << std::endl;
        return 1;
    }
    assert(std::abs(points[0].x - cloud.x[0]) < 1e-6f);
    assert(std::abs(points[0].y - cloud.y[0]) < 1e-6f);
    assert(std::abs(points[0].z - cloud.z[0]) < 1e-6f);
    std::cout << "[PASS] PointCloud and PointXYZ loader counts and coordinates match exactly (" << points.size() << " points)." << std::endl;

    return 0;
}

