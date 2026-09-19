#include <iostream>
#include <vector>
#include "include/rvpoint.h"
#include "simple_pcd_loader.h"

int main() {
    std::vector<rvpoint::PointXYZ> points;
    std::vector<std::string> candidates = {
        "data/0000000000.pcd",
        "data/pcd_compressed/0000000090.pcd",
        "data/region_growing_rgb_tutorial.pcd",
        "0000000000.pcd",
        "bunny.pcd",
        "../data/0000000000.pcd"
    };
    
    int count = -1;
    std::string loaded_path;
    for (const auto &path : candidates) {
        count = rvpoint::loadPCD(path, points);
        if (count > 0) {
            loaded_path = path;
            break;
        }
    }
    
    if (count <= 0) {
        std::cerr << "[FAIL] Failed to load any test PCD" << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << count << " points from " << loaded_path << std::endl;
    
    if (count > 0) {
        std::cout << "Point 0: " << points[0].x << " " << points[0].y << " " << points[0].z << std::endl;
        std::cout << "[PASS] Successfully loaded " << count << " points." << std::endl;
        return 0;
    } else {
        std::cerr << "[FAIL] No points loaded from " << loaded_path << std::endl;
        return 1;
    }
}

