#include <iostream>
#include <vector>
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

int main() {
    std::vector<rvv_pcl::PointXYZ> points;
    std::string filename = "bunny.pcd";
    
    int count = rvv_pcl::loadPCD(filename, points);
    if (count < 0) count = rvv_pcl::loadPCD("data/" + filename, points);
    if (count < 0) count = rvv_pcl::loadPCD("../data/" + filename, points);
    if (count < 0) count = rvv_pcl::loadPCD("../" + filename, points);
    if (count < 0) count = rvv_pcl::loadPCD("/workspace/data/" + filename, points);
    if (count < 0) count = rvv_pcl::loadPCD("/workspace/" + filename, points);
    
    if (count < 0) {
        std::cerr << "[FAIL] Failed to load " << filename << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << count << " points from " << filename << std::endl;
    
    if (count > 0) {
        std::cout << "Point 0: " << points[0].x << " " << points[0].y << " " << points[0].z << std::endl;
        std::cout << "[PASS] Successfully loaded " << count << " points." << std::endl;
        return 0;
    } else {
        std::cerr << "[FAIL] No points loaded from " << filename << std::endl;
        return 1;
    }
}

