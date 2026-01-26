#include <iostream>
#include <vector>
#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"

int main() {
    std::vector<rvv_pcl::PointXYZ> points;
    std::string filename = "bunny.pcd";
    
    int count = rvv_pcl::loadPCD(filename, points);
    
    if (count < 0) {
        std::cerr << "[FAIL] Failed to load " << filename << std::endl;
        return 1;
    }
    
    std::cout << "Loaded " << count << " points from " << filename << std::endl;
    
    if (count == 397) {
        std::cout << "Point 0: " << points[0].x << " " << points[0].y << " " << points[0].z << std::endl;
        std::cout << "[PASS] Correct number of points loaded." << std::endl;
        return 0;
    } else {
        std::cerr << "[FAIL] Expected 397 points, got " << count << std::endl;
        return 1;
    }
}
