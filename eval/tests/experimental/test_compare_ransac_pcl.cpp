#include "io/simple_pcd_loader.h"
#include "include/rvpoint.h"
#include <iostream>
#include <vector>
#include <cmath>

using namespace rvpoint;

int main() {
    std::string path = "data/pcd_compressed/0000000007.pcd";
    std::vector<PointXYZ> pts;
    if (loadPCD(path, pts) < 0) return 1;

    std::vector<float> x(pts.size()), y(pts.size()), z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { x[i] = pts[i].x; y[i] = pts[i].y; z[i] = pts[i].z; }
    PointCloudSoA raw_cloud{x.data(), y.data(), z.data(), pts.size()};

    std::vector<PointXYZ> ds(pts.size());
    size_t nd = voxel_grid_downsamp_rvv_v2(raw_cloud, ds.data(), 0.10f);
    ds.resize(nd);

    // Let's check how many inliers exist for different plane angles on 0000000007.pcd
    std::cout << "Downsampled pts: " << nd << std::endl;

    return 0;
}
