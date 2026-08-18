#include "include/rvv_pcl.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace rvv_pcl {

// ============================================================================
// Scalar Implementation
// ============================================================================
std::size_t radius_search_sc(const PointXYZ* cloud, std::size_t n, 
                             PointXYZ query, float radius, 
                             int* indices, float* dists, int max_nn) {
    std::size_t count = 0;
    float r2 = radius * radius;

    for (std::size_t i = 0; i < n; ++i) {
        float dx = cloud[i].x - query.x;
        float dy = cloud[i].y - query.y;
        float dz = cloud[i].z - query.z;
        float d2 = dx*dx + dy*dy + dz*dz;

        if (d2 <= r2) {
            if (count < max_nn) {
                indices[count] = i;
                if (dists) dists[count] = d2; // Return squared distance
            }
            count++;
        }
    }
    // Return total found (even if > max_nn, typical PCL behavior is to return found count, 
    // but we only write up to max_nn. Let's return written count to be safe/simple C-style)
    return (count > max_nn) ? max_nn : count; 
}

// ============================================================================
// RVV Implementation
// ============================================================================
std::size_t radius_search_rvv(const PointCloudSoA& cloud, 
                              PointXYZ query, float radius, 
                              int* indices, float* dists, int max_nn) {
    if (cloud.n == 0) return 0;
    
    // 1. Compute distances for ALL points using the optimized kernel
    // (In a real massive PointCloud, we involves chunking, but here N < RAM)
    std::vector<float> all_dists(cloud.n);
    get_dist_sq_rvv(cloud.x, cloud.y, cloud.z, query.x, query.y, query.z, all_dists.data(), cloud.n);

    // 2. Filter (Scalar for now)
    // Vectorizing the "compress" operation (keeping indices where pred is true)
    // is possible with vcompress, but the main cost is usually the distance calculation.
    std::size_t count = 0;
    float r2 = radius * radius;

    for (std::size_t i = 0; i < cloud.n; ++i) {
        if (all_dists[i] <= r2) {
            if (count < max_nn) {
                indices[count] = i;
                if (dists) dists[count] = all_dists[i];
            }
            count++;
        }
    }
    return (count > max_nn) ? max_nn : count;
}

} // namespace rvv_pcl
