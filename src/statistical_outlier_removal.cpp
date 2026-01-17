#include "include/rvv_pcl.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

namespace rvv_pcl {

// ============================================================================
// Scalar Implementation
// ============================================================================
std::size_t sor_sc(const PointXYZ* in, std::size_t n,
                   PointXYZ* out, int k, float alpha) {
    if (n == 0) return 0;
    std::vector<float> mean_dists(n);
    std::vector<float> dists(n);

    // 1. Compute mean K-NN distance for each point
    for(size_t i=0; i<n; ++i) {
        // Compute distances to all other points
        for(size_t j=0; j<n; ++j) {
            float dx = in[i].x - in[j].x;
            float dy = in[i].y - in[j].y;
            float dz = in[i].z - in[j].z;
            dists[j] = dx*dx + dy*dy + dz*dz;
        }

        // Find k nearest neighbors (1 to k, 0 is self)
        std::partial_sort(dists.begin(), dists.begin() + k + 1, dists.end());
        
        float sum = 0;
        for(int j=1; j<=k; ++j) sum += std::sqrt(dists[j]);
        mean_dists[i] = sum / k;
    }

    // 2. Compute Global Statistics
    float global_sum = 0;
    for(float d : mean_dists) global_sum += d;
    float global_mean = global_sum / n;

    float variance_sum = 0;
    for(float d : mean_dists) variance_sum += (d - global_mean)*(d - global_mean);
    float global_std = std::sqrt(variance_sum / n);
    
    // 3. Filter
    float thresh = global_mean + alpha * global_std;
    std::size_t count = 0;
    for(size_t i=0; i<n; ++i) {
        if(mean_dists[i] <= thresh) {
            out[count++] = in[i];
        }
    }
    return count;
}

// ============================================================================
// RVV Implementation
// ============================================================================
std::size_t sor_rvv(const PointCloudSoA& in,
                    PointXYZ* out, int k, float alpha) {
    if (in.n == 0) return 0;
    std::vector<float> mean_dists(in.n);
    std::vector<float> dists(in.n); 

    // 1. Compute mean K-NN distance using RVV Kernel
    for(size_t i=0; i<in.n; ++i) {
        // Accelerate the inner O(N) loop with vector instructions
        get_dist_sq_rvv(in.x, in.y, in.z, in.x[i], in.y[i], in.z[i], dists.data(), in.n);
        
        // Scalar Sort (still fast enough relative to N^2 distance calc)
        std::partial_sort(dists.begin(), dists.begin() + k + 1, dists.end());
        
        float sum = 0;
        for(int j=1; j<=k; ++j) sum += std::sqrt(dists[j]);
        mean_dists[i] = sum / k;
    }

    // 2. Compute Global Statistics (Scalar reduction - effectively O(N))
    // Could be vectorized with reductions, but N is small relative to the O(N^2) above.
    float global_sum = 0;
    for(float d : mean_dists) global_sum += d;
    float global_mean = global_sum / in.n;

    float variance_sum = 0;
    for(float d : mean_dists) variance_sum += (d - global_mean)*(d - global_mean);
    float global_std = std::sqrt(variance_sum / in.n);

    // 3. Filter
    float thresh = global_mean + alpha * global_std;
    std::size_t count = 0;
    for(size_t i=0; i<in.n; ++i) {
        if(mean_dists[i] <= thresh) {
            out[count].x = in.x[i];
            out[count].y = in.y[i];
            out[count].z = in.z[i];
            count++;
        }
    }
    return count;
}

} // namespace rvv_pcl
