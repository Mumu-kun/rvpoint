#include "include/rvv_pcl.h"
#include <cmath>
#include <map>
#include <tuple>
#include <algorithm>

namespace rvv_pcl {

// ============================================================================
// Scalar Implementation
// ============================================================================
std::size_t voxel_grid_downsamp_sc(const PointXYZ* in, std::size_t n,
                                   PointXYZ* out, float leaf_size) {
    if (n == 0) return 0;
    
    // Key: (vx, vy, vz), Value: (Sum coordinates, Count)
    // Using map to group points belonging to the same voxel
    std::map<std::tuple<int, int, int>, std::pair<PointXYZ, int>> grid;
    float inv_leaf = 1.0f / leaf_size;

    for (std::size_t i = 0; i < n; ++i) {
        int vx = std::floor(in[i].x * inv_leaf);
        int vy = std::floor(in[i].y * inv_leaf);
        int vz = std::floor(in[i].z * inv_leaf);
        auto key = std::make_tuple(vx, vy, vz);
        
        grid[key].first.x += in[i].x;
        grid[key].first.y += in[i].y;
        grid[key].first.z += in[i].z;
        grid[key].second++;
    }

    // Compute centroids and write to output
    std::size_t count = 0;
    for (auto& kv : grid) {
        float f = 1.0f / kv.second.second;
        out[count].x = kv.second.first.x * f;
        out[count].y = kv.second.first.y * f;
        out[count].z = kv.second.first.z * f;
        count++;
    }
    return count;
}

// ============================================================================
// RVV Implementation
// ============================================================================
std::size_t voxel_grid_downsamp_rvv(const PointCloudSoA& in,
                                    PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;

    std::map<std::tuple<int, int, int>, std::pair<PointXYZ, int>> grid;
    float inv_leaf = 1.0f / leaf_size;

    // In a full RVV implementation, we would also vectorize the aggregation (sorting/reducing).
    // However, hash map insertion is inherently scalar.
    // We WILL vectorize the transformation of coordinates to voxel indices.
    
    // Temporary storage for indices could be allocated, but for this hybrid approach
    // we will process in chunks (stripming) and then insert into map scalar-wise.
    // This allows us to use RVV for the FP math (mul + floor).

    size_t n = in.n;
    size_t i = 0;

    // Arrays to hold chunk results
    // Max VLEN is usually reasonable, assume max 256 or 512 elements for buffer logic if needed,
    // but here we just loop standard strip mining.


    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);

        // Load X, Y, Z
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);

        // Scale: val * inv_leaf
        vfloat32m8_t vsx = __riscv_vfmul_vf_f32m8(vx, inv_leaf, vl);
        vfloat32m8_t vsy = __riscv_vfmul_vf_f32m8(vy, inv_leaf, vl);
        vfloat32m8_t vsz = __riscv_vfmul_vf_f32m8(vz, inv_leaf, vl);

        // Floor: Uses vfcvt... but since we want integer indices:
        // C++ std::floor returns float. RVV vfcvt_x_f_v converts float to int (truncate).
        // Correct floor for positive/negative:
        // Custom floor logic or just cast if we assume logical behavior. 
        // For simplicity in this demo, we assume the conversion intrinsics available.
        // vcvt.x.f.v (which truncates toward zero) is not exactly floor for negatives.
        // We will stick to the scalar insertion for correctness if intricate instructions missing,
        // BUT let's do the floating point scale in vector.
        
        // Storing back to analyze one by one (Hybrid)
        // Ideally we would compress/scatter, but map insertion is serial.
        float raw_sx[vl], raw_sy[vl], raw_sz[vl];
        __riscv_vse32_v_f32m8(raw_sx, vsx, vl);
        __riscv_vse32_v_f32m8(raw_sy, vsy, vl);
        __riscv_vse32_v_f32m8(raw_sz, vsz, vl);

        for(size_t j=0; j<vl; ++j) {
            // "Scalar" part of the loop (Map Insertion)
            int idx_x = std::floor(raw_sx[j]);
            int idx_y = std::floor(raw_sy[j]);
            int idx_z = std::floor(raw_sz[j]);

            auto key = std::make_tuple(idx_x, idx_y, idx_z);
            grid[key].first.x += in.x[i+j];
            grid[key].first.y += in.y[i+j];
            grid[key].first.z += in.z[i+j];
            grid[key].second++;
        }

        i += vl;
    }

    // Compute centroids
    std::size_t count = 0;
    for (auto& kv : grid) {
        float f = 1.0f / kv.second.second;
        out[count].x = kv.second.first.x * f;
        out[count].y = kv.second.first.y * f;
        out[count].z = kv.second.first.z * f;
        count++;
    }
    return count;
}

} // namespace rvv_pcl
