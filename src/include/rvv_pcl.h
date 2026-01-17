#pragma once
#include <cstddef>
#include <vector>

#ifdef __riscv_vector
  #include <riscv_vector.h>
#endif

namespace rvv_pcl {

struct PointXYZ {
  float x, y, z;
};

// Structure of Arrays (SoA) layout - Critical for RVV performance
struct PointCloudSoA {
  float* x;
  float* y;
  float* z;
  std::size_t n;
};

// Voxel Grid Downsampling
// Returns number of points in the output
std::size_t voxel_grid_downsamp_sc(const PointXYZ* in, std::size_t n,
                                   PointXYZ* out, float leaf_size);

std::size_t voxel_grid_downsamp_rvv(const PointCloudSoA& in,
                                    PointXYZ* out, float leaf_size);

} // namespace rvv_pcl
