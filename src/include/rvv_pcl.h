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

// 2) Statistical Outlier Removal
std::size_t sor_sc(const PointXYZ* in, std::size_t n,
                   PointXYZ* out, int k, float alpha);

std::size_t sor_rvv(const PointCloudSoA& in,
                    PointXYZ* out, int k, float alpha);

// 3) Normal Estimation (output normals per point)
void normal_estimation_sc(const PointXYZ* in, std::size_t n,
                          float* nx, float* ny, float* nz, int k);

void normal_estimation_rvv(const PointCloudSoA& in,
                           float* nx, float* ny, float* nz, int k);

// 4) Radius Search
std::size_t radius_search_sc(const PointXYZ* cloud, std::size_t n, 
                             PointXYZ query, float radius, 
                             int* indices, float* dists, int max_nn);

std::size_t radius_search_rvv(const PointCloudSoA& cloud, 
                              PointXYZ query, float radius, 
                              int* indices, float* dists, int max_nn);

// 5) RANSAC Plane Fitting
// Returns number of inliers. 'model' must be float[4] (a,b,c,d).
int ransac_plane_sc(const PointXYZ* cloud, std::size_t n, 
                    float dist_thresh, int max_iters, float* model);

int ransac_plane_rvv(const PointCloudSoA& cloud, 
                     float dist_thresh, int max_iters, float* model);

// Helper: Squared Euclidean Distance Kernel (RVV)
// Computes d2[i] = dist_sq( (x[i],y[i],z[i]), q )
void get_dist_sq_rvv(const float* x, const float* y, const float* z,
                     float qx, float qy, float qz,
                     float* out_d2, std::size_t n);

} // namespace rvv_pcl
