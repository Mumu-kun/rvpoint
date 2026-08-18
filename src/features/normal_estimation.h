#pragma once

#include "core/point_types.h"
#include <cstddef>

namespace rvpoint {

class Octree;
class SpatialHash;
class PointerOctree;

/**
 * @brief Normal Estimation (Scalar Reference).
 */
void normal_estimation_sc(const PointXYZ *in, std::size_t n, float *nx,
                          float *ny, float *nz, int k, float radius,
                          float vp_x = 0, float vp_y = 0, float vp_z = 0,
                          int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV with Octree).
 */
void normal_estimation_rvv(const PointCloudSoA &in, const Octree &octree,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV with SpatialHash).
 */
void normal_estimation_rvv(const PointCloudSoA &in, const SpatialHash &hash,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV with PointerOctree).
 */
void normal_estimation_rvv(const PointCloudSoA &in, const PointerOctree &octree,
                           float *nx, float *ny, float *nz, int k, float radius,
                           float vp_x = 0, float vp_y = 0, float vp_z = 0,
                           int eigen_iters = 4);

/**
 * @brief Normal Estimation (RVV, self-contained — builds PointerOctree internally).
 */
void normal_estimation_rvv(const PointCloudSoA &in, float *nx, float *ny,
                           float *nz, int k, float radius, float vp_x = 0,
                           float vp_y = 0, float vp_z = 0, int eigen_iters = 4);

} // namespace rvpoint
