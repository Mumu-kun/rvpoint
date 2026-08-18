#pragma once

#include "core/point_types.h"
#include <cstddef>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

/**
 * @brief Helper: Squared Euclidean Distance Kernel (RVV).
 *
 * Computes d^2 = (x-qx)^2 + (y-qy)^2 + (z-qz)^2 for 'n' points using vector
 * instructions.
 */
void get_dist_sq_rvv(const float *x, const float *y, const float *z, float qx,
                     float qy, float qz, float *out_d2, std::size_t n);

/**
 * @brief Fused Gather-Filter Kernel (RVV).
 *
 * Reads x,y,z at specified 'indices' (indirect addressing), computes distance
 * to query, and stores matching indices/distances.
 */
void get_inds_in_radius_rvv(const float *x, const float *y, const float *z,
                            const int *subset_indices, std::size_t n, float qx,
                            float qy, float qz, float r2,
                            std::vector<int> &out_indices,
                            std::vector<float> &out_dists);

} // namespace rvpoint
