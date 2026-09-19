#pragma once

#include "core/point_types.h"
#include <cstddef>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

/**
 * @brief Maximum vector register capacity for LMUL=8 across RISC-V VLEN architectures.
 *
 * Sized for hardware VLEN up to 1024 bits:
 *   Max float32 elements with LMUL=8: (1024 / 32) * 8 = 256 elements (1 KB stack buffer).
 *   Max mask bytes for vbool4_t:      256 / 8 = 32 bytes.
 */
constexpr std::size_t kMaxVectorFloatsM8 = 256;
constexpr std::size_t kMaxVectorMaskBytesM8 = kMaxVectorFloatsM8 / 8; // 32 bytes

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
