#pragma once

#include "core/point_types.h"
#include <cstddef>

namespace rvpoint {

/**
 * @brief Voxel Grid Downsampling (Scalar Reference).
 *
 * Reduces the number of points by creating a 3D voxel grid over the input point
 * cloud. All points within each voxel are approximated by their centroid.
 */
std::size_t voxel_grid_downsamp_sc(const PointXYZ *in, std::size_t n,
                                   PointXYZ *out, float leaf_size);

/**
 * @brief Voxel Grid Downsampling (RVV Hybrid — DEPRECATED, use _rvv_v2).
 */
[[deprecated("Use voxel_grid_downsamp_rvv_v2 — fully vectorized, no std::map")]]
std::size_t voxel_grid_downsamp_rvv(const PointCloudSoA &in, PointXYZ *out,
                                    float leaf_size);

/**
 * @brief Voxel Grid Downsampling (Fully Vectorized RVV, Sort-Based).
 *
 * Eliminates std::map by using a sort-based grouping approach:
 *  1. Vectorized bounding box computation (vfmin/vfmax + reductions)
 *  2. Vectorized voxel key computation (vfsub + vfmul + vfcvt_rtz + vmul/vadd)
 *  3. Sort indices by linear voxel key (groups same-voxel points contiguously)
 *  4. Vectorized centroid reduction per group (vluxei32 gather + vfredusum)
 */
std::size_t voxel_grid_downsamp_rvv_v2(const PointCloudSoA &in, PointXYZ *out,
                                       float leaf_size);

} // namespace rvpoint
