#pragma once

#include "rvv_pcl.h"
#include <cstdint>
#include <vector>

namespace rvv_pcl {

/**
 * @brief Strategy 1: Grid-Caravan Bounding Box Pruned SOR (O(N_local * Q / VL)).
 * Computes tile AABB + radius, queries SpatialHash cells, and streams candidate points into vector registers.
 */
std::size_t sor_grid_caravan(const PointCloudSoA &in, PointXYZ *out, int k, float alpha, float search_radius = 0.5f);

/**
 * @brief Strategy 2: Pure Hardware SIMD Register Selection SOR (O(N * Q * K / VL)).
 * Keeps K=20 top distance registers resident in hardware vector registers (zero stack heap operations).
 */
std::size_t sor_caravan_simd_select(const PointCloudSoA &in, PointXYZ *out, int k, float alpha);

/**
 * @brief Strategy 3: Coarse Voxel Centroid Streaming SOR (O(N_voxels * Q / VL)).
 * Streams coarse voxel centroids to reject 50-100 points per vector distance check.
 */
std::size_t sor_caravan_voxel(const PointCloudSoA &in, PointXYZ *out, int k, float alpha, float leaf_size = 0.2f);

/**
 * @brief Strategy 4: Fixed-Radius Bitmask & Distance Sum Reduction SOR (O(N * Q / VL)).
 * Replaces K-NN sorting with fixed-radius bitmask distance summation using masked vector addition.
 */
std::size_t sor_caravan_radius_bitmask(const PointCloudSoA &in, PointXYZ *out, float search_radius, float alpha);

} // namespace rvv_pcl
