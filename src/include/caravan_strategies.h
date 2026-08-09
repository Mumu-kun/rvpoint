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

} // namespace rvv_pcl
