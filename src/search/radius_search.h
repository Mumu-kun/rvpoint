#pragma once

#include "core/point_types.h"
#include <cstddef>

namespace rvpoint {

/**
 * @brief Radius Search (Scalar Reference).
 */
std::size_t radius_search_sc(const PointXYZ *cloud, std::size_t n,
                             PointXYZ query, float radius, int *indices,
                             float *dists, int max_nn);

/**
 * @brief Radius Search (RVV Optimized).
 */
std::size_t radius_search_rvv(const PointCloudSoA &cloud, PointXYZ query,
                              float radius, int *indices, float *dists,
                              int max_nn);

} // namespace rvpoint
