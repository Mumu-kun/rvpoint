#pragma once

#include "core/point_types.h"
#include <cstddef>

namespace rvpoint {

class Octree;
class SpatialHash;
class PointerOctree;

/**
 * @brief Statistical Outlier Removal (Scalar Reference).
 *
 * Removes points that are further away from their neighbors compared to the
 * average.
 */
std::size_t sor_sc(const PointXYZ *in, std::size_t n, PointXYZ *out, int k,
                   float alpha);

/**
 * @brief Statistical Outlier Removal (RVV Optimized).
 */
std::size_t sor_rvv(const PointCloudSoA &in, PointXYZ *out, int k, float alpha);

/**
 * @brief Index-Accelerated SOR using Octree (O(N log N)).
 */
std::size_t sor_octree(const PointCloudSoA &in, const Octree &tree, PointXYZ *out,
                       int k, float alpha, float search_radius = 0.5f);

/**
 * @brief Index-Accelerated SOR using SpatialHash (O(N)).
 */
std::size_t sor_spatial_hash(const PointCloudSoA &in, const SpatialHash &hash, PointXYZ *out,
                             int k, float alpha, float search_radius = 0.5f);

/**
 * @brief Index-Accelerated SOR using PointerOctree (O(N log N)).
 */
std::size_t sor_pointer_octree(const PointCloudSoA &in, const PointerOctree &tree, PointXYZ *out,
                               int k, float alpha, float search_radius = 0.5f);

} // namespace rvpoint
