#pragma once

#include <cstddef>
#include <vector>

namespace rvpoint {

/**
 * @brief Standard 3D Point structure.
 * Simple structure representing a point in 3D space with float coordinates.
 */
struct PointXYZ {
  float x, y, z;
};

/**
 * @brief Point Cloud stored in Structure of Arrays (SoA) layout.
 *
 * This layout is critical for RISC-V Vector (RVV) performance as it allows
 * for unit-stride loads/stores (vle32.v / vse32.v), which are significantly
 * faster than strided gather/scatter operations required for Array of
 * Structures (AoS).
 */
struct PointCloudSoA {
  float *x = nullptr;      /**< Pointer to array of X coordinates */
  float *y = nullptr;      /**< Pointer to array of Y coordinates */
  float *z = nullptr;      /**< Pointer to array of Z coordinates */
  std::size_t n = 0;       /**< Number of points in the cloud */
};

} // namespace rvpoint
