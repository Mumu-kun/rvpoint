#pragma once

#include <cstddef>
#include <cstdint>

namespace rvpoint {

/**
 * @brief Standard 3D Point structure.
 * Simple structure representing a point in 3D space with float coordinates.
 */
struct PointXYZ {
  float x, y, z;
};

struct PointXYZI {
  float x, y, z;
  float intensity;
};

struct PointXYZRGB {
  float x, y, z;
  uint8_t r, g, b;
};

struct Normal {
  float normal_x, normal_y, normal_z;
  float curvature;
};

struct PointNormal {
  float x, y, z;
  float normal_x, normal_y, normal_z;
  float curvature;
};

/**
 * @brief Point Cloud stored in Structure of Arrays (SoA) layout.
 */
struct PointCloudSoA {
  float *x = nullptr;      /**< Pointer to array of X coordinates */
  float *y = nullptr;      /**< Pointer to array of Y coordinates */
  float *z = nullptr;      /**< Pointer to array of Z coordinates */
  std::size_t n = 0;       /**< Number of points in the cloud */
};

} // namespace rvpoint

namespace rvv_pcl = rvpoint;
