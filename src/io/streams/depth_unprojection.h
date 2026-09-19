#pragma once

#include <cstdint>
#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Vectorized pinhole depth map unprojection directly into PointCloud SoA.
 *
 * @param depth Raw float depth map pointer (metric meters)
 * @param conf Raw uint8 confidence map pointer (0: Low, 1: Medium, 2: High; can be nullptr)
 * @param w Width of depth map (e.g. 256)
 * @param h Height of depth map (e.g. 192)
 * @param fx Focal length X
 * @param fy Focal length Y
 * @param cx Principal point X
 * @param cy Principal point Y
 * @param min_conf Minimum confidence threshold (typically 2)
 * @param min_range Minimum valid range in meters (typically 0.25m)
 * @param max_range Maximum valid range in meters (typically 4.5m)
 * @param[out] cloud Pre-allocated PointCloud destination buffer
 */
void unproject_depth_map(const float* depth,
                         const uint8_t* conf,
                         uint32_t w, uint32_t h,
                         float fx, float fy, float cx, float cy,
                         uint8_t min_conf,
                         float min_range,
                         float max_range,
                         PointCloud& cloud);

} // namespace rvpoint

