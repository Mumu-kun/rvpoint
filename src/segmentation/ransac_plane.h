#pragma once

#include "core/point_types.h"
#include <cstddef>

namespace rvpoint {

/**
 * @brief RANSAC Plane Fitting (Scalar Reference).
 */
int ransac_plane_sc(const PointXYZ *cloud, std::size_t n, float dist_thresh,
                    int max_iters, float *model,
                    float collinear_thresh = 1e-6f,
                    float probability = 0.99f);

/**
 * @brief RANSAC Plane Fitting (RVV Optimized).
 */
int ransac_plane_rvv(const PointCloudSoA &cloud, float dist_thresh,
                     int max_iters, float *model,
                     float collinear_thresh = 1e-6f,
                     float probability = 0.99f);

/**
 * @brief Extract Plane Inliers (RVV Optimized).
 */
std::size_t extract_plane_inliers_rvv(const PointCloudSoA &cloud,
                                      const float *model, float dist_thresh,
                                      PointXYZ *inliers);

/**
 * @brief Extract Plane Outliers (RVV Optimized).
 */
std::size_t extract_plane_outliers_rvv(const PointCloudSoA &cloud,
                                       const float *model, float dist_thresh,
                                       PointXYZ *outliers);

/**
 * @brief Extract Both Inliers and Outliers in Single Pass (RVV Optimized).
 */
void extract_plane_inliers_outliers_rvv(const PointCloudSoA &cloud,
                                        const float *model, float dist_thresh,
                                        PointXYZ *inliers, PointXYZ *outliers,
                                        std::size_t &n_inliers,
                                        std::size_t &n_outliers);

} // namespace rvpoint
