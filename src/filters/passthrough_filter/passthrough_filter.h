#pragma once

#include <cstddef>
#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Axis interval range.
 */
struct AxisRange {
    float min_val = -1e9f;
    float max_val = 1e9f;

    bool contains(float val) const noexcept {
        return val >= min_val && val <= max_val;
    }
};

/**
 * @brief PassThroughFilter configuration.
 */
struct PassThroughParams {
    AxisRange x_range{-1e9f, 1e9f};
    AxisRange y_range{-1e9f, 1e9f};
    AxisRange z_range{-1e9f, 1e9f};
    bool keep_inliers = true; ///< true: keep points inside range; false: keep points outside
};

/**
 * @brief Zero-heap axis-aligned bounding interval and elevation slicing filter.
 *
 * Slices 3D point clouds along X, Y, and/or Z axes. Can be used for:
 * 1. Ground and ceiling removal (e.g. Z in [0.03m, 0.60m]).
 * 2. Forward sensor range gating (e.g. X in [0.10m, 4.00m]).
 * 3. Driving corridor lateral cropping (e.g. Y in [-2.0m, +2.0m]).
 *
 * Accelerated via RVV 1.0 SIMD vector comparisons and stream compaction.
 */
class PassThroughFilter {
public:
    explicit PassThroughFilter(PassThroughParams params = PassThroughParams{});
    explicit PassThroughFilter(float min_z, float max_z);

    const PassThroughParams& params() const noexcept { return params_; }
    void set_params(const PassThroughParams& params) noexcept { params_ = params; }

    void set_limits_z(float min_z, float max_z) noexcept {
        params_.z_range = {min_z, max_z};
    }

    void set_limits_x(float min_x, float max_x) noexcept {
        params_.x_range = {min_x, max_x};
    }

    void set_limits_y(float min_y, float max_y) noexcept {
        params_.y_range = {min_y, max_y};
    }

    void set_keep_inliers(bool keep) noexcept {
        params_.keep_inliers = keep;
    }

    /**
     * @brief Filters input points according to configured axis limits.
     * @param in Input point cloud.
     * @param out Output point cloud receiving passed points.
     * @param rejected Optional destination cloud receiving rejected points.
     */
    void filter(const PointCloud& in, PointCloud& out, PointCloud* rejected = nullptr) const;

    void operator()(const PointCloud& in, PointCloud& out, PointCloud* rejected = nullptr) const {
        filter(in, out, rejected);
    }

private:
    PassThroughParams params_;
};

} // namespace rvpoint
