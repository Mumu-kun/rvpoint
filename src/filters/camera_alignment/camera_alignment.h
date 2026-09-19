#pragma once

#include <cstddef>
#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Configuration parameters for CameraAlignment extrinsics leveling.
 */
struct CameraAlignmentParams {
    float camera_pitch_deg = 10.0f;     ///< Nominal fallback tilt (degrees, pitched down towards floor)
    float mount_height_m = 0.12f;       ///< Camera optical center height above ground (meters)
    float mount_x_offset_m = 0.05f;     ///< Forward offset from vehicle origin (meters)
    float mount_y_offset_m = 0.0f;      ///< Lateral offset from centerline (meters, +Left)
    float max_ground_angle_deg = 15.0f; ///< Max angular discrepancy between gravity UP and plane normal before rejecting (degrees)
};

/**
 * @brief Gravity-leveled optical-to-ISO 8855 body frame transformer.
 *
 * Implements closed-form orthonormal rotation and translation from optical camera
 * coordinates (+X right, +Y down, +Z forward) into the ISO 8855 vehicle body frame
 * (+X forward, +Y left, +Z up, with Z = 0 at the level physical ground surface).
 *
 * Invariant: The body +Z axis ALWAYS aligns strictly with gravity (anti-parallel to g_cam).
 * When a ground PlaneModel is provided, it is validated against gravity (angle <= max_ground_angle_deg);
 * if accepted, the exact vertical height along the gravity axis h_vert = d / (n . u_z) is calibrated.
 * If the plane disagrees with gravity (> max_ground_angle_deg, e.g. a wall), it is rejected
 * and falls back to nominal mount_height_m.
 */
class CameraAlignment {
public:
    explicit CameraAlignment(CameraAlignmentParams params = CameraAlignmentParams{})
        : params_(params) {}

    const CameraAlignmentParams& params() const noexcept { return params_; }
    void set_params(const CameraAlignmentParams& params) noexcept { params_ = params; }

    /**
     * @brief Validates whether a ground plane normal agrees with gravity within max_ground_angle_deg.
     * @param gravity Live gravity vector [gx, gy, gz].
     * @param ground_plane Candidate ground plane equation (ax + by + cz + d = 0).
     * @param out_vertical_height Optional output receiving vertical distance from camera to plane along gravity.
     * @return true if plane is accepted as valid ground, false if rejected (wall/obstacle).
     */
    bool is_ground_plane_valid(const float gravity[3],
                               const PlaneModel& ground_plane,
                               float* out_vertical_height = nullptr) const;

    /**
     * @brief Computes extrinsics using live CoreMotion gravity vector [gx, gy, gz].
     */
    void compute_extrinsics(const float gravity[3], float R[9], float t[3]) const;

    /**
     * @brief Computes extrinsics using an optical-frame ground PlaneModel (ax + by + cz + d = 0).
     *
     * Invariant: Body Z-axis aligns with nominal gravity. Validates plane angle against nominal gravity;
     * if valid, calibrates vertical height h_vert = d / (n . u_z). If rejected, falls back to mount_height_m.
     */
    void compute_extrinsics(const PlaneModel& ground_plane, float R[9], float t[3]) const;

    /**
     * @brief Computes extrinsics using gravity for orientation and validated ground plane for vertical height.
     *
     * Body Z-axis ALWAYS aligns with gravity. If angle(n, u_z) <= max_ground_angle_deg,
     * t_z = h_vert = d / (n . u_z). Otherwise, falls back to nominal mount_height_m.
     */
    void compute_extrinsics(const float gravity[3], const PlaneModel& ground_plane, float R[9], float t[3]) const;

    /**
     * @brief Transform point cloud from camera optical frame to ISO 8855 vehicle body frame.
     * @param cam_cloud Input points in camera coordinates.
     * @param gravity Live gravity vector in camera frame [gx, gy, gz].
     * @param body_cloud Output points in ISO 8855 body coordinates (+X forward, +Y left, +Z up).
     */
    void transform_to_body(const PointCloud& cam_cloud,
                           const float gravity[3],
                           PointCloud& body_cloud) const;

    /**
     * @brief Transform points using optical ground PlaneModel (aligns ground plane exactly to Z = 0).
     */
    void transform_to_body(const PointCloud& cam_cloud,
                           const PlaneModel& ground_plane,
                           PointCloud& body_cloud) const;

    /**
     * @brief Transform points using gravity for orientation and PlaneModel for vertical leveling.
     */
    void transform_to_body(const PointCloud& cam_cloud,
                           const float gravity[3],
                           const PlaneModel& ground_plane,
                           PointCloud& body_cloud) const;

    void operator()(const PointCloud& cam_cloud,
                    const float gravity[3],
                    PointCloud& body_cloud) const {
        transform_to_body(cam_cloud, gravity, body_cloud);
    }

    void operator()(const PointCloud& cam_cloud,
                    const PlaneModel& ground_plane,
                    PointCloud& body_cloud) const {
        transform_to_body(cam_cloud, ground_plane, body_cloud);
    }

private:
    CameraAlignmentParams params_;
};

} // namespace rvpoint
