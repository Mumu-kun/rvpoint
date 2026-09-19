#pragma once

#include <cstdint>
#include <algorithm>
#include "core/point_types.h"
#include "control/actuators/motor_actuator.h"

namespace rvpoint {

/**
 * @brief Parameters defining the dynamic vehicle stopping envelope and hazard sensitivity.
 */
struct SafetyCorridorParams {
    float car_width_m = 0.18f;           ///< Vehicle track width (meters)
    float car_height_m = 0.20f;          ///< Vehicle height / roof clearance (meters)
    float lateral_margin_m = 0.04f;      ///< Safety buffer on each side (meters)
    float min_forward_dist_m = 0.05f;    ///< Front bumper minimum threshold (meters)
    float base_margin_m = 0.20f;         ///< Minimum stopping distance when stationary (meters)
    float reaction_time_s = 0.05f;       ///< Processing & actuation latency buffer (seconds)
    float max_decel_mps2 = 1.5f;         ///< Conservative braking deceleration (m/s^2)
    float min_obstacle_height_m = 0.03f; ///< Clearance above ground to avoid residual ground noise
    uint32_t trigger_point_threshold = 10; ///< Minimum obstacle points inside corridor to trip E-stop
};

/**
 * @brief Result payload from forward corridor safety evaluation.
 */
struct SafetyEvaluationResult {
    bool emergency_stop_triggered = false;
    uint32_t intrusion_points_count = 0;
    float stopping_distance_m = 0.0f;
    float nearest_hazard_x_m = 1e9f;
};

/**
 * @brief Instantaneous reactive safety corridor evaluator and software E-stop trigger.
 *
 * Checks non-ground obstacle points against a dynamic stopping corridor parameterized by
 * vehicle speed vx. If obstacle points intrude into the volume, trips an emergency stop
 * and commands MotorActuator::emergency_brake() within sub-millisecond latency.
 */
class ForwardCorridorSafetyFilter {
public:
    explicit ForwardCorridorSafetyFilter(SafetyCorridorParams params = SafetyCorridorParams{})
        : params_(params) {}

    const SafetyCorridorParams& params() const noexcept { return params_; }
    void set_params(const SafetyCorridorParams& params) noexcept { params_ = params; }

    /**
     * @brief Compute dynamic stopping distance d_stop = d_margin + vx * t_reaction + vx^2 / (2 * a_max).
     */
    float compute_stopping_distance(float vx_mps) const noexcept {
        float v = (vx_mps > 0.0f) ? vx_mps : 0.0f;
        return params_.base_margin_m + v * params_.reaction_time_s + (v * v) / (2.0f * params_.max_decel_mps2);
    }

    /**
     * @brief Evaluate obstacle point cloud against dynamic safety corridor.
     * @param obstacles Point cloud in ISO 8855 body frame (+X forward, +Y left, +Z up).
     * @param vx_mps Current vehicle forward velocity in m/s.
     * @return SafetyEvaluationResult detailing hazard presence, point count, and nearest hazard.
     */
    SafetyEvaluationResult evaluate(const PointCloud& obstacles, float vx_mps) const;

    /**
     * @brief Evaluates obstacles and directly actuates emergency_brake() if a hazard is detected.
     * @param obstacles Obstacle point cloud in body frame.
     * @param vx_mps Current vehicle forward velocity.
     * @param actuator Motor actuator seam to brake upon hazard detection.
     * @param result_out Optional pointer to receive detailed safety result.
     * @return true if emergency stop was triggered, false if corridor is clear.
     */
    bool evaluate_and_actuate(const PointCloud& obstacles,
                              float vx_mps,
                              MotorActuator& actuator,
                              SafetyEvaluationResult* result_out = nullptr);

private:
    SafetyCorridorParams params_;
};

} // namespace rvpoint

