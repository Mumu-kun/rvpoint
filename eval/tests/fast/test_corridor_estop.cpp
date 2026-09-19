#include <cmath>
#include <iostream>
#include <vector>
#include <cstdlib>

#include "filters/corridor_safety_filter/corridor_safety_filter.h"
#include "control/actuators/mock_motor_actuator.h"

using namespace rvpoint;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "    [FAIL] " << msg << " (" #cond ") at line " << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

static void test_stopping_distance_formula() {
    std::cout << "[1] Verifying dynamic stopping distance math..." << std::endl;
    SafetyCorridorParams params;
    params.base_margin_m = 0.20f;
    params.reaction_time_s = 0.05f;
    params.max_decel_mps2 = 1.5f;

    ForwardCorridorSafetyFilter filter(params);

    // Static
    float d0 = filter.compute_stopping_distance(0.0f);
    TEST_CHECK(std::abs(d0 - 0.20f) < 1e-4f, "Static d_stop must equal base margin");

    // Negative velocity clamped
    float d_neg = filter.compute_stopping_distance(-0.4f);
    TEST_CHECK(std::abs(d_neg - 0.20f) < 1e-4f, "Negative velocity clamped to base margin");

    // Dynamic at 0.5 m/s: 0.20 + 0.5*0.05 + 0.25 / 3.0 = 0.20 + 0.025 + 0.08333 = 0.30833m
    float d_half = filter.compute_stopping_distance(0.5f);
    TEST_CHECK(std::abs(d_half - 0.308333f) < 1e-4f, "d_stop at 0.5 m/s");

    // Dynamic at 1.0 m/s: 0.20 + 1.0*0.05 + 1.0 / 3.0 = 0.20 + 0.05 + 0.33333 = 0.58333m
    float d_one = filter.compute_stopping_distance(1.0f);
    TEST_CHECK(std::abs(d_one - 0.583333f) < 1e-4f, "d_stop at 1.0 m/s");

    std::cout << "    [PASS] Dynamic stopping distance mathematically verified." << std::endl;
}

static void test_hazard_corridor_bounds() {
    std::cout << "[2] Verifying safety corridor spatial bounding volume..." << std::endl;
    SafetyCorridorParams params;
    params.car_width_m = 0.18f;
    params.lateral_margin_m = 0.04f; // Half-width margin: 0.09 + 0.04 = 0.13m
    params.car_height_m = 0.20f;
    params.min_obstacle_height_m = 0.03f;
    params.base_margin_m = 0.25f;
    params.trigger_point_threshold = 5;

    ForwardCorridorSafetyFilter filter(params);

    PointCloud cloud;
    // 10 points laterally outside corridor (Y = 0.20m > 0.13m)
    for (int i = 0; i < 10; ++i) cloud.push_back(0.15f, 0.20f, 0.10f);
    // 10 points overhead (Z = 0.25m > 0.20m)
    for (int i = 0; i < 10; ++i) cloud.push_back(0.15f, 0.0f, 0.25f);
    // 10 points below min height (ground noise, Z = 0.01m < 0.03m)
    for (int i = 0; i < 10; ++i) cloud.push_back(0.15f, 0.0f, 0.01f);
    // 10 points beyond forward stopping distance (X = 0.40m > 0.25m)
    for (int i = 0; i < 10; ++i) cloud.push_back(0.40f, 0.0f, 0.10f);

    SafetyEvaluationResult res = filter.evaluate(cloud, 0.0f);
    TEST_CHECK(res.intrusion_points_count == 0, "No points should penetrate safety volume");
    TEST_CHECK(!res.emergency_stop_triggered, "E-stop must not trigger on out-of-volume points");

    std::cout << "    [PASS] Lateral, vertical, ground clearance, and range boundaries verified." << std::endl;
}

static void test_noise_threshold_gating() {
    std::cout << "[3] Verifying noise rejection threshold gating..." << std::endl;
    SafetyCorridorParams params;
    params.trigger_point_threshold = 10;
    ForwardCorridorSafetyFilter filter(params);

    PointCloud cloud;
    // Inject 5 stray noise points directly in the corridor
    for (int i = 0; i < 5; ++i) {
        cloud.push_back(0.15f, 0.0f, 0.08f);
    }

    SafetyEvaluationResult res = filter.evaluate(cloud, 0.0f);
    TEST_CHECK(res.intrusion_points_count == 5, "Should register 5 intrusion points");
    TEST_CHECK(!res.emergency_stop_triggered, "Sub-threshold count (5 < 10) must NOT trigger E-stop");

    // Add 6 more points (total 11 > 10)
    for (int i = 0; i < 6; ++i) {
        cloud.push_back(0.12f + i * 0.01f, 0.02f, 0.09f);
    }

    res = filter.evaluate(cloud, 0.0f);
    TEST_CHECK(res.intrusion_points_count == 11, "Should register 11 intrusion points");
    TEST_CHECK(res.emergency_stop_triggered, "Suprathreshold count (11 >= 10) MUST trigger E-stop");
    TEST_CHECK(std::abs(res.nearest_hazard_x_m - 0.12f) < 1e-4f, "Nearest hazard distance recorded");

    std::cout << "    [PASS] False-positive suppression threshold verified." << std::endl;
}

static void test_reactive_actuation_estop() {
    std::cout << "[4] Verifying direct reactive E-stop motor braking..." << std::endl;
    SafetyCorridorParams params;
    params.trigger_point_threshold = 10;
    ForwardCorridorSafetyFilter filter(params);

    MockMotorActuator mock_motor;
    // Simulate car cruising forward at 50% duty cycle
    mock_motor.set_duty_cycles(0.5f, 0.5f);
    TEST_CHECK(mock_motor.last_command().duty_left == 0.5f, "Motor initialized forward");
    TEST_CHECK(!mock_motor.is_emergency_stopped(), "Motor not in E-stop");

    // 15 obstacle points intruding into forward stopping path
    PointCloud obstacles;
    for (int i = 0; i < 15; ++i) {
        obstacles.push_back(0.15f, 0.0f, 0.10f);
    }

    SafetyEvaluationResult result;
    bool triggered = filter.evaluate_and_actuate(obstacles, 0.4f, mock_motor, &result);

    TEST_CHECK(triggered, "evaluate_and_actuate returned true");
    TEST_CHECK(result.emergency_stop_triggered, "Result flagged emergency stop");
    TEST_CHECK(mock_motor.is_emergency_stopped(), "Mock motor placed into E-STOP state");
    TEST_CHECK(mock_motor.last_command().duty_left == 0.0f, "Left motor duty cut to 0.0 immediately");
    TEST_CHECK(mock_motor.last_command().duty_right == 0.0f, "Right motor duty cut to 0.0 immediately");

    std::cout << "    [PASS] Reactive reflex instantaneous power cutoff verified." << std::endl;
}

static void test_dynamic_velocity_expansion() {
    std::cout << "[5] Verifying dynamic corridor expansion with speed..." << std::endl;
    SafetyCorridorParams params;
    params.base_margin_m = 0.20f;
    params.trigger_point_threshold = 5;
    ForwardCorridorSafetyFilter filter(params);

    PointCloud cloud;
    // Obstacle at X = 0.45m
    for (int i = 0; i < 10; ++i) {
        cloud.push_back(0.45f, 0.0f, 0.10f);
    }

    // At stationary (vx = 0), d_stop = 0.20m -> Obstacle at 0.45m is safe!
    SafetyEvaluationResult res_static = filter.evaluate(cloud, 0.0f);
    TEST_CHECK(!res_static.emergency_stop_triggered, "Obstacle outside stationary envelope");

    // At speed (vx = 1.0 m/s), d_stop = 0.583m -> Obstacle at 0.45m is an immediate collision hazard!
    SafetyEvaluationResult res_fast = filter.evaluate(cloud, 1.0f);
    TEST_CHECK(res_fast.emergency_stop_triggered, "Obstacle inside high-speed envelope");
    TEST_CHECK(res_fast.intrusion_points_count == 10, "All 10 points detected in expanded envelope");

    std::cout << "    [PASS] Dynamic speed-dependent hazard envelope expansion verified." << std::endl;
}

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_corridor_estop (Forward Safety Corridor & E-Stop)" << std::endl;
    std::cout << "============================================================" << std::endl;

    test_stopping_distance_formula();
    test_hazard_corridor_bounds();
    test_noise_threshold_gating();
    test_reactive_actuation_estop();
    test_dynamic_velocity_expansion();

    std::cout << "============================================================" << std::endl;
    std::cout << " test_corridor_estop PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;
    return 0;
}

