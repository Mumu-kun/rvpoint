#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>
#include <fstream>
#include "control/actuators/motor_actuator.h"

namespace rvpoint {

/**
 * @brief Identifies each wheel of the 4-wheel omni chassis.
 */
enum class WheelId {
    FL = 0, // Front-Left
    FR = 1, // Front-Right
    RL = 2, // Rear-Left
    RR = 3  // Rear-Right
};

/**
 * @brief Configuration for a single physical motor driver channel.
 */
struct MotorPinConfig {
    int channel_index = 0;              // 0..3 (0: Board1 ChA, 1: Board1 ChB, 2: Board2 ChA, 3: Board2 ChB)
    int pwm_pin = -1;                   // GPIO number for software PWM, or PWM channel
    std::string sysfs_pwm_path = "";    // e.g. "/sys/class/pwm/pwmchip0/pwm0" (if hardware PWM)
    int in1_pin = -1;                   // Direction GPIO line 1
    int in2_pin = -1;                   // Direction GPIO line 2
    bool invert = false;                // Reverse polarity if true
    float trim = 1.0f;                  // Default scale factor [0.5, 1.5]
    float trim_forward = 1.0f;          // Forward scale factor [0.5, 1.5]
    float trim_reverse = 1.0f;          // Reverse scale factor [0.5, 1.5]
    float deadband_forward = -1.0f;     // Forward deadband (<0 uses global config)
    float deadband_reverse = -1.0f;     // Reverse deadband (<0 uses global config)
};

/**
 * @brief Configuration for the Axle-Partitioned Dual L298N driver.
 */
struct DualL298NConfig {
    bool use_hardware_pwm = false;
    int pwm_frequency_hz = 250;         // 200 - 500 Hz optimal for L298N
    float deadband = 0.20f;             // Stiction compensation offset [0.0, 0.4]
    bool enable_watchdog = true;        // 200 ms deadman watchdog
    uint64_t watchdog_timeout_ms = 200; // Timeout before auto-brake

    // Mapping for each of the 4 physical wheels
    MotorPinConfig fl{1, -1, "", 72, 73, false, 1.0f};
    MotorPinConfig fr{0, -1, "", 74, 71, true, 1.0f};
    MotorPinConfig rl{3, -1, "", 70, 91, true, 1.0f};
    MotorPinConfig rr{2, -1, "", 47, 48, false, 1.0f};

    // Helper to get pin config by wheel
    MotorPinConfig& get_wheel_config(WheelId wheel) {
        switch (wheel) {
            case WheelId::FL: return fl;
            case WheelId::FR: return fr;
            case WheelId::RL: return rl;
            case WheelId::RR: return rr;
        }
        return fl;
    }

    const MotorPinConfig& get_wheel_config(WheelId wheel) const {
        switch (wheel) {
            case WheelId::FL: return fl;
            case WheelId::FR: return fr;
            case WheelId::RL: return rl;
            case WheelId::RR: return rr;
        }
        return fl;
    }

    bool load_from_json(const std::string& filepath);
    bool save_to_json(const std::string& filepath) const;
};

/**
 * @brief Axle-Partitioned Quad-Channel Omni-Tank Motor Actuator Driver.
 *
 * Implements rvpoint::MotorActuator (ADR-0007, ADR-0015).
 * Drives two discrete L298N dual H-bridge modules partitioned by axle
 * (Board 1: Front-Left & Front-Right; Board 2: Rear-Left & Rear-Right)
 * under Omni-Tank 2-DoF kinematics.
 *
 * Supports:
 * - Dual PWM backend: Linux sysfs hardware PWM or high-resolution software timer PWM
 * - Active braking (LOW/LOW)
 * - 200 ms deadman watchdog
 * - Per-wheel polarity inversion and trim calibration
 * - Mock simulation fallback when running off-target (CI/QEMU)
 */
class DualL298NActuator : public MotorActuator {
public:
    explicit DualL298NActuator(const DualL298NConfig& config = DualL298NConfig{});
    ~DualL298NActuator() override;

    // --- MotorActuator Seam Interface ---
    void set_duty_cycles(float duty_left, float duty_right) override;
    void emergency_brake() override;
    bool is_emergency_stopped() const noexcept override;
    void reset_emergency_stop() override;

    // --- Direct Hardware & Calibration Control ---
    /**
     * @brief Directly set duty cycle for a specific raw channel (0..3).
     * Used by the Calibration Wizard to identify which physical wheel is on which channel.
     */
    void set_raw_channel_duty(int channel_index, float duty);

    /**
     * @brief Directly set individual wheel duty cycles [-1.0, 1.0].
     */
    void set_wheel_duties(float fl, float fr, float rl, float rr);

    /**
     * @brief Access the active configuration.
     */
    DualL298NConfig& config() { return config_; }
    const DualL298NConfig& config() const { return config_; }

    /**
     * @brief Check whether driver is running in simulation/mock mode.
     */
    bool is_simulated() const noexcept { return is_simulated_; }

    /**
     * @brief Temporarily enable/disable watchdog (e.g. during calibration prompts).
     */
    void set_watchdog_enabled(bool enabled) { watchdog_enabled_ = enabled; }

    /**
     * @brief Re-initialize hardware GPIOs and PWM with current config.
     */
    bool initialize_hardware();

    /**
     * @brief Release hardware pins and stop background worker threads.
     */
    void shutdown_hardware();

private:
    DualL298NConfig config_;
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> watchdog_enabled_{true};
    std::atomic<bool> watchdog_tripped_{false};
    std::atomic<bool> running_{false};
    bool is_simulated_ = false;

    // Target duty commands for the 4 channels [-1.0, 1.0]
    std::atomic<float> target_duties_[4]{{0.0f}, {0.0f}, {0.0f}, {0.0f}};
    std::atomic<uint64_t> last_command_time_ns_{0};

    // File descriptors for sysfs GPIO values (opened once for fast writes)
    int gpio_fds_[16]; // Up to 16 GPIO lines (4 PWM + 8 direction + spare)

    // Background software PWM and watchdog worker thread
    std::thread worker_thread_;
    void worker_loop();

    // Helper functions for low-level GPIO
    bool export_gpio(int pin);
    bool set_gpio_direction(int pin, const std::string& dir);
    int open_gpio_value_fd(int pin);
    void write_gpio_fast(int fd, int value);
    void write_gpio(int pin, int value);

    // Helpers for sysfs PWM
    bool init_sysfs_pwm(const std::string& pwm_path, int freq_hz);
    void set_sysfs_pwm_duty(const std::string& pwm_path, float duty, int freq_hz);

    void apply_channel_hardware(int ch, float duty);
    static uint64_t get_current_time_ns();
};

} // namespace rvpoint
