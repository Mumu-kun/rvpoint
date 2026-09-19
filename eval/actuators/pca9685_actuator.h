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

enum class WheelId {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};

struct MotorConfig {
    int motor = 1;              // 1..4 (Motor 1, 2, 3, 4)
    bool invert = false;        // Reverse polarity if true
    float trim = 1.0f;
    float deadband_forward = 0.12f;
    float deadband_reverse = 0.12f;
};

struct PCA9685Config {
    int bus = 4;
    std::string address = "0x60";
    float freq_hz = 200.0f;
    std::string mapping = "adafruit";
    float deadband = 0.12f;
    bool enable_watchdog = true;
    uint64_t watchdog_timeout_ms = 200;

    MotorConfig fl{1, false, 1.0f, 0.12f, 0.12f};
    MotorConfig fr{2, false, 1.0f, 0.12f, 0.12f};
    MotorConfig rl{3, false, 1.0f, 0.12f, 0.12f};
    MotorConfig rr{4, false, 1.0f, 0.12f, 0.12f};

    MotorConfig& get_wheel_config(WheelId wheel) {
        switch (wheel) {
            case WheelId::FL: return fl;
            case WheelId::FR: return fr;
            case WheelId::RL: return rl;
            case WheelId::RR: return rr;
        }
        return fl;
    }

    const MotorConfig& get_wheel_config(WheelId wheel) const {
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

class PCA9685Actuator : public MotorActuator {
public:
    explicit PCA9685Actuator(const PCA9685Config& config = PCA9685Config{});
    ~PCA9685Actuator() override;

    void set_duty_cycles(float duty_left, float duty_right) override;
    void emergency_brake() override;
    bool is_emergency_stopped() const noexcept override;
    void reset_emergency_stop() override;

    void set_wheel_duties(float fl, float fr, float rl, float rr);
    PCA9685Config& config() { return config_; }
    const PCA9685Config& config() const { return config_; }
    bool is_simulated() const noexcept { return is_simulated_; }
    void set_watchdog_enabled(bool enabled) { watchdog_enabled_ = enabled; }

    bool initialize_hardware();
    void shutdown_hardware();

private:
    PCA9685Config config_;
    std::atomic<bool> emergency_stop_{false};
    std::atomic<bool> watchdog_enabled_{true};
    std::atomic<bool> watchdog_tripped_{false};
    std::atomic<bool> running_{false};
    bool is_simulated_ = false;

    int i2c_fd_ = -1;
    std::thread worker_thread_;
    std::atomic<uint64_t> last_command_time_ns_{0};

    void worker_loop();
    void hw_set_duty(int channel, float duty);
    void hw_drive_motor(int motor_num, float duty, const std::string& dir);
    static uint64_t get_current_time_ns();
};

} // namespace rvpoint
