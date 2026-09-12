#include "control/actuators/mock_motor_actuator.h"

#include <algorithm>

namespace rvpoint {

void MockMotorActuator::set_duty_cycles(float duty_left, float duty_right) {
    if (e_stopped_) {
        // Enforce active emergency clamp
        duty_left = 0.0f;
        duty_right = 0.0f;
    }

    // Clamp duty to [-1.0, 1.0]
    duty_left = std::max(-1.0f, std::min(1.0f, duty_left));
    duty_right = std::max(-1.0f, std::min(1.0f, duty_right));

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    command_history_.push_back(MotorDutyCommand{duty_left, duty_right, ts});
}

void MockMotorActuator::emergency_brake() {
    e_stopped_ = true;
    brake_count_++;

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

    command_history_.push_back(MotorDutyCommand{0.0f, 0.0f, ts});
}

void MockMotorActuator::reset_emergency_stop() {
    e_stopped_ = false;
}

MotorDutyCommand MockMotorActuator::last_command() const noexcept {
    if (command_history_.empty()) {
        return MotorDutyCommand{0.0f, 0.0f, 0};
    }
    return command_history_.back();
}

void MockMotorActuator::clear_history() {
    command_history_.clear();
    brake_count_ = 0;
    e_stopped_ = false;
}

} // namespace rvpoint
