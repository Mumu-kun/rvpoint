#pragma once

#include <chrono>
#include <vector>

#include "control/actuators/motor_actuator.h"

namespace rvpoint {

/**
 * @brief In-memory mock actuator recording commands and E-stop triggers for desk/CI testing.
 */
class MockMotorActuator : public MotorActuator {
public:
    MockMotorActuator() = default;
    ~MockMotorActuator() override = default;

    void set_duty_cycles(float duty_left, float duty_right) override;
    void emergency_brake() override;
    bool is_emergency_stopped() const noexcept override { return e_stopped_; }
    void reset_emergency_stop() override;

    // Inspection methods for test assertions
    size_t command_count() const noexcept { return command_history_.size(); }
    size_t brake_count() const noexcept { return brake_count_; }
    MotorDutyCommand last_command() const noexcept;
    const std::vector<MotorDutyCommand>& history() const noexcept { return command_history_; }

    void clear_history();

private:
    bool e_stopped_ = false;
    size_t brake_count_ = 0;
    std::vector<MotorDutyCommand> command_history_;
};

} // namespace rvpoint

