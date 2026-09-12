#include <cassert>
#include <cmath>
#include <iostream>

#include "include/rvpoint.h"

int main() {
    std::cout << "============================================================" << std::endl;
    std::cout << " Running test_motor_actuator (Prototype Actuation Seam)" << std::endl;
    std::cout << "============================================================" << std::endl;

    rvpoint::MockMotorActuator mock;

    // 1. Initial state
    assert(!mock.is_emergency_stopped());
    assert(mock.command_count() == 0);
    assert(mock.brake_count() == 0);
    std::cout << "    [PASS] Initial state verified (idle, unstopped)." << std::endl;

    // 2. Normal command transmission
    mock.set_duty_cycles(0.5f, -0.25f);
    assert(mock.command_count() == 1);
    auto cmd = mock.last_command();
    assert(std::abs(cmd.duty_left - 0.5f) < 1e-5f);
    assert(std::abs(cmd.duty_right - (-0.25f)) < 1e-5f);
    assert(cmd.timestamp_ns > 0);
    std::cout << "    [PASS] Duty cycle commands recorded accurately." << std::endl;

    // 3. Out-of-bounds clamping
    mock.set_duty_cycles(1.5f, -2.0f);
    cmd = mock.last_command();
    assert(std::abs(cmd.duty_left - 1.0f) < 1e-5f);
    assert(std::abs(cmd.duty_right - (-1.0f)) < 1e-5f);
    std::cout << "    [PASS] Out-of-bounds duty cycle clamping [-1.0, 1.0] verified." << std::endl;

    // 4. Emergency Brake Trigger
    mock.emergency_brake();
    assert(mock.is_emergency_stopped());
    assert(mock.brake_count() == 1);
    cmd = mock.last_command();
    assert(std::abs(cmd.duty_left) < 1e-5f);
    assert(std::abs(cmd.duty_right) < 1e-5f);
    std::cout << "    [PASS] Emergency brake cuts power to 0.0 instantaneously." << std::endl;

    // 5. Subsequent commands while E-stopped remain clamped to 0.0
    mock.set_duty_cycles(0.8f, 0.8f);
    assert(mock.is_emergency_stopped());
    cmd = mock.last_command();
    assert(std::abs(cmd.duty_left) < 1e-5f);
    assert(std::abs(cmd.duty_right) < 1e-5f);
    std::cout << "    [PASS] Commands rejected/clamped during active E-stop state." << std::endl;

    // 6. Reset E-stop
    mock.reset_emergency_stop();
    assert(!mock.is_emergency_stopped());
    mock.set_duty_cycles(0.3f, 0.3f);
    cmd = mock.last_command();
    assert(std::abs(cmd.duty_left - 0.3f) < 1e-5f);
    assert(std::abs(cmd.duty_right - 0.3f) < 1e-5f);
    std::cout << "    [PASS] Emergency stop recovery verified." << std::endl;

    std::cout << "============================================================" << std::endl;
    std::cout << " test_motor_actuator PASSED ALL CHECKS" << std::endl;
    std::cout << "============================================================" << std::endl;

    return 0;
}
