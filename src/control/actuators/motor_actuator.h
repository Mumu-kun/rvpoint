#pragma once

#include <cstdint>

namespace rvpoint {

/**
 * @brief Commanded normalized motor duty cycles [-1.0, 1.0].
 *
 * Negative indicates reverse, positive indicates forward, 0.0 is idle/neutral.
 */
struct MotorDutyCommand {
    float duty_left = 0.0f;
    float duty_right = 0.0f;
    uint64_t timestamp_ns = 0;
};

/**
 * @brief Actuation Seam interface (ADR-0007).
 *
 * Decouples navigation algorithms and emergency braking from physical H-bridge PWM
 * and GPIO drivers, enabling deterministic headless testing in CI.
 */
class MotorActuator {
public:
    virtual ~MotorActuator() = default;

    /**
     * @brief Command normalized motor duty cycles [-1.0, 1.0] for differential drive.
     * @param duty_left Left motor bank duty cycle [-1.0, 1.0]
     * @param duty_right Right motor bank duty cycle [-1.0, 1.0]
     */
    virtual void set_duty_cycles(float duty_left, float duty_right) = 0;

    /**
     * @brief Trigger instant active emergency brake and clamp duty cycles to 0.0.
     */
    virtual void emergency_brake() = 0;

    /**
     * @brief Check whether the actuator is currently in emergency-stopped failsafe state.
     */
    virtual bool is_emergency_stopped() const noexcept = 0;

    /**
     * @brief Reset emergency stop state after safety clearing.
     */
    virtual void reset_emergency_stop() = 0;
};

} // namespace rvpoint
