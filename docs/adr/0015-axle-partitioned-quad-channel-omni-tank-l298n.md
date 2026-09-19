# Axle-Partitioned Quad-Channel Omni-Tank with Dual L298N Drivers

Implement a hardware driver and standalone validation suite for an omni-wheel chassis actuated by two discrete L298N dual H-bridge modules partitioned by axle (Front Module: FL/FR, Rear Module: RL/RR), driven directly via Linux sysfs hardware PWM or high-resolution software timer PWM and GPIO direction pins under an Omni-Tank differential kinematic model.

## Status
Accepted

## Context
ADR-0001 originally established a 2-Channel Paired Omni-Tank topology where left-side and right-side motors were wired in parallel to a single dual H-bridge. In practice, wiring two DC motors in parallel to a single L298N channel introduces current-sharing imbalances (due to differing internal motor winding resistances) and prevents per-wheel speed trim calibration. Furthermore, routing motor pairs across opposite chassis ends into side-partitioned drivers complicates physical wire harnessing.

The physical vehicle utilizes two discrete L298N dual H-bridge modules located at the front and rear axles respectively:
- **Front Module**: Channel A drives Front-Left (FL), Channel B drives Front-Right (FR).
- **Rear Module**: Channel A drives Rear-Left (RL), Channel B drives Rear-Right (RR).

All four omnidirectional wheels face forward, with transverse passive rollers eliminating tire scrub during turning. While the chassis possesses four independent physical actuators (requiring 4 PWM signals and 8 direction GPIO lines), its kinematics are governed by the 2-DoF Omni-Tank model ($v_x, \omega_z$).

Additionally, configuring four hardware PWM channels on Linux SBCs (such as the Orange Pi RV2 SpacemiT K1) can encounter device-tree pinmux conflicts. A resilient bring-up strategy requires software timer PWM capability as a fallback.

## Decision
1. **Axle-Partitioned Hardware Topology**:
   - Dedicate L298N Board 1 to the Front Axle (Channel A = FL, Channel B = FR).
   - Dedicate L298N Board 2 to the Rear Axle (Channel A = RL, Channel B = RR).
   - Software mixer maps body twist $[v_x, \omega_z]^T$ to left and right bank velocities ($v_L = v_x - \frac{L}{2}\omega_z, v_R = v_x + \frac{L}{2}\omega_z$) and distributes them to the four motor channels:
     $$v_{\text{FL}} = v_L \cdot \text{trim}_{\text{FL}}, \quad v_{\text{RL}} = v_L \cdot \text{trim}_{\text{RL}}$$
     $$v_{\text{FR}} = v_R \cdot \text{trim}_{\text{FR}}, \quad v_{\text{RR}} = v_R \cdot \text{trim}_{\text{RR}}$$
2. **Dual-Backend PWM Engine**:
   - Support Linux sysfs hardware PWM (`/sys/class/pwm/pwmchipX/pwmY`) when configured in device tree.
   - Provide high-resolution POSIX software timer PWM fallback (`timerfd` / `clock_nanosleep` thread at 200–500 Hz) toggling GPIO lines directly for friction-free hardware bring-up.
3. **Active Braking & Failsafe Watchdog**:
   - Direction pins use L298N fast active brake (`LOW/LOW`) during deceleration and stop.
   - Maintain a 200 ms deadman watchdog timer and POSIX signal handlers (`SIGINT`, `SIGTERM`, `SIGSEGV`) to immediately cut PWM to 0% and engage active braking.
4. **Decoupled Actuator Seam**:
   - Implement the driver in `eval/actuators/dual_l298n_actuator.{h,cpp}` conforming to `rvpoint::MotorActuator` ([ADR-0007](0007-two-seam-architecture.md)).
   - Provide a standalone multi-mode hardware validation runner in `eval/pipelines/validate_motors.cpp` with individual polarity testing, directional verification, stiction ramp sweeping, and terminal teleoperation.
   - Store pin and trim configurations in an external JSON/INI config file with CLI argument overrides.

## Consequences
- **Positive**: 
  - Clean front/rear wire harnessing without stretching motor cables across the length of the chassis.
  - Software trim calibration compensates for individual motor manufacturing variances.
  - Software PWM fallback allows immediate prototype validation on physical hardware without requiring custom device-tree compilation.
  - Full compatibility with the existing `MotorActuator` perception/navigation seam.
- **Negative**:
  - Requires 12 digital pins (4 PWM + 8 GPIO) on the host SBC header instead of 6 pins.
  - Software PWM consumes a small fraction of one CPU core when hardware PWM is not active.
