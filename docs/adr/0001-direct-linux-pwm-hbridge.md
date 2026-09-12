# Direct Linux PWM and GPIO H-Bridge Actuation in Paired Omni-Tank Topology

Directly interface discrete H-bridge motor drivers (L298N/TB6612) to the Orange Pi RV2 (SpacemiT K1) 40-pin header using a 2-Channel Paired Omni-Tank wiring topology, Linux hardware PWM (`/sys/class/pwm`), and `libgpiod` direction pins.

## Status
Accepted

## Context
Standard autonomous mobile robotics stacks commonly offload low-level motor PWM and encoder timing to an auxiliary MCU over UART. However, the Orange Pi RV2 possesses a 40-pin expansion header with native hardware PWM channels and high-speed GPIOs directly controlled by the SpacemiT K1 SoC. The user already possesses discrete H-bridge drivers wired directly to this header.

## Decision
Drive the vehicle's 4 motors in a **2-Channel Paired Omni-Tank** topology:
1. Wire both Left motors in parallel to H-bridge Channel A (driven by `PWM0` + 2 GPIO direction pins).
2. Wire both Right motors in parallel to H-bridge Channel B (driven by `PWM1` + 2 GPIO direction pins).
3. Treat the 4-wheel omni chassis as a scrub-free differential drive unicycle model ($v_L = v_x - \frac{L}{2}\omega_z, v_R = v_x + \frac{L}{2}\omega_z$).
4. To mitigate safety risks inherent to driving motors directly from userspace Linux (where a process segfault or thread lock could leave motors spinning at full throttle), implement an active 50 Hz watchdog thread and POSIX signal handlers (`SIGINT`, `SIGTERM`, `SIGSEGV`) to instantly zero PWM duty cycles and pull direction pins `LOW/LOW` (active brake) upon process interruption or packet timeout (> 200 ms).

## Consequences
- **Positive**: Zero extra hardware BOM; zero serial transmission latency; avoids device-tree pinmux conflicts by using exactly 2 hardware PWM channels; frictionless zero-radius pivot turns.
- **Negative**: Requires strict userspace watchdog monitoring; software-locked to differential kinematics (cannot crab laterally without rewiring to 4 independent channels).
