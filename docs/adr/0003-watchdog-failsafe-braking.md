# Instant Hard-Cutoff Userspace Watchdog Fail-Safe

Enforce an instant hard-cutoff fail-safe policy on the Orange Pi RV2 that zeroes all motor PWM duty cycles and sets H-bridge direction pins to active brake if UDP packets or perception updates stall for more than 200 milliseconds.

## Status
Accepted

## Context
Because motor control is driven directly from the Orange Pi RV2 via Linux sysfs PWM and GPIO direction pins without an intermediate microcontroller (e.g. STM32/ESP32), a network dropout, thread deadlock, or software crash could leave the vehicle's motors actively powered, causing runaway collisions.

## Decision
Implement a dedicated userspace watchdog thread (`std::jthread` running at 50 Hz on Core 4) paired with atomic timestamp heartbeats and POSIX signal handlers (`SIGINT`, `SIGTERM`, `SIGSEGV`):
1. If the elapsed duration since the last valid UDP perception packet exceeds 200 ms, the watchdog thread immediately forces hardware PWM duty cycles to 0% and pulls all direction pins to `LOW/LOW` (active H-bridge motor brake).
2. Install a crash handler (`sigaction`) to ensure clean emergency braking if the main process receives an unhandled signal.
3. Avoid gradual deceleration ramps upon packet loss, as a vehicle traveling at 0.4 m/s moves only 8 cm in 200 ms, preventing obstacle and wall impacts.

## Consequences
- **Positive**: Strict, deterministic safety envelope protecting physical hardware, sensors, and the environment during wireless testing.
- **Negative**: Jerky braking if the Wi-Fi link experiences jitter above 200 ms (mitigated by operating on a clean 5 GHz Wi-Fi band).

