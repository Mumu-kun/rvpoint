# Two-Seam Architecture for Headless CI and Deterministic Simulation

Decouple the entire mobile autonomy pipeline across exactly two external seams (`StreamSource` and `MotorActuator`) enclosing two deep modules (`PerceptionEngine` and `Navigator`), enabling 100% headless desktop and CI testing without physical hardware.

## Status
Accepted

## Context
Embedded robotics code frequently couples network sockets (`recvfrom`) and hardware actuation (`/sys/class/pwm`, `libgpiod`) directly into the main perception loop. This creates high coupling, prevents automated unit testing on developer laptops, and requires an active physical car and live Wi-Fi stream just to verify algorithmic changes.

## Decision
Enforce a **Two-Seam Architecture** separating I/O from core compute:
1. **The Ingestion Seam (`StreamSource`)**:
   - Interface: `poll_frame(StreamFrame& out) -> bool`.
   - Production Adapter: `UdpStreamSource` (non-blocking POSIX BSD socket on 5 GHz Wi-Fi).
   - Test / Replay Adapter: `MockPcdStreamSource` (replays local `.pcd` files with synthetic VIO trajectories).
2. **The Actuation Seam (`MotorActuator`)**:
   - Interface: `set_duty_cycles(float left, float right)`, `emergency_brake()`.
   - Production Adapter: `LinuxSysfsPwmActuator` (talks to `/sys/class/pwm/` and `libgpiod`).
   - Test / CI Adapter: `MockMotorActuator` (logs commanded duty cycles in memory; asserts on E-stop latency in unit tests).
3. **Deep Modules**: Centralize point cloud processing inside `PerceptionEngine` (`librvpoint.a`) and decision planning inside `Navigator` (`eval/pipelines/`), each presenting a single, minimal entry point.

## Consequences
- **Positive**: Closed-loop autonomy tests execute in $< 15\,\text{ms}$ on any standard PC in CI; complete isolation between hardware bugs and algorithmic regressions; zero virtual function overhead on hot vector paths (seams are invoked exclusively at the 30 Hz / 50 Hz frame boundary).
- **Negative**: Requires maintaining lightweight mock adapters alongside production drivers.

