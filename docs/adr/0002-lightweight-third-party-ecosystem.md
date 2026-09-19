# Standalone Lightweight C++ Architecture Over Full ROS 2

Implement the complete perception, actuation, and telemetry pipeline as a standalone C++ binary using established lightweight open-source libraries for non-PCL tasks, avoiding ROS 2.

## Status
Accepted

## Context
Deploying full ROS 2 (rclcpp, DDS/FastDDS, colcon, tf2) on a RISC-V SBC running Linux introduces substantial runtime memory overhead (typically >250 MB RSS for basic nodes), complex multi-node inter-process communication (IPC) context-switching penalties, and build/packaging friction on non-standard RISC-V distributions. The project's primary focus is evaluating RVPoint's hardware vector-accelerated point cloud algorithms on RVV 1.0.

## Decision
Keep the entire runtime inside a single optimized C++ multi-threaded binary (`eval/pipelines/pipeline_iphone_vehicle.cpp`).
Integrate standard, lightweight open-source components for non-point-cloud roles:
- Network Ingestion: POSIX BSD sockets (`sys/socket.h`) for zero-copy UDP reception.
- GPIO / PWM Actuation: Linux sysfs PWM (`/sys/class/pwm`) and standard `libgpiod`.
- Live Telemetry: Lightweight Foxglove WebSocket server (`foxglove/ws-protocol` C++ or header-only WebSocket) transmitting binary Protobuf/JSON frames.
- Point Cloud Perception: RVPoint (`librvpoint.a`) with hardware-accelerated RVV 1.0 kernels.

## Consequences
- **Positive**: Sub-25ms end-to-end latency, sub-30MB total RAM footprint, zero DDS daemon friction, single-binary cross-compilation with standard CMake toolchain.
- **Negative**: Replaces ROS 2 node modularity with clean C++ thread barriers and ring buffers.

