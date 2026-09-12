# ADR 0013: Multi-Core Scheduling Modes and Slot Retention Lifecycle

- **Status**: accepted
- **Deciders**: Fahad, Antigravity
- **Date**: 2026-09-11

## Context & Decision

Point cloud perception on the 8-core SpacemiT K1 RISC-V SoC (`rv64gcv`) requires addressing two fundamental architectural tensions:
1. **Latency vs. Throughput in Multi-Core Scheduling**: Closed-loop vehicle safety demands minimal reaction latency ($<20\text{ ms}$ from LiDAR pulse to steering/braking), whereas 3D mapping and SLAM demand maximal throughput ($>70\text{ FPS}$). Naive thread spawning or running 16 unmanaged threads across stages creates severe thread over-subscription and L2 cache thrashing across the SpacemiT K1's dual 4-core clusters.
2. **Inter-Frame Temporal Dataflow**: Cross-frame algorithms (Scan-to-Submap ICP, Moving Object Tracking, running elevation models) require historical and persistent state across consecutive frames, while external systems (ROS 2 nodes, shared-memory visualizers, global HD maps) require zero-copy access without duplicating buffers into the pipeline register file.

We decided to formalize **Three Multi-Core Scheduling Modes** in `PipelineManager` and **Four Slot Retention Policies** in `RegisterFile`:

### 1. Three Formal Execution Modes (`ExecutionMode`)

1. **`ExecutionMode::IntraFrame` (Default / Low-Latency Mode)**:
   - All 8 CPU cores execute each pipeline stage sequentially along the DAG.
   - Intra-stage data parallelism uses OpenMP vectorization (parallel radix sort, 8-core spatial slab clustering).
   - Minimizes end-to-end latency ($<18\text{ ms}$), zero inter-frame queuing delay; primary mode for real-time obstacle avoidance and emergency braking.

2. **`ExecutionMode::TemporalPipelined` (High-Throughput Mode via Cluster Islanding)**:
   - Partitions the DAG into Front-End and Back-End stage groups pinned to the SpacemiT K1's physical hardware clusters:
     - **Cluster 0 (Cores 0–3, 1 MB L2)**: Front-End (Ingestion $\to$ Voxel Downsample $\to$ RANSAC Ground Plane) processing Frame $N+1$ with 4-thread OpenMP.
     - **Cluster 1 (Cores 4–7, 1 MB L2)**: Back-End (Spatial Grid Build $\to$ Euclidean Clustering $\to$ Bounding Boxes) processing Frame $N$ with 4-thread OpenMP.
   - Boosts throughput by $+30\%$ ($>70\text{ FPS}$) at the cost of 1 frame buffer latency ($+8\text{ ms}$), with zero thread over-subscription and zero cross-cluster L2 cache line evictions.

3. **`ExecutionMode::TaskFarm` (Stream & Batch Worker Pool)**:
   - An asynchronous worker pool where $W$ worker threads each independently process an entire frame from end to end using pre-allocated, private worker `RegisterFile` banks.
   - **Stream-Compatible**: Ingests live continuous sensor streams (e.g. 50 Hz or 100 Hz streams) via non-blocking submission (`pm.submit(cloud)` or `pm.step_async(cloud)`), absorbing bursty traffic without dropping frames.
   - **Ordered Egress**: Employs an optional `StreamResequencer` (lock-free re-order buffer) to emit processed perception results in monotonic timestamp order.
   - Also serves as the execution engine for high-speed offline dataset sweeps and benchmarking (`scripts/bench/`).

### 2. Four Slot Retention Policies (`SlotRetention`)

1. **`Retention::Ephemeral` (Default)**:
   - Registered intermediate buffers (`downsampled_cloud`, `clusters`). Automatically reset by `reset_frame()` to capacity-retaining zero-length states at each frame boundary.
2. **`Retention::Persistent`**:
   - Long-lived state registers (`running_ground_model`, `tracker_kalman_state`, `odometry_pose`). Survives `reset_frame()` untouched, allowing downstream nodes to read and update running state across consecutive frames.
3. **`Retention::History<N>`**:
   - Temporal ring buffer (e.g. `History<2>` for $[t]$ and $[t-1]$). Managed via automatic pointer ping-pong swapping at frame boundaries, allowing differential kernels (Scan-to-Scan ICP, optical flow, velocity estimators) to read the frozen previous frame with zero manual copying.
4. **`Retention::External` (`ExternalSlot`)**:
   - Non-owning pointer binding directly to memory managed outside the pipeline (e.g. global SLAM submaps, shared-memory IPC rings, hardware device DMA buffers), providing zero-copy read/write access for pipeline stages.

---

## Consequences

- **Unified Pipeline Definition**: Developers write the perception pipeline DAG once; the exact same code runs live in low-latency mode on the car, high-throughput pipelined mode for SLAM, or batch task-farmed for offline dataset evaluations.
- **Zero Thread Contention**: Cluster Islanding guarantees exactly 8 active threads on 8 cores, eliminating context switching storms.
- **Zero-Copy External Integration**: External SLAM submaps and IPC streams bind directly into register slots without heap allocations or data duplication.
- **Explicit Temporal State**: Cross-frame dataflow is fully declarative and inspectable via telemetry probes.

