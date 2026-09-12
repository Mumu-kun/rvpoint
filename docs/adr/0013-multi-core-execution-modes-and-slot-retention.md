# ADR 0013: Multi-Core Scheduling Modes and Slot Retention Lifecycle

- **Status**: accepted
- **Deciders**: Fahad, Antigravity
- **Date**: 2026-09-11

## Context & Decision

Point cloud perception on the 8-core SpacemiT K1 RISC-V SoC (`rv64gcv`) requires addressing two fundamental architectural tensions:
1. **Latency vs. Throughput in Multi-Core Scheduling**: Closed-loop vehicle safety demands minimal reaction latency ($<20\text{ ms}$ from LiDAR pulse to steering/braking), whereas 3D mapping and SLAM demand maximal throughput ($>70\text{ FPS}$). Naive thread spawning or running 16 unmanaged threads across stages creates severe thread over-subscription and L2 cache thrashing across the SpacemiT K1's dual 4-core clusters.
2. **Inter-Frame Temporal Dataflow**: Cross-frame algorithms (Scan-to-Submap ICP, Moving Object Tracking, running elevation models) require historical and persistent state across consecutive frames, while external systems (ROS 2 nodes, shared-memory visualizers, global HD maps) require zero-copy access without duplicating buffers into the pipeline register file.

We decided to formalize **Two Frame Execution Modes** in `PipelineManager`, **Two Intra-Frame Kernel Strategies**, a hardware **Cluster Topology Affinity System**, and **Four Slot Lifetime Policies** in `FrameContext`:

### 1. Frame Execution Modes (`ExecutionMode`)

1. **`ExecutionMode::IntraFrame` (Low-Latency Mode)**:
   - Processes one frame at a time across available cores to minimize end-to-end latency ($<18\text{ ms}$) for real-time collision avoidance and closed-loop vehicle control.
   - Internal stage acceleration can employ either:
     - **`KernelParallel`**: Fine-grained `#pragma omp parallel for` across points within individual kernels (ROR, Cardano normal estimation, voxel radix sort) querying a shared spatial index.
     - **`SpatialSlabEngine`**: Geometric domain decomposition slicing obstacle points into $K$ equal-population slabs along a spatial axis, executing local grid construction, ROR filtering, and intra-slab clustering in parallel before a fast boundary stitching pass.
     - **`ForwardCellClustering`**: Standalone cell-centric clustering evaluating pairwise point connectivity across 13 canonical forward neighborhood vectors (half of 26 3D neighbors) via zero-heap Union-Find, cutting redundant checks in half.
   - Stages declare their parallelization capability; non-slab stages (like global RANSAC fitting) run with KernelParallel parallelism, avoiding nested OpenMP conflicts via `enforce_anti_oversubscription()` (`omp_set_max_active_levels(1)`).

2. **`ExecutionMode::FrameWorkerPool` (High-Throughput Streaming Mode)**:
   - An asynchronous worker pool where $W$ worker threads each independently process an entire frame from end to end using pre-allocated, private worker `FrameContext` banks cloned from a validated prototype.
   - Zero lock contention and zero synchronization between stages during frame processing.
   - Supports continuous sensor streams ($>60\text{ FPS}$) and offline dataset evaluation.
   - Can optionally combine with `ClusterTopology` to pin individual worker pipelines to specific CPU clusters (e.g. Worker 0 on Cluster 0, Worker 1 on Cluster 1).

*(Note: DAG-level task parallelism (`#pragma omp task` or multi-stage pipelining queues across clusters) was explicitly rejected due to scheduler overhead, thread over-subscription, and poor scalability on linear point cloud pipelines).*

### 2. Hardware Cluster Topology (`ClusterTopology`)

- Formalizes the SpacemiT K1's physical silicon layout:
  - **Cluster 0**: Cores 0–3 sharing 1 MB L2 cache.
  - **Cluster 1**: Cores 4–7 sharing 1 MB L2 cache.
- Provides programmatic POSIX affinity (`pthread_setaffinity_np`) to pin intra-frame execution or FrameWorkerPool worker instances to specific `CoreCluster` definitions, eliminating cross-cluster L2 cache invalidations.

### 3. Four Slot Lifetime Policies (`SlotLifetime`)

1. **`SlotLifetime::Ephemeral` (Default)**:
   - Registered intermediate buffers (`downsampled_cloud`, `clusters`). Automatically reset by `reset_frame()` to capacity-retaining zero-length states at each frame boundary.
2. **`SlotLifetime::Persistent`**:
   - Long-lived state registers (`running_ground_model`, `tracker_kalman_state`, `odometry_pose`). Survives `reset_frame()` untouched, allowing downstream nodes to read and update running state across consecutive frames.
3. **`SlotLifetime::History<N>`**:
   - Temporal ring buffer (e.g. `History<2>` for $[t]$ and $[t-1]$). Managed via automatic pointer ping-pong swapping at frame boundaries, allowing differential kernels (Scan-to-Scan ICP, optical flow, velocity estimators) to read the frozen previous frame with zero manual copying.
4. **`SlotLifetime::External` (`ExternalSlot`)**:
   - Non-owning pointer binding directly to memory managed outside the pipeline (e.g. global SLAM submaps, shared-memory IPC rings, hardware device DMA buffers), providing zero-copy read/write access for pipeline stages.

---

## Consequences

- **Unified Pipeline Definition**: Developers write the perception pipeline DAG once; the exact same code runs live in low-latency mode on the car, high-throughput pipelined mode for SLAM, or batch task-farmed for offline dataset evaluations.
- **Zero Thread Contention**: Cluster Islanding guarantees exactly 8 active threads on 8 cores, eliminating context switching storms.
- **Zero-Copy External Integration**: External SLAM submaps and IPC streams bind directly into register slots without heap allocations or data duplication.
- **Explicit Temporal State**: Cross-frame dataflow is fully declarative and inspectable via telemetry probes.

