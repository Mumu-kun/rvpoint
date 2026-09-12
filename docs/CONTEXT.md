# RVPoint Domain Model: Core Library & Autonomous Vehicle

Unified domain model and canonical terminology for the RVPoint high-performance RISC-V Vector (RVV 1.0) point cloud perception library and autonomous vehicle control stack.

## Core Library & Representation

**PointCloud**:
An owning contiguous Structure-of-Arrays (SoA) point cloud container storing separate `x`, `y`, and `z` floating-point vectors to maximize unit-stride vector memory bandwidth.
_Avoid_: AoS point arrays, std::vector<PointXYZ> in vector hotpaths

**PointCloudView**:
A lightweight, non-owning contiguous slice view (`x`, `y`, `z` raw float pointers and point count `n`) providing zero-copy access into PointCloud memory buffers.
_Avoid_: Deep copies, passing full owning containers to read-only kernels

**Base Pitch ($\Delta$)**:
The single source of truth spatial voxel step size from which all downstream neighborhood search radii ($2.5\Delta$), RANSAC planar tolerances ($0.5\Delta$), and cluster linkage tolerances ($1.25\Delta$) are proportionally derived.
_Avoid_: Arbitrary tuning constants, independent parameter grids

**Resolution Preset**:
Standardized Base Pitch ($\Delta$) scale profiles tailored to specific physical scan domains (0.01m dense object, 0.02m standard tabletop, 0.05m outdoor LiDAR).
_Avoid_: Ad-hoc voxel configs, hardcoded thresholds

**Simulation Budget**:
Pre-simulation decimation threshold categorizing point counts by hardware cache residency and simulation wall-clock feasibility.
_Avoid_: Arbitrary downsampling, point cut

**Region of Interest (ROI)**:
Simulated execution window isolating algorithmic compute kernels from disk I/O and setup overhead using m5 pseudo-instructions or CLI flags.
_Avoid_: Full-trace capture, noisy benchmarking

**Zero-Vtable Algorithm Class**:
A stateful, non-virtual C++ class with value semantics implementing the Functor Protocol (`operator()(const In&..., Out&..., Param...)`). It encapsulates algorithm configuration, internal pre-allocated scratch workspaces, and an enum-based backend selector (`Backend::RVV`, `Backend::Scalar`, `Backend::Auto`). When added to a pipeline, it is moved and owned by value within the node; when shared across stages (e.g. spatial index grids), it is stored as a slot in the RegisterFile.
_Avoid_: Abstract filter base class, polymorphic interface, vtable hierarchy, external scratch wrappers

**Stage-Owned Scratch**:
Internal pre-allocated workspace memory (hash tables, PRNG states, candidate arrays, radix buffers) managed entirely within an algorithm class instance to guarantee zero heap allocations during execution.
_Avoid_: Dynamic allocation in hot loop, per-frame malloc, std::vector::resize in steady state

**Slotted Register File (RegisterFile)**:
An extensible, heterogeneous typed execution context storing pre-allocated representations (point clouds, spatial index grids, surface normals, planes, cluster indices) accessed via registered `SlotId` handles, pre-allocated at setup time to guarantee zero heap allocations and 1-cycle direct memory access.
_Avoid_: Point-cloud-only context, string-keyed blackboard in hot loop, std::unordered_map or std::any in perception loop

**SlotId**:
A 16-bit lightweight integer handle resolved once during pipeline setup representing an offset into the RegisterFile, enabling single-instruction pointer dereferencing on RISC-V.
_Avoid_: Runtime string hashing, string tags in hot path

**ConfigStore**:
A symmetrical parameter repository mirroring the data register file, storing typed configuration values (floats, ints, booleans) accessible by name or `ParamId` for dynamic runtime parameter tuning.
_Avoid_: Hardcoded constants, static defines for tunable thresholds

**ParamId**:
A 16-bit lightweight integer handle representing an offset into the ConfigStore, allowing kernels to read configuration values with zero dictionary lookups.
_Avoid_: String-based parameter lookups in per-frame code

**Tagged Positional Binding**:
Compile-time typed descriptor pattern (`in<T>`, `out<T>`, `param<T>`) binding node parameters to register slots with compile-time type verification.
_Avoid_: Untyped void* buffers, implicit index binding

**Parameter Reconfiguration Hook**:
A lifecycle callback triggered when a structural parameter (e.g. spatial grid cell size, octree depth) is updated, allowing spatial indices or pre-allocated scratch tables to reinitialize before the next frame without steady-state runtime overhead.
_Avoid_: Rebuilding indices every frame unconditionally, ignoring parameter changes

**PipelineManager**:
The top-level orchestrator that manages stage lifecycles, computes stage execution order via Kahn's DAG topological sort, binds SlotId dataflow connections, and provides dynamic parameter entry points.
_Avoid_: Monolithic runner script, hardcoded stage sequencers

**Declarative Probes**:
Lightweight callbacks attached directly to register slots (`pm.add_probe<T>()`) that execute after stage runs to stream Foxglove MCAPs, log telemetry, or inspect intermediate clouds without modifying algorithmic kernels.
_Avoid_: Dedicated probe bus class, hardcoded printf/file-saving inside algorithm loops

**DynamicConfigManager**:
A lock-free, double-buffered parameter snapshot manager (RCU pattern) that allows background threads or GUIs to tune perception thresholds on the fly without stalling the real-time vector loop.
_Avoid_: Mutex-guarded parameters, per-parameter atomics in inner loop

**Pipeline Inversion**:
An execution ordering topology where RANSAC plane segmentation is executed directly on downsampled points before building spatial indices, reducing spatial index construction and outlier filtering workloads to non-ground obstacle points only.
_Avoid_: Traditional forward sequencing, indexing the full cloud

**Ground Normal Prior**:
An extrinsic orientation vector (e.g. $[0, 0, 1]$) used by RANSAC plane segmentation to filter candidate hypotheses in $O(1)$ time via dot-product testing before evaluating cloud inliers.
_Avoid_: Fitting arbitrary dominant planes without normal orientation constraints

**Inlier Covariance Refinement**:
Analytical refinement of the winning RANSAC plane normal using the cross-product of the inlier covariance matrix columns, yielding sub-millimeter planar normal accuracy without non-linear optimization.
_Avoid_: Unrefined 3-point sample plane equations

**SPRT RANSAC**:
Sequential Probability Ratio Test accelerated random sample consensus with uniform 2048-point strided sample screening and dynamic iteration adaptation.
_Avoid_: Fixed-iteration RANSAC, evaluating the entire cloud on every hypothesis

**Symmetric 13-Offset Clustering**:
A directional spatial partitioning strategy evaluating only the 13 positive forward neighbor cells (`kForwardOffsets[13][3]`), cutting inter-cell neighbor distance checks in half without duplicate comparisons.
_Avoid_: Full 26-neighbor bidirectional scans, pairwise $O(N^2)$ checks

**ClusterResult (CSR)**:
Flat Compressed Sparse Row cluster representation storing concatenated point indices and cluster boundary offsets with zero nested heap vectors.
_Avoid_: std::vector<std::vector<int>>, heap reallocations per cluster

**FusedFilterNormals**:
Single-pass geometric fusion evaluating radius outlier thresholds while simultaneously estimating 3D surface normals from local covariance matrices in a single spatial grid traversal pass.
_Avoid_: Two-pass filtering followed by independent normal estimation

**Pipeline 3D Ultra (Pipeline 3D Ultimate)**:
The canonical 10-stage evaluation pipeline combining VoxelGrid downsampling, Fast 3D Spatial Grid indexing, outlier filtering (ROR/SOR), Cardano closed-form surface normal estimation, RVV SPRT RANSAC ground plane fitting, and Euclidean clustering. Standardized under ADR-0012 to execute via the Slotted Register-File PipelineManager.
_Avoid_: Hardcoded monolithic execution loops, unversioned pipeline aliases

**SpatialGridBuilder**:
A stateless zero-allocation DAG functor that constructs or updates a `Fast3DSpatialGrid` inside a pre-allocated `RegisterFile` slot in-place. Allows downstream spatial search stages (ROR, normal estimation) to consume shared spatial acceleration structures without redundant index builds or dynamic heap allocations.
_Avoid_: In-lambda ad-hoc grid allocations, per-stage independent grid rebuilding

**Radius Outlier Removal (ROR)**:
A spatial density filter that removes points with fewer than $K$ neighbors within a Euclidean ball of radius $R$, accelerated via uniform spatial grid hashing.
_Avoid_: Statistical outlier removal (when radius density is intended), brute-force radius filter

**SlotRetention**:
The temporal lifecycle contract of a register slot across consecutive frames: `Ephemeral` (reset every frame), `Persistent` (survives frame resets for running accumulators and state estimators), `History<N>` (ring-buffered across $N$ frames with automatic ping-pong pointer swapping for differential algorithms like ICP or velocity estimation), and `External` (non-owning pointer viewing memory managed outside the pipeline).
_Avoid_: Manual per-stage frame caching, global static state

**ExternalSlot**:
A non-owning register slot that binds directly to external application memory (e.g. global SLAM submaps, shared-memory IPC buffers, or hardware sensor queues) allowing nodes to read and write external state with zero copy and zero framework allocation.
_Avoid_: Deep-copying external state into registers, artificial wrapper buffers

**ExecutionMode**:
The multi-core scheduling paradigm chosen for the PipelineManager:
- `IntraFrame`: All 8 CPU cores execute one stage at a time in sequence using intra-stage data-parallel vectorization, delivering minimal latency ($<18\text{ ms}$) for live obstacle avoidance.
- `TemporalPipelined`: Core clusters process overlapping frames concurrently ($N+1$ on Cluster 0, $N$ on Cluster 1) via Cluster Islanding, delivering maximal streaming throughput ($>70\text{ FPS}$) for mapping and SLAM.
- `TaskFarm`: An asynchronous worker pool where $W$ worker threads each independently process an entire frame from end to end. Fully compatible with live continuous sensor streams as well as offline dataset playback.
_Avoid_: Uncontrolled thread over-subscription, hardcoded threading models

**StreamResequencer**:
A lock-free monotonic re-order buffer used in TaskFarm streaming mode that reassembles asynchronously completed frames into strict chronological timestamp order before dispatching to downstream consumers.
_Avoid_: Out-of-order frame delivery to controllers, blocking worker threads

**ClusterIslanding**:
A hardware-conscious thread affinity topology for the 8-core SpacemiT K1 (two 4-core clusters with independent L2 caches) that pins early pipeline stages to Cluster 0 (Cores 0–3) and late pipeline stages to Cluster 1 (Cores 4–7) to eliminate thread over-subscription and cross-cluster L2 cache evictions during temporal pipelining.
_Avoid_: Global thread contention, 16-thread over-subscription on 8 cores

**Loop-Carried Temporal Hazard**:
A data dependency where an early pipeline stage on Frame $N+1$ requires the computed output of a late stage on Frame $N$, creating a pipeline stall unless mitigated via double-buffering or decoupled estimator state.
_Avoid_: Unsynchronized cross-frame reads, race conditions between overlapping frames


## Vehicle & Kinematics

**Omni-Tank Mode**:
A kinematic operating mode where a four-wheel omnidirectional chassis is driven as a two-degree-of-freedom differential drive vehicle by pairing left and right wheel velocities.
_Avoid_: Holonomic mode, mecanum drive, skid steer

**Kinematic Omni-Tank Model**:
The mathematical formulation relating chassis body twist $[v_x, \omega_z]^T$ to left and right wheel bank linear velocities $v_L, v_R$ given track width $L$ and wheel radius $r$.
_Avoid_: Skid steer model, unicycle model

**Paired Omni-Tank**:
A hardware wiring topology where the two left DC motors are connected in parallel to H-bridge Channel A and the two right DC motors are connected in parallel to Channel B, requiring exactly two hardware PWM channels and four GPIO direction pins.
_Avoid_: 4-channel independent drive, individual motor control

**Pivot Turn**:
A zero-radius pure yaw rotation about the geometric center of the vehicle achieved by commanding equal and opposite velocities to the left and right wheel banks.
_Avoid_: Spin turn, point turn, donut

**H-Bridge Bridge**:
The direct software interface running on the Linux host that translates high-level linear and angular velocity commands into sysfs hardware PWM duty cycles and GPIO direction logic.
_Avoid_: Motor controller, ESC firmware, microcontroller bridge

**Watchdog Heartbeat**:
An atomic timestamp updated by the perception loop and monitored at 50 Hz by an independent safety thread that triggers an instant hard cutoff (PWM 0%, direction LOW/LOW) if no packet is received for 200 ms.
_Avoid_: Soft timer, graceful deceleration, timeout monitor


## Closed-Loop Control & Estimation

**Body-Level Decoupled Dual PID**:
A control architecture containing two independent PID loops regulating forward linear velocity $v_x$ and angular yaw rate $\omega_z$ directly from body-frame observables before mixing into differential wheel commands.
_Avoid_: Dual wheel PID, cascaded motor PID, uncoupled PID

**VIO State Observer**:
A discrete recursive filter (low-pass IIR or linear Kalman filter) that reconstructs instantaneous ground-truth linear velocity and yaw rate from discrete, noisy VIO pose updates.
_Avoid_: Naive differentiator, finite difference, raw velocity

**Deadband Stiction Compensation**:
A feed-forward voltage offset added to the control output that overcomes static gearbox and motor brush friction below the motor's stall threshold.
_Avoid_: Min PWM cutoff, deadzone jump, static bias

**Integrator Anti-Windup**:
A clamping mechanism that halts or back-calculates integral error accumulation whenever commanded PWM duty cycles saturate at physical hardware limits ($\pm 100\%$).
_Avoid_: Integral reset, saturation leak


## Spatial Coordinates & Frames

**ARKit Optical Frame**:
The right-handed camera coordinate frame defined by Apple ARKit (X right, Y up, Z backwards out of the screen).
_Avoid_: Camera frame, iOS frame

**Vehicle Body Frame**:
The right-handed ISO 8855 / ROS REP 105 vehicle coordinate frame fixed at the rear-axle ground projection (X forward, Y left, Z up).
_Avoid_: Base frame, car frame, robot frame

**Extrinsics Matrix**:
The rigid $SE(3)$ transformation matrix $\mathbf{T}_{\text{body}}^{\text{cam}}$ mapping 3D coordinates from the ARKit Optical Frame into the Vehicle Body Frame.
_Avoid_: Calibration matrix, pose offset

**Drift Correction Transform**:
The rigid $SE(3)$ offset $\mathbf{T}_{\text{map}}^{\text{odom}}$ continuously updated by Scan-to-Submap ICP to reconcile raw drifting VIO odometry with the true global arena map.
_Avoid_: Map transform, SLAM pose, global correction


## Perception & Mapping

**Corridor Hazard Check**:
A low-latency safety filter that tests whether any non-ground points occupy a rectangular volumetric safety envelope directly ahead of the vehicle.
_Avoid_: Safety zone, collision box, bumper check

**Oriented Bounding Box (OBB)**:
A minimum-area 3D bounding prism oriented along an obstacle's dominant planar heading angle, computed via Andrew's Monotone Chain convex hull and Rotating Calipers.
_Avoid_: Axis-aligned bounding box, AABB, bounding rectangle

**Rolling Local Costmap**:
A body-centric or odometry-stabilized 2.5D elevation grid representing obstacle occupancy, traversability gradient, and euclidean obstacle clearance around the moving vehicle.
_Avoid_: Global map, occupancy grid, SLAM map

**Separable Distance Transform**:
A two-pass vectorizable algorithm that computes the exact Euclidean distance from every free cell in the costmap to the nearest obstacle boundary in $O(N)$ linear time.
_Avoid_: Euclidean distance map, flood fill, grassfire transform

**Scan-to-Submap ICP**:
A point-cloud-aided localization module using RVPoint's Cardano 3D normals and linearized Gauss-Newton point-to-plane ICP to align keyframe scans against a persistent arena submap.
_Avoid_: 2D scan matching, AMCL, NDT


## Navigation & Mission Planning

**Interactive Goal Pose**:
A 2D navigation destination coordinate $(X^*, Y^*, \theta^*)$ transmitted from Foxglove Studio over WebSockets via the interactive 3D goal tool.
_Avoid_: Destination waypoint, target coordinate, click-to-go

**Global A* Planner**:
A graph-search path planner running on the global arena occupancy grid that computes the shortest collision-free waypoint trajectory to the goal.
_Avoid_: Dijkstra, RRT, global path generator

**Pure Pursuit Tracker**:
A geometric path tracking algorithm that calculates the instantaneous steering curvature needed to intercept a designated lookahead distance along the global path.
_Avoid_: Path follower, trajectory tracker, Stanley controller

**Hierarchical Autonomy FSM**:
A top-level finite state machine managing system modes between IDLE, TRACKING, LOCAL_AVOIDANCE, RECOVERY_DEADLOCK, and FAILSAFE.
_Avoid_: State manager, autonomy loop, mission sequencer


## Telemetry & Transport

**Binary UDP Ingestion**:
A lightweight, zero-copy network transport that streams ARKit dToF depth vertices and 6-DoF VIO transformation matrices directly into memory-aligned SoA buffers over 5 GHz Wi-Fi.
_Avoid_: ROS transport, TCP stream, HTTP API

**Mock Stream Replayer**:
A standalone development utility that transmits recorded point clouds and synthesized VIO transformation matrices over UDP to test perception and actuation without requiring a live iPhone.
_Avoid_: Fake data generator, synthetic sensor, simulator

**Foxglove WebSocket Bridge**:
A lightweight WebSocket server transmitting Foxglove Studio schemas (foxglove.PointCloud, foxglove.BoundingBox3D, foxglove.Grid) for live browser debugging and visualization.
_Avoid_: Web visualizer, RViz server
