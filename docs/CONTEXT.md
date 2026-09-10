# RVPoint Domain Model: Core Library & Autonomous Vehicle

Unified domain model and canonical terminology for the RVPoint high-performance RISC-V Vector (RVV 1.0) point cloud perception library and autonomous vehicle control stack.

## Core Library & Representation

**PointCloudSoA**:
Contiguous structure-of-arrays representation storing separated x, y, and z floating-point buffers to maximize vector load bandwidth.
_Avoid_: AoS point arrays, std::vector<PointXYZ> in vector hotpaths

**Point Cloud Decimation**:
Adaptive geometric voxel sampling that downsamples dense point clouds to an exact target point count while preserving geometric bounds and cluster features.
_Avoid_: Random subsampling, arbitrary truncation

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
