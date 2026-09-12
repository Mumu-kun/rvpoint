# Complete Autonomous Vehicle Perception & Control Pipeline Specification
## Tiered End-to-End Autonomy Architecture for Orange Pi RV2 (RISC-V 64-bit RVV 1.0) & iPhone 14 Pro LiDAR

---

## 1. Executive Summary & Full-Stack System Architecture

This specification defines the complete, full-stack autonomous vehicle architecture for an omnidirectional model vehicle running on an **Orange Pi RV2** (SpacemiT K1 8-core RISC-V SoC with RVV 1.0 vector extensions).

The vehicle ingests live dToF LiDAR depth point clouds and 6-DoF Visual-Inertial Odometry (VIO) from an onboard **iPhone 14 Pro** over a low-latency 5 GHz Wi-Fi UDP link. Motor actuation is driven directly from the Orange Pi RV2 via hardware PWM (`/sys/class/pwm`) and direction GPIOs (`libgpiod`) into discrete H-bridges in a **2-Channel Paired Omni-Tank** wiring topology.

The architecture covers the entire spectrum of mobile autonomy: from low-level safety watchdogs and reactive bumper reflexes, to 3D object detection, rolling costmaps, global $A^*$ path planning, scan-to-submap ICP drift correction, and closed-loop VIO velocity PID regulation.

```
       +-----------------------------------------------------------------------+
       |                     iPhone 14 Pro (ARKit 6-DoF VIO)                   |
       |  - 256x192 dToF Depth (0.3m - 4.5m range)                             |
       |  - 4x4 Transformation Matrix T_world_cam (64 bytes float @ 60 Hz)     |
       +-----------------------------------+-----------------------------------+
                                           | 5 GHz Wi-Fi (Binary UDP Datagrams)
                                           v
+=========================================================================================+
| [SEAM 1: StreamSource]                                                                  |
|   ├── Production Adapter: UdpStreamSource (non-blocking POSIX socket on port 8765)      |
|   └── Desk/CI Test Adapter: MockPcdStreamSource (replays local PCDs & synthetic VIO)    |
+==========================================+==============================================+
                                           |
                                           v
+-----------------------------------------------------------------------------------------+
|                      Orange Pi RV2 (SpacemiT K1 rv64gcv @ 1.6 GHz)                      |
|                                                                                         |
|  [DEEP MODULE 1: PerceptionEngine] (librvpoint.a RVV 1.0)                               |
|   - Interface: process(const PointCloudSoA& in, PerceptionOutput& out)                  |
|   - Implementation:                                                                     |
|       • Tier 1: Static Elevation Slicing (PassThrough RVV) + Startup Ground RANSAC      |
|       • Tier 1: Forward Safety Dynamic TTC Corridor Envelope -> Instant E-Stop Reflex  |
|       • Tier 2A: Euclidean Clustering + Fast Bounding Discs (Evasion) + 3D OBB (Viz)   |
|       • Tier 3: Rolling 2.5D Costmap + RVV 1.0 Separable Distance Transform             |
|       • Tier 5: Scan-to-Submap Point-to-Plane ICP (Cardano analytical normals)          |
|                                                                                         |
|  [DEEP MODULE 2: Navigator] (eval/pipelines/)                                           |
|   - Interface: update(const PerceptionOutput& perc, const float pose[16], double t)     |
|   - Implementation:                                                                     |
|       • Tier 4: Foxglove Goal Pose Receiver + Global 2D Grid A* Path Planner            |
|       • Tier 4: Pure Pursuit Path Tracker + DWA-Style Circular Arc Tentacles            |
|       • Tier 6: VIO State Observer (Alpha-Beta Filter on Differentiated Poses)          |
|       • Tier 6: Body-Level Decoupled Dual PID Controller (Anti-Windup & Stiction Bias)  |
|       • Tier 6: Hierarchical Autonomy FSM (IDLE -> TRACKING -> RECOVERY_DEADLOCK)       |
+------------------------------------------+----------------------------------------------+
                                           | Commanded Motor Duty [-1.0, 1.0]
                                           v
+=========================================================================================+
| [SEAM 2: MotorActuator]                                                                 |
|   ├── Production Adapter: LinuxSysfsPwmActuator (/sys/class/pwm/ & libgpiod)            |
|   └── Desk/CI Test Adapter: MockMotorActuator (records duty cycles & verifies watchdog) |
+==========================================+==============================================+
                                           | Hardware PWM (pwm0, pwm1) + 4 GPIO Direction
                                           v
       +-----------------------------------------------------------------------+
       |                  Discrete Dual H-Bridge (L298N / TB6612)              |
       |  - Channel A: Left Front + Left Rear Motors (Paired)                  |
       |  - Channel B: Right Front + Right Rear Motors (Paired)                |
       +-----------------------------------+-----------------------------------+
                                           | High-Current DC Motor Drive
                                           v
       +-----------------------------------------------------------------------+
       |                4-Wheel Omni Chassis (Omni-Tank Mode)                  |
       |  - Differential cruise + Zero-radius Pivot Turns for deadlock escape  |
       +-----------------------------------------------------------------------+
```

---

## 2. The 7-Tier Autonomous Engineering Roadmap

```
+---------------------------------------------------------------------------------------+
| TIER 0: Infrastructure & Hardware Primitives (The Skeleton)                           |
| [TICKET-0.1] Zero-copy POSIX UDP Ingestion & PointCloudSoA Unpacker                   |
| [TICKET-0.2] Linux Sysfs PWM (pwm0, pwm1) & libgpiod H-Bridge Bridge with Watchdog   |
| [TICKET-0.3] Embedded Header-Only Foxglove WebSocket Telemetry Server (Port 8765)     |
| [TICKET-0.4] Python/C++ Mock Stream Replayer (scripts/bench/mock_iphone_stream.py)    |
| [TICKET-0.5] Minimal iOS Swift ARKit UDP Streaming Application                       |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 1: Reactive Safety & Emergency Braking (Instant Reflex)                          |
| [TICKET-1.1] ARKit Extrinsics Normalization (T_body_cam) & Ground Plane RANSAC        |
| [TICKET-1.2] Forward Safety Corridor Hazard Envelope -> Instant Software E-Stop       |
| [TICKET-1.3] Closed-Loop Bench Test: Autonomous Stop in Front of Wall/Obstacle        |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 2A: 3D Object Detection & Pivot Evasion (Deliberative 3D Geometry)               |
| [TICKET-2.1] RVV-Accelerated Euclidean Clustering on Non-Ground Obstacles             |
| [TICKET-2.2] 2D Convex Hull (Andrew's Monotone Chain) & 3D Oriented Bounding Box     |
| [TICKET-2.3] Lateral Evasion & Intelligent Pivot Selector (Left vs Right Decision)    |
| [TICKET-2.4] Closed-Loop Bench Test: Detect Obstacle, Pivot Away, Resume Cruise       |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 3: Local Spatial Memory & 2.5D Rolling Costmap (Dynamic Steering)                |
| [TICKET-3.1] VIO-Stabilized Rolling Multi-Frame Costmap (150x150, 4cm cells)          |
| [TICKET-3.2] Vectorized Separable Distance Transform (O(N) Clearance Field via RVV)   |
| [TICKET-3.3] Dynamic Tentacle / Gap-Finder Trajectory Generator & Curvature Control   |
| [TICKET-3.4] Closed-Loop Bench Test: Smooth Evasion of Staggered Obstacles            |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 4: Global Mission, Waypoint Navigation & Path Planning (Directed Autonomy)       |
| [TICKET-4.1] Interactive Foxglove 3D Goal Pose Receiver (ws-protocol endpoint)        |
| [TICKET-4.2] Global 2D Grid A* Path Planner (Shortest collision-free arena path)      |
| [TICKET-4.3] Pure Pursuit Waypoint Path Tracker (Lookahead steering curvature)        |
| [TICKET-4.4] Integration: Global Path Guided by Local Tentacle Obstacle Clearance    |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 5: Persistent Spatial Memory & Point-Cloud-Aided Drift Correction (SLAM Refinement|
| [TICKET-5.1] Persistent Global Arena Submap Accumulator & Disk Serialization (.pcd)   |
| [TICKET-5.2] Analytical Cardano Normal Estimation on Keyframe Point Clouds            |
| [TICKET-5.3] 6-DoF Point-to-Plane ICP Registration against Global Arena Submap        |
| [TICKET-5.4] Drift Correction Transform (T_map_odom) Closed-Loop Update               |
+---------------------------------------------------------------------------------------+
                                           |
                                           v
+---------------------------------------------------------------------------------------+
| TIER 6: Closed-Loop Regulation, Autonomy FSM & Recovery Behaviors (Robust Autonomy)   |
| [TICKET-6.1] VIO State Observer (Alpha-Beta Filter on Differentiated Poses)           |
| [TICKET-6.2] Body-Level Decoupled Dual PID Speed & Yaw Rate Regulator (Anti-Windup)   |
| [TICKET-6.3] Hierarchical Autonomy Finite State Machine (IDLE, TRACKING, RECOVERY)    |
| [TICKET-6.4] Full Arena Closed-Loop Verification: Point-to-Point Navigation in Arena  |
+---------------------------------------------------------------------------------------+
```

---

## 3. Detailed Engineering Tickets by Tier

### TIER 0: Infrastructure & Hardware Foundation

#### `[TICKET-0.1]` Zero-Copy POSIX UDP Ingestion Socket
- **Goal**: Ingest streaming binary packets from the iPhone over 5 GHz Wi-Fi with $< 3\,\text{ms}$ socket latency.
- **Data Protocol**:
  - Header: `uint32_t magic` (`0x52565054` - "RVPT"), `uint32_t seq`, `uint64_t timestamp_ns`.
  - Odometry: `float T_world_cam[16]` (64 bytes column-major).
  - Point Cloud: `uint32_t num_points`, followed by contiguous `float x[], y[], z[]`.
- **Implementation**: Zero-copy POSIX socket (`recvfrom` / `recvmmsg`) in `eval/pipelines/pipeline_iphone_vehicle.cpp` unpacking directly into `rvpoint::PointCloudSoA`.

#### `[TICKET-0.2]` Linux Sysfs PWM & libgpiod H-Bridge Bridge with Watchdog
- **Goal**: Drive 4 motors in Paired Omni-Tank mode directly from the Orange Pi RV2 40-pin header.
- **Interfaces**:
  - `PWM0` (`/sys/class/pwm/pwmchip0/pwm0`): Left pair motor speed.
  - `PWM1` (`/sys/class/pwm/pwmchip0/pwm1`): Right pair motor speed.
  - 4x GPIO direction pins via `libgpiod`: Forward (`HIGH/LOW`), Reverse (`LOW/HIGH`), Active Brake (`LOW/LOW`).
- **Fail-Safe Watchdog**:
  - Independent 50 Hz thread monitoring atomic timestamp `std::atomic<uint64_t> last_packet_time`.
  - If elapsed time $> 200\,\text{ms}$ or on process signal (`SIGINT`, `SIGSEGV`), instantly drive PWM duty to $0\%$ and pull direction pins `LOW/LOW`.

#### `[TICKET-0.3]` Embedded Header-Only Foxglove WebSocket Telemetry Server
- **Goal**: Real-time live browser visualization and command channel on port `8765`.
- **Streams**:
  - `foxglove.PointCloud`: Raw and ground-filtered 3D points.
  - `foxglove.BoundingBox3DArray`: Extracted 3D obstacle boxes.
  - `foxglove.Grid`: 2.5D Rolling Costmap and Euclidean clearance field.
  - `foxglove.PoseInFrame`: Current vehicle pose and global goal pose.

#### `[TICKET-0.4]` Python/C++ Mock Stream Replayer Utility
- **Goal**: Allow full desk development and algorithm benchmarking without requiring a live iPhone.
- **File**: `scripts/bench/mock_iphone_stream.py`.
- **Functionality**: Reads recorded point clouds from `data/pcd_compressed/` and transmits UDP datagrams with synthetic circular/straight-line VIO matrices to `127.0.0.1:8765` at 30 Hz.

#### `[TICKET-0.5]` Minimal iOS Swift ARKit UDP Streaming Application
- **Goal**: Self-contained Swift application running on iPhone 14 Pro.
- **Functionality**: Configures `ARWorldTrackingConfiguration` with `sceneDepth`, captures `ARFrame.sceneDepth` and `ARFrame.camera.transform`, packs the 64-byte matrix + depth vertices, and fires UDP packets to the Orange Pi's IP address.

---

### TIER 1: Reactive Safety & Emergency Braking

#### `[TICKET-1.1]` ARKit Extrinsics Normalization & Ground Plane RANSAC
- **Goal**: Transform optical points into ISO 8855 body frame and remove ground points.
- **Transformation**:
  $$\mathbf{P}_{\text{body}} = \mathbf{T}_{\text{body}}^{\text{cam}} \mathbf{P}_{\text{optical}}$$
  where $\mathbf{T}_{\text{body}}^{\text{cam}}$ accounts for mounting height ($h \approx 0.12\,\text{m}$) and tilt angle ($\theta \approx 10^\circ$).
- **Ground Filtering**: Fast SPRT RANSAC Plane Estimator (`rvpoint::SPRTPlaneEstimator`) fits the dominant floor plane and segments points with distance $|d| > 0.03\,\text{m}$ as obstacles.

#### `[TICKET-1.2]` Forward Safety Corridor Hazard Envelope
- **Goal**: Sub-millisecond reactive reflex triggering an immediate halt if an obstacle enters the dynamic stopping volume.
- **Corridor Envelope**:
  $$X \in [0.05\,\text{m}, d_{\text{stop}}(v_x)], \quad Y \in \left[-\frac{W_{\text{car}}}{2} - \delta, \frac{W_{\text{car}}}{2} + \delta\right], \quad Z \in [0.03\,\text{m}, H_{\text{car}}]$$
  where $d_{\text{stop}}(v_x) = d_{\text{margin}} + v_x \cdot t_{\text{reaction}} + \frac{v_x^2}{2 a_{\text{max}}}$.
- **Reaction**: If obstacle count $> 10$ points, immediately command $v_{\text{cmd}} = 0$, overriding all higher-level path planners.

#### `[TICKET-1.3]` Closed-Loop Bench Test: Autonomous Stop
- **Test**: Drive car forward towards a static cardboard box at $0.4\,\text{m/s}$; verify car stops cleanly at $\ge 0.3\,\text{m}$ distance with zero collision.

---

### TIER 2A: 3D Object Detection & Pivot Evasion

#### `[TICKET-2.1]` RVV-Accelerated 3D Euclidean Clustering
- **Goal**: Partition non-ground obstacle points into discrete physical objects.
- **Algorithm**: `rvpoint::EuclideanClusterer` using `Fast3DSpatialGrid` with cluster tolerance $d_{\text{tol}} = 0.08\,\text{m}$ and size bounds $[30, 2000]$ points.

#### `[TICKET-2.2]` 2D Convex Hull & 3D Oriented Bounding Box (OBB)
- **Goal**: Compute precise bounding geometry and orientation for each cluster without external libraries.
- **Steps**:
  1. Project 3D cluster points to 2D $(X_i, Y_i)$.
  2. Compute 2D Convex Hull via **Andrew's Monotone Chain** ($O(K \log K)$).
  3. Compute Minimum Area Bounding Box via **Rotating Calipers** ($O(K)$).
  4. Compute vertical elevation bounds $[Z_{\text{min}}, Z_{\text{max}}]$.
  5. Output: Center $(c_x, c_y, c_z)$, extents $(e_x, e_y, e_z)$, and yaw angle $\theta_{\text{box}}$.

#### `[TICKET-2.3]` Lateral Evasion & Intelligent Pivot Selector
- **Goal**: When an obstacle enters the warning zone ($d < 1.2\,\text{m}$):
  - If obstacle centroid $c_y > 0$ (left of centerline), bias steering right ($\omega_z < 0$).
  - If obstacle centroid $c_y < 0$ (right of centerline), bias steering left ($\omega_z > 0$).
  - If $|c_y| < 0.15\,\text{m}$ and distance $< 0.5\,\text{m}$, halt forward motion ($v_x = 0$) and execute a zero-radius **Pivot Turn** toward the side with greater open clearance until the corridor is clear for $> 300\,\text{ms}$.

#### `[TICKET-2.4]` Closed-Loop Bench Test: Pivot & Evasion
- **Test**: Place an obstacle centered on the track; car approaches, halts at $0.5\,\text{m}$, pivots $45^\circ$, navigates around the box, and resumes forward path.

---

### TIER 3: Local Spatial Memory & 2.5D Rolling Costmap

#### `[TICKET-3.1]` VIO-Stabilized Rolling Multi-Frame Costmap
- **Goal**: Maintain a body-centric $150 \times 150$ grid ($6\,\text{m} \times 6\,\text{m}$ at $4\,\text{cm/cell}$) accumulating LiDAR points across frames using iPhone VIO poses $\mathbf{T}_{\text{world}}^{\text{cam}}$.
- **Advantage**: Retains memory of obstacles even when the camera pivots away from them during a turn.

#### `[TICKET-3.2]` Vectorized Separable Distance Transform
- **Goal**: Compute exact Euclidean distance from every grid cell to the nearest obstacle.
- **Algorithm**: Parabolic envelope distance transform (Felzenszwalb & Huttenlocher). Vectorized with RVV 1.0 intrinsics for $< 3\,\text{ms}$ execution on the SpacemiT K1.

#### `[TICKET-3.3]` Dynamic Tentacle / Gap-Finder Trajectory Generator
- **Goal**: Evaluate an array of 17 candidate circular trajectory arcs ("tentacles") against the clearance field:
  $$J(\kappa) = \alpha \cdot \text{Clearance}(\kappa) + \beta \cdot \text{Progress}(\kappa) - \gamma \cdot |\kappa|$$
  Select best curvature $\kappa^* \to$ command $v_{\text{left}} = v(1 - \kappa^* L/2), v_{\text{right}} = v(1 + \kappa^* L/2)$.

---

### TIER 4: Global Mission, Waypoint Navigation & Path Planning

#### `[TICKET-4.1]` Interactive Foxglove 3D Goal Pose Receiver
- **Goal**: Enable user to click a 2D/3D Goal Pose on the Foxglove Studio map view.
- **Implementation**: Foxglove WebSocket subscriber decoding destination $(X^*, Y^*, \theta^*)$ and injecting it into the global planner.

#### `[TICKET-4.2]` Global 2D Grid A* Path Planner
- **Goal**: Compute the globally optimal collision-free waypoint path from current vehicle pose to the goal coordinate.
- **Algorithm**: 8-connected $A^*$ search on the accumulated arena occupancy grid with Euclidean distance heuristic and safety inflation radius ($R_{\text{inflate}} = 0.22\,\text{m}$).

#### `[TICKET-4.3]` Pure Pursuit Waypoint Path Tracker
- **Goal**: Calculate vehicle steering curvature to track the global $A^*$ path smoothly.
- **Formulation**:
  $$\kappa_{\text{pursuit}} = \frac{2 \sin\alpha}{L_{\text{lookahead}}}$$
  where $\alpha$ is the angle between current vehicle heading and the lookahead point on the global path at distance $L_{\text{lookahead}} \approx 0.5\,\text{m}$.

#### `[TICKET-4.4]` Hierarchical Fusion: Global Waypoints Guided by Local Tentacles
- **Goal**: Fuse global path guidance with local costmap obstacle avoidance.
- **Objective Function**: The tentacle evaluator adds a target heading alignment term towards the Pure Pursuit lookahead point:
  $$J(\kappa) = w_{\text{clearance}} \cdot \text{Clearance}(\kappa) + w_{\text{goal}} \cdot \cos(\theta_{\text{tentacle}} - \theta_{\text{lookahead}}) - w_{\kappa} \cdot |\kappa|$$

---

### TIER 5: Persistent Spatial Memory & Drift Correction

#### `[TICKET-5.1]` Persistent Arena Submap Accumulator & Disk Serialization
- **Goal**: Retain a persistent point cloud map of the entire arena and save/load it to `output/maps/arena_map.pcd`.
- **Keyframe Insertion**: Insert incoming point cloud into the submap whenever vehicle travel $> 0.25\,\text{m}$ or yaw rotation $> 15^\circ$.

#### `[TICKET-5.2]` Analytical Cardano Surface Normal Estimation
- **Goal**: Compute unit surface normals $\mathbf{n}_i$ for submap and keyframe points using RVPoint's Cardano analytical eigensolver (`rvpoint::estimate_normals_cardano`).

#### `[TICKET-5.3]` 6-DoF Point-to-Plane ICP Registration
- **Goal**: Align keyframe point clouds against the persistent arena submap to estimate VIO drift.
- **Formulation**: Linearized Gauss-Newton minimizing point-to-plane residual $r_i = (\mathbf{R}\mathbf{s}_i + \mathbf{t} - \mathbf{d}_i) \cdot \mathbf{n}_i$. Solves $6 \times 6$ Hessian $\mathbf{H} \Delta \mathbf{\xi} = \mathbf{b}$ via closed-form Cholesky factorization in $< 5\,\text{ms}$ on SpacemiT K1.

#### `[TICKET-5.4]` Drift Correction Transform ($\mathbf{T}_{\text{map}}^{\text{odom}}$) Closed-Loop Update
- **Goal**: Maintain coordinate consistency:
  $$\mathbf{T}_{\text{map}}^{\text{body}} = \mathbf{T}_{\text{map}}^{\text{odom}} \cdot \mathbf{T}_{\text{odom}}^{\text{body}}$$
  Updates $\mathbf{T}_{\text{map}}^{\text{odom}}$ in the background every $0.5\,\text{s}$ so global planning and Foxglove visualization remain drift-free throughout hours of operation.

---

### TIER 6: Closed-Loop Regulation & Autonomy FSM

#### `[TICKET-6.1]` VIO State Observer (Alpha-Beta Filter on Differentiated Poses)
- **Goal**: Cleanly extract ground-truth body linear velocity $\hat{v}_x$ and yaw rate $\hat{\omega}_z$ from discrete, noisy VIO poses at 60 Hz without noise amplification.

#### `[TICKET-6.2]` Body-Level Decoupled Dual PID Speed & Yaw Rate Regulator
- **Goal**: Maintain exact commanded linear speed ($0.4\,\text{m/s}$) and yaw rate regardless of battery discharge (12.6V $\to$ 11.1V) or floor friction, using anti-windup clamping and stiction compensation.

#### `[TICKET-6.3]` Hierarchical Autonomy Finite State Machine (FSM)
- **Goal**: Robustly handle mission transitions and corner traps (`IDLE`, `GLOBAL_TRACKING`, `LOCAL_AVOIDANCE`, `RECOVERY_DEADLOCK`, `GOAL_REACHED`, `FAILSAFE`).

#### `[TICKET-6.4]` Full Arena End-to-End Verification
- **Test**: Set a goal across the $4\,\text{m} \times 4\,\text{m}$ room with 4 scattered obstacle boxes; car plans global path, smoothly evades obstacles, recovers from a simulated corner trap, and reaches target within $0.1\,\text{m}$ accuracy.

---

## 4. Vehicle Modeling, State Estimation & Closed-Loop Control Architecture

### 4.1 Kinematic Omni-Tank Model (Physics of Omni Wheels)

The vehicle possesses 4 omnidirectional wheels arranged axially along the sides of the chassis.
- Wheel radius: $r = 0.030\,\text{m}$ (60 mm wheel diameter).
- Effective track width: $L = 0.180\,\text{m}$ (transverse distance between left and right contact patches).
- Wheelbase: $B = 0.160\,\text{m}$ (longitudinal distance between front and rear axles).

```
          ^ X (Forward)
          |
    [FL]  |  [FR]          FL / RL wired in parallel -> Left H-Bridge Channel
     ||   |   ||           FR / RR wired in parallel -> Right H-Bridge Channel
     ||---|---||
     |    |    |           Rollers on omni perimeter rotate PERPENDICULAR to wheel.
     |----+----> Y (Left)  In lateral direction (Y), rolling resistance is near-zero!
     |    |    |
     ||---|---||
     ||       ||
    [RL]     [RR]
```

#### The Scrub-Free Differential Advantage:
In a standard 4-wheel drive car with rubber tires, executing a zero-radius turn ($v_x = 0, \omega_z \ne 0$) forces the tires to **skid laterally** against the floor. This causes severe mechanical scrub, high stiction torque, unpredictable center-of-rotation (ICR) shifts, and high motor current draw.

On an **omni-wheel chassis**:
- In the longitudinal axis $\hat{\mathbf{x}}$, the wheel motor commands traction via the wheel rim.
- In the lateral axis $\hat{\mathbf{y}}$, the passive barrel rollers roll freely with negligible friction ($\mu_{\text{roll}} \approx 0.02 \ll \mu_{\text{rubber}} \approx 0.8$).
- Therefore, when turning, front and rear wheels translate sideways on their passive rollers with **zero lateral scrub**.
- The kinematics reduce strictly and cleanly to the **ideal unicycle differential drive model**:

$$\begin{bmatrix} v_L \\ v_R \end{bmatrix} = \begin{bmatrix} 1 & -\frac{L}{2} \\ 1 & \frac{L}{2} \end{bmatrix} \begin{bmatrix} v_x \\ \omega_z \end{bmatrix}$$

Inverse transformation (Forward Kinematics):
$$v_x = \frac{v_R + v_L}{2}, \quad \omega_z = \frac{v_R - v_L}{L}$$

---

### 4.2 Motor & Actuator Dynamics

Each wheel is driven by a DC brushed gearmotor powered through an H-bridge.
The electrical dynamics are given by:
$$V_a(t) = R_a i_a(t) + K_e \omega_m(t)$$
and mechanical rotor acceleration is:
$$J_{\text{eq}} \dot{\omega}_m(t) + B_{\text{eq}} \omega_m(t) = K_t i_a(t) - \tau_{\text{stiction}} \operatorname{sgn}(\omega_m)$$

Combining these yields a classic first-order lag with non-linear stiction deadband:
$$\tau_{\text{mech}} \dot{v}_x(t) + v_x(t) = K_{\text{gain}} \cdot u(t) - v_{\text{deadband}} \operatorname{sgn}(u)$$
where $u \in [-1.0, 1.0]$ is the commanded PWM duty cycle ratio.
- Due to gearbox stiction, motors do not turn below $|u| \approx 0.15$ (the **deadband**).
- As battery voltage sags from 12.6V down to 11.1V, $K_{\text{gain}}$ drops by $\approx 12\%$, requiring closed-loop integral compensation.

---

### 4.3 VIO State Observer (Velocity & Yaw Rate Reconstructor)

Because the system does not use physical wheel encoders, actual ground-truth vehicle speed and yaw rate are observed from the iPhone 14 Pro's 60 Hz VIO pose stream.

Given consecutive rigid-body transformations $\mathbf{T}_{k-1} = [\mathbf{R}_{k-1} \mid \mathbf{t}_{k-1}]$ and $\mathbf{T}_k = [\mathbf{R}_k \mid \mathbf{t}_k]$ with timestamps $t_{k-1}, t_k$ ($\Delta t = t_k - t_{k-1}$):

1. **Longitudinal Displacement in Body Frame**:
   $$\Delta \mathbf{t}_k^{\text{body}} = \mathbf{R}_k^T (\mathbf{t}_k - \mathbf{t}_{k-1})$$
   $$v_{\text{raw}, k} = \frac{\Delta x_k^{\text{body}}}{\Delta t_k}$$

2. **Heading Yaw Displacement**:
   $$\Delta \mathbf{R} = \mathbf{R}_{k-1}^T \mathbf{R}_k$$
   $$\Delta \theta_k = \operatorname{atan2}(R_{10}, R_{00})$$
   $$\omega_{\text{raw}, k} = \frac{\Delta \theta_k}{\Delta t_k}$$

3. **Recursive Low-Pass Filter ($\alpha$-$\beta$ Observer)**:
   To prevent derivative amplification of high-frequency visual jitter, the raw measurements pass through an exponential moving average:
   $$\hat{v}_k = (1 - \alpha_v) \hat{v}_{k-1} + \alpha_v v_{\text{raw}, k} \quad (\alpha_v = 0.25)$$
   $$\hat{\omega}_k = (1 - \alpha_\omega) \hat{\omega}_{k-1} + \alpha_\omega \omega_{\text{raw}, k} \quad (\alpha_\omega = 0.30)$$

---

### 4.4 Body-Level Decoupled Dual PID Controller

Instead of running separate PIDs on unmeasured wheel velocities, the controller operates in the **body frame**, directly regulating $[v_x, \omega_z]^T$:

```
Commanded [v_cmd, omega_cmd]         VIO Estimated [v_hat, omega_hat]
         |                                         |
         v                                         v
+-----------------------------------------------------------------------+
|                 Body-Level Dual PID Controller                        |
|                                                                       |
|  1. Linear Velocity Loop:                                             |
|     e_v = v_cmd - v_hat                                               |
|     u_v = Kp_v*e_v + Ki_v*Int(e_v) - Kd_v*(dv_hat/dt) + FF_v(v_cmd)   |
|                                                                       |
|  2. Angular Yaw Rate Loop:                                            |
|     e_w = omega_cmd - omega_hat                                       |
|     u_w = Kp_w*e_w + Ki_w*Int(e_w) - Kd_w*(dw_hat/dt) + FF_w(omega)    |
|                                                                       |
|  3. Stiction Compensation:                                            |
|     u_v += sgn(u_v) * V_deadband                                      |
+-----------------------------------+-----------------------------------+
                                    | [u_v, u_w]
                                    v
+-----------------------------------------------------------------------+
|                    Kinematic Inverse Tank Mixer                       |
|                                                                       |
|  u_left  = u_v - u_w                                                  |
|  u_right = u_v + u_w                                                  |
|                                                                       |
|  Saturation Handling & Priority Yaw Scaling:                          |
|  If max(|u_left|, |u_right|) > 1.0:                                   |
|    scale = 1.0 / max(|u_left|, |u_right|)                             |
|    u_left *= scale, u_right *= scale                                  |
|                                                                       |
|  Anti-Windup: Freeze Integrators when saturated                       |
+-----------------------------------+-----------------------------------+
                                    | Commanded Duty [-1.0, 1.0]
                                    v
+-----------------------------------------------------------------------+
|        H-Bridge Hardware Bridge (/sys/class/pwm & libgpiod)           |
+-----------------------------------------------------------------------+
```

#### Key Controller Features:
1. **Feed-Forward Term**:
   $$u_{\text{ff}, v} = K_{\text{ff}, v} \cdot v_{\text{cmd}}, \quad u_{\text{ff}, \omega} = K_{\text{ff}, \omega} \cdot \omega_{\text{cmd}}$$
   Enables instant throttle response; the PID error terms only need to compensate for friction variations and battery sag.
2. **Derivative-on-Measurement**:
   Computes $D = -K_d \frac{d\hat{y}}{dt}$ instead of $K_d \frac{de}{dt}$, completely eliminating **derivative kick** when the path planner issues sudden velocity step commands.
3. **Conditional Back-Calculation Anti-Windup**:
   Whenever $u_{\text{left}}$ or $u_{\text{right}}$ hits the physical limit ($\pm 1.0$) and the error $e(t)$ has the same sign as $u$, integral accumulation is frozen:
   $$\text{If } (|u| \ge 1.0 \text{ and } e \cdot u > 0) \implies \dot{I} = 0$$
4. **Stiction Injection**:
   Overcomes gearbox stiction by adding a minimum offset $V_{\text{deadband}} \approx 0.15$ whenever non-zero motion is commanded.

---

### 4.5 C++ Header Implementation Reference

```cpp
namespace rvpoint {

struct VelocitySetpoint {
    float v_linear_mps = 0.0f;     // Desired body linear speed (e.g. 0.4 m/s)
    float omega_yaw_radps = 0.0f;   // Desired body yaw rate (e.g. 1.5 rad/s for pivot)
};

struct MotorDutyCommand {
    float duty_left = 0.0f;   // [-1.0, 1.0]
    float duty_right = 0.0f;  // [-1.0, 1.0]
};

class VioStateObserver {
public:
    void update(const float T_world_cam[16], double timestamp_sec) {
        if (prev_time_ < 0.0) {
            prev_time_ = timestamp_sec;
            extract_pose(T_world_cam, prev_x_, prev_y_, prev_yaw_);
            return;
        }
        double dt = timestamp_sec - prev_time_;
        if (dt <= 1e-4) return;

        float curr_x, curr_y, curr_yaw;
        extract_pose(T_world_cam, curr_x, curr_y, curr_yaw);

        // Body-frame longitudinal displacement
        float dx_world = curr_x - prev_x_;
        float dy_world = curr_y - prev_y_;
        float dx_body = dx_world * std::cos(prev_yaw_) + dy_world * std::sin(prev_yaw_);
        float raw_v = static_cast<float>(dx_body / dt);

        // Yaw rate
        float dyaw = curr_yaw - prev_yaw_;
        while (dyaw > M_PI) dyaw -= 2.0f * M_PI;
        while (dyaw < -M_PI) dyaw += 2.0f * M_PI;
        float raw_omega = static_cast<float>(dyaw / dt);

        // Alpha-Beta exponential filtering
        v_hat_ = (1.0f - alpha_v_) * v_hat_ + alpha_v_ * raw_v;
        omega_hat_ = (1.0f - alpha_w_) * omega_hat_ + alpha_w_ * raw_omega;

        prev_x_ = curr_x; prev_y_ = curr_y; prev_yaw_ = curr_yaw;
        prev_time_ = timestamp_sec;
    }

    float linear_velocity() const { return v_hat_; }
    float yaw_rate() const { return omega_hat_; }

private:
    float alpha_v_ = 0.25f, alpha_w_ = 0.30f;
    float v_hat_ = 0.0f, omega_hat_ = 0.0f;
    float prev_x_ = 0.0f, prev_y_ = 0.0f, prev_yaw_ = 0.0f;
    double prev_time_ = -1.0;
    static void extract_pose(const float T[16], float& x, float& y, float& yaw) {
        x = T[12]; y = T[13];
        yaw = std::atan2(T[1], T[0]);
    }
};

class DecoupledDualPidController {
public:
    MotorDutyCommand compute(const VelocitySetpoint& sp, float v_hat, float w_hat, float dt) {
        // 1. Linear velocity PID with Derivative-on-Measurement
        float e_v = sp.v_linear_mps - v_hat;
        float d_v = -(v_hat - prev_v_hat_) / dt;
        prev_v_hat_ = v_hat;

        float u_v = (kp_v_ * e_v) + int_v_ + (kd_v_ * d_v) + (kff_v_ * sp.v_linear_mps);
        if (std::abs(sp.v_linear_mps) > 1e-3f) {
            u_v += (u_v > 0.0f ? deadband_ : -deadband_);
        }

        // 2. Angular yaw rate PID with Derivative-on-Measurement
        float e_w = sp.omega_yaw_radps - w_hat;
        float d_w = -(w_hat - prev_w_hat_) / dt;
        prev_w_hat_ = w_hat;

        float u_w = (kp_w_ * e_w) + int_w_ + (kd_w_ * d_w) + (kff_w_ * sp.omega_yaw_radps);

        // 3. Differential Mixer
        float raw_left = u_v - u_w;
        float raw_right = u_v + u_w;

        // 4. Priority Scaling if saturated
        float max_val = std::max(std::abs(raw_left), std::abs(raw_right));
        bool saturated = false;
        if (max_val > 1.0f) {
            raw_left /= max_val;
            raw_right /= max_val;
            saturated = true;
        }

        // 5. Anti-windup conditional integration
        if (!saturated || (e_v * u_v <= 0.0f)) {
            int_v_ = std::clamp(int_v_ + ki_v_ * e_v * dt, -0.4f, 0.4f);
        }
        if (!saturated || (e_w * u_w <= 0.0f)) {
            int_w_ = std::clamp(int_w_ + ki_w_ * e_w * dt, -0.4f, 0.4f);
        }

        return {std::clamp(raw_left, -1.0f, 1.0f), std::clamp(raw_right, -1.0f, 1.0f)};
    }

private:
    float kp_v_ = 1.2f, ki_v_ = 2.5f, kd_v_ = 0.04f, kff_v_ = 1.5f;
    float kp_w_ = 0.6f, ki_w_ = 1.2f, kd_w_ = 0.02f, kff_w_ = 0.4f;
    float deadband_ = 0.15f;
    float int_v_ = 0.0f, int_w_ = 0.0f;
    float prev_v_hat_ = 0.0f, prev_w_hat_ = 0.0f;
};

} // namespace rvpoint
```

---

## 5. Hardware Pinout & Actuation Topology

| Signal | Function | Orange Pi RV2 Header Pin | Linux Interface |
|---|---|---|---|
| **PWM_L** | Left Motors Speed | Pin 32 (PWM0 / GPIO) | `/sys/class/pwm/pwmchip0/pwm0` |
| **DIR_L1** | Left Motors Direction A | Pin 11 (GPIO) | `libgpiod` line |
| **DIR_L2** | Left Motors Direction B | Pin 13 (GPIO) | `libgpiod` line |
| **PWM_R** | Right Motors Speed | Pin 33 (PWM1 / GPIO) | `/sys/class/pwm/pwmchip0/pwm1` |
| **DIR_R1** | Right Motors Direction A | Pin 15 (GPIO) | `libgpiod` line |
| **DIR_R2** | Right Motors Direction B | Pin 16 (GPIO) | `libgpiod` line |
| **GND** | Logic Common Ground | Pin 6 / Pin 14 | Logic Ground Rail |

---

## 6. Complete Verification Matrix (Tiers 0 through 6)

| Tier | Test Case | Target Metric / Pass Criteria |
|---|---|---|
| **0.1** | `test_udp_ingestion` | 30 FPS mock UDP packets ingested with $< 3\,\text{ms}$ socket latency; zero drops. |
| **0.2** | `test_pwm_watchdog` | `kill -9` or packet stall causes complete motor stop within $< 100\,\text{ms}$. |
| **0.3** | `test_foxglove_ws` | Browser connects to `ws://orangepi:8765`; renders live point cloud & vehicle state. |
| **1.1** | `test_ground_filter` | SPRT RANSAC separates floor points with $> 98\%$ accuracy on flat room. |
| **1.2** | `test_estop_corridor` | Simulated barrier in safety envelope triggers zero-velocity command in $< 2\,\text{ms}$. |
| **2.1** | `test_obb_extraction` | 3D OBB computed for obstacle; dimensions and center within $\pm 3\,\text{cm}$ tolerance. |
| **2.2** | `test_pivot_evasion` | Vehicle detects obstacle, executes clean zero-radius pivot turn until clear. |
| **3.1** | `test_costmap_clearance` | $150 \times 150$ distance transform executes in $< 3\,\text{ms}$ via RVV 1.0 intrinsics. |
| **3.2** | `test_tentacle_planner` | Vehicle smoothly maneuvers around staggered slalom obstacles. |
| **4.1** | `test_global_astar` | $A^*$ finds optimal collision-free waypoint path on $150 \times 150$ grid in $< 10\,\text{ms}$. |
| **4.2** | `test_pure_pursuit` | Vehicle tracks waypoint path with cross-track error $< 0.05\,\text{m}$. |
| **5.1** | `test_submap_icp` | Point-to-plane ICP achieves $< 2\,\text{cm}$ translational error against submap. |
| **5.2** | `test_drift_correction` | VIO intentional offset corrected by ICP; map consistency preserved over 10 min. |
| **6.1** | `test_velocity_pid` | Vehicle maintains $0.4\,\text{m/s} \pm 0.02\,\text{m/s}$ across battery range (12.6V down to 11.1V). |
| **6.2** | `test_recovery_deadlock`| Box placed directly surrounding car triggers reverse $\to$ $90^\circ$ pivot $\to$ escape. |
| **6.3** | `test_full_arena_mission`| Car navigates from start to user-selected goal across arena with 4 obstacles. |
