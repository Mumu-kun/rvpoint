# Dual Obstacle Representation: Bounding Discs for Evasion, 3D OBBs for Telemetry

Employ a dual obstacle geometry representation: compute lightweight Bounding Discs (Centroid + Maximum Radius) for the high-frequency vehicle steering and collision evasion loop, while computing full 3D Oriented Bounding Boxes (OBBs) via Rotating Calipers strictly for Foxglove Studio visual telemetry.

## Status
Accepted

## Context
After 3D Euclidean clustering, downstream planning requires geometric bounds to steer around obstacles. Computing exact 3D Oriented Bounding Boxes (OBBs) using 2D Convex Hull (Andrew's Monotone Chain, $O(K \log K)$) and Rotating Calipers ($O(K)$) yields precise bounding prisms. However, executing polygon-to-polygon collision checks (Separating Axis Theorem) across multiple rotated boxes inside the 50 Hz control loop introduces unnecessary computational overhead and orientation singularities on symmetric objects (e.g. cylindrical poles, circular cones).

## Decision
Separate the representation between the real-time control path and the visualization path:
1. **Real-Time Evasion Path (High Priority)**: Extract the obstacle centroid $\mathbf{c} = [c_x, c_y]^T$ and bounding radius $R = \max_i \|\mathbf{p}_i - \mathbf{c}\|$ using an $O(K)$ vectorized reduction (`__riscv_vfredmax`). Treat obstacles as isotropic safety discs ($R_{\text{effective}} = R + R_{\text{vehicle}} + d_{\text{margin}}$). This enables trivial, branchless 2D point-to-circle clearance evaluation.
2. **Telemetry & Visualizer Path (Low Priority / Background)**: Compute full 2D Convex Hull + Rotating Calipers 3D OBBs ($c_x, c_y, c_z, e_x, e_y, e_z, \theta$) and transmit them over WebSockets to Foxglove Studio for human observation and demonstration.

## Consequences
- **Positive**: Evasion steering evaluates in $< 20\,\mu\text{s}$ per cluster; isotropic clearance buffers guarantee collision-free margins regardless of obstacle yaw; 3D telemetry retains full visual fidelity.
- **Negative**: Bounding discs slightly overestimate clearance margins for high-aspect-ratio rectangular obstacles (e.g. long walls).

