# Scan-to-Submap Point-to-Plane ICP for VIO Drift Correction

Periodically align keyframe point clouds against a persistent global arena submap using RVPoint's analytical Cardano surface normal estimator and linearized point-to-plane ICP to estimate and correct ARKit VIO drift.

## Status
Accepted

## Context
While iPhone ARKit Visual-Inertial Odometry provides high-rate (60 Hz) low-latency 6-DoF pose estimates, it inevitably suffers from unbounded translational and yaw drift over extended operational runs, especially during rapid zero-radius pivot turns or when visual features are sparse. To ensure consistent global navigation and persistent spatial memory, this drift must be corrected.

## Decision
Maintain a persistent global arena submap in the Orange Pi RV2 memory.
Trigger a background registration task every 0.5 seconds or following a pivot turn:
1. Extract analytical surface normals on the downsampled keyframe using RVPoint's closed-form Cardano eigensolver.
2. Align the keyframe against the persistent submap via linearized point-to-plane ICP solving $6 \times 6$ normal equations via Cholesky decomposition.
3. Compute the drift correction matrix $\mathbf{T}_{\text{map}}^{\text{odom}} = \mathbf{T}_{\text{map}}^{\text{body}} (\mathbf{T}_{\text{odom}}^{\text{body}})^{-1}$.
4. Broadcast this correction so the global planner and path tracker always operate in a drift-free coordinate frame.

## Consequences
- **Positive**: Eliminates VIO long-term drift; provides true spatial memory; directly leverages and showcases RVPoint's native 3D vector capabilities (`librvpoint.a`).
- **Negative**: Adds periodic background compute (estimated $< 5\,\text{ms}$ on SpacemiT K1).

