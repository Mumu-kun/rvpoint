# Static Elevation Slicing over Per-Frame RANSAC for Flat Indoor Navigation

Use an RVV 1.0 vectorized PassThrough elevation slice ($Z \in [Z_{\text{floor}}, Z_{\text{ceiling}}]$) during nominal driving, executing SPRT plane RANSAC only once at startup to calibrate floor height and tilt.

## Status
Accepted

## Context
Removing floor points is mandatory before obstacle clustering and costmap generation. While RANSAC plane fitting dynamically adapts to sloping surfaces, executing RANSAC on every frame at 30 Hz consumes $\sim 1.5\,\text{ms}$ per frame on the SpacemiT K1. In flat indoor arena environments ($4\,\text{m} \times 4\,\text{m}$) where camera extrinsics ($\mathbf{T}_{\text{body}}^{\text{cam}}$) rigidly level the point cloud to the ground plane ($Z \approx 0.00\,\text{m}$), fitting a plane every 33 ms is computationally redundant.

## Decision
1. **Startup Calibration Phase**: When the vehicle boots or is stationary, execute `rvpoint::SPRTPlaneEstimator` on initial frames to fit the dominant ground plane equation $ax + by + cz + d = 0$ and fine-tune extrinsics pitch and height.
2. **Nominal Driving Phase**: Freeze the ground plane height $Z_{\text{floor\_thresh}} \approx 0.03\,\text{m}$. Filter ground and ceiling points using an RVV 1.0 vectorized PassThrough filter (`__riscv_vmfgt_vf` + `__riscv_vmflt_vf` + `__riscv_vcompress_vm`).

## Consequences
- **Positive**: Reduces ground filtering latency from $1.5\,\text{ms}$ down to $< 0.05\,\text{ms}$ ($30\times$ speedup); frees $> 96\%$ of CPU cycles on the SpacemiT K1 for downstream clustering and SLAM; runs as a single-pass branchless vector stream.
- **Negative**: Will classify ramps or severe floor tilts as obstacles unless the vehicle stops and triggers recalibration.

