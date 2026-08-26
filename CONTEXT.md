# RVPoint Domain Model

RVPoint is a high-performance, RISC-V Vector (RVV 1.0) accelerated 3D point cloud perception library and microarchitectural evaluation suite.

## Language

### Core Representation

**PointCloudSoA**:
Contiguous structure-of-arrays representation storing separated x, y, and z floating-point buffers to maximize vector load bandwidth.
_Avoid_: AoS point arrays, std::vector<PointXYZ> in vector hotpaths

**Point Cloud Decimation**:
Adaptive geometric voxel sampling that downsamples dense point clouds to an exact target point count while preserving geometric bounds and cluster features.
_Avoid_: Random subsampling, arbitrary truncation

### Perception Pipelines

**RVPoint Ultra Pipeline**:
The 10-stage hardware-vectorized perception pipeline combining spatial hashing, fused statistical outlier removal, closed-form Cardano normal estimation, SPRT RANSAC, and disjoint-set clustering.
_Avoid_: Legacy pipeline, baseline pipeline

**PCL Native Pipeline**:
The reference upstream Point Cloud Library perception pipeline compiling official PCL modules statically for RISC-V scalar baseline comparison.
_Avoid_: Synthetic baseline, mock PCL

### Pipeline Configuration

**Base Pitch ($\Delta$)**:
The single source of truth spatial voxel step size from which all downstream neighborhood search radii ($2.5\Delta$), RANSAC planar tolerances ($0.5\Delta$), and cluster linkage tolerances ($1.25\Delta$) are proportionally derived.
_Avoid_: Arbitrary tuning constants, independent parameter grids

**Resolution Preset**:
Standardized Base Pitch ($\Delta$) scale profiles tailored to specific physical scan domains (0.01m dense object, 0.02m standard tabletop, 0.05m outdoor LiDAR).
_Avoid_: Ad-hoc voxel configs, hardcoded thresholds

### Microarchitectural Evaluation

**Simulation Budget**:
Pre-simulation decimation threshold categorizing point counts by hardware cache residency and simulation wall-clock feasibility.
_Avoid_: Arbitrary downsampling, point cut

**Region of Interest (ROI)**:
Simulated execution window isolating algorithmic compute kernels from disk I/O and setup overhead using m5 pseudo-instructions or CLI flags.
_Avoid_: Full-trace capture, noisy benchmarking

