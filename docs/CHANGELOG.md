# RVPoint: RISC-V Vector Point Cloud Library — Conceptual Changelog

This document maintains a detailed, conceptually structured record of all architectural updates, algorithmic enhancements, vectorization strategies, microarchitectural optimizations, and infrastructural changes across the evolution of the **RVPoint** codebase.

---

## [Unreleased / Recent Refactoring] - 2026-07-28

### 1. Vectorized Algorithmic Evolution & Memory Alignment

#### Fully Vectorized Voxel Grid Downsampling (`voxel_grid_downsamp_rvv_v2`)
* **Conceptual Problem**: The initial RVV hybrid implementation (`voxel_grid_downsamp_rvv`) used RVV vector instructions for coordinate scaling but relied on `std::map` key insertions for grouping points into voxels. The $O(N \log N)$ scalar bottleneck of tree insertions dominated runtime, nullifying vector speedups at large point counts ($N > 10^5$).
* **Vectorized Solution**: Introduced `voxel_grid_downsamp_rvv_v2`, replacing `std::map` with a sort-based vector reduction:
  1. **Bounding Box & Voxel Key Calculation**: Vectorized `vfmin`/`vfmax` reductions compute point cloud extents. Vector floating-point-to-integer conversion (`vfcvt_rtz`) maps $(x,y,z)$ coordinates to a linear 3D voxel index.
  2. **Index Sorting**: Sorts point indices by linear voxel key, grouping all points residing in the same spatial voxel into contiguous memory ranges.
  3. **Vector Gather & Reduction**: Uses vector indexed gather loads (`vluxei32`) and vector reduction sums (`vfredusum`) to calculate centroid coordinates directly in RVV vector registers.
* **Deprecation Notice**: Annotated `voxel_grid_downsamp_rvv` with `[[deprecated]]` to guide applications toward `_v2`.

#### Pointer Octree Architecture (`PointerOctree`)
* **Conceptual Motivation**: Comparing traditional linear/Morton-indexed spatial octrees against pointer-based node hierarchies under RISC-V Vector execution.
* **Cache & Vector Layout**: Designed `PointerOctreeNode` with contiguous leaf coordinate arrays (`leaf_x`, `leaf_y`, `leaf_z`). During leaf-node radius search, this structure permits unit-stride vector loads (`vle32.v`) instead of high-latency strided loads (`vlse32.v`) or gather operations (`vluxei32`).
* **Dual Execution Paths**: Provided `radiusSearch` (RVV accelerated) and `radiusSearchScalar` for head-to-head benchmarking.

#### Uniform API Signature Standardization
* **Structure of Arrays (SoA) vs. Array of Structures (AoS)**: Explicitly partitioned C-style interface functions into scalar baseline calls (`*_sc`) operating on AoS (`PointXYZ*`) and vector calls (`*_rvv`, `*_rvv_v2`) operating on SoA (`PointCloudSoA`).
* **Coverage**: Standardized across [rvv_pcl.h](file:///e:/FahadProject/rvpoint/src/include/rvv_pcl.h), `normal_estimation.cpp`, `octree.cpp`, `ransac_plane.cpp`, `spatial_hashing.cpp`, `statistical_outlier_removal.cpp`, and `voxel_grid_downsamp.cpp`.

---

### 2. Infrastructure & Automated Testing Suite

#### Containerized Verification Overhaul
* **Aggregate Test Execution (`verify_container.sh`)**: Replaced calls to legacy CMake targets with a auto-discovery runner `run_tests()`. The updated script scans for all built `test_*/rvv_test` binaries, executes them under QEMU RISC-V emulation (`qemu-riscv64`), and outputs a structured Pass/Fail matrix.
* **Environment Setup (`docs/setup.md`)**: Created comprehensive setup documentation covering Docker environment setup, gem5 Docker-in-Docker integration, and `manuel313/gem5_v25` container paths.

#### Automated Experiment & Benchmark Harnesses
* **Batch Workload Processing**: Added [scripts/run_batch_pcd_compressed.sh](file:///e:/FahadProject/rvpoint/scripts/run_batch_pcd_compressed.sh) to execute pipeline workloads across compressed point cloud archives.
* **Pointer Octree Evaluation**: Added [scripts/run_pointer_octree_bench.sh](file:///e:/FahadProject/rvpoint/scripts/run_pointer_octree_bench.sh) and [tests/benchmark_pointer_octree_real.cpp](file:///e:/FahadProject/rvpoint/tests/benchmark_pointer_octree_real.cpp).
* **Gem5 Microarchitecture Comparison**: Added [scripts/run_gem5_compare.sh](file:///e:/FahadProject/rvpoint/scripts/run_gem5_compare.sh) to capture detailed cycle, instruction count, and vector register usage metrics.

---

### 3. Repository Optimization & Dataset Housekeeping

* **Binary Data Compression**: Removed raw, uncompressed binary `.pcd` test datasets (`0000000010.pcd`, `bunny.pcd`, `indoor_scene.pcd`, `living_room.pcd`, `table_scene_lms400.pcd`) from version control to minimize repository footprint and git clone overhead.
* **Compressed Workload Archive**: Transitioned test harnesses to extract data on demand from `data/pcd_compressed.zip`.

---

## [v0.4.0] - Spatial Hashing & Advanced Neighbor Search

### 1. Architectural Concepts & Features

#### Spatial Hash Grid (`SpatialHash`)
* **Conceptual Design**: Provides an $O(1)$ spatial lookup table for point cloud queries where tree-building overhead is prohibitive.
* **Grid Discretization**: Hashes 3D points into discrete integer cells using prime-number multiplier hash functions.
* **RVV Query Acceleration**: Accelerates neighbor candidate distance checks using SIMD floating-point distance math over SoA cell contents.

#### Caravan-Based Radius Search
* **Multi-Query Vectorization**: Processes multiple query points simultaneously ("caravan" style) to saturate RISC-V Vector registers (`vlen`), maximizing vector register reuse across adjacent queries.

---

## [v0.3.0] - Class Structuring & Pipeline Automation

### 1. Object-Oriented Wrappers & Export Tools

#### High-Level C++ Abstractions
* Structured functional kernels into reusable class interfaces: `rvv_pcl::VoxelGridFilter`, `rvv_pcl::SORFilter`, `rvv_pcl::RANSACFilter`, `rvv_pcl::NormalEstimation`, and `rvv_pcl::OctreeNeighborSearch`.
* Encapsulated configuration parameters (e.g., `leaf_size`, `mean_k`, `std_dev_mul`, `distance_threshold`) with standard setter methods.

#### Pipeline Exporter (`pipeline_export.cpp`)
* Added automated pipeline tool to sequence multiple processing stages (Voxel Grid -> SOR -> Normal Estimation -> RANSAC Plane Extraction) and output benchmark metrics for end-to-end evaluation.

---

## [v0.2.0] - Kernel Optimizations & QEMU 9 Toolchain

### 1. Microarchitectural Vector Tuning

#### Inlined Statistical Outlier Removal (SOR) Kernel
* Inlined distance computing vector loops directly into neighbor search loops, eliminating function-call stack overhead within hot loops.
* Utilized vector mask registers (`vbool*`) to evaluate $k$-nearest neighbor distance thresholds in parallel.

#### Toolchain Upgrade
* Updated build system and execution scripts to target **QEMU 9.x**, supporting the official RISC-V Vector Extension 1.0 (`rv64gcv`).

---

## [v0.1.0] - Initial RVPoint Core

### 1. Initial Implementation

* Foundation RISC-V Vector point cloud library implementing:
  * SoA point cloud representation (`PointCloudSoA`).
  * Normal Estimation via 3D Covariance Matrix PCA eigen-decomposition.
  * RANSAC 3D Plane fitting with vector distance check masks.
  * Basic Voxel Grid Downsampling and Statistical Outlier Removal (SOR).
