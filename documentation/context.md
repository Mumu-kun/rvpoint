# RVPoint Project Context

## Project Summary

RVPoint is a research-focused point cloud processing library implemented in C++ for RISC-V with a strong emphasis on the RISC-V Vector Extension (RVV). It combines point cloud algorithms, spatial indexing strategies, and RVV accelerations to create a platform for exploring performance improvements in 3D data processing on vector-capable RISC-V hardware.

## Domain

- 3D point cloud processing
- RISC-V vector computing
- Spatial indexing and neighbor search
- Performance optimization for embedded/edge systems
- Geometry processing and segmentation

## Key Goals

1. **RVV-accelerated point cloud algorithms**
   - Provide both scalar and vectorized implementations for baseline comparison.

2. **Research on spatial search structures**
   - Implement and compare Octree, PointerOctree, SpatialHash, Caravan query packs, and global vector scans.

3. **Pipeline profiling and ablation analysis**
   - Measure stage-level performance and identify the most important bottlenecks.

4. **Practical evaluation on RISC-V platforms**
   - Target QEMU RV64GCV emulation and real RISC-V hardware such as Banana Pi BPI-F3.

## Technology Stack

- Language: C++17
- Build: CMake
- Target ISA: RISC-V 64-bit with Vector extension (`rv64gcv`)
- Vector backend: RISC-V Vector intrinsics (`<riscv_vector.h>`)
- Testing: native test binaries compiled via CMake and run under QEMU / RISC-V environment
- Scripts: Bash/Powershell helpers for container setup, verification, benchmarking, and pipeline profiling

## Core Algorithms

1. **Voxel Grid Downsampling**
   - Scalar baseline with `std::map` grouping
   - RVV sort-based downsample variant (`voxel_grid_downsamp_rvv_v2`)

2. **Statistical Outlier Removal (SOR)**
   - Scalar brute-force baseline
   - RVV distance kernel plus priority selection
   - Index-accelerated variants with Octree, SpatialHash, and PointerOctree

3. **Normal Estimation**
   - Covariance-based normal fitting using local neighbor search
   - RVV-enhanced path using spatial indexes and vectorized kernels

4. **Radius Search**
   - Scalar and RVV global scan implementations
   - Support for spatial search assisted radius queries

5. **RANSAC Plane Fitting**
   - Scalar consensus algorithm
   - RVV inlier counting and extraction

6. **Euclidean Clustering**
   - BFS-based region growing
   - RVV-accelerated radius query kernel for faster cluster extraction

## Spatial Indexing

The project explores several neighbor search designs:

- **Octree**: classic recursive partitioning.
- **SpatialHash**: uniform grid cell hash with RVV-assisted radius queries.
- **PointerOctree**: leaf nodes storing contiguous point coordinates for better RVV access patterns.
- **Caravan Query-Pack**: batched query handling using tile-based pruning and vector registers.

## Important Data Structures

- `PointXYZ` — simple AoS point representation for scalar code and output.
- `PointCloudSoA` — Structure of Arrays format used by RVV code.
- `Octree`, `SpatialHash`, `PointerOctree` — spatial search structures.
- `ClusterIndices` / `EuclideanClustering` — clustering infrastructure.

## Repository Structure

- `CMakeLists.txt` — main project build file.
- `src/include/rvv_pcl.h` — public API header.
- `src/rvv_common.cpp` — shared RVV kernels and utilities.
- `src/voxel_grid_downsamp.cpp` — downsampling algorithms.
- `src/statistical_outlier_removal.cpp` — outlier filtering.
- `src/normal_estimation.cpp` — normal estimation.
- `src/ransac_plane.cpp` — plane fitting.
- `src/euclidean_clustering.cpp` — clustering.
- `src/neighbor_search/` — spatial index implementations.
- `src/pointer_octree/` — pointer-based octree implementation.
- `src/tools/` — pipeline and benchmarking utilities.
- `tests/` — unit, integration, and benchmark tests.
- `scripts/` — environment setup, build wrappers, verify scripts, and profiling helpers.

## Innovations

This project includes the following notable research / implementation contributions:

- RVV-first design with a preference for SoA data layout.
- Vectorized voxel key generation and sort-based grouping for downsampling.
- RVV-friendly priority selection for K-NN computations in SOR.
- Spatial-index accelerated filtering methods to reduce $O(N^2)$ costs.
- `PointerOctree` leaf packing to improve RVV gather efficiency.
- RVV inlier counting for RANSAC using vector masks and popcount.
- Zero-overhead profiling macros and exportable stage timing metrics.

## Use Cases

This repository is useful for:

- researching RVV-based geometry processing,
- comparing spatial indexing strategies in point cloud pipelines,
- experimenting with performance engineering for RISC-V,
- building a starting point for RISC-V-enabled 3D perception software.

## Future Research Directions

Potential extension areas that align with this codebase:

- adding multi-threading along with RVV acceleration,
- supporting denser point clouds and streaming data,
- evaluating RVV performance on physical RISC-V hardware at different VLEN sizes,
- integrating more advanced segmentation or object detection algorithms,
- exploring sparse data structures and compressed point cloud representations.

## Context for Literature Review

When querying an LLM or writing a literature review, use this project context to frame questions about:

- RISC-V Vector Extension optimizations for geometry kernels
- SoA vs AoS layout decisions for SIMD data processing
- spatial indexing tradeoffs for point cloud neighbor search
- scan-based vector algorithms vs tree-based search methods
- RANSAC acceleration in a vectorized and embedded environment
- profiling-driven algorithm selection in research pipelines

## Suggested Next-Question Prompts

Examples of useful follow-up prompts for an LLM:

- "Compare RVV-based vectorized distance computation with GPU-style SIMD strategies for point cloud filtering."
- "What are the latest advances in spatial hashing for high-dimensional point search?"
- "How can RANSAC be adapted to take advantage of wide vector units in embedded systems?"
- "What are common techniques for zero-overhead instrumentation in C++ research code?"
- "How does the `PointerOctree` design compare to standard octree implementations for vectorized neighbor queries?"
