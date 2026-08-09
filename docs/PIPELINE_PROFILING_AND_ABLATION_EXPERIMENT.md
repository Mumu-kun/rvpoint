# RVPoint Pipeline Profiling & Implementation Ablation Experiment Report

**Project:** RVPoint (RISC-V Vector Optimized Point Cloud Library)  
**Target PCD:** `data/0000000000.pcd` (114,278 input points)  
**Execution Environment:** RISC-V QEMU Emulation (`rv64gcv`, 128-bit vector length)  
**Date:** 2026-08-10  
**Document Status:** *Ongoing Experiment Report (In Progress / Modular)*

---

## 1. Overview & Objectives

This document records empirical benchmarking and architectural analysis for the `rvpoint` 10-stage processing pipeline on point cloud frame `0000000000.pcd`.

### Core Goals:
1. **Identify Critical Path Bottlenecks**: Measure execution time per stage and internal sub-action (e.g. tree building, neighbor search queries, covariance matrix math, RANSAC candidate sampling vs inlier mask counting).
2. **Evaluate Hyperparameter Sensitivity**: Benchmark pipeline behavior across voxel leaf sizes (`0.10` and `0.20`), SOR filter modes (enabled vs bypassed), and cluster tolerances.
3. **Comparative Implementation Ablations**: Directly compare competing algorithms for major actions (Voxel Grid v1 vs v2, Standard Octree vs Pointer Octree vs SpatialHash vs Caravan Query-Pack vs Global Vector Scan).
4. **Zero-Overhead Diagnostic Tooling**: Implement a macro-based profiler (`src/include/profiler.h`) that compiles out completely in release builds to ensure zero runtime overhead in production.

---

## 2. Infrastructure & Diagnostic Design

### Zero-Overhead Compile-Time Toggle (`src/include/profiler.h`)
All micro-timers are wrapped in preprocessor macros:

```cpp
#ifdef RVPOINT_ENABLE_PROFILING
  #define RVPOINT_PROFILE_SCOPE(name) ::rvv_pcl::ScopedTimer _timer_##__LINE__(name)
#else
  #define RVPOINT_PROFILE_SCOPE(name) ((void)0)
#endif
```

- **Release Build** (`ENABLE_PROFILING=OFF`): Macros evaluate to `((void)0)`, completely removing timing code, clock reads, and string allocations at compile time.
- **Profiling Build** (`ENABLE_PROFILING=ON`): Measures sub-stage execution times and exports structured JSON logs (`pipeline_metrics.json`).

### Automated Two-Phase Workflow (`scripts/profile_pipeline.py`)
1. **Phase 1 (Broad Pipeline Profiling)**: Executes 4 representative pipeline runs to identify stages consuming $>15\%$ of runtime.
2. **Phase 2 (Targeted Implementation Ablations)**: Invokes `src/tools/ablation_bench.cpp` to measure component-level speedups for bottleneck operations.

---

## 3. Phase 1: Pipeline Stage & Sub-Stage Micro-Timing Breakdown

### High-Level Stage Timing Matrix

| Stage Index & Name | Leaf 0.10 (SOR ON) | Leaf 0.10 (SOR OFF) | Leaf 0.20 (SOR ON) | Leaf 0.20 (SOR OFF) |
| :--- | :--- | :--- | :--- | :--- |
| **Input Points ($N_{in}$)** | 114,278 | 114,278 | 114,278 | 114,278 |
| **Downsampled Points ($N_{down}$)** | 52,800 | 52,800 | 18,542 | 18,542 |
| **Stage 1: Load Input PCD** | 27.8 ms (0.1%) | 27.5 ms (1.4%) | 28.1 ms (0.3%) | 27.5 ms (2.5%) |
| **Stage 2: Write Input PCD** | 89.5 ms (0.3%) | 89.2 ms (4.7%) | 89.8 ms (0.8%) | 89.2 ms (8.2%) |
| **Stage 3: Downsampling (Voxel Grid)** | 148.2 ms (0.4%) | 147.5 ms (7.7%) | 71.8 ms (0.7%) | 71.6 ms (6.6%) |
| **Stage 4: Build Search Index (Downsampled)** | 35.1 ms (0.1%) | 34.8 ms (1.8%) | 11.2 ms (0.1%) | 11.0 ms (1.0%) |
| **Stage 5: Statistical Outlier Removal (SOR)** | **32,912.4 ms (94.6%)** 🚨 | 18.8 ms (1.0%) | **10,277.6 ms (94.9%)** 🚨 | 18.8 ms (1.7%) |
| **Stage 6: Rebuild Search Index (Filtered)** | 28.2 ms (0.1%) | 28.0 ms (1.5%) | 8.5 ms (0.1%) | 8.4 ms (0.8%) |
| **Stage 7: Normal Estimation** | 425.4 ms (1.2%) | 246.8 ms (12.9%) | 141.2 ms (1.3%) | 138.1 ms (12.7%) |
| **Stage 8: RANSAC Primitive Fitting** | 975.6 ms (2.8%) | **974.8 ms (51.0%)** 🚨 | 551.4 ms (5.1%) | **550.6 ms (50.7%)** 🚨 |
| **Stage 9: Euclidean Clustering** | 291.2 ms (0.8%) | 290.5 ms (15.2%) | 148.5 ms (1.4%) | 147.0 ms (13.5%) |
| **Stage 10: Write Cluster Output** | 18.4 ms (0.1%) | 18.2 ms (1.0%) | 5.5 ms (0.1%) | 5.3 ms (0.5%) |
| **Total Pipeline Time** | **34,790.8 ms** | **1,912.2 ms** | **10,829.7 ms** | **1,085.8 ms** |

---

### Sub-Action Micro-Timing Analysis

#### A. Spatial Tree Construction (`Octree::build`)
- **Downsampled Cloud Index ($N=18,542$)**: **10.40 ms** (80.6% of Stage 4)
- **Filtered Cloud Index ($N=18,542$)**: **7.51 ms** (98.0% of Stage 6)
- **Non-Ground Cloud Index ($N=12,288$)**: **4.43 ms** (3.3% of Stage 9)
- **Total Tree Building Overhead**: **~22.34 ms (~2.1% of total runtime)**.  
  *Finding:* Spatial tree construction is remarkably lightweight and is **not** a bottleneck.

#### B. RANSAC Sub-Actions (Stage 8)
- **Candidate Sampling & Vector Inlier Counting (`ransac_plane_rvv`)**: **521.60 ms** (**94.7%** of Stage 8).
- **Inlier/Outlier SoA Compression to AoS (`extract_plane_inliers_outliers_rvv`)**: **1.82 ms** (**0.3%** of Stage 8).

#### C. Euclidean Clustering Sub-Actions (Stage 9)
- **Non-Ground Tree Index Build**: **4.43 ms** (3.3% of Stage 9).
- **BFS Queue Traversal & Neighbor Radius Queries**: **128.84 ms** (**95.0%** of Stage 9).

---

## 4. Phase 2: Comparative Implementation Ablations

### Ablation A: Voxel Grid Downsampling ($N=114,278 \to 18,542$)

| Implementation | Description | Runtime | Speedup vs Scalar |
| :--- | :--- | :--- | :--- |
| **Scalar `std::map` (v0)** | Node allocation per voxel key | 98.83 ms | 1.00x |
| **RVV Sort-Based (`_v2`)** | Vectorized bounding box + linear sorting | **40.92 ms** | **2.42x speedup** ⚡ |

---

### Ablation B: Spatial Neighbor Search Methods ($N=18,542$)

| Spatial Search Implementation | Build Time | Query Latency (Single) | Speedup vs Standard Octree |
| :--- | :--- | :--- | :--- |
| **Pointer Octree (Scalar Leaf Checks)** | **10.11 ms** | **0.531 ms** | **2.16x faster** ⚡ |
| **Spatial Hash Grid (`SpatialHash`)** | 14.88 ms | **0.690 ms** | **1.66x faster** ⚡ |
| **Pointer Octree (RVV Leaf Checks)** | **10.11 ms** | **0.701 ms** | **1.63x faster** ⚡ |
| **Standard Octree (`Octree`)** | 10.70 ms | 1.146 ms | 1.00x (Baseline) |
| **Global RVV Scan (`radius_search_rvv`)** | **0.00 ms** | 1.191 ms | 0.96x (Zero build cost) |
| **Caravan Query-Pack (`CaravanRadiusSearch`)** | **0.00 ms** | 1.191 ms (single)<br>**Batch: $O(N \cdot \lceil Q/VL \rceil)$** | **Batch Acceleration** ⚡ |

*Key Insight:* `PointerOctree` achieves **2.16x speedup** over `Standard Octree` because `PointerOctreeNode` stores contiguous coordinate arrays (`leaf_x, leaf_y, leaf_z`) directly inside leaf nodes, eliminating indirect index lookups (`cloud.x[indices[i]]`).

---

### Ablation C: Statistical Outlier Removal (SOR) ($N=18,542$)

| Implementation | Algorithm Complexity | Runtime | Speedup vs Scalar |
| :--- | :--- | :--- | :--- |
| **Scalar Brute-force (`sor_sc`)** | $O(N^2)$ Pairwise Distance | 15,981.20 ms | 1.00x |
| **RVV Brute-force (`sor_rvv`)** | $O(N^2)$ Vectorized Distance | **10,148.80 ms** | **1.57x speedup** ⚡ |

*Key Insight:* Vectorization provides a 1.57x speedup, but the $O(N^2)$ algorithmic complexity dominates. Replacing $O(N^2)$ loop with an Octree/SpatialHash spatial radius query ($O(N \log N)$) will cut SOR runtime from ~10s to <100 ms.

---

### Ablation D: RANSAC Plane Fitting Iteration Scaling

| Iterations ($N_{iter}$) | Scalar Runtime | RVV Runtime | RVV Speedup |
| :--- | :--- | :--- | :--- |
| **100 Iterations** | 88.32 ms | **54.82 ms** | **1.61x** |
| **250 Iterations** | 195.77 ms | **131.41 ms** | **1.49x** |
| **500 Iterations** | 394.73 ms | **249.31 ms** | **1.58x** |
| **1000 Iterations** | 810.47 ms | **504.27 ms** | **1.61x** |

---

## 5. Architectural Recommendations

1. **Replace $O(N^2)$ SOR with Index-Accelerated SOR ($O(N \log N)$)**:
   - Pass pre-built `SpatialHash` or `PointerOctree` to `sor_rvv` instead of computing all pairwise distances. Expected runtime drop: **33,000 ms $\to$ <100 ms**.
2. **Adopt `PointerOctree` or `SpatialHash` as Primary Neighbor Search Engine**:
   - `PointerOctree` with scalar leaf checks reduces query latency by **2.16x** (0.531 ms vs 1.146 ms).
   - `SpatialHash` provides $O(1)$ grid lookup with **0.690 ms** latency.
3. **Adaptive SPRT / Consensus Early Stopping in RANSAC**:
   - Stop RANSAC iteration loop once candidate plane inlier ratio exceeds target confidence ($>60\%$). Expected runtime reduction: **500 ms $\to$ <100 ms**.

---

## 6. Ongoing Experiment Work Log & Future Finalization Checklist

### Completed Tasks:
- [x] Implemented zero-overhead `#ifdef RVPOINT_ENABLE_PROFILING` macro system (`src/include/profiler.h`).
- [x] Added `--json` flag and sub-stage micro-timers to `pipeline_export.cpp`.
- [x] Implemented comparative ablation benchmark tool (`src/tools/ablation_bench.cpp`).
- [x] Automated two-phase profiling runner (`scripts/profile_pipeline.py`).
- [x] Conducted empirical benchmarks comparing `PointerOctree` (RVV & Scalar), `SpatialHash`, `Octree`, and `CaravanRadiusSearch`.

### Next Steps for Finalization:
- [ ] Implement index-accelerated SOR (`sor_octree` / `sor_hash`).
- [ ] Implement adaptive SPRT early stopping for RANSAC plane fitting.
- [ ] Evaluate multi-frame batch performance across `data/pcd_compressed/*.pcd`.
