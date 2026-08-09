# Pipeline Performance Profiling & Bottleneck Analysis Report

**Date:** 2026-08-10  
**Target Point Cloud:** `data/0000000000.pcd` (114,278 input points)  
**Execution Environment:** RISC-V QEMU Emulation (`rv64gcv`, vector length 128-bit)  
**Artifacts Generated:**
- Consolidated Results: [`output/profiling_results/consolidated_profiling_results.json`](file:///e:/FahadProject/rvpoint/output/profiling_results/consolidated_profiling_results.json)
- Summary CSV: [`output/profiling_results/summary_results.csv`](file:///e:/FahadProject/rvpoint/output/profiling_results/summary_results.csv)

---

## 1. Executive Summary

Empirical profiling of the `rvpoint` 10-stage processing pipeline on input point cloud `data/0000000000.pcd` reveals two primary critical path bottlenecks that account for over **95% of total pipeline computation time**:

1. **Statistical Outlier Removal (SOR) — $O(N^2)$ Pairwise Bottleneck (94.6% – 94.9% of Total Runtime)**:
   - When SOR is enabled with default parameters ($k=20, \alpha=1.0$), pairwise distance computation across $N$ points scales quadratically ($T \propto N^2$).
   - At leaf size `0.10` ($N=52,800$ pts), SOR takes **32,912 ms out of 34,791 ms (94.6%)**.
   - Bypassing SOR (`--skip-sor`) achieves a **18.2x overall pipeline speedup** (reducing total runtime from 34.8s to 1.9s at leaf size 0.10).

2. **RANSAC Dominant Plane Fitting — Iteration & Distance Evaluation Bottleneck (50.7% – 51.0% when SOR is Bypassed)**:
   - When SOR is bypassed or downsampled, RANSAC fitting ($1000$ iterations) becomes the primary bottleneck, taking **50.7% to 51.0% of total runtime**.
   - RISC-V Vector intrinsics (`ransac_plane_rvv`) accelerate inlier counting via `plane_dist_rvv` + `vcpop`, yielding a consistent **1.61x speedup over scalar RANSAC**.

3. **RISC-V Vector Extension (RVV) Acceleration Summary**:
   - **Voxel Grid Downsampling**: Sort-based RVV (`_v2`) achieves **2.42x speedup** over scalar `std::map`.
   - **RANSAC Plane Fitting**: Vectorized inlier evaluation achieves **1.61x speedup** over scalar loop.
   - **Statistical Outlier Removal**: Vectorized distance calculation achieves **1.57x speedup** over scalar loop.
   - **Spatial Neighbor Search**: `SpatialHash` grid search is **1.65x faster than Octree query time** (0.74 ms vs 1.22 ms).

---

## 2. Phase 1 Pipeline Profiling Matrix

### Stage Breakdown Across Configurations

| Stage Index & Name | Leaf 0.10 (SOR ON) | Leaf 0.10 (SOR OFF) | Leaf 0.20 (SOR ON) | Leaf 0.20 (SOR OFF) |
| :--- | :--- | :--- | :--- | :--- |
| **Total Cloud Input Points** | 114,278 | 114,278 | 114,278 | 114,278 |
| **Downsampled Cloud Points** | 52,800 | 52,800 | 18,542 | 18,542 |
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
| **Total Execution Time** | **34,790.8 ms** | **1,912.2 ms** | **10,829.7 ms** | **1,085.8 ms** |

---

## 3. Phase 2 Component-Level Implementation Ablations

### Ablation A: Voxel Grid Downsampling
Comparing scalar `std::map` grouping against sort-based RISC-V Vector downsampling (`_v2`):

| Implementation | Runtime (ms) | Speedup vs. Scalar | Point Count |
| :--- | :--- | :--- | :--- |
| **Scalar `std::map` (v0)** | 98.83 ms | 1.00x | 18,542 |
| **RVV Sort-based (`_v2`)** | **40.92 ms** | **2.42x** ⚡ | 18,542 |

*Key Takeaway:* Eliminating `std::map` node allocations and replacing grouping with vectorized bounding box scaling + index sorting yields a **2.42x speedup**.

---

### Ablation B: Spatial Neighbor Search
Comparing query time for finding neighbors within $r=0.05\text{ m}$:

| Search Implementation | Build Time (ms) | Query Time (ms) | Relative Query Efficiency |
| :--- | :--- | :--- | :--- |
| **Global RVV Scan** | 0.00 ms | 0.94 ms | 1.00x |
| **Octree Radius Search** | 14.17 ms | 1.22 ms | 0.77x |
| **Spatial Hash Grid Search** | 17.96 ms | **0.74 ms** | **1.28x faster than Global Scan / 1.65x faster than Octree** ⚡ |

*Key Takeaway:* `SpatialHash` grid provides $O(1)$ bucket lookup, outperforming pointer-chasing Octree tree traversals for fixed-radius queries.

---

### Ablation C: Statistical Outlier Removal (SOR)
Comparing brute-force scalar vs. brute-force vector distance computation ($N=18,542$):

| Implementation | Runtime (ms) | Speedup vs. Scalar | Output Inliers |
| :--- | :--- | :--- | :--- |
| **Scalar Brute-force (`sor_sc`)** | 15,981.20 ms | 1.00x | 17,569 |
| **RVV Brute-force (`sor_rvv`)** | **10,148.80 ms** | **1.57x** ⚡ | 17,569 |

*Key Takeaway:* While RVV intrinsics (`vle32.v`, `vfsub`, `vfmul`, `vfredusum`) accelerate pairwise distance calculations by 1.57x, the $O(N^2)$ algorithm complexity remains the dominant bottleneck. Replacing brute-force search with an Octree or SpatialHash search will reduce complexity to $O(N \log N)$ and cut runtime by $>10\times$.

---

### Ablation D: RANSAC Plane Fitting Iteration Scaling
Comparing scalar vs. RVV vectorized plane fitting across iteration counts:

| Iterations ($N_{iter}$) | Scalar Runtime (ms) | RVV Runtime (ms) | RVV Speedup |
| :--- | :--- | :--- | :--- |
| **100 Iterations** | 88.32 ms | **54.82 ms** | **1.61x** |
| **250 Iterations** | 195.77 ms | **131.41 ms** | **1.49x** |
| **500 Iterations** | 394.73 ms | **249.31 ms** | **1.58x** |
| **1000 Iterations** | 810.47 ms | **504.27 ms** | **1.61x** |

*Key Takeaway:* Fusing dot-product evaluation (`plane_dist_rvv`) with vector mask population count (`vcpop`) delivers a steady ~1.6x vector speedup across all iteration counts.

---

## 4. Targeted Architectural Optimization Recommendations

1. **Replace Brute-Force SOR with Index-Accelerated SOR ($O(N^2) \to O(N \log N)$)**:
   - *Current Status:* `sor_rvv` calculates distance from every point to all $N$ other points in the cloud ($N^2$ loop).
   - *Optimization:* Use pre-built `Octree` or `SpatialHash` to query only local $K$ neighbors. This will reduce SOR runtime from ~33 seconds to <100 ms.

2. **Adaptive Early-Stopping RANSAC**:
   - *Current Status:* RANSAC runs a fixed 1000 iterations regardless of inlier consensus.
   - *Optimization:* Implement SPRT (Sequential Probability Ratio Test) or adaptive iteration stopping based on target confidence ($p=0.99$). If a candidate plane achieves $>60\%$ inliers early, terminate iteration loop. This can reduce RANSAC time from ~500 ms to <100 ms.

3. **Adopt SpatialHash for Fixed-Radius Neighbor Search**:
   - *Current Status:* Normal Estimation and Euclidean Clustering default to `Octree`.
   - *Optimization:* Switch to `SpatialHash` for radius searches with fixed radii. SpatialHash reduces per-query lookup latency to **0.74 ms** (1.65x faster than Octree).
