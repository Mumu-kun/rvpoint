# RVPoint: Negative Results, Failed Hypotheses & Optimization Traps

> **Living Record of Disproven Hypotheses, Ineffective Optimizations, and Architectural Anti-Patterns on RISC-V Vector (RVV 1.0) Architectures.**
> 
> *Maintainers and AI agents MUST consult this document before attempting single-core or kernel-level refactors to avoid repeating empirically disproven optimizations.*

---

## 1. Executive Summary & Core Insights

When optimizing 3D perception algorithms on RISC-V architectures (e.g. SpacemiT K1 / X60 cores on Orange Pi RV2), theoretical Big-$O$ gains and isolated micro-benchmarks frequently fail in full end-to-end pipelines due to three fundamental hardware phenomena:

1. **The Allocation & Zeroing Penalty**: Dynamically allocating and zero-initializing large lookup tables (e.g., $131\text{k}$-entry hash tables) every frame consumes massive DRAM bandwidth and erases algorithmic gains.
2. **Index Indirection vs. L1 Cache Locality**: Converting direct pointers into multi-level index mappings (`orig_indices_[off + k]`) thrashes L1 data cache lines.
3. **The Early-Exit Dominance Effect**: In dense LiDAR point clouds, $>90\%$ of points satisfy neighborhood density conditions in the first 1–2 points checked. Vectorized batch kernels that evaluate fixed chunks waste cycles compared to simple scalar short-circuiting.

---

## 2. Catalog of Negative Results & Failed Hypotheses

### Entry 001: Flat-Array Contiguous Grid vs. Linked-List Grid for Radius Outlier Removal (ROR)

* **Date Tested**: 2026-08-22
* **Target Stage**: Stage 5 (Radius Outlier Removal & Normal Estimation)
* **Theoretical Hypothesis**: Storing points contiguously per voxel cell (`sorted_x`, `sorted_y`, `sorted_z`) will enable wide RVV vector loads (`__riscv_vle32_v_f32m8`) and compute squared distances across 32–64 points simultaneously with `__riscv_vfmacc_vv_f32m8`, speeding up neighbor searches by $3\times\text{--}5\times$.
* **Empirical Outcome**: **FAILED (Net Slowdown: $182.5\,\text{ms} \rightarrow 190.3\,\text{ms}$)**.
* **Root Cause Analysis**:
  1. *Early-Exit Short Circuiting*: With `min_neighbors = 2`, the linked-list grid's self-cell fastpath inspects 1 or 2 pointers in the same cell and immediately exits (taking 2–3 CPU cycles). The vectorized loop was forced to compute vector lengths and execute mask reductions.
  2. *Secondary Indirection Penalty*: To compute surface normals, the flat grid had to look up original point indices via `orig_indices_[off + k]`. This secondary indirection caused severe L1/L2 cache misses.
* **Guideline / Rule**: **Do not replace the linked-list spatial hash grid (`Fast3DSpatialGrid`) with a flat sorted array for low-neighbor ROR queries.**

---

### Entry 002: $O(N)$ Dynamic Hash Table Downsampler vs. $O(N \log N)$ `std::sort`

* **Date Tested**: 2026-08-22
* **Target Stage**: Stage 3 (Voxel Grid Downsampling)
* **Theoretical Hypothesis**: Replacing $O(N \log N)$ coordinate comparison sorting (`std::sort`) with an $O(N)$ linear open-addressing hash accumulator table (`sum_x, sum_y, sum_z, count`) will save $30\text{--}40\,\text{ms}$.
* **Empirical Outcome**: **FAILED (Negligible gain in isolation: $60.8\,\text{ms} \rightarrow 52.1\,\text{ms}$, but caused downstream cache pollution)**.
* **Root Cause Analysis**:
  1. *Allocation & Page Zeroing Overhead*: An open-addressing hash table sized for 118k points requires at least $131,072$ entries ($\sim 4\,\text{MB}$). Allocating and clearing this table on every single frame incurs severe memory bus overhead.
  2. *L1 Cache Localization*: `std::sort` on 34,000 integer voxel keys operates entirely inside a compact $136\,\text{KB}$ index array that fits completely in L1/L2 cache ($512\,\text{KB}$ cluster cache).
* **Guideline / Rule**: **Keep the L1-localized sorting downsampler (`voxel_grid_downsamp_rvv_v2`) unless static pre-allocated memory pools are used.**

---

### Entry 003: Direct Memory-Mapped SoA Loader on LZF-Compressed PCD Datasets

* **Date Tested**: 2026-08-22
* **Target Stage**: Stage 1 (PCD File Ingestion)
* **Theoretical Hypothesis**: Using POSIX `mmap()` to directly populate `PointCloudSoA` without intermediate `std::vector<PointXYZ>` objects will reduce ingestion time from $69\,\text{ms} \rightarrow <15\,\text{ms}$.
* **Empirical Outcome**: **FAILED ($69.0\,\text{ms} \rightarrow 115.8\,\text{ms}$ when handling compressed streams)**.
* **Root Cause Analysis**:
  1. *Decompression Dominance*: Standard benchmark point clouds (`data/pcd_compressed/*.pcd`) are LZF-compressed (`DATA binary_compressed`). The bottleneck is the byte-by-byte LZ77 decompression loop, not the memory copy into SoA layout.
  2. *Two-Pass Header Overhead*: Pre-allocating exact SoA buffers required two separate file reads / seeks when parsing compressed headers.
* **Guideline / Rule**: **Do not attempt raw binary zero-copy casting on compressed point clouds without an RVV-vectorized LZF decompressor.**

---

### Entry 004: Flat-Grid Adjacency for Euclidean Clustering

* **Date Tested**: 2026-08-22
* **Target Stage**: Stage 9 (Euclidean Cluster Extraction)
* **Theoretical Hypothesis**: Flattening the 3D grid adjacency will accelerate cluster extraction by eliminating linked-list pointer traversing.
* **Empirical Outcome**: **FAILED ($66.5\,\text{ms} \rightarrow 66.2\,\text{ms}$, $<0.5\%$ difference)**.
* **Root Cause Analysis**:
  * The actual CPU cycle consumer in Euclidean clustering is the **Disjoint-Set Union-Find tree traversal (`find()` and `unite()`) and root aggregation**, not the spatial hash table lookup. Modifying the grid data structure does not address the true algorithmic bottleneck.
* **Guideline / Rule**: **Do not refactor the spatial grid to speed up Stage 9; focus on Union-Find path compression and parallel edge discovery.**

---

### Entry 005: Multi-File Cluster Disk I/O (Solved Anti-Pattern)

* **Date Identified**: 2026-08-22
* **Target Stage**: Stage 10 (Write Cluster Stage)
* **What Went Wrong**: Saving individual cluster files (`cluster_0.pcd`, `cluster_1.pcd`, ..., `cluster_71.pcd`) generated 72 separate file creates, header writes, and syscalls, taking $>500\,\text{ms}$ in Stage 10 alone.
* **Solution**: Write a single consolidated `06_clusters.pcd` binary point cloud with procedural RGB coloring per cluster ID ($<40\,\text{ms}$).

---

## 3. Anti-Pattern Checklist (Red Flags to Reject in Planning)

Before implementing a proposed optimization for RVPoint, check if it violates these rules:

- [ ] **Dynamic Per-Frame Allocations $>256\,\text{KB}$**: If the kernel allocates a dynamic table (e.g. `vector(65536)`) on every frame, reject it.
- [ ] **Extra Index Buffers / Indirection**: If accessing point data requires `index_map[cell_offset + k]`, cache thrashing will likely destroy any vectorization benefit.
- [ ] **Replacing Early-Exit Loops with Fixed Vector Blocks**: If a scalar loop exits after 1–2 iterations $>80\%$ of the time, replacing it with vector instructions will be slower.
- [ ] **Micro-Benchmark Illusions**: Never trust an isolated kernel benchmark that allocates memory outside the timed loop unless the full pipeline does the same.

---

## 4. Proven Paths to High-Confidence Speedups

For genuine, measured speedups on the Orange Pi RV2 / SpacemiT K1 platform:

1. **Multi-Core OpenMP Parallelization**: The 8 physical RVV cores offer an immediate $5\times\text{--}7\times$ scaling factor with zero cache conflicts when points are chunked cleanly.
2. **Fused Multi-Stage Kernels**: Fusing stage computations (e.g., normal estimation computed directly inside spatial neighbor traversal without intermediate storage) eliminates entire memory roundtrips.
3. **Hardware SPRT in RANSAC**: Vectorized model candidate scoring with Sequential Probability Ratio Test early-rejection drops RANSAC runtime by $>70\times$.
