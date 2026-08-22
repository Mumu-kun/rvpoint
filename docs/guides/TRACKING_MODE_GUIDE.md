# Tracking Mode (Keyframe Registration) — Architecture & CLI Guide

Comprehensive guide for the RVV-accelerated **Tracking Mode** (Point-to-Plane ICP Registration & Geometric Propagation Pipeline) in RVPoint.

---

## 1. Executive Summary & Pipeline Concept

In sequential LiDAR streams, computing the full perception pipeline (Voxel Grid $\rightarrow$ Statistical Outlier Removal $\rightarrow$ Normal Estimation $\rightarrow$ RANSAC Plane Segmentation $\rightarrow$ Euclidean Clustering) independently on every single frame is computationally redundant. 

**Tracking Mode** leverages temporal continuity:
1. **Keyframe Mode (Frame $0, K, 2K, \dots$)**: Executes the full perception pipeline to discover major geometric primitives (planes, object clusters, and surface normals).
2. **Tracking Mode (Intermediate Frames)**: Estimates inter-frame sensor motion via RVV Point-to-Plane ICP, propagates known geometric models forward in time, verifies them against incoming points, and passes only unmatched *residual* points to the full perception pipeline.

```text
Keyframe Mode (Full Pipeline)            Tracking Mode (9 Stages)
┌─────────────────────────────────┐      ┌─────────────────────────────────┐
│ [1] Voxel Grid Downsampling     │      │ [1] Voxel Downsampling          │
│ [2] Statistical Outlier Removal │      │ [2] Correspondence Search (RVV) │
│ [3] Normal Estimation (Cardano) │───┐  │ [3] Residual + Jacobian (RVV)   │
│ [4] RANSAC Plane Segmentation   │   │  │ [4] Tree-Sum Reduction 6×6      │
│ [5] Euclidean Clustering        │   │  │ [5] Solve 6×6 (Scalar Cholesky) │
└─────────────────────────────────┘   │  │ [6] Propagate Models (RVV)      │
                                      └─►│ [7] Verify Models (RVV)         │
                                         │ [8] Split Confirmed/Residual    │
                                         │ [9] Update Spatial Index        │
                                         │                                 │
                                         │ Residual ──► Small RANSAC/EC    │
                                         └─────────────────────────────────┘
```

---

## 2. The 9-Stage Pipeline & RVV Vectorization Strategy

| Stage | Name | Algorithm / Kernel | Parallelism | RVV Configuration |
| :--- | :--- | :--- | :--- | :--- |
| **Stage 1** | Voxel Downsampling | `voxel_grid_downsamp_rvv_v2()` | 🟢 Data-Parallel | LMUL = `m4` (`vfmin`, `vfmax`, `vfmul`, `vfloor`) |
| **Stage 2** | Correspondence Search | `ICPRegistration::findCorrespondences` | 🟢 Parallel per Query | LMUL = `m8` / `m2` (`vluxei32`, `vfsub_vf`, `vfmacc_vv`) |
| **Stage 3** | Residual + Jacobian | `residual_jacobian_rvv()` | 🟢 Parallel per Pair | LMUL = `m8` (`vfsub_vv`, `vfmul_vv`, `vfmacc_vv`, `vfnmsac_vv`) |
| **Stage 4** | Reduction to $6 \times 6$ | Fused with Stage 3 | 🟡 Tree Reduction | LMUL = `m8` (`vfredusum_vs_f32m8_f32m1`) |
| **Stage 5** | Solve for Motion | `ICPRegistration::solve6x6` | 🔴 Scalar (Optimal) | Plain C++ Cholesky ($6 \times 6$ system is too small to vectorize) |
| **Stage 6** | Propagate Models | `propagate_transform_rvv()` | 🟢 Data-Parallel | LMUL = `m8` (`vfmv_v_f`, `vfmacc_vf` batched matrix multiply) |
| **Stage 7** | Verify Models | `verify_plane_rvv`, `verify_cluster_rvv` | 🟢 Data-Parallel | LMUL = `m8` (`vmfle_vf`, `vmfge_vf`, `vmand_mm`, `vcpop_m`) |
| **Stage 8** | Split Points | `compact_points_rvv()` | 🟡 Parallel + Compact | LMUL = `m8` (`vcompress_vm_f32m8`, `vsse32_v_f32m8`) |
| **Stage 9** | Update Spatial Index | `SpatialHash::insertPoints` | 🟡 Hybrid | Vectorized hash keying + scalar hash map insertion |

---

## 3. Mathematical Formulation

### A. Point-to-Plane ICP Residual & Jacobian (Stages 2–4)
For each correspondence pair $(\mathbf{p}_{\text{src}, i}, \mathbf{p}_{\text{tgt}, i})$ with target normal $\mathbf{n}_i$:
$$\mathbf{p}'_i = \mathbf{R}\mathbf{p}_{\text{src}, i} + \mathbf{t}$$
$$r_i = \mathbf{n}_i^\top (\mathbf{p}'_i - \mathbf{p}_{\text{tgt}, i})$$
$$\mathbf{J}_i = \begin{bmatrix} (\mathbf{p}'_i \times \mathbf{n}_i)^\top & \mathbf{n}_i^\top \end{bmatrix} \in \mathbb{R}^{1 \times 6}$$

The normal equations are accumulated via vector tree reduction:
$$\mathbf{J}^\top\mathbf{J} = \sum_{i=1}^{N} \mathbf{J}_i^\top \mathbf{J}_i \in \mathbb{R}^{6 \times 6}, \quad \mathbf{J}^\top\mathbf{r} = \sum_{i=1}^{N} \mathbf{J}_i^\top r_i \in \mathbb{R}^{6 \times 1}$$

### B. Transformation Update & SE(3) Propagation (Stages 5–6)
The linear system is solved via Cholesky decomposition:
$$\mathbf{J}^\top\mathbf{J} \, \Delta\mathbf{x} = -\mathbf{J}^\top\mathbf{r}, \quad \Delta\mathbf{x} = [\alpha, \beta, \gamma, t_x, t_y, t_z]^\top$$

The incremental update is mapped to $SE(3)$ via exponential map / Rodrigues formula and composed with the previous estimate $\mathbf{T}_{k+1} = \exp(\Delta\mathbf{x}) \circ \mathbf{T}_k$.

Stored geometric models are propagated forward:
- **Planes**: $\mathbf{n}' = \mathbf{R}\mathbf{n}, \quad d' = d - \mathbf{n}'^\top\mathbf{t}$
- **Clusters**: $\mathbf{c}' = \mathbf{R}\mathbf{c} + \mathbf{t}$
- **Normals**: $\mathbf{n}' = \mathbf{R}\mathbf{n}$

---

## 4. CLI Commands & Execution Guide

All commands run under the standard RVPoint runners.

### A. Run Fast Unit Tests
```bash
# Run full unit test suite (including tracking registration & pipeline tests):
./scripts/test.sh

# Run individual tracking unit tests directly under QEMU:
./scripts/run.sh test_tracking_registration
./scripts/run.sh test_tracking_pipeline
```

### B. Pure In-Memory Compute Benchmark (Zero Disk I/O)
Measures pure algorithmic compute speedup across multiple frames without disk I/O distortion:
```bash
# Synthetic scene benchmark (10 frames, 5000 points):
./scripts/run.sh tracking_mode_bench --synthetic output 10 5000

# Benchmark on real recorded PCD dataset:
./scripts/run.sh tracking_mode_bench data/pcd_compressed/0000000080.pcd output 10 5
```

### C. Multi-Frame PCD Comparison Pipeline
Processes sequential PCD frames, compares Mode A (Full Pipeline) vs. Mode B (Tracking Mode), exports processed PCD clouds, and outputs comparison metrics JSON:
```bash
# Process 20 frames with keyframe interval of 5:
./scripts/run.sh tracking_pcd_compare data/pcd_compressed output/pcd_compare 20 5
```

---

## 5. Zero-Disk-I/O Timing Methodology

In autonomous driving perception, file I/O operations (parsing ASCII/compressed PCD headers and disk writes) take orders of magnitude longer than vector kernel computation. 

To ensure research-grade benchmark accuracy:
1. **Pre-Loading Phase**: All input frames are loaded and decompressed into contiguous memory arrays *before* timers start.
2. **Compute Phase**: Timers strictly encapsulate algorithm kernel calls (`TrackingPipeline::runFullPipeline` or `TrackingPipeline::processTrackingFrame`).
3. **Export Phase**: Disk serialization (`savePCD`) and JSON generation (`saveJSON`) occur *after* all compute timers stop.

---

## 6. Antigravity IDE & WSL2 Environment Setup

To build, test, and run RVPoint in Antigravity IDE on Windows:

### Step 1: Open Antigravity IDE Terminal
Open the built-in terminal in Antigravity IDE (`Ctrl + \`` or from the bottom panel).

### Step 2: Access the Ubuntu WSL2 Environment
Execute commands targeting the WSL2 distribution containing `/opt/riscv`:
```bash
wsl -d Ubuntu
```

### Step 3: Activate Environment & Build
Inside the WSL terminal:
```bash
cd /mnt/d/CAPSTONE_FINAL/rvpoint
source env/activate.sh
./scripts/build.sh
./scripts/test.sh
```

### Step 4: One-Liner Commands from Windows Shell / IDE Tasks
You can also run commands directly from Windows PowerShell or IDE tasks without entering WSL interactively:
```powershell
wsl -d Ubuntu bash -c "cd /mnt/d/CAPSTONE_FINAL/rvpoint && source env/activate.sh && ./scripts/test.sh"
wsl -d Ubuntu bash -c "cd /mnt/d/CAPSTONE_FINAL/rvpoint && source env/activate.sh && ./scripts/run.sh tracking_mode_bench --synthetic output 10 5000"
```
