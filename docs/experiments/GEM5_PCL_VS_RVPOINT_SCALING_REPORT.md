# gem5 Microarchitectural Scaling Evaluation: Upstream PCL vs. RVPoint Ultra Pipeline

> **Comparative Microarchitectural Cycle & Cache Profiling on RISC-V 64-bit (`rv64gcv`) SpacemiT K1 Core Model.**

---

## 1. Executive Summary & Experimental Methodology

This benchmark experiment provides a reproducible, cycle-accurate microarchitectural evaluation comparing the upstream open-source **Point Cloud Library (PCL 1.14.0)** against the **RVPoint 3D Ultra Vector Pipeline** across varying point cloud densities.

### Simulated Microarchitecture (SpacemiT K1 Core Preset)
- **CPU Model**: `MinorCPU` (in-order, dual-issue superscalar pipeline).
- **Architecture**: RISC-V 64-bit (`rv64gcv`).
- **Vector Unit Configuration**: `VLEN = 256` bits, `ELEN = 64` bits.
- **Cache Subsystem**: L1 Data Cache (32 KB, 2-way associative), L1 Instruction Cache (32 KB), 512 MB physical address space.

---

## 2. Single Source of Truth Base Pitch (Δ) Parameter Derivation

To guarantee mathematical equivalence and prevent parameter drift across point cloud densities, all pipeline stage thresholds are automatically derived from the root spatial step size $\Delta$:

```text
                       ┌───────────────────────────────────────────────┐
                       │   SINGLE SOURCE OF TRUTH: Base Pitch (Δ)      │
                       │   (e.g., Δ = 0.02m for Standard Tabletop)     │
                       └───────────────────────┬───────────────────────┘
                                               │
         ┌─────────────────────────────────────┼─────────────────────────────────────┐
         │ (Scale Factor: 1.0)                 │ (Scale Factor: 2.5)                 │ (Scale Factor: 0.5)
         ▼                                     ▼                                     ▼
┌─────────────────────────────┐       ┌─────────────────────────────┐       ┌─────────────────────────────┐
│    1. Voxel Downsampling    │       │ 2. Neighborhood & Normals   │       │  3. RANSAC & Segmentation   │
├─────────────────────────────┤       ├─────────────────────────────┤       ├─────────────────────────────┤
│ • Grid Cell Pitch: Δ        │       │ • Search Radius: R = 2.5Δ   │       │ • Plane Inlier Tol: ε = 0.5Δ│
│ • Uniform density baseline  │       │ • Guaranteed ≥15 neighbors  │       │ • Sub-voxel plane accuracy  │
│ • Replaces cell w/ centroid │       │ • Avoids empty search balls │       │ • Prevents noise inclusion  │
└─────────────────────────────┘       └─────────────────────────────┘       └─────────────────────────────┘
```

| Pipeline Stage | Scale Multiplier | Tabletop Preset ($\Delta = 0.02\text{ m}$) | Algorithmic Rationale |
| :--- | :---: | :---: | :--- |
| **Voxel Downsampling** | $1.0 \times \Delta$ | $0.02\text{ m}$ ($2.0\text{ cm}$) | Establishes uniform lattice density. |
| **Spatial Radius Search (SOR/Normals)** | $2.5 \times \Delta$ | $0.05\text{ m}$ ($5.0\text{ cm}$) | Encompasses $\ge 15\text{--}30$ adjacent voxels for stable covariance estimation. |
| **RANSAC Plane Inlier Tolerance** | $0.5 \times \Delta$ | $0.01\text{ m}$ ($1.0\text{ cm}$) | Restricts consensus strictly to true planar surfaces. |
| **Euclidean Cluster Proximity** | $1.25 \times \Delta$ | $0.025\text{ m}$ ($2.5\text{ cm}$) | Connects intra-object voxels while preserving inter-object gaps. |

---

## 3. How to Run the Experiment

### Step 1: Bootstrap Upstream PCL Cross-Compilation
```bash
# Inside WSL2 'rvpoint' environment:
source env/activate.sh
./scripts/setup_pcl_riscv.sh
```

### Step 2: Build Pipeline Targets
```bash
./scripts/build.sh --target pcl_native_pipeline
./scripts/build.sh --target pipeline_3d_ultra
```

### Step 3: Launch Multi-Point Scaling Sweep
```bash
# Default sweep (1,000, 5,000, 10,000 points):
python3 scripts/bench/sweep_gem5_pcl_vs_rvpoint.py --pcd data/01_table_scene_lms400.pcd --delta 0.02

# Full sweep with opt-in large point clouds (up to 50,000 points):
python3 scripts/bench/sweep_gem5_pcl_vs_rvpoint.py --pcd data/01_table_scene_lms400.pcd --delta 0.02 --include-large
```

---

## 4. Output Artifacts & Metrics Format

Each sweep run saves complete cycle-accurate results under `results/gem5/pcl_vs_rvpoint_sweep_<timestamp>/`:
- `summary.json`: Raw structured microarchitectural metrics for every target and point count.
- `SCALING_REPORT.md`: Formatted Markdown comparison table.
- Individual `stats.txt` and `sim.log` directories for full gem5 trace inspection.
