# RVPoint: RISC-V Vector Optimized Point Cloud Library

**RVPoint** is a high-performance, zero-dependency 3D Point Cloud Processing Library purpose-built for **RISC-V 64-bit vector architectures (`rv64gcv`)** with hardware RVV 1.0 acceleration.

Drop-in hardware-accelerated replacements for core Point Cloud Library (PCL) algorithms across robotics, LiDAR perception, and autonomous driving.

---

## 📚 Documentation Hierarchy (Single Source of Truth)

| Document | Primary Audience | Scope & Content |
| :--- | :--- | :--- |
| [**Architecture Specification**](docs/ARCHITECTURE.md) | System Architects & Researchers | Mathematical formulations (Cardano analytical eigensolver), SoA vector memory alignment (`vle32.v`), and isolated build caching. |
| [**Workspace Reference**](docs/WORKSPACE.md) | Developers & Operators | Target lookup matrix, canonical build/test commands, multi-environment setup, and simulation workflows. |
| [**Instruction Manual**](docs/guides/INSTRUCTION_MANUAL.md) | Engineers & Students | Exhaustive pedagogical handbook, RVV vector assembly theory, step-by-step pipeline walkthroughs, and team SOPs. |
| [**gem5 Benchmarking Guide**](docs/guides/GEM5_BENCHMARKING_GUIDE.md) | Performance Engineers | Universal, target-agnostic microarchitectural simulation & hardware profiling guide. |
| [**Foxglove Visualization**](docs/guides/FOXGLOVE_MCAP_WALKTHROUGH.md) | Perception Engineers | MCAP export and 3D bounding box / cluster streaming in Foxglove Studio. |

---

## 🚀 Key Hardware-Accelerated Features

1. **Structure-of-Arrays (SoA) Layout**: Direct contiguous `x`, `y`, `z` streaming via unit-stride `__riscv_vle32_v_f32m8` vector loads.
2. **Analytical Cardano Normal Estimation**: Solves $3 \times 3$ covariance characteristic polynomials in closed form ($>4\times$ faster than iterative Jacobi rotations).
3. **SPRT Vector RANSAC**: Sequential Probability Ratio Test with vectorized plane distance evaluation and early hypothesis rejection.
4. **O(1) Spatial Hash & Pointer Octree**: Tight AABB sphere-box culling and unified radius neighbor searches.
5. **Linear-Time Euclidean Clustering**: Disjoint-Set / Union-Find clustering with vector candidate gathering.

---

## 🛠️ Quickstart

### 1. Environment Setup

#### Option A: Docker Dev Container (Recommended)
Open the workspace in VS Code and select **"Reopen in Container"** (provisions GCC 14.2+, QEMU 9.x, and RVV 1.0 toolchain in `/opt/riscv`).

#### Option B: Native WSL2 (`rvpoint` distro)
```bash
# Sourcing the environment adds cross-compilers and QEMU to PATH:
source env/activate.sh
```

### 2. Download Official Benchmark Datasets
```bash
./scripts/get_data.sh
```

### 3. Build & Run Tests
```bash
# Build all library targets, pipelines, and benchmarks:
./scripts/build.sh

# Run 12 essential fast unit & regression tests:
./scripts/test.sh
```

### 4. Execute Pipelines & Microarchitectural Simulations
```bash
# QEMU functional emulation with adaptive decimation:
./scripts/run.sh --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

# Cycle-accurate gem5 simulation (SpacemiT K1 / MinorCPU model):
./scripts/gem5/run_sim.sh --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write
```

---

## 📂 Repository Layout

```text
rvpoint/
├── src/            # Pure C++17 library (librvpoint.a) - NO main()
│   ├── core/       # Point types, SoA structures, RVV primitives, profiler
│   ├── features/   # Surface normal estimation (Cardano closed-form)
│   ├── filters/    # Voxel downsampling & Statistical Outlier Removal
│   ├── search/     # Octree, SpatialHash, PointerOctree, Caravan search
│   ├── segmentation/ # RANSAC plane fitting & Euclidean clustering
│   ├── io/         # Zero-dependency simple PCD file reader & writer
│   └── include/    # Public umbrella header (rvpoint.h)
├── eval/           # All executables & pipelines (pipeline_3d_ultimate, benchmarks, tests)
├── env/            # Toolchain activators & CMake toolchains (riscv.cmake)
├── scripts/        # Runners & automation (build.sh, run.sh, test.sh, gem5/run_sim.sh)
├── docs/           # Specifications, architecture, and guides (ARCHITECTURE.md, WORKSPACE.md)
├── data/           # Input PCD point clouds
└── output/         # Unified destination for all generated PCDs, MCAPs, and metrics
```