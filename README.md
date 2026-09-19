# RVPoint: RISC-V Vector Optimized Point Cloud Library

**RVPoint** is a high-performance, zero-dependency 3D Point Cloud Processing Library purpose-built for **RISC-V 64-bit vector architectures (`rv64gcv`)** with hardware RVV 1.0 acceleration.

Drop-in hardware-accelerated replacements for core Point Cloud Library (PCL) algorithms across robotics, LiDAR perception, and autonomous driving.

---

## 📚 Documentation Hierarchy (Single Source of Truth)

| Document | Primary Audience | Scope & Content |
| :--- | :--- | :--- |
| [**Architecture Specification**](docs/ARCHITECTURE.md) | System Architects & Researchers | Mathematical formulations (Cardano analytical eigensolver), SoA vector memory alignment (`vle32.v`), slotted pipeline DAG, and multi-core models. |
| [**Workspace Reference**](docs/WORKSPACE.md) | Developers & Operators | Target lookup matrix, canonical build/test commands, multi-environment setup, and simulation workflows. |
| [**Domain Context & Glossary**](docs/CONTEXT.md) | Developers & AI Agents | Ubiquitous language, domain entities, mathematical invariants, and zero-heap pipeline concepts. |
| [**Codebase Design & RVV Standards**](docs/guides/CODEBASE_DESIGN_AND_RVV_STANDARDS.md) | Contributors & Engineers | Deep module design, zero-vtable invariants (ADR-0011), RVV 1.0 LMUL allocation, and hot-path zero-heap rules. |
| [**Command Reference Guide**](docs/guides/COMMANDS.md) | Operators & Evaluators | Complete CLI syntax for pipelines, benchmarks, dual backends, and multi-core sweeps. |
| [**gem5 Benchmarking Guide**](docs/guides/GEM5_BENCHMARKING_GUIDE.md) | Performance Engineers | Universal, target-agnostic microarchitectural simulation & hardware profiling guide. |
| [**Foxglove Visualization**](docs/guides/FOXGLOVE_MCAP_WALKTHROUGH.md) | Perception Engineers | MCAP export and 3D bounding box / cluster streaming in Foxglove Studio. |

---

## 🚀 Key Hardware-Accelerated Features

1. **Structure-of-Arrays (SoA) & View Semantics**: Contiguous `PointCloud` (owning) and `PointCloudView` (non-owning) enabling sequential `__riscv_vle32_v_f32m8` vector loads.
2. **Slotted Register-File Pipeline Engine**: Zero-copy topological DAG scheduler with tagged positional bindings and dynamic parameter tuning (ADR-0012).
3. **Analytical Cardano Normal Estimation**: Solves $3 \times 3$ covariance characteristic polynomials in closed form ($>4\times$ faster than iterative Jacobi rotations).
4. **SPRT Vector RANSAC**: Sequential Probability Ratio Test with vectorized plane distance evaluation and early hypothesis rejection.
5. **Zero-Heap Functor Primitives**: Stateful, zero-vtable classes (`VoxelGrid`, `RadiusOutlierRemoval`, `StatisticalOutlierRemoval`, `EuclideanClustering`) with stage-owned scratch buffers (ADR-0010, ADR-0011).
6. **Dual-Backend Parity**: Bit-exact and epsilon-exact mathematical alignment between RVV 1.0 vector intrinsics and Scalar C++ implementations.

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

# Run 13 essential fast unit & regression tests (RVV 1.0):
./scripts/test.sh

# Run 12 fast unit tests (Scalar fallback backend):
./scripts/test.sh --backend scalar
```

### 4. Execute Pipelines & Microarchitectural Simulations
```bash
# Slotted register-file perception pipeline prototype (ADR-0012):
./scripts/run.sh pipeline_prototype data/pcd_compressed/0000000090.pcd

# Full 10-stage evaluation pipeline:
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd --no-write --progress

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
│   ├── pipeline/   # RegisterFile, PipelineManager, Tagged Binding, Hooks (ADR-0012)
│   ├── io/         # Zero-dependency simple PCD file reader & writer
│   └── include/    # Public umbrella header (rvpoint.h)
├── eval/           # All executables & pipelines (pipeline_3d_ultimate, benchmarks, tests)
├── env/            # Toolchain activators & CMake toolchains (riscv.cmake)
├── scripts/        # Runners & automation (build.sh, run.sh, test.sh, gem5/run_sim.sh)
├── docs/           # Specifications, architecture, and guides (ARCHITECTURE.md, WORKSPACE.md)
├── data/           # Input PCD point clouds
└── output/         # Unified destination for all generated PCDs, MCAPs, and metrics
```
