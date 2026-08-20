# RVPoint Workspace Specification

> **Definitive Workspace & System Architecture Guide for Human Developers and AI Coding Agents.**  
> **Repository**: `rvpoint` (formerly `rvv_pcl`)  
> **Primary Namespace**: `rvpoint` (alias: `rvv_pcl`)  
> **Primary Umbrella Header**: `#include "include/rvpoint.h"`  
> **Primary CMake Target**: `rvpoint` (alias: `rvvpcl`, `rvv_pcl`)  
> **Architecture Reference**: [ARCHITECTURE.md](ARCHITECTURE.md)  

---

## 1. Quick Context Bootstrap (For AI Agents & Developers)

| Key Property | Canonical Value | Notes |
| :--- | :--- | :--- |
| **C++ Standard** | `C++17` | Minimum requirement |
| **Target Hardware** | RISC-V 64-bit with RVV 1.0 (`rv64gcv`) | Tested on QEMU 9.x & SpacemiT K1 hardware |
| **Toolchain Requirement** | RISC-V GCC 14.2+ (`riscv64-unknown-linux-gnu-gcc`) | Provided in `/opt/riscv` |
| **Build System** | CMake 3.16+ | Automatic recursive target & source discovery |
| **Primary Data Type** | `rvpoint::PointCloudSoA` | Contiguous Structure-of-Arrays for vectorization |
| **AoS Data Type** | `rvpoint::PointXYZ` | Point type for standard interfaces |
| **Default Output Dir** | `output/` | All PCDs, MCAPs, and benchmark logs write here |

---

## 2. Directory Layout & Roles

```text
rvpoint/
├── CMakeLists.txt              # Unified automated CMake build system
├── README.md                   # Human project overview & quickstart
├── AGENTS.md                   # Specific AI agent rules and execution policy
│
├── src/                        # [1] 100% PURE LIBRARY (librvpoint.a) - NO main()
│   ├── core/                   # Point types, SoA structures, RVV common helpers, profiler
│   ├── features/               # Surface normal estimation (Cardano analytical closed-form)
│   ├── filters/                # Voxel grid downsampling & Statistical Outlier Removal
│   ├── search/                 # Octree, SpatialHash, PointerOctree, Caravan search
│   ├── segmentation/           # RANSAC plane fitting & Euclidean clustering
│   ├── io/                     # Zero-dependency simple PCD file reader & writer
│   └── include/                # Public umbrella header (rvpoint.h) & backward aliases
│
├── eval/                       # [2] ALL EXECUTABLES & EVALUATION SUITE
│   ├── pipelines/              # All 9 standalone perception pipelines (flat)
│   ├── benchmarks/             # Standalone benchmark utilities & profiling drivers (flat)
│   ├── tests/                  # Two-Tier Test Suite
│   │   ├── fast/               # 12 Essential Fast Tests (run by default)
│   │   └── experimental/       # Deep research audits & algorithmic sweeps (run on demand)
│   └── notebooks/              # Research & Kaggle/gem5 notebooks
│
├── env/                        # [3] ENVIRONMENT & TOOLCHAINS
│   ├── cmake/                  # CMake toolchain files (riscv.cmake, riscv_linux.cmake)
│   ├── linux/                  # Linux / container installer scripts
│   ├── wsl.sh                  # WSL quick-launcher
│   ├── setup.sh                # Main setup entrypoint
│   └── activate.sh             # Environment activator
│
├── scripts/                    # [4] AUTOMATION & RUNNERS
│   ├── build.sh, run.sh, test.sh # Core runners
│   ├── viz/                    # Visualization & MCAP tools (export_mcap.py, serve_mcap.py)
│   ├── gem5/                   # gem5 simulation & cycle benchmarks
│   └── bench/                  # Batch sweeps & baseline comparison scripts
│
├── docs/                       # [5] DOCUMENTATION & SPECS
│   ├── ARCHITECTURE.md         # Core library architecture & algorithm spec
│   ├── WORKSPACE.md            # This specification document
│   ├── CHANGELOG.md            # Version & release history
│   ├── guides/                 # Walkthroughs, manuals (INSTRUCTION_MANUAL.md, etc.)
│   ├── experiments/            # Profiling reports & ablation experiments
│   ├── presentation/           # Slide decks & Marp presentations
│   └── plans/                  # Design & implementation plans
│
├── data/                       # [6] INPUT DATASETS (data/pcd_compressed/*.pcd)
└── output/                     # [7] UNIFIED OUTPUT DESTINATION (git-ignored)
```

---

## 3. Environment-Independent Execution Policy

RVPoint supports execution across multiple runtime environments (Native Linux, Docker Dev Containers, Windows WSL2, CI/CD runners):

### A. Inside Native Linux, Docker Dev Container, or Active Environment
When running inside an environment where the RISC-V toolchain is already active:
```bash
# Build:
./scripts/build.sh

# Run Fast Tests:
./scripts/test.sh

# Run Target:
./scripts/run.sh <target_name>
```

### B. From Windows Host (via WSL2)
When executing from Windows/PowerShell, commands should target the designated `rvpoint` distribution:
```bash
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/build.sh"
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/test.sh"
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/run.sh <target_name>"
```
*(Or launch interactively via `./env/wsl.sh`)*.

---

## 4. Inviolate Architectural Rules ("Do's and Don'ts")

### Must Do:
1. **Always activate environment toolchain**: Ensure `/opt/riscv` is in `PATH` by sourcing `env/activate.sh`.
2. **Always place pure library code in `src/`**: Code in `src/` is compiled into `librvpoint.a`. Keep it free of `int main()`.
3. **Always place executable tools in `eval/`**: Perception pipelines belong in `eval/pipelines/`, benchmarks in `eval/benchmarks/`, tests in `eval/tests/`.
4. **Always write outputs to `output/`**: Default all file outputs to `output/` or `output/<category>/`.
5. **Always prefer `PointCloudSoA` for vector kernels**: Use contiguous `x`, `y`, `z` buffers to enable unit-stride `__riscv_vle32_v_f32m8` vector loads.

### Never Do:
1. **Never add `int main()` inside `src/`**: This will break `librvpoint.a` compilation.
2. **Never hardcode absolute machine paths**: Never hardcode user paths, drive letters, or machine-specific locations in code or scripts.
3. **Never write outputs to workspace root**: Keep root clean. Never write `--no-write/`, `--progress/`, or temporary logs to root.
4. **Never commit generated artifacts**: `output/`, `results/`, and `build/` must remain in `.gitignore`.

---

## 5. Target Navigation & Feature Map

| Feature / Algorithm | Header Path | Implementation Path | Test Path |
| :--- | :--- | :--- | :--- |
| **Point Types & SoA** | [`src/core/point_types.h`](../src/core/point_types.h) | Header-only | [`eval/tests/fast/test_voxel_grid.cpp`](../eval/tests/fast/test_voxel_grid.cpp) |
| **RVV Vector Helpers** | [`src/core/rvv_common.h`](../src/core/rvv_common.h) | [`src/core/rvv_common.cpp`](../src/core/rvv_common.cpp) | [`eval/tests/fast/test_rvv_features.c`](../eval/tests/fast/test_rvv_features.c) |
| **Voxel Grid Filter** | [`src/filters/voxel_grid.h`](../src/filters/voxel_grid.h) | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | [`eval/tests/fast/test_voxel_grid.cpp`](../eval/tests/fast/test_voxel_grid.cpp) |
| **SOR Outlier Filter**| [`src/filters/statistical_outlier_removal.h`](../src/filters/statistical_outlier_removal.h) | [`src/filters/statistical_outlier_removal.cpp`](../src/filters/statistical_outlier_removal.cpp) | [`eval/tests/fast/test_sor.cpp`](../eval/tests/fast/test_sor.cpp) |
| **Normal Estimation**| [`src/features/normal_estimation.h`](../src/features/normal_estimation.h) | [`src/features/normal_estimation.cpp`](../src/features/normal_estimation.cpp) | [`eval/tests/fast/test_normal.cpp`](../eval/tests/fast/test_normal.cpp) |
| **Pointer Octree** | [`src/search/pointer_octree.h`](../src/search/pointer_octree.h) | [`src/search/pointer_octree.cpp`](../src/search/pointer_octree.cpp) | [`eval/tests/fast/test_octree.cpp`](../eval/tests/fast/test_octree.cpp) |
| **Spatial Hashing** | [`src/search/spatial_hashing.h`](../src/search/spatial_hashing.h) | [`src/search/spatial_hashing.cpp`](../src/search/spatial_hashing.cpp) | [`eval/tests/fast/test_octree.cpp`](../eval/tests/fast/test_octree.cpp) |
| **Radius Search** | [`src/search/radius_search.h`](../src/search/radius_search.h) | [`src/search/radius_search.cpp`](../src/search/radius_search.cpp) | [`eval/tests/fast/test_radius.cpp`](../eval/tests/fast/test_radius.cpp) |
| **Plane RANSAC** | [`src/segmentation/ransac_plane.h`](../src/segmentation/ransac_plane.h) | [`src/segmentation/ransac_plane.cpp`](../src/segmentation/ransac_plane.cpp) | [`eval/tests/fast/test_ransac.cpp`](../eval/tests/fast/test_ransac.cpp) |
| **Euclidean Clustering**| [`src/segmentation/euclidean_clustering.h`](../src/segmentation/euclidean_clustering.h) | [`src/segmentation/euclidean_clustering.cpp`](../src/segmentation/euclidean_clustering.cpp) | [`eval/tests/fast/test_euclidean_clustering.cpp`](../eval/tests/fast/test_euclidean_clustering.cpp) |
| **PCD Loader** | [`src/io/simple_pcd_loader.h`](../src/io/simple_pcd_loader.h) | Header-only | [`eval/tests/fast/test_loader.cpp`](../eval/tests/fast/test_loader.cpp) |
| **Public API** | [`src/include/rvpoint.h`](../src/include/rvpoint.h) | Header-only | All pipelines |

---

## 6. Canonical Command Reference

### Build Commands
```bash
# Build all library targets, tools, and tests:
./scripts/build.sh

# Clean rebuild from scratch:
./scripts/build.sh --clean

# Build scalar baseline backend:
./scripts/build.sh --backend scalar
```

### Test Commands
```bash
# Run 12 Essential Fast Tests (runs in seconds):
./scripts/test.sh

# Run all tests (fast + experimental sweeps):
./scripts/test.sh --all

# Run single unit test:
./scripts/run.sh test_voxel_grid
```

### Pipeline Execution Commands
```bash
# Run Recommended Ultimate 3D Pipeline (Full PCD Export):
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Run Ultimate 3D Pipeline (Pure Compute Benchmark, No Disk I/O):
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Run 10-Stage Scientific Evaluation Pipeline:
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Run Standalone Neighbor Search Benchmark:
./scripts/run.sh neighbor_search_sor_bench data/pcd_compressed/0000000090.pcd output/neighbor_results
```
