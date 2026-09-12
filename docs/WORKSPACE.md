# RVPoint Workspace Specification

> **Definitive Workspace & System Architecture Guide for Human Developers and AI Coding Agents.**
> **Repository**: `rvpoint` (formerly `rvv_pcl`)
> **Primary Namespace**: `rvpoint` (alias: `rvv_pcl`)
> **Primary Umbrella Header**: `#include "include/rvpoint.h"`
> **Primary CMake Target**: `rvpoint` (static library `librvpoint.a`)
> **Architecture Reference**: [ARCHITECTURE.md](ARCHITECTURE.md)

---

## 1. Quick Context Bootstrap

| Key Property | Canonical Value | Notes |
| :--- | :--- | :--- |
| **C++ Standard** | `C++17` | Required across all library and test code |
| **Target Hardware** | RISC-V 64-bit with RVV 1.0 (`rv64gcv`) | SpacemiT K1 Octa-Core SoC / QEMU 9.x |
| **Toolchain Requirement** | RISC-V GCC 14.2+ (`riscv64-unknown-linux-gnu-gcc`) | Located in `/opt/riscv` |
| **Build System** | CMake 3.16+ | Automatic target & source discovery |
| **Primary Data Container** | `rvpoint::PointCloud` | Owning contiguous Structure-of-Arrays (SoA) |
| **Primary Data View** | `rvpoint::PointCloudView` | Non-owning contiguous slice view |
| **Default Output Destination** | `output/` | Unified destination for PCDs, MCAPs, and metrics |

---

## 2. Inviolate Workspace Hierarchy (7-Folder Root Policy)

The root directory strictly maintains this 7-folder structure. **NEVER create arbitrary folders or files at the workspace root.**

```text
rvpoint/
├── CMakeLists.txt              # Unified automated build system (auto-discovers targets)
├── README.md                   # Human project overview & quickstart
├── AGENTS.md                   # Mandatory instructions for AI coding assistants
│
├── src/                        # [1] 100% PURE LIBRARY (librvpoint.a) - NO main()
│   ├── core/                   # Point types, SoA structures, RVV primitives, profiler
│   ├── features/               # Surface normal estimation (Cardano closed-form) & FusedFilterNormals
│   ├── filters/                # Voxel downsampling, SOR, ROR, filter concepts
│   ├── search/                 # Octree, SpatialHash, PointerOctree, Caravan, Fast3DSpatialGrid
│   ├── segmentation/           # RANSAC plane fitting & Euclidean clustering (BFS & Union-Find)
│   ├── pipeline/               # Slotted RegisterFile, PipelineManager, TaggedBinding
│   ├── io/                     # Zero-dependency simple PCD file reader & writer
│   └── include/                # Public umbrella header (rvpoint.h)
│
├── eval/                       # [2] ALL EXECUTABLES & EVALUATION SUITE
│   ├── pipelines/              # Standalone perception pipelines (pipeline_prototype.cpp, etc.)
│   ├── benchmarks/             # Standalone benchmark utilities & profiling drivers
│   ├── tests/                  # Two-Tier Test Suite
│   │   ├── fast/               # 13 Essential Fast Tests (runs on every build)
│   │   └── experimental/       # Algorithmic sweeps & research audits
│   └── notebooks/              # Research & Kaggle/gem5 notebooks
│
├── demonstration/              # [3] LIVE HARDWARE DEMOS, STREAMERS & VISUALIZERS
│   ├── LIDAR_APP_IOS/          # iOS Swift LiDAR streaming capture application
│   ├── server_main.py          # Real-time multi-threaded perception daemon & visualizer
│   ├── pcd_server.py           # TCP PCD sync broadcaster
│   └── receive_scans.py        # Client receiver utility
│
├── env/                        # [4] ENVIRONMENT & TOOLCHAINS
│   ├── cmake/                  # CMake toolchain files (riscv.cmake, riscv_linux.cmake)
│   ├── linux/                  # Linux / container installer scripts
│   ├── wsl.sh                  # Quick WSL launcher
│   ├── setup.sh                # Main setup entrypoint
│   └── activate.sh             # Environment activator
│
├── scripts/                    # [5] AUTOMATION & RUNNERS
│   ├── build.sh, run.sh, test.sh # Core runners
│   ├── viz/                    # Visualization & MCAP tools (export_mcap.py, serve_mcap.py)
│   ├── gem5/                   # gem5 simulation & cycle benchmarks
│   └── bench/                  # Batch sweeps & baseline comparison scripts
│
├── docs/                       # [6] DOCUMENTATION & SPECS
│   ├── ARCHITECTURE.md         # Core library architecture & algorithm spec
│   ├── WORKSPACE.md            # This workspace specification
│   ├── CONTEXT.md              # Domain model glossary & terminology
│   ├── CHANGELOG.md            # Version & release history
│   ├── guides/                 # Walkthroughs & engineering standards
│   ├── experiments/            # Profiling reports & architectural research
│   ├── presentation/           # Slide decks & Marp presentations
│   └── plans/                  # Active design documents & roadmaps
│
├── data/                       # [7] INPUT POINT CLOUD DATASETS (data/pcd_compressed/*.pcd)
└── output/                     # [8] UNIFIED OUTPUT DESTINATION (git-ignored)
```

---

## 3. Environment & Execution Policy

### A. Inside Native Linux / Docker Dev Containers
```bash
source env/activate.sh
./scripts/build.sh
./scripts/test.sh
./scripts/run.sh pipeline_prototype
```

### B. From Windows Host (via WSL2 `rvpoint` Distribution)
Target the custom `rvpoint` WSL2 distribution containing `/opt/riscv`:
```bash
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/build.sh"
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/test.sh"
wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/run.sh pipeline_prototype"
```
*(Or launch interactively via `./env/wsl.sh`)*. Run Windows host commands using PowerShell or Git Bash.

---

## 4. Target Navigation & Component Map

| Feature / Algorithm | Header Path | Implementation Path | Fast Test Target |
| :--- | :--- | :--- | :--- |
| **Point Types & Views** | [`src/core/point_types.h`](../src/core/point_types.h) | Header-only | [`test_concepts`](../eval/tests/fast/test_concepts.cpp) |
| **Voxel Grid Filter** | [`src/filters/voxel_grid.h`](../src/filters/voxel_grid.h) | [`src/filters/voxel_grid_downsamp.cpp`](../src/filters/voxel_grid_downsamp.cpp) | [`test_voxel_grid`](../eval/tests/fast/test_voxel_grid.cpp) |
| **SOR Filter** | [`src/filters/statistical_outlier_removal.h`](../src/filters/statistical_outlier_removal.h) | [`src/filters/statistical_outlier_removal.cpp`](../src/filters/statistical_outlier_removal.cpp) | [`test_sor`](../eval/tests/fast/test_sor.cpp) |
| **ROR Filter** | [`src/filters/radius_outlier_removal.h`](../src/filters/radius_outlier_removal.h) | [`src/filters/radius_outlier_removal.cpp`](../src/filters/radius_outlier_removal.cpp) | [`test_ror`](../eval/tests/fast/test_ror.cpp) |
| **Normal Estimation** | [`src/features/normal_estimation.h`](../src/features/normal_estimation.h) | [`src/features/normal_estimation.cpp`](../src/features/normal_estimation.cpp) | [`test_normal`](../eval/tests/fast/test_normal.cpp) |
| **Fused Filter & Normals** | [`src/features/fused_filter_normals.h`](../src/features/fused_filter_normals.h) | [`src/features/fused_filter_normals.cpp`](../src/features/fused_filter_normals.cpp) | [`test_fused_filter_normals`](../eval/tests/fast/test_fused_filter_normals.cpp) |
| **Fast 3D Spatial Grid** | [`src/search/fast_3d_spatial_grid.h`](../src/search/fast_3d_spatial_grid.h) | [`src/search/fast_3d_spatial_grid.cpp`](../src/search/fast_3d_spatial_grid.cpp) | [`test_radius`](../eval/tests/fast/test_radius.cpp) |
| **Pointer Octree** | [`src/search/pointer_octree.h`](../src/search/pointer_octree.h) | [`src/search/pointer_octree.cpp`](../src/search/pointer_octree.cpp) | [`test_concepts`](../eval/tests/fast/test_concepts.cpp) |
| **Plane RANSAC** | [`src/segmentation/ransac_plane.h`](../src/segmentation/ransac_plane.h) | [`src/segmentation/ransac_plane.cpp`](../src/segmentation/ransac_plane.cpp) | [`test_ransac`](../eval/tests/fast/test_ransac.cpp) |
| **Euclidean Clustering** | [`src/segmentation/euclidean_clustering.h`](../src/segmentation/euclidean_clustering.h) | [`src/segmentation/euclidean_clustering.cpp`](../src/segmentation/euclidean_clustering.cpp) | [`test_euclidean_clustering`](../eval/tests/fast/test_euclidean_clustering.cpp) |
| **Pipeline Engine** | [`src/pipeline/pipeline_manager.h`](../src/pipeline/pipeline_manager.h) | [`src/pipeline/pipeline_manager.cpp`](../src/pipeline/pipeline_manager.cpp) | [`test_pipeline`](../eval/tests/fast/test_pipeline.cpp) |
| **PCD File I/O** | [`src/io/simple_pcd_loader.h`](../src/io/simple_pcd_loader.h) | Header-only | [`test_loader`](../eval/tests/fast/test_loader.cpp) |
| **Umbrella Header** | [`src/include/rvpoint.h`](../src/include/rvpoint.h) | Header-only | [`test_pipeline_walkthrough`](../eval/tests/fast/test_pipeline_walkthrough.cpp) |

---

## 5. Canonical Command Reference

### Build Commands
```bash
# Build librvpoint.a, pipelines, and all tests on default RVV backend:
./scripts/build.sh

# Build scalar reference backend:
./scripts/build.sh --backend scalar

# Clean rebuild:
./scripts/build.sh --clean
```

### Test Commands
```bash
# Run 13 essential fast unit tests on RVV 1.0 backend:
./scripts/test.sh

# Run 12 fast tests on Scalar fallback backend (verifies dual-path parity):
./scripts/test.sh --backend scalar

# Run specific unit test:
./scripts/run.sh test_ransac
./scripts/run.sh test_euclidean_clustering
./scripts/run.sh test_fused_filter_normals
```

### Perception Pipeline Execution (QEMU Emulation)
```bash
# Run Slotted Register-File Pipeline Prototype on sample PCD:
./scripts/run.sh pipeline_prototype data/01_table_scene_lms400.pcd
```
