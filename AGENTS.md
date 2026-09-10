# RVPoint Agent & Developer Guidelines

> **Mandatory behavioral and structural instructions for AI coding assistants (Antigravity, Claude, Copilot, Cursor) working on the RVPoint codebase.**

---

## 1. Environment & Toolchain Execution Policy

RVPoint targets RISC-V 64-bit vector architectures (`rv64gcv`) and requires GCC 14.2+ with RVV 1.0 intrinsics support (located in `/opt/riscv`).

### Multi-Environment Execution Reference:

1. **Inside Native Linux / Docker Dev Containers / CI Runners**:
   ```bash
   source env/activate.sh
   ./scripts/build.sh
   ./scripts/test.sh
   ./scripts/run.sh <target_name>
   ```

2. **From Windows Host (via WSL2 `rvpoint` Distribution)**:
   Always target the custom `rvpoint` WSL distribution which contains the `/opt/riscv` cross-compilation toolchain:
   ```bash
   wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/build.sh"
   wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/test.sh"
   wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/run.sh <target_name>"
   ```
   *(Or interactively using `./env/wsl.sh`)*.
   run common windows tools/commands using git bash not wsl

3. **General Agent Invariants**:
   - **Environment Agnostic**: Never hardcode host machine paths, user directories, or drive letters. Use repository-relative paths.
   - **Do Not Auto-Commit Design Plans**: Keep planning and design artifacts in `docs/plans/` local unless explicitly requested to commit by the user.

---

## 2. Inviolate Workspace Hierarchy (7-Folder Root Policy)

The root directory must strictly maintain this 7-folder structure. **NEVER create arbitrary folders or files at the workspace root.**

```text
rvpoint/
├── CMakeLists.txt              # Unified automated build system (auto-discovers targets)
├── README.md                   # Human project overview & quickstart
├── AGENTS.md                   # This instruction file for AI agents
│
├── src/                        # [1] 100% PURE LIBRARY (librvpoint.a) - NO main()
│   ├── core/                   # Point types, SoA structures, RVV primitives, profiler
│   ├── features/               # Surface normal estimation (Cardano closed-form)
│   ├── filters/                # Voxel downsampling (v2) & Statistical Outlier Removal
│   ├── search/                 # Octree, SpatialHash, PointerOctree, Caravan search
│   ├── segmentation/           # RANSAC plane fitting & Euclidean clustering
│   ├── io/                     # Zero-dependency simple PCD file reader & writer
│   └── include/                # Public umbrella header (rvpoint.h) & rvv_pcl forwarder
│
├── eval/                       # [2] ALL EXECUTABLES & EVALUATION SUITE
│   ├── pipelines/              # All standalone perception pipelines (pipeline_3d_intra, etc.)
│   ├── benchmarks/             # Standalone benchmark utilities (ablation_bench, etc.)
│   ├── tests/                  # Two-tier test suite (fast/ and experimental/)
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
│   ├── WORKSPACE.md            # Dual-audience workspace specification
│   ├── CHANGELOG.md            # Version & release history
│   ├── guides/                 # Walkthroughs, manuals (INSTRUCTION_MANUAL.md, etc.)
│   ├── experiments/            # Profiling reports & ablation experiments
│   ├── presentation/           # Slide decks & Marp presentations
│   └── plans/                  # Design & implementation plans
│
├── data/                       # [7] INPUT POINT CLOUD DATASETS (data/pcd_compressed/*.pcd)
└── output/                     # [8] UNIFIED OUTPUT DESTINATION (git-ignored)
```

---

## 3. Rules for Adding Code & Extending the Project

### A. Library Code (`src/`)
- **NO `main()` in `src/`**: Every file in `src/` is compiled into static library `librvpoint.a`. Adding `int main()` anywhere in `src/` will break the build.
- **Canonical Namespace**: Always write new classes and algorithms inside `namespace rvpoint { ... }`.
- **Public API Exposure**: When creating new modules, expose their public header inside `src/include/rvpoint.h`.
- **Data Representation**: Prefer `PointCloudSoA` (`in.x`, `in.y`, `in.z` contiguous buffers) over AoS `std::vector<PointXYZ>` for vector-accelerated algorithms to enable sequential `__riscv_vle32_v_f32m8` loads.
- **Design & Vector Standards**: Consult [`docs/guides/CODEBASE_DESIGN_AND_RVV_STANDARDS.md`](docs/guides/CODEBASE_DESIGN_AND_RVV_STANDARDS.md) for deep module interface guidelines, seam placement, and RVV 1.0 intrinsics invariants (unit-stride, LMUL allocation, zero-heap loops).

### B. Executables & Perception Pipelines (`eval/`)
- **Pipelines**: Place end-to-end perception pipelines in `eval/pipelines/<pipeline_name>.cpp`.
- **Benchmarks**: Place standalone benchmarks in `eval/benchmarks/<benchmark_name>.cpp`.
- **Tests**: Place fast, self-contained unit and regression tests in `eval/tests/fast/test_<feature>.cpp`.
- **Automated Discovery**: `CMakeLists.txt` automatically globs and registers any new `.cpp` files in `eval/pipelines/`, `eval/benchmarks/`, and `eval/tests/`. There is no need to manually edit `CMakeLists.txt` when adding new tools.

### C. Output Directory Protocol (`output/`)
- **All generated files MUST write to `output/`**:
  - Point cloud outputs $\rightarrow$ `output/*.pcd` or `output/<experiment_name>/*.pcd`
  - MCAP visualization files $\rightarrow$ `output/*.mcap`
  - Benchmark metrics / JSON logs $\rightarrow$ `output/*.json`
- **Never write outputs to root**: Do not create accidental root directories like `./--no-write` or `./--progress` due to CLI argument mishaps.

### D. Scripts Organization (`scripts/`)
- When creating new automation scripts:
  - MCAP/Foxglove or visual tools $\rightarrow$ `scripts/viz/`
  - Gem5 simulation & cycle scripts $\rightarrow$ `scripts/gem5/`
  - Multi-frame dataset batch sweeps $\rightarrow$ `scripts/bench/`

---

## 4. Verification Workflow for Agents

Before completing any task, agents MUST verify that the changes build cleanly and tests pass:

```bash
# 1. Clean build all library targets, tools, and tests:
./scripts/build.sh

# 2. Run fast test suite:
./scripts/test.sh

# 3. Test execution of a target:
./scripts/run.sh <target_name>
```
*(Wrap with `wsl -d rvpoint bash -c "source env/activate.sh && <cmd>"` when on Windows host).*
