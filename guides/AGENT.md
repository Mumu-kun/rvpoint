# AI Agent Guide: RVPoint

**Purpose**: Quick reference for AI agents working on this codebase  
**Last Updated**: 2026-04-07  
**Project Stack**: C++17, RVV intrinsics, QEMU 9.2, RISC-V GNU Toolchain (GCC 14), Docker

---

## Quick Start for Agents

### First Steps When Starting Work

1. **Read this file first** — it captures all design decisions and what's been done
2. **Check git log for recent changes**:
   ```bash
   git log --oneline -10
   ```
3. **Understand the environment**: Everything runs inside Docker via `scripts/rvpoint.sh`
4. **Check build status**:
   ```bash
   ls bin/   # If populated, last build is still valid
   ```

---

## Command Patterns

### The Only Script You Need

```bash
# Interactive menu (preferred — user doesn't have to remember commands)
./scripts/rvpoint.sh

# Or with direct subcommands (agent-friendly, non-interactive):
./scripts/rvpoint.sh build
./scripts/rvpoint.sh test
./scripts/rvpoint.sh bench
./scripts/rvpoint.sh bench-pipeline   # End-to-end pipeline benchmark
./scripts/rvpoint.sh shell            # Drop into container
```

`rvpoint.sh` handles everything: builds the Docker image if missing, creates or reuses the persistent container `rvpoint-dev`, then delegates to the relevant script inside.

### Running Commands Inside Container Directly

```bash
# Build
docker exec -it rvpoint-dev bash scripts/build.sh --toolchain linux

# Run a single test
docker exec -it rvpoint-dev bash -c "
    qemu-riscv64 -cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot bin/test_voxel_grid
"

# Run benchmark
docker exec -it rvpoint-dev bash scripts/run.sh bench
```

### Building Docker Image (only when Dockerfile changes)

```bash
docker build -f .devcontainer/Dockerfile -t rvpoint .devcontainer/
```

### CMake Build (inside container)

```bash
cmake -S /workspace -B /workspace/build \
      -DCMAKE_TOOLCHAIN_FILE=/workspace/cmake/riscv_linux.cmake
cmake --build /workspace/build -j$(nproc)
```

---

## Project Architecture

### Goal

Build a **lightweight, efficient PCL library optimized with RISC-V Vector (RVV) extensions** for edge deployment. Point cloud processing pressure can be distributed to RISC-V edge SoCs instead of running entirely on server-side GPUs.

### Data Layout (Critical Design Decision)

```
PointXYZ     — AoS (Array of Structures): {float x, y, z}  ← scalar functions
PointCloudSoA — SoA (Structure of Arrays): float* x, *y, *z ← RVV functions
```

SoA is mandatory for RVV: it enables unit-stride `vle32.v` loads instead of expensive strided gathers. All RVV functions take `PointCloudSoA`; all scalar reference functions take `PointXYZ*`.

### File Structure

```
rvpoint/
├── src/
│   ├── include/
│   │   ├── rvv_pcl.h           ← Full public API (read this first)
│   │   └── simple_pcd_loader.h ← PCD file I/O (ASCII + binary)
│   ├── rvv_common.cpp          ← Shared RVV kernels (distance, gather-filter)
│   ├── voxel_grid_downsamp.cpp ← 3 versions: scalar, rvv hybrid, rvv_v2 (sort-based)
│   ├── statistical_outlier_removal.cpp
│   ├── normal_estimation.cpp   ← Requires pre-built Octree for RVV path
│   ├── radius_search.cpp
│   ├── ransac_plane.cpp        ← Also contains extract_plane_inliers/outliers
│   ├── octree.cpp
│   └── spatial_hashing.cpp
├── tests/
│   ├── benchmark.cpp           ← Per-algorithm rdinstret benchmark (N=1024)
│   ├── benchmark_pipeline.cpp  ← [TO BE CREATED] E2E pipeline benchmark
│   ├── test_pipeline_walkthrough.cpp ← Full pipeline demo (loads bunny.pcd)
│   ├── test_voxel_grid.cpp
│   ├── test_sor.cpp
│   ├── test_normal.cpp
│   ├── test_radius.cpp
│   ├── test_ransac.cpp
│   ├── test_octree.cpp
│   └── test_spatial_hash_comparison.cpp
├── scripts/
│   ├── rvpoint.sh              ← [NEW] Interactive CLI (start here)
│   ├── run.sh                  ← Build + run tests/benchmarks
│   ├── build.sh                ← CMake build wrapper
│   └── visualize_result.py     ← Matplotlib PCD viewer
├── cmake/
│   ├── riscv_linux.cmake       ← riscv64-unknown-linux-gnu (QEMU-compatible)
│   └── riscv.cmake             ← riscv64-unknown-elf (Spike/bare-metal)
├── .devcontainer/
│   ├── Dockerfile              ← QEMU 9.2, GCC 14, Spike, all tools
│   └── devcontainer.json
├── data/                       ← PCD test datasets
├── results/                    ← Benchmark reports (timestamped .txt)
├── CMakeLists.txt
└── AGENT.md                    ← This file
```

### Algorithms Implemented

| Algorithm | Scalar Fn | RVV Fn | Notes |
|-----------|-----------|--------|-------|
| Voxel Grid | `voxel_grid_downsamp_sc` | `voxel_grid_downsamp_rvv_v2` | v2 is sort-based (no std::map), always prefer v2 |
| SOR | `sor_sc` | `sor_rvv` | Both O(n²) NN; RVV distances vectorized |
| Normal Estimation | `normal_estimation_sc` | `normal_estimation_rvv` | RVV requires pre-built Octree |
| Radius Search | `radius_search_sc` | `radius_search_rvv` | Global linear scan, fully vectorized |
| RANSAC Plane | `ransac_plane_sc` | `ransac_plane_rvv` | Inlier counting via vcpop |
| Extract Inliers | — | `extract_plane_inliers_rvv` | No scalar version |
| Extract Outliers | — | `extract_plane_outliers_rvv` | No scalar version |
| Extract Both | — | `extract_plane_inliers_outliers_rvv` | Single-pass, more efficient |

### Spatial Data Structures

- **Octree** (`Octree` class): Used by `normal_estimation_rvv`. Build with `octree.setInputCloud(soa); octree.build();` then call `octree.radiusSearch(...)`.
- **SpatialHash** (`SpatialHash` class): Alternative to Octree for fixed-radius search. Not yet integrated into main pipeline.

### Key RVV Kernels (in `rvv_common.cpp`)

- `get_dist_sq_rvv` — vectorized squared distance from N points to 1 query (LMUL=m8)
- `get_inds_in_radius_rvv` — fused gather-filter: given index array, gathers coordinates via `vluxei32`, computes distances, compresses results (LMUL=m2)

---

## Current Status

### What's Done (as of 2026-04-07)

- ✅ All 6 core algorithms: scalar + RVV implementations
- ✅ Two spatial indices: Octree + SpatialHash
- ✅ Per-algorithm benchmark (`tests/benchmark.cpp`): measures `rdinstret` per algorithm at N=1024
- ✅ Integration test (`tests/test_pipeline_walkthrough.cpp`): loads bunny.pcd, runs full pipeline
- ✅ Docker environment with QEMU 9.2 + GCC 14 (full RVV v1.0 support)
- ✅ Interactive CLI (`scripts/rvpoint.sh`)
- ✅ `voxel_grid_downsamp_rvv_v2`: fully vectorized sort-based approach (eliminates std::map)

### What's Next (priority order)

1. **`tests/benchmark_pipeline.cpp`** — End-to-end pipeline benchmark (see plan below)
2. Integrate SpatialHash into pipeline as optional alternative to Octree
3. Test at realistic cloud sizes (10K–100K points)
4. Measure Octree build cost explicitly in benchmark
5. Add ICP (Iterative Closest Point) for registration use case

---

## The Priority Task: E2E Pipeline Benchmark

**File to create**: `tests/benchmark_pipeline.cpp`  
**CMakeLists.txt**: Add `benchmark_pipeline` target after existing `benchmark` target.

### What it does

Measures the full pipeline cost (scalar vs RVV) at N = 1K / 4K / 16K:

```
Stage 1: Voxel Grid Downsampling   → voxel_grid_downsamp_sc  vs  voxel_grid_downsamp_rvv_v2
Stage 2: RANSAC Plane Fitting      → ransac_plane_sc          vs  ransac_plane_rvv
Stage 3: Extract Plane Outliers    → (same fn: extract_plane_outliers_rvv, shown separately)
Stage 4: Statistical Outlier Removal → sor_sc                 vs  sor_rvv
Stage 5: Octree Build              → [0 / not needed]         vs  octree.build() [COUNTED]
Stage 6: Normal Estimation         → normal_estimation_sc     vs  normal_estimation_rvv
─────────────────────────────────────────────────────────────────
TOTAL                              → sum stages 1-4,6         vs  sum all 6 stages
```

### Key design rules for this benchmark

1. **Generate synthetic data before any `rdinstret` read** — data gen is scaffolding, not algorithm cost
2. **AoS↔SoA conversions between stages are NOT timed** — in a real system data stays in SoA
3. **Octree build IS timed for RVV** — it's real pipeline cost (scalar doesn't need it)
4. **Both AoS and SoA prepared from same data before measurement starts**
5. **Pre-allocate all output buffers** before measurement windows

### Synthetic data spec

```
N points total = size under test (1024, 4096, 16384)
80% object points: uniform random in [0, 100]³
20% ground plane: z = 0.0, x/y in [0, 100]
Seed: 42 (reproducible)
```

### Output format

```
============================================================
  RVPoint End-to-End Pipeline Benchmark (rdinstret)
  Synthetic cloud: 80% object + 20% ground plane
============================================================

N = 1024
Stage                | Scalar(ins)  | RVV(ins)     | Ratio
------------------------------------------------------------
VoxelGrid            | ...          | ...          | x.xxX
RANSAC               | ...          | ...          | x.xxX
ExtractPlane         | (shared)     | ...          | ---
SOR                  | ...          | ...          | x.xxX
OctreeBuild          | ---          | ...          | ---
NormalEst            | ...          | ...          | x.xxX
------------------------------------------------------------
TOTAL (pipeline)     | ...          | ...          | x.xxX
============================================================
```

Report saved to: `results/benchmark_pipeline_YYYYMMDD_HHMMSS.txt`

### Parameters

```
leaf_size  = 1.0f      (for 0-100 range — ~1% density reduction)
ransac_thresh = 0.5f   (ground at z=0, object points have z > ~0.5)
ransac_iters = 500
sor k = 10, alpha = 1.0f
normal k = 10, radius = 2.0f
```

### CMakeLists.txt addition

After the existing `benchmark` target (line ~90):
```cmake
add_executable(benchmark_pipeline tests/benchmark_pipeline.cpp)
target_link_libraries(benchmark_pipeline PRIVATE rvvpcl)
```

---

## Common Tasks for Agents

### Task 1: Running the Full Test Suite

```bash
./scripts/rvpoint.sh test
# or non-interactively:
./scripts/rvpoint.sh
# Select option 2
```

### Task 2: Running a Specific Algorithm Test

```bash
# Inside container:
docker exec -it rvpoint-dev bash -c "
    qemu-riscv64 -cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot bin/test_sor
"
```

### Task 3: Adding a New Algorithm

1. Add function declarations to `src/include/rvv_pcl.h`
2. Implement in `src/<algorithm>.cpp` (scalar + RVV)
3. Add source to `list(APPEND SOURCES ...)` in `CMakeLists.txt`
4. Write test in `tests/test_<algorithm>.cpp`
5. Add executable to `CMakeLists.txt`
6. Add to `ALL_TESTS` in `scripts/run.sh`

### Task 4: Checking Benchmark Results

```bash
ls -lt results/           # List all reports, newest first
cat results/benchmark_report_<latest>.txt
```

### Task 5: Pipeline Walkthrough with Real Data

```bash
docker exec -it rvpoint-dev bash -c "
    qemu-riscv64 -cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot \
        bin/test_pipeline_walkthrough data/bunny.pcd
"
```

---

## Critical Gotchas for Agents

### QEMU Version Must Be 9.2+

**Gotcha**: QEMU 7.x has bugs with `vfcvt_rtz`, `vfredosum`, and integer vector ops.

```
QEMU 9.2 is installed at: /opt/qemu/bin/qemu-riscv64
Symlinked to:            /usr/bin/qemu-riscv64
```

Always use `-cpu rv64,v=true,vlen=128` flag. Without it, RVV intrinsics will crash.

### Linux Toolchain Needs Sysroot

**Gotcha**: The `riscv64-unknown-linux-gnu` toolchain produces dynamically linked binaries.

```bash
# WRONG — will segfault on QEMU
qemu-riscv64 -cpu rv64,v=true,vlen=128 bin/test_sor

# RIGHT — provide sysroot for dynamic linker
qemu-riscv64 -cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot bin/test_sor
```

### Normal Estimation Requires Octree

**Gotcha**: `normal_estimation_rvv` requires a **pre-built Octree** passed as argument. If you call it without building the octree first, it will crash or produce garbage.

```cpp
// CORRECT usage pattern:
Octree octree;
octree.setInputCloud(soa);
octree.build();  // ← Must call this first
normal_estimation_rvv(soa, octree, nx, ny, nz, k, radius);
```

### Voxel Grid: Always Use v2

**Gotcha**: `voxel_grid_downsamp_rvv` (v1) still uses `std::map` internally — it's a hybrid, not fully vectorized. Always prefer `voxel_grid_downsamp_rvv_v2` which is the fully vectorized sort-based version.

### Ground Truth Generation Is Not Pipeline Cost

**Gotcha**: In `test_pipeline_walkthrough.cpp`, Step 1.5 generates a synthetic ground plane **inside the benchmark scope** if `rdinstret` is used. For clean benchmarks, always generate test data **before** starting instruction counting.

### `#error` Guard on RVV

**Gotcha**: `rvv_pcl.h` has a hard `#error` if compiled without RVV support:

```cpp
#ifndef __riscv_vector
#error "RISC-V Vector (RVV) support is mandatory. Compile with -march=rv64gcv"
#endif
```

This means all code using the library **must** be compiled with `-march=rv64gcv`. The CMake toolchain files handle this automatically.

---

## What the Benchmarks Measure

### Instruction Counting (rdinstret)

Both benchmark files use RISC-V's `rdinstret` CSR:

```cpp
struct Timer {
    uint64_t start;
    void reset() { asm volatile("rdinstret %0" : "=r" (start)); }
    uint64_t elapsed() {
        uint64_t end;
        asm volatile("rdinstret %0" : "=r" (end));
        return end - start;
    }
};
```

This counts **retired instructions** on the QEMU emulated CPU. It's the correct metric for QEMU (wall-clock time is meaningless on an emulator).

### Why Not Wall-Clock Time?

QEMU emulates RISC-V at variable speed depending on host load. `rdinstret` gives deterministic, host-independent counts.

---

## Safety Checks

### Before Running Destructive Commands

Never run these without confirmation:
- `docker rm rvpoint-dev` — destroys the persistent container
- `rm -rf build/` — wipes the build (rebuild takes time)
- `docker rmi rvpoint` — destroys the image (rebuild takes 15-20 min)

The `rvpoint.sh` script asks for confirmation before image rebuild.

### Before Editing Core Headers

`src/include/rvv_pcl.h` is the public API. Changes here affect all tests and benchmarks. Read the full header before modifying.

---

## Git Policy

Commit only when the user explicitly asks. Never force push. Never amend published commits.

Read-only git operations (status, diff, log, show) are always safe.

---

## Session Logs (Hard Limits & Lessons Learned)

Each working session has a dated log in `guides/session_YYYY-MM-DD.md`.
These record environmental constraints, bugs hit, and fixes applied — things that burned time and shouldn't be repeated.

**When starting a session**: check the latest session file for recent hard limits.
**When ending a session**: append any new hard limits you hit to today's session file, or create it if it doesn't exist.

Current logs:
- [`guides/session_2026-04-07.md`](session_2026-04-07.md) — Docker swap impossibility, container RAM cap, gem5 OOM

---

## Recommended Reading Order for New Agents

1. **This file** — architecture and current status
2. **`src/include/rvv_pcl.h`** — full public API
3. **`tests/benchmark.cpp`** — how measurements work (Timer, DualStream)
4. **`tests/test_pipeline_walkthrough.cpp`** — how the full pipeline flows
5. **`src/rvv_common.cpp`** — the core shared RVV kernels
6. **`scripts/run.sh`** — how tests/benchmarks are invoked

---

## Agent Best Practices

### DO ✅
- Read files before editing — never guess content
- Use `docker exec -it rvpoint-dev` for all commands inside the container
- Check `bin/` exists before trying to run tests
- Use `voxel_grid_downsamp_rvv_v2` (not v1) in any new code
- Include Octree build cost when benchmarking the RVV normal estimation path
- Test at multiple cloud sizes (1K, 4K, 16K) to show scaling behavior

### DON'T ❌
- Run QEMU without `-cpu rv64,v=true,vlen=128` — RVV instructions will crash
- Run linux-toolchain binaries without `-L /opt/riscv/sysroot`
- Include test data generation inside `rdinstret` measurement windows
- Use `voxel_grid_downsamp_rvv` (v1) in new code — use v2
- Call `normal_estimation_rvv` without pre-building the Octree
- Benchmark wall-clock time on QEMU — use rdinstret only
