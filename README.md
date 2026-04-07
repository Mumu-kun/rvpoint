# RVPoint: RISC-V Vector Optimized Point Cloud Library

**RVPoint** is a specialized Point Cloud Library (PCL) built for **RISC-V** architectures, leveraging **RVV** (RISC-V Vector Extension) intrinsics to accelerate core 3D processing algorithms.

Optimized for **RV64GCV** (targeting `v1.0` vector spec).

> **[Online Code Documentation](https://mumu-kun.github.io/rvpoint/)**

## Documentation & Resources

| Guide | Description |
|-------|-------------|
| [**Instruction Manual**](@ref instruction_manual) | **The Master Guide**. API documentation, Team SOPs, and Intrinsic Naming conventions. |
| [**Pipeline Demo**](@ref pipeline_demo) | **Quick-start**. Walkthrough of the optimized Octree/SpatialHash pipeline. |
| [**Performance Report**](@ref performance_report) | Benchmarks comparing different spatial indexing methods. |
| [**Vector Theory**](@ref vector_theory) | Geometric logic behind the RVV implementations. |

## Key Features

5 core point cloud processing algorithms, each with an optimized **RVV** path alongside a standard Scalar reference.

| Algorithm | Function Name | RVV Optimization |
| :--- | :--- | :--- |
| **Voxel Grid Downsampling** | `voxel_grid_downsamp_rvv` | Vectorized coordinate scaling & projection. |
| **Statistical Outlier Removal** | `sor_rvv` | Accelerated K-NN distance calculation using `get_dist_sq_rvv` kernel. |
| **Normal Estimation** | `normal_estimation_rvv` | Accelerated neighbor search for covariance matrix building. |
| **Radius Search** | `radius_search_rvv` | Vectorized global distance scan & filter mask. |
| **RANSAC Plane Fitting** | `ransac_plane_rvv` | High-throughput inlier counting using vector masks (`vmfle`, `vcpop`). |
| **Spatial Hashing** | `SpatialHash::radiusSearch` | O(1) cell lookup with RVV fused gather-filter candidate processing. |

## Setup

### Prerequisites
- **Docker Desktop** (or Docker Engine)
- **VS Code** with "Dev Containers" extension (optional, for IDE integration)

### Dev Container (Recommended)
1. Clone and open:
   ```bash
   git clone https://github.com/Mumu-kun/rvpoint.git
   cd rvpoint
   code .
   ```
2. Click **"Reopen in Container"** when prompted in VS Code.
3. The container provides: **GCC 14** (Linux + bare-metal toolchains), **QEMU 9.2**, **Spike v1.1.0**, **CMake**, and all dependencies.

### Docker (Manual)

**Build the image:**
```bash
docker build -f .devcontainer/Dockerfile -t rvpoint .
```

**Run interactively:**
```bash
docker run -it --rm -v $(pwd):/workspace -w /workspace rvpoint /bin/bash
```

**Run a single command:**
```bash
docker run --rm -v $(pwd):/workspace -w /workspace rvpoint bash scripts/run.sh test
```

### Custom Toolchain Path
If you installed the toolchain outside the container:
```bash
export RISCV_PATH=/path/to/riscv
```

## Building

A central build script handles CMake configuration and compilation with caching:

```bash
scripts/build.sh                          # default: linux toolchain
scripts/build.sh --toolchain elf          # bare-metal toolchain
scripts/build.sh --clean                  # force full reconfigure
scripts/build.sh --clean --toolchain elf  # clean + specific toolchain
```

- Skips CMake configure if already configured (fast incremental builds)
- Auto-detects toolchain switches and reconfigures when needed
- Outputs binaries to `bin/`

## Running Tests

Use `scripts/run.sh` to build and run tests or benchmarks:

```bash
# Run all tests
scripts/run.sh test

# Run specific tests (prefix "test_" is optional)
scripts/run.sh test voxel_grid sor ransac

# List all available targets
scripts/run.sh --list
```

### Available tests

| Test | Algorithm |
|------|-----------|
| `test_scalar` | Basic scalar operations |
| `test_vector` | Basic vector operations |
| `rvv_test` | RVV feature verification |
| `test_voxel_grid` | Voxel Grid Downsampling |
| `test_sor` | Statistical Outlier Removal |
| `test_normal` | Normal Estimation |
| `test_radius` | Radius Search |
| `test_ransac` | RANSAC Plane Fitting |
| `test_octree` | Octree spatial index |
| `test_spatial_hash_comparison` | Spatial Hash vs Octree |
| `test_pipeline_walkthrough` | Full PCL pipeline |

## Benchmarking

### rdinstret Benchmark (via `run.sh`)
Runs all algorithms inside QEMU, counting instructions via the `rdinstret` CSR:
```bash
scripts/run.sh bench
```
Outputs a formatted table to stdout and saves a timestamped report to `results/`.

### QEMU Trace Benchmark
Counts Translation Block executions using QEMU's `-one-insn-per-tb` tracing:
```bash
scripts/run_trace_benchmark.sh
```
Saves results to `results/report_sc_rvv_qemu.txt`.

## Full Verification

Runs a clean build and all algorithm tests:
```bash
scripts/verify_container.sh
```

## Project Structure

```
.
├── CMakeLists.txt              # Build configuration
├── cmake/
│   ├── riscv.cmake             # Bare-metal (elf) toolchain
│   └── riscv_linux.cmake       # Linux (glibc) toolchain
├── src/
│   ├── include/
│   │   ├── rvv_pcl.h           # Public API header
│   │   └── simple_pcd_loader.h # PCD file loader
│   ├── voxel_grid_downsamp.cpp
│   ├── statistical_outlier_removal.cpp
│   ├── normal_estimation.cpp
│   ├── radius_search.cpp
│   ├── ransac_plane.cpp
│   ├── octree.cpp
│   ├── spatial_hashing.cpp
│   └── rvv_common.cpp          # Shared RVV kernels
├── tests/                      # Test and benchmark sources
├── scripts/
│   ├── build.sh                # Central build script (cached)
│   ├── run.sh                  # Test and benchmark runner
│   ├── run_trace_benchmark.sh  # QEMU trace-based benchmark
│   ├── verify_container.sh     # Full verification suite
│   ├── setup_git_hooks.sh      # Git pre-push hook setup
│   ├── download_datasets.sh    # Download bunny.pcd dataset
│   └── visualize_result.py     # 3D point cloud visualization
├── .devcontainer/              # Docker environment config
└── .github/workflows/          # GitHub Actions CI
```

## Toolchain

The container ships with **GCC 14** (2025.01.20 nightly) providing two toolchain variants:

| Toolchain | Compiler Prefix | Use Case |
|-----------|----------------|----------|
| **Linux (default)** | `riscv64-unknown-linux-gnu-` | Full application builds, file I/O, benchmarks |
| **Bare-metal** | `riscv64-unknown-elf-` | Minimal verification, no OS dependencies |

Both use `-march=rv64gcv -mabi=lp64d` and support RVV 1.0 intrinsics, fractional LMUL, tuple types, and auto-vectorization.

## CI/CD

- **GitHub Actions**: Compiles and runs all tests on every `push` and `pull_request`.
- **Local Git Hooks**: `pre-push` hook available via `bash scripts/setup_git_hooks.sh`.

## License
[MIT](LICENSE)
