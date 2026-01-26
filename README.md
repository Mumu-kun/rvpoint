# RVPoint: RISC-V Vector Optimized Point Cloud Library

**RVPoint** is a specialized Point Cloud Library (PCL) built for **RISC-V** architectures, leveraging **RVGe** (RISC-V Vector Extension) intrinsics to accelerate core 3D processing algorithms. 

Optimized for **RV64GCV** (targeting `v0.10+` vector specs).

> 📘 **[Online Code Documentation](https://mumu-kun.github.io/rvpoint/)**

##  Documentation & Resources

Explore these guides for a deeper understanding of the codebase and its performance:

| Guide | Description |
|-------|-------------|
| [**Instruction Manual**](@ref instruction_manual) | **The Master Guide**. Contains API documentation, Team SOPs, and Intrinsic Naming conventions. |
| [**Pipeline Demo**](@ref pipeline_demo) | **Recommended for Quick-start**. Detailed walkthrough of the optimized Octree/SpatialHash pipeline. |
| [**Performance Report**](@ref performance_report) | Benchmarks comparing different spatial indexing methods. |
| [**Vector Theory**](@ref vector_theory) | Geometric logic behind the RVV implementations. |

## 🚀 Key Features

This library implements 5 core point cloud processing algorithms, each with a highly optimized **RVV** path alongside a standard Scalar reference path.

| Algorithm | Function Name | RVV Optimization |
| :--- | :--- | :--- |
| **Voxel Grid Downsampling** | `voxel_grid_downsamp_rvv` | Vectorized coordinate scaling & projection. |
| **Statistical Outlier Removal** | `sor_rvv` | Accelerated K-NN distance calculation using `get_dist_sq_rvv` kernel. |
| **Normal Estimation** | `normal_estimation_rvv` | Accelerated neighbor search for covariance matrix building. |
| **Radius Search** | `radius_search_rvv` | Vectorized global distance scan & filter mask. |
| **RANSAC Plane Fitting** | `ransac_plane_rvv` | High-throughput inlier counting using vector masks (`vmfle`, `vcpop`). |
| **Spatial Hashing** | `SpatialHash::radiusSearch` | O(1) cell lookup with RVV fused gather-filter candidate processing. |

## 🛠️ Installation & Setup

### Prerequisites
*   **Docker Desktop** (or Docker Engine)
*   **VS Code** with "Dev Containers" extension.

### Quick Start (Recommended)
1.  **Clone the Repository**:
    ```bash
    git clone https://github.com/Mumu-kun/rvpoint.git
    cd rvpoint
    ```
2.  **Open in Dev Container**:
    *   Open VS Code (`code .`).
    *   Click "Reopen in Container" when prompted.
    *   *This automatically sets up the GCC 13+ (or GCC 14 if configured) RISC-V Toolchain, QEMU, CMake, and all dependencies.*

### Running with Docker (Manual)
If you prefer to run Docker commands manually in your terminal, use these commands:

**Option 1: Open Already Built Image**
(Requires an existing image tagged as `rvpoint-built`)
```bash
docker run -it --rm -v $(pwd):/workspace -w /workspace rvpoint-built /bin/bash
```

**Option 2: Clean Build & Run (Simulate New User)**
Test the build process from scratch:

1. **Build the Image**:
    ```bash
    docker build -f .devcontainer/Dockerfile -t rvpoint-clean-test .
    ```

2. **Run the Clean Image**:
    ```bash
    docker run -it --rm -v $(pwd):/workspace -w /workspace rvpoint-clean-test /bin/bash
    ```

### Custom Toolchain Path
If you installed the toolchain in a custom location, set the `RISCV_PATH` environment variable before running scripts or building:
```bash
export RISCV_PATH=/path/to/riscv
```

## 🏗️ Building & Testing

We provide a master script that handles linting, building, and verifying all algorithms in one go.

### Run Full Verification
```bash
scripts/verify_container.sh
```
This script performs:
1.  **Code Format Check** (`clang-format`)
2.  **Static Analysis** (`cppcheck`)
3.  **CMake Configuration** (Targeting `rv64gcv`)
4.  **Compilation**
5.  **QEMU Emulation Tests** (Runs all 5 algo tests)

### Manual Build
If you want to build manually:
```bash
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv.cmake # (Or rely on env vars set by container)
make
```

### Running Individual Tests
After building, you can run specific tests using QEMU:
```bash
# Example: Run RANSAC test
/opt/riscv/bin/qemu-riscv64 -cpu max build_cmake/test_ransac
```

## 📂 Project Structure

```
.
├── src/
│   ├── include/rvv_pcl.h       # Public API Header
│   ├── rvv_common.cpp          # Reusable RVV Kernels (Distance, etc.)
│   ├── voxel_grid_downsamp.cpp # Voxel Grid Implementation
│   ├── statistical_outlier...  # SOR Implementation
│   ├── normal_estimation.cpp   # Normal Estimation Implementation
│   ├── radius_search.cpp       # Radius Search Implementation
│   ├── octree.cpp              # Octree spatial index
│   ├── spatial_hashing.cpp     # RVV Optimized Hash Grid
│   └── ransac_plane.cpp        # RANSAC Implementation
├── tests/                      # Unit Tests (C++)
├── scripts/
│   └── verify_container.sh     # Master CI/CD script
├── .devcontainer/              # Docker Environment Config
└── .github/workflows/          # GitHub Actions CI
```

## 🤖 CI/CD Pipeline

This repository is protected by a robust CI/CD pipeline:
*   **GitHub Actions**: Automatically compiles and runs all tests on every `push` and `pull_request` using a fresh Ubuntu+RISC-V environment.
*   **Local Git Hooks**: A `pre-push` hook is available to prevent pushing broken code.
    *   Enable with: `bash scripts/setup_git_hooks.sh`

## 📊 Benchmarking

We support instruction-level benchmarking using QEMU's tracing feature. This allows for accurate "architectural instruction count" comparison between Scalar and RVV implementations, independent of emulation speed.

### Running the Benchmark
To generate the instruction count report:
```bash
scripts/run_trace_benchmark.sh
```
This will:
1.  Build the benchmark executable.
2.  Run each algorithm in initialization-only (`setup`), Scalar (`sc`), and RVV (`rvv`) modes.
3.  Use QEMU trace logs to count executed Translation Blocks (TBs).
4.  Subtract baseline overhead to report accurate kernel instruction counts.
5.  Output results to `results/report_sc_rvv_qemu.txt`.

### Expected Results
You should see significant instruction reduction for compute-bound kernels (e.g., Normal Estimation ~5x reduction).

### Verify without CMake (Manual Compilation)
You can manually compile and verify each function against its scalar counterpart using the provided script or manual commands.

**Using the script:**
```bash
scripts/verify_manual.sh
```

**Individual commands:**

*Voxel Grid:*
```bash
/opt/riscv/bin/riscv64-unknown-elf-g++ -march=rv64gcv -mabi=lp64d -I src/include src/rvv_common.cpp src/voxel_grid_downsamp.cpp tests/test_voxel_grid.cpp -o test_voxel_rvv_manual
qemu-riscv64 -cpu rv64,v=true,vlen=128 ./test_voxel_rvv_manual
```

*RANSAC Plane:*
```bash
/opt/riscv/bin/riscv64-unknown-elf-g++ -march=rv64gcv -mabi=lp64d -I src/include src/rvv_common.cpp src/ransac_plane.cpp tests/test_ransac.cpp -o test_ransac_rvv_manual
qemu-riscv64 -cpu rv64,v=true,vlen=128 ./test_ransac_rvv_manual
```

*Radius Search:*
```bash
/opt/riscv/bin/riscv64-unknown-elf-g++ -march=rv64gcv -mabi=lp64d -I src/include src/rvv_common.cpp src/radius_search.cpp tests/test_radius.cpp -o test_radius_rvv_manual
qemu-riscv64 -cpu rv64,v=true,vlen=128 ./test_radius_rvv_manual
```

*Statistical Outlier Removal:*
```bash
/opt/riscv/bin/riscv64-unknown-elf-g++ -march=rv64gcv -mabi=lp64d -I src/include src/rvv_common.cpp src/statistical_outlier_removal.cpp tests/test_sor.cpp -o test_sor_rvv_manual
qemu-riscv64 -cpu rv64,v=true,vlen=128 ./test_sor_rvv_manual
```



## ⚡ GCC 14 Upgrade & Linux Toolchain

We have upgraded the toolchain to **GCC 14** (supports RVV 1.0, Auto-vectorization, Tuple types).
We now provide **two** toolchains in the container:
1.  **Embedded/ELF** (`riscv64-unknown-elf-`): Default, for bare-metal/simulated verification.
2.  **Linux/Glibc** (`riscv64-unknown-linux-gnu-`): For building full Linux applications.

### How to use
1.  **Rebuild Dev Container**:
    *   Command Palette (`Ctrl+Shift+P`) -> `Dev Containers: Rebuild Container`.
    *   *This will automatically install both GCC 14 toolchains.*
2.  **Verify Setup**:
    ```bash
    scripts/verify_container.sh
    ```
    *This runs tests for both toolchains.*

### Building for Linux
To build using the Linux (glibc) toolchain, use the specialized CMake file:
```bash
mkdir build_linux
cd build_linux
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv_linux.cmake
make
```

### Detailed Verification Steps (Manual)
If you want to verify specific GCC 14 capabilities manually, run these commands:

**1. Verify Toolchain Version & Environment**
```bash
echo $RISCV_PATH  # Should output /opt/riscv (or your custom path)
riscv64-unknown-elf-gcc --version
```
*(If RISCV_PATH is empty, run `export RISCV_PATH=/opt/riscv` first)*

**2. Verify New RVV 1.0 Features (Fractional LMUL)**
```bash
# Compile
$RISCV_PATH/bin/riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -o tests/test_rvv_features tests/test_rvv_features.c
# Run (Expect "vl=2" for mf2)
qemu-riscv64 -cpu rv64,v=true,vlen=128 tests/test_rvv_features
```

**3. Verify Tuple Types & Segmented Operations**
```bash
# Compile
$RISCV_PATH/bin/riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -o tests/test_tuples tests/test_tuples.c
# Run
qemu-riscv64 -cpu rv64,v=true,vlen=128 tests/test_tuples
```

**4. Verify Auto-Vectorization**
```bash
# Compile to assembly
$RISCV_PATH/bin/riscv64-unknown-elf-gcc -O3 -march=rv64gcv -mabi=lp64d -S -o tests/test_autovec.s tests/test_autovec.c
# Check for vector instructions (should show 'vle32.v', 'vadd.vv', etc.)
grep -E "vle|vadd|vse" tests/test_autovec.s
```

### New Features Enabled
*   **Auto-Vectorization**: `-O3 -march=rv64gcv` now automatically vectorizes standard loops.
*   **RVV 1.0 Intrinsics**: Full support for fractional LMUL (`mf2`) and Tuple types (`vfloat32m1x2_t`).
*   **Tests**: See `tests/test_rvv_features.c` and `tests/test_tuples.c` for examples.


## 📄 License
[MIT](LICENSE)