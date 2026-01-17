# RVPoint: RISC-V Vector Optimized Point Cloud Library

**RVPoint** is a specialized Point Cloud Library (PCL) built for **RISC-V** architectures, leveraging **RVGe** (RISC-V Vector Extension) intrinsics to accelerate core 3D processing algorithms. 

Optimized for **RV64GCV** (targeting `v0.10+` vector specs).

## 🚀 Key Features

This library implements 5 core point cloud processing algorithms, each with a highly optimized **RVV** path alongside a standard Scalar reference path.

| Algorithm | Function Name | RVV Optimization |
| :--- | :--- | :--- |
| **Voxel Grid Downsampling** | `voxel_grid_downsamp_rvv` | Vectorized coordinate scaling & projection. |
| **Statistical Outlier Removal** | `sor_rvv` | Accelerated K-NN distance calculation using `get_dist_sq_rvv` kernel. |
| **Normal Estimation** | `normal_estimation_rvv` | Accelerated neighbor search for covariance matrix building. |
| **Radius Search** | `radius_search_rvv` | Vectorized global distance scan & filter mask. |
| **RANSAC Plane Fitting** | `ransac_plane_rvv` | High-throughput inlier counting using vector masks (`vmfle`, `vcpop`). |

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
    *   *This automatically sets up the GCC 13+ RISC-V Toolchain, QEMU, CMake, and all dependencies.*

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

## 📄 License
[MIT](LICENSE)
