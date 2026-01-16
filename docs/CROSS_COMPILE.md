# RISC-V Cross-Compilation Guide

This document explains how to cross-compile the RVPoint project for RISC-V architecture.

## Overview

The project uses:
- **vcpkg** for dependency management (spdlog, tl-expected, catch2, benchmark)
- **riscv64-linux-gnu-gcc/g++** for cross-compilation
- **QEMU** or **Spike** for testing
- Custom vcpkg triplet for RISC-V targets

## Files Structure

```
rvpoint/
├── cmake/
│   └── riscv64-linux-gnu.cmake    # RISC-V toolchain file
├── triplets/
│   └── riscv64-linux.cmake        # vcpkg RISC-V triplet
├── scripts/
│   └── build.sh                    # Build script
└── .devcontainer/
    ├── Dockerfile                  # Dev container with tools
    └── devcontainer.json           # Container configuration
```

## Quick Start

### Native Build (x86_64)
```bash
./scripts/build.sh
```

### RISC-V Cross-Compilation (Scalar)
```bash
./scripts/build.sh --riscv
```

### RISC-V with Vector Extension (RVV)
```bash
./scripts/build.sh --riscv --rvv
```

### Clean Build
```bash
./scripts/build.sh --riscv --clean
```

## How It Works

### 1. vcpkg Integration

The build system uses vcpkg's chainloading feature:

1. Main toolchain: `vcpkg.cmake` (from vcpkg)
2. Chainloaded toolchain: `cmake/riscv64-linux-gnu.cmake` (RISC-V specific)

**Build script flow:**
```bash
cmake -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
      -DVCPKG_TARGET_TRIPLET=riscv64-linux \
      -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake
```

### 2. Custom Triplet

The `triplets/riscv64-linux.cmake` file tells vcpkg:
- Target architecture: `riscv64`
- System: `Linux`
- Compiler toolchain to use

```cmake
set(VCPKG_TARGET_ARCHITECTURE riscv64)
set(VCPKG_CMAKE_SYSTEM_NAME Linux)
set(VCPKG_CHAINLOAD_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/riscv64-linux-gnu.cmake)
```

### 3. RISC-V Toolchain

The `cmake/riscv64-linux-gnu.cmake` configures:
- Compilers: `riscv64-linux-gnu-gcc/g++`
- System root: `/usr/riscv64-linux-gnu`
- QEMU/Spike emulator for testing

### 4. Compiler Flags

The main `CMakeLists.txt` sets architecture flags:
- Scalar: `-march=rv64gc`
- Vector: `-march=rv64gcv` (when `--rvv` is used)

## Testing

Tests automatically run in QEMU after building:

```bash
cd build
ctest --output-on-failure
```

Or use the alias:
```bash
test-qemu
```

## Troubleshooting

### Issue: Dependencies not found

**Problem:** vcpkg can't find dependencies for RISC-V

**Solution:** Install dependencies for the RISC-V triplet:
```bash
vcpkg install spdlog:riscv64-linux
vcpkg install tl-expected:riscv64-linux
vcpkg install catch2:riscv64-linux
vcpkg install benchmark:riscv64-linux
```

Or use manifest mode (automatic):
```bash
cmake -B build --preset riscv
```

### Issue: Compiler not found

**Problem:** `riscv64-linux-gnu-gcc` not in PATH

**Solution:** Install the cross-compiler:
```bash
apt-get install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu
```

### Issue: Tests fail to run

**Problem:** QEMU not found or not configured

**Solution:**
1. Install QEMU: `apt-get install qemu-user`
2. Verify: `qemu-riscv64 --version`

### Issue: Wrong compiler being used

**Problem:** CMake uses native gcc instead of riscv64-linux-gnu-gcc

**Solution:** Make sure you're using the `--riscv` flag:
```bash
./scripts/build.sh --riscv
```

Or manually specify:
```bash
cmake -B build \
  -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_TARGET_TRIPLET=riscv64-linux \
  -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake
```

## Manual Build (Without Script)

If you prefer to run CMake directly:

```bash
# Configure
cmake -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_TARGET_TRIPLET=riscv64-linux \
  -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_RVV=OFF

# Build
cmake --build build --parallel

# Test
cd build && ctest --output-on-failure
```

## Environment Variables

Key environment variables (set in `.devcontainer/devcontainer.json`):

- `VCPKG_ROOT=/opt/vcpkg` - vcpkg installation directory
- `VCPKG_DEFAULT_TRIPLET=x64-linux` - Default triplet for native builds
- `VCPKG_OVERLAY_TRIPLETS=/workspace/triplets` - Custom triplets directory

## Architecture Flags

### Scalar (rv64gc)
```
-march=rv64gc
```
Includes:
- **I**: Integer base
- **M**: Integer multiplication/division
- **A**: Atomic instructions
- **F**: Single-precision floating-point
- **D**: Double-precision floating-point
- **C**: Compressed instructions

### Vector (rv64gcv)
```
-march=rv64gcv
```
Adds:
- **V**: Vector extension (RVV)

QEMU configuration:
```
qemu-riscv64 -cpu rv64,v=true,vlen=256
```

## References

- [vcpkg Documentation](https://vcpkg.io/)
- [CMake Toolchain Files](https://cmake.org/cmake/help/latest/manual/cmake-toolchains.7.html)
- [RISC-V GNU Toolchain](https://github.com/riscv-collab/riscv-gnu-toolchain)
- [QEMU RISC-V](https://www.qemu.org/docs/master/system/target-riscv.html)
