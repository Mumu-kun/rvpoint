# Building PCL-RISC-V

This guide covers building and testing PCL-RISC-V on various platforms.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Native Build (x86/ARM)](#native-build)
- [RISC-V Cross-Compilation](#riscv-cross-compilation)
- [Docker Build](#docker-build)
- [Build Options](#build-options)
- [Running Tests](#running-tests)
- [Troubleshooting](#troubleshooting)

## Prerequisites

### Ubuntu/Debian

**For native builds:**
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    git
```

**For RISC-V cross-compilation:**
```bash
sudo apt install -y \
    gcc-riscv64-linux-gnu \
    g++-riscv64-linux-gnu \
    qemu-user \
    qemu-system-misc
```

**For Spike (optional, for validation):**
```bash
sudo apt install -y spike device-tree-compiler

# Or build from source:
git clone https://github.com/riscv-software-src/riscv-isa-sim.git
cd riscv-isa-sim
mkdir build && cd build
../configure --prefix=/usr/local
make -j$(nproc)
sudo make install
```

## Native Build

For quick development iteration on your host machine:

```bash
# Clone repository
git clone https://github.com/yourusername/pcl-riscv.git
cd pcl-riscv

# Configure and build
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build

# Run tests
cd build && ctest --output-on-failure
```

## RISC-V Cross-Compilation

### Using the build script (recommended):

```bash
# Scalar version
./scripts/build.sh --riscv

# With RVV support
./scripts/build.sh --riscv --rvv

# Debug build with RVV
./scripts/build.sh --riscv --rvv --debug
```

### Manual CMake configuration:

```bash
# Configure
cmake -B build -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_RVV=ON \
    -DBUILD_TESTS=ON

# Build
cmake --build build

# Test (uses QEMU automatically)
cd build && ctest --output-on-failure
```

### Using Spike instead of QEMU:

```bash
cmake -B build \
    -DCMAKE_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake \
    -DUSE_SPIKE_EMULATOR=ON \
    -DENABLE_RVV=ON
cmake --build build
cd build && ctest
```

## Docker Build

For a reproducible environment:

```bash
# Build Docker image
docker build -t pcl-riscv -f docker/Dockerfile .

# Run container
docker run -it -v $(pwd):/workspace pcl-riscv

# Inside container, build project
cd /workspace
./scripts/build.sh --riscv --rvv
```

## Build Options

Available CMake options:

| Option | Values | Default | Description |
|--------|--------|---------|-------------|
| `CMAKE_BUILD_TYPE` | Debug, Release | Release | Build configuration |
| `ENABLE_RVV` | ON, OFF | OFF | Enable RISC-V Vector extension |
| `USE_CUSTOM_ISA` | ON, OFF | OFF | Enable custom PCL instructions |
| `BUILD_TESTS` | ON, OFF | ON | Build unit tests |
| `BUILD_EXAMPLES` | ON, OFF | ON | Build example programs |
| `BUILD_BENCHMARKS` | ON, OFF | ON | Build benchmarks |
| `USE_SPIKE_EMULATOR` | ON, OFF | OFF | Use Spike instead of QEMU |

Example with multiple options:

```bash
cmake -B build \
    -DCMAKE_BUILD_TYPE=Debug \
    -DENABLE_RVV=ON \
    -DBUILD_EXAMPLES=ON \
    -DUSE_SPIKE_EMULATOR=OFF
```

## Running Tests

### All tests:
```bash
cd build && ctest
```

### Specific test:
```bash
./build/tests/pcl_tests --gtest_filter=PointCloudTest.AddPoints
```

### Verbose output:
```bash
cd build && ctest --verbose --output-on-failure
```

### List available tests:
```bash
./build/tests/pcl_tests --gtest_list_tests
```

## Running Examples

```bash
# Basic usage
./build/examples/example_basic

# Point types demonstration
./build/examples/example_point_types

# Backend comparison
./build/examples/example_backends
```

## Running Benchmarks

```bash
./build/benchmarks/benchmark_suite
```

## Troubleshooting

### QEMU not found

**Error:** `CMAKE_CROSSCOMPILING_EMULATOR` not set

**Solution:**
```bash
sudo apt install qemu-user
# Verify: qemu-riscv64 --version
```

### Toolchain not found

**Error:** `riscv64-linux-gnu-gcc` not found

**Solution:**
```bash
sudo apt install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu
# Verify: riscv64-linux-gnu-gcc --version
```

### RVV intrinsics not found

**Error:** `riscv_vector.h: No such file or directory`

**Solution:** Ensure your RISC-V toolchain supports RVV. GCC 12+ or Clang 14+ recommended.

```bash
# Check GCC version
riscv64-linux-gnu-gcc --version

# If too old, build from source or use Docker image
```

### Tests fail in QEMU

**Error:** Tests timeout or crash

**Possible causes:**
1. QEMU version too old (need 8.0+ for RVV)
2. Missing RVV CPU features

**Solution:**
```bash
# Check QEMU version
qemu-riscv64 --version

# Explicitly specify CPU with RVV
qemu-riscv64 -cpu rv64,v=true,vlen=256 ./test_program
```

### Spike not found

**Error:** `spike` command not found

**Solution:** Install or build Spike:
```bash
# Option 1: Package manager (if available)
sudo apt install spike

# Option 2: Build from source
git clone https://github.com/riscv-software-src/riscv-isa-sim.git
cd riscv-isa-sim
mkdir build && cd build
../configure --prefix=/usr/local
make -j$(nproc)
sudo make install
```

### Build performance

For faster builds:
```bash
# Use Ninja instead of Make
cmake -B build -G Ninja

# Use ccache
sudo apt install ccache
export CC="ccache gcc"
export CXX="ccache g++"

# Parallel build
cmake --build build --parallel $(nproc)
```

## IDE Integration

### VS Code

Install the CMake Tools extension, then:
1. Open project folder
2. Select kit (gcc or riscv64-linux-gnu-gcc)
3. Select variant (Debug/Release)
4. Build with F7 or click "Build" in status bar

### CLion

1. Open project
2. Go to Settings → Build → Toolchains
3. Add RISC-V toolchain with path to `riscv64-linux-gnu-gcc`
4. Select toolchain in CMake profile

## Next Steps

- Read [API documentation](API.md)
- See [examples/](../examples/) for usage
- Check [CONTRIBUTING.md](CONTRIBUTING.md) for development guidelines
