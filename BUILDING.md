# Building RVPoint

Quick reference for building the RVPoint project.

## Prerequisites

- CMake 3.15+
- Ninja build system
- vcpkg (for dependencies)
- For RISC-V: `riscv64-linux-gnu-gcc/g++`, QEMU

## Build Commands

### Native Build (x86_64)
```bash
./scripts/build.sh
```

### RISC-V Cross-Compilation
```bash
# Scalar version (rv64gc)
./scripts/build.sh --riscv

# With RVV (RISC-V Vector extension)
./scripts/build.sh --riscv --rvv

# Debug build
./scripts/build.sh --riscv --debug

# Clean build
./scripts/build.sh --riscv --clean
```

### Using Aliases (in dev container)
```bash
build           # Native build
build-riscv     # RISC-V scalar
build-rvv       # RISC-V with vectors
test-qemu       # Run tests in QEMU
```

## Manual CMake Commands

### Native Build
```bash
cmake -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build --parallel
```

### RISC-V Cross-Compilation
```bash
cmake -B build -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
  -DVCPKG_TARGET_TRIPLET=riscv64-linux \
  -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_RVV=OFF

cmake --build build --parallel
```

## Build Options

| Option | Description | Default |
|--------|-------------|---------|
| `--debug` | Build in Debug mode | Release |
| `--rvv` | Enable RISC-V Vector extension | OFF |
| `--riscv` | Cross-compile for RISC-V | Native |
| `--spike` | Use Spike emulator | QEMU |
| `--clean` | Clean before building | No |

## CMake Options

| CMake Variable | Description | Default |
|----------------|-------------|---------|
| `BUILD_TESTS` | Build unit tests | ON |
| `BUILD_EXAMPLES` | Build examples | ON |
| `BUILD_BENCHMARKS` | Build benchmarks | ON |
| `ENABLE_RVV` | Enable RVV support | OFF |
| `USE_CUSTOM_ISA` | Enable custom instructions | OFF |

## Testing

```bash
# After building
cd build
ctest --output-on-failure

# Or use alias (in dev container)
test-qemu
```

## Troubleshooting

### Dependencies not building for RISC-V

The first RISC-V build will take longer as vcpkg compiles all dependencies from source. Be patient!

### "Compiler not found" error

Make sure you're in the dev container or have installed:
```bash
apt-get install gcc-riscv64-linux-gnu g++-riscv64-linux-gnu
```

### Tests fail in QEMU

Install QEMU user-mode emulation:
```bash
apt-get install qemu-user
```

## More Information

- [Cross-Compilation Guide](docs/CROSS_COMPILE.md) - Detailed explanation
- [Architecture Documentation](docs/ARCHITECTURE.md) - Project structure
- [Contributing Guide](CONTRIBUTING.md) - Development workflow

## Quick Start (Complete Workflow)

```bash
# 1. Open in dev container (VS Code)
# 2. Build for RISC-V with vectors
./scripts/build.sh --riscv --rvv

# 3. Tests run automatically
# 4. Check the build output
ls -lh build/
```

That's it! The build system handles everything else automatically.
