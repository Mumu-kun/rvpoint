# Development Script (`dev.sh`)

A smart build and run script for PCL-RISC-V development that automatically detects when rebuilds are needed and uses ccache for faster compilation.

## Features

- **Smart Rebuild Detection**: Only rebuilds when source files have changed
- **Configuration Change Detection**: Automatically rebuilds when CMake options change (e.g., RVV flag)
- **Separate Build Directories**: Maintains separate build caches for different configurations:
  - `build/native`: x86-64 builds
  - `build/riscv`: RISC-V scalar builds (rv64gc)
  - `build/riscv-rvv`: RISC-V vector builds (rv64gcv)
- **ccache Integration**: Speeds up rebuilds by caching compilations
- **Unified Build & Run**: Combines building and running in one command
- **Multi-Architecture Support**: Works with both native (x86-64) and RISC-V builds
- **Flexible Options**: Build-only, run-only, or build-and-run modes

## Quick Start

```bash
# Build and run native tests
./scripts/dev.sh --run-tests

# Build and run RISC-V examples
./scripts/dev.sh --riscv --run-examples

# Build and run benchmarks with RVV enabled
./scripts/dev.sh --riscv --rvv --run-benchmarks

# Force a clean rebuild and run tests
./scripts/dev.sh --clean --run-tests

# Only run tests (skip build)
./scripts/dev.sh --skip-build --run-tests

# Run specific executables (just like run.sh)
./scripts/dev.sh rvpoint_tests
./scripts/dev.sh --riscv error_handling_example
./scripts/dev.sh --riscv --rvv error_handling_example

# Switch between RVV configurations without rebuilding
./scripts/dev.sh --riscv --run-tests        # Uses build/riscv
./scripts/dev.sh --riscv --rvv --run-tests  # Uses build/riscv-rvv
```

## Options

### Build Options
- `--debug`: Build in Debug mode (default: Release)
- `--rvv`: Enable RISC-V Vector extension
- `--riscv`: Cross-compile for RISC-V
- `--spike`: Use Spike instead of QEMU for testing
- `--clean`: Clean build directory before building
- `--force`: Force rebuild even if not needed
- `--skip-build`: Skip build step, only run

### Run Options
- `--run-tests`: Run tests after building
- `--run-examples`: Run examples after building
- `--run-benchmarks`: Run benchmarks after building
- `--run-file FILE`: Run specific executable file after building
- `--run-args ARGS`: Pass additional arguments to run script

## How It Works

1. **ccache Setup**: Automatically detects and configures ccache for faster rebuilds
2. **Build Detection**: Compares timestamps of source files vs build artifacts
3. **Configuration Monitoring**: Checks if CMake options (like RVV) have changed since last build
4. **Separate Directories**: Uses different build directories for different configurations to avoid conflicts
5. **Smart Building**: Only builds when necessary, otherwise skips to save time
6. **Integrated Testing**: Runs the specified targets using the existing run.sh script

## Examples

```bash
# Development workflow: build and test frequently
./scripts/dev.sh --run-tests

# Cross-platform testing: test both architectures
./scripts/dev.sh --run-tests
./scripts/dev.sh --riscv --run-tests

# Performance testing with RVV
./scripts/dev.sh --riscv --rvv --run-benchmarks

# Quick iteration: skip build when you know it's up to date
./scripts/dev.sh --skip-build --run-examples
```</content>
<parameter name="filePath">/workspace/docs/DEV_SCRIPT.md