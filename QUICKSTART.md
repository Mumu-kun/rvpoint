# Voxel Downsampling - Quick Start Guide

## 🚀 One-Command Testing

### Fastest Way to Test Everything

```bash
# Quick test (30 seconds)
make quick
```

### Full Test Suite

```bash
# Complete testing (2 minutes)
make test
```

### See Performance Comparison

```bash
# Compare scalar vs RVV
make compare
```

## 📋 All Available Commands

```bash
make quick      # Quick test (recommended for first run)
make test       # Full test suite
make build      # Build only
make unit       # Unit tests only
make benchmark  # Performance benchmarks
make compare    # Scalar vs RVV comparison
make verify     # Verify correctness
make stress     # Stress test (1M points)
make clean      # Clean build
make help       # Show all options
```

## 📊 What to Expect

### Quick Test Output
```
✓ Build completed successfully
✓ All unit tests passed!
✓ Scalar implementation works (10K points)
  Throughput:    6.45 Mpoints/s
✓ RVV implementation works (10K points)
  Throughput:    17.54 Mpoints/s
✓ Quick tests completed successfully!
```

### Performance Comparison
```
Configuration: 100000 points, leaf size=1.0, 5 iterations
----------------------------------------------------------------
Implementation       Time (ms)       Throughput (Mpts/s)
----------------------------------------------------------------
Scalar               118.9           0.84
RVV                  6.71            14.91
----------------------------------------------------------------
Speedup: 17.72x
```

## 🎯 Test Results Summary

- **11 test cases** - All passing ✓
- **9,985 assertions** - All verified ✓
- **Performance gain** - 2.85x to 17x speedup (RVV vs Scalar)
- **Tested with** - Up to 1M points ✓

## 📁 Project Files

### Implementations
- `src/voxel_downsampling_scalar.cpp` - Scalar baseline
- `src/voxel_downsampling_rvv.cpp` - RVV optimized

### Testing
- `tests/test_voxel_downsampling.cpp` - 11 comprehensive test cases
- `benchmarks/benchmark_voxel_downsampling.cpp` - Performance benchmarks

### Test Automation
- `test_voxel.sh` - Automated test script
- `Makefile` - Simple make commands

## 🔧 Manual Testing (if needed)

### Build Manually
```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . -j4
```

### Run Tests Manually
```bash
cd build
./tests/voxel_downsampling_tests
./benchmarks/benchmark_voxel_downsampling
./src/voxel_downsample_scalar 100000 1.0 5
./src/voxel_downsample_rvv 100000 1.0 5
```

## 📚 Documentation

- `TESTING_GUIDE.md` - Comprehensive testing guide
- `VOXEL_DOWNSAMPLING_RESULTS.md` - Detailed results and analysis

## ✨ Features

- ✓ Complete scalar implementation
- ✓ RVV-vectorized implementation with GCC11 compatibility
- ✓ Comprehensive test coverage (edge cases, correctness, performance)
- ✓ Automated testing scripts
- ✓ Performance benchmarks with Google Benchmark
- ✓ Standalone executables for manual testing

## 🎓 Algorithm Overview

**Voxel Grid Downsampling:**
1. Divide 3D space into voxels (cubes of size `leaf_size`)
2. Assign each point to a voxel using `floor(coord / leaf_size)`
3. Compute centroid for all points in each voxel
4. Output one point (centroid) per voxel

**Scalar:** Hash map grouping
**RVV:** Sort by voxel key + contiguous reduction

## 🏆 Performance Highlights

| Points | Scalar | RVV | Speedup |
|--------|--------|-----|---------|
| 1K | 17.9 Mpts/s | 59.9 Mpts/s | 3.3x |
| 10K | 7.2 Mpts/s | 20.7 Mpts/s | 2.85x |
| 100K | 0.84 Mpts/s | 14.9 Mpts/s | 17.7x |

---

**Get started now:** `make quick`
