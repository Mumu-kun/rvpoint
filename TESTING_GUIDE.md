# Voxel Downsampling Testing Guide

Complete guide for testing and benchmarking voxel grid downsampling implementations.

## Quick Start

### Option 1: Using Make (Simplest)

```bash
# Run quick tests (fastest)
make quick

# Run all tests
make test

# Run benchmarks
make benchmark

# Compare implementations
make compare

# Show all options
make help
```

### Option 2: Using Test Script

```bash
# Make script executable (one time)
chmod +x test_voxel.sh

# Run quick tests
./test_voxel.sh quick

# Run all tests
./test_voxel.sh all

# Show all options
./test_voxel.sh help
```

## Available Test Commands

### Using Make

| Command | Description | Time |
|---------|-------------|------|
| `make quick` | Build + unit tests + basic checks | ~30s |
| `make test` | Complete test suite | ~2min |
| `make build` | Build project only | ~20s |
| `make unit` | Unit tests only | ~1s |
| `make benchmark` | Performance benchmarks | ~10s |
| `make compare` | Scalar vs RVV comparison | ~15s |
| `make verify` | Verify correctness | ~1s |
| `make stress` | Stress test (1M points) | ~1min |
| `make clean` | Clean build directory | <1s |

### Using Test Script

```bash
# Quick verification
./test_voxel.sh quick

# Full test suite
./test_voxel.sh all

# Individual test stages
./test_voxel.sh build         # Build only
./test_voxel.sh unit          # Unit tests
./test_voxel.sh ctest         # CTest suite
./test_voxel.sh standalone    # Test executables
./test_voxel.sh benchmark     # Benchmarks
./test_voxel.sh compare       # Performance comparison
./test_voxel.sh verify        # Correctness check
./test_voxel.sh stress        # Stress test
```

## Test Output Examples

### Quick Test Output
```
======================================================================
Quick Test Mode
======================================================================

✓ Build completed successfully
✓ All unit tests passed!
✓ Scalar implementation works (10K points)
  Throughput:    6.45 Mpoints/s
✓ RVV implementation works (10K points)
  Throughput:    17.54 Mpoints/s
✓ Quick tests completed successfully!
```

### Performance Comparison Output
```
======================================================================
Performance Comparison: Scalar vs RVV
======================================================================

Configuration: 100000 points, leaf size=1.0, 5 iterations
----------------------------------------------------------------
Implementation       Time (ms)       Throughput (Mpts/s)
----------------------------------------------------------------
Scalar               13.8            7.23
RVV                  4.84            20.65
----------------------------------------------------------------
Speedup: 2.85x
```

### Unit Test Output
```
======================================================================
Running Unit Tests
======================================================================

All tests passed (9985 assertions in 11 test cases)

Test cases:
  ✓ Voxel downsampling - empty input
  ✓ Voxel downsampling - single point
  ✓ Voxel downsampling - two points in same voxel
  ✓ Voxel downsampling - two points in different voxels
  ✓ Voxel downsampling - multiple points forming a grid
  ✓ Voxel downsampling - negative coordinates
  ✓ Voxel downsampling - larger leaf size
  ✓ Voxel downsampling - random cloud comparison
  ✓ Voxel downsampling - reduction ratio
  ✓ Voxel downsampling - centroid accuracy
  ✓ Voxel downsampling - stress test
```

## Manual Testing

### Building Manually

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake
cmake --build . -j4
```

### Running Tests Manually

```bash
cd build

# Unit tests
./tests/voxel_downsampling_tests

# Specific test case
./tests/voxel_downsampling_tests -c "test name"

# Benchmarks
./benchmarks/benchmark_voxel_downsampling

# Standalone executables
./src/voxel_downsample_scalar 100000 1.0 5
./src/voxel_downsample_rvv 100000 1.0 5
```

## Test Parameters

Standalone executables accept three parameters:
```bash
./voxel_downsample_[scalar|rvv] <num_points> <leaf_size> <iterations>
```

**Examples:**
```bash
# Small test
./src/voxel_downsample_scalar 10000 1.0 3

# Medium test
./src/voxel_downsample_scalar 100000 2.0 5

# Large test
./src/voxel_downsample_scalar 1000000 1.0 3

# Different leaf sizes
./src/voxel_downsample_scalar 100000 0.5 3  # More downsampling
./src/voxel_downsample_scalar 100000 5.0 3  # Less downsampling
```

## Continuous Integration

### Pre-commit Hook

Add to `.git/hooks/pre-commit`:
```bash
#!/bin/bash
./test_voxel.sh quick
if [ $? -ne 0 ]; then
    echo "Tests failed! Commit aborted."
    exit 1
fi
```

### CI Pipeline Example

```yaml
# .github/workflows/test.yml
name: Test
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Run tests
        run: make test
```

## Troubleshooting

### Build Fails

```bash
# Clean and rebuild
make clean
make build
```

### Tests Fail

```bash
# Check specific test
./test_voxel.sh unit

# View detailed output
cd build && ./tests/voxel_downsampling_tests -v
```

### Performance Issues

```bash
# Check build type (should be Release)
cd build && cmake -LA | grep CMAKE_BUILD_TYPE

# Rebuild with optimizations
make clean
make build
```

## Test Coverage

### What's Tested

✓ **Edge Cases**
- Empty input
- Single point
- Negative coordinates
- Various leaf sizes

✓ **Correctness**
- Voxel assignment
- Centroid calculation
- Scalar vs RVV equivalence

✓ **Performance**
- Small clouds (1K points)
- Medium clouds (10K-100K points)
- Large clouds (1M points)
- Different leaf sizes

✓ **Robustness**
- Stress testing
- Memory handling
- Long-running tests

## Performance Targets

| Point Count | Scalar | RVV | Target Speedup |
|-------------|--------|-----|----------------|
| 1K | ~18 Mpts/s | ~60 Mpts/s | 3x+ |
| 10K | ~7 Mpts/s | ~20 Mpts/s | 2.8x+ |
| 100K | ~7 Mpts/s | ~20 Mpts/s | 2.8x+ |

## Additional Resources

- [VOXEL_DOWNSAMPLING_RESULTS.md](VOXEL_DOWNSAMPLING_RESULTS.md) - Detailed results
- [src/voxel_downsampling_scalar.cpp](src/voxel_downsampling_scalar.cpp) - Scalar implementation
- [src/voxel_downsampling_rvv.cpp](src/voxel_downsampling_rvv.cpp) - RVV implementation
- [tests/test_voxel_downsampling.cpp](tests/test_voxel_downsampling.cpp) - Test suite
