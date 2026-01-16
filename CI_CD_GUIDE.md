# CI/CD Guide for Voxel Downsampling

## Overview

This project uses GitHub Actions for continuous integration and testing. The CI/CD pipeline ensures code quality, correctness, and performance for the voxel downsampling implementations.

## CI/CD Workflows

### 1. Main CI Workflow (`.github/workflows/ci.yml`)

The main CI workflow runs on every push to `main` and `dev` branches, and on pull requests to `main`.

**Jobs:**

- **native-build**: Builds and tests on native x86_64 architecture
  - Matrix: Release and Debug builds
  - Runs all unit tests including voxel downsampling tests
  - Validates standalone executables

- **riscv-build**: Cross-compiles for RISC-V architecture
  - Matrix: Release/Debug × Scalar/RVV
  - Tests with QEMU emulation
  - Validates RISC-V specific optimizations

- **voxel-downsampling-tests**: Comprehensive voxel downsampling validation
  - Matrix: Release/Debug × Multiple leaf sizes (0.5, 1.0, 2.0, 5.0)
  - Tests all edge cases and scenarios
  - Validates performance requirements

- **performance-tests**: Benchmark regression testing
  - Runs Google Benchmark suite
  - Generates performance metrics
  - Uploads benchmark results as artifacts

- **code-quality**: Static analysis and formatting checks
  - cppcheck for code quality
  - clang-format for style consistency
  - Specific checks for voxel downsampling code

### 2. Voxel Downsampling CI (`.github/workflows/voxel_ci.yml`)

A dedicated workflow that only runs when voxel downsampling files change.

**Jobs:**

- **quick-check**: Fast validation that required files exist
- **voxel-tests**: Comprehensive test suite
  - Matrix: Release/Debug × GCC/Clang
  - All test categories: edge cases, comparisons, stress tests
  - Standalone executable validation
- **performance**: Dedicated performance benchmarks
- **quality**: Code quality specific to voxel downsampling
- **ci-success**: Summary job that ensures all tests passed

## Local Validation

### Validate Before Push

Run the validation script before pushing to ensure CI will pass:

```bash
export VCPKG_ROOT=/opt/vcpkg  # Set your vcpkg path
./scripts/validate_ci.sh
```

This script runs:
1. Native Release build
2. All voxel downsampling tests
3. Edge case tests
4. Stress tests
5. Scalar vs RVV comparison tests
6. Performance tests
7. Standalone executable tests
8. Code quality checks (cppcheck)
9. Debug build
10. Debug tests
11. Benchmarks (if available)

### Manual Build and Test

```bash
# Build Release
./scripts/build.sh

# Build Debug
./scripts/build.sh --debug

# Build for RISC-V
./scripts/build.sh --riscv

# Build for RISC-V with RVV
./scripts/build.sh --riscv --rvv

# Run specific tests
cd build/native
ctest -R voxel_downsampling_tests --output-on-failure

# Run specific test categories
./tests/voxel_downsampling_tests '[voxel][edge]'
./tests/voxel_downsampling_tests '[voxel][comparison]'
./tests/voxel_downsampling_tests '[voxel][stress]'

# Run benchmarks
./benchmarks/benchmark_voxel_downsampling
```

## Test Categories

### Unit Tests ([`tests/test_voxel_downsampling.cpp`](tests/test_voxel_downsampling.cpp))

| Tag | Description | Test Count |
|-----|-------------|------------|
| `[voxel][scalar]` | Basic functionality tests | 7 |
| `[voxel][comparison]` | Scalar vs RVV equivalence | 2 |
| `[voxel][edge]` | Boundary and edge cases | 4 |
| `[voxel][realistic]` | Real-world scenarios | 1 |
| `[voxel][stress]` | Large dataset tests | 1 |
| `[voxel][performance]` | Performance validation | 1 |
| `[voxel][accuracy]` | Centroid accuracy | 1 |
| `[voxel][stability]` | Memory and stability | 1 |

### Test Scenarios

**Basic Tests:**
- Empty input
- Single point
- Two points in same/different voxels
- Grid patterns
- Negative coordinates
- Various leaf sizes

**Edge Cases:**
- Very small leaf sizes (0.01)
- Very large leaf sizes (1000.0)
- Extreme coordinates (±1000)
- Voxel boundary precision
- Duplicate points
- Variable density clouds

**Stress Tests:**
- 1 million points
- Multiple consecutive operations
- Chained downsampling

**Performance Tests:**
- Reduction ratio validation
- Different leaf sizes
- Benchmark suite (1K, 10K, 100K, 1M points)

## CI/CD Best Practices

### Before Committing

1. Run `./scripts/validate_ci.sh` to test locally
2. Ensure all tests pass
3. Check code quality with cppcheck
4. Review benchmark results if performance-critical

### Automated Validation

The `git_push.sh` script now includes automatic CI validation:

```bash
./git_push.sh  # Automatically runs validation before commit
```

### CI Failure Debugging

If CI fails:

1. **Check the workflow logs** in GitHub Actions
2. **Download test artifacts** from failed jobs
3. **Reproduce locally**:
   ```bash
   # For native-build failures
   ./scripts/build.sh
   cd build/native
   ctest --output-on-failure -V

   # For specific test failures
   ./tests/voxel_downsampling_tests '[failed-tag]' -v
   ```

4. **Check VCPKG_ROOT**: Ensure environment variable is set correctly
5. **Verify dependencies**: All required packages installed

### Common CI Issues and Fixes

| Issue | Solution |
|-------|----------|
| VCPKG_ROOT not set | Added `env:` block with VCPKG_ROOT in workflow |
| Tests not running | Added explicit ctest commands with `-R` filter |
| Benchmark failures | Reduced min_time and repetitions for CI |
| Code quality failures | Run `cppcheck` locally before push |
| Standalone exec fails | Test with `echo` pipe locally |

## Performance Benchmarks

### Benchmark Configuration

- **Min time**: 0.2s per benchmark in CI (for speed)
- **Repetitions**: 5 (to get stable averages)
- **Output format**: JSON for automated analysis
- **Test sizes**: 1K, 10K, 100K, 1M points

### Expected Performance

- **Scalar**: Baseline performance using hash map
- **RVV**: 2.85x-17x speedup over scalar (depending on data)

### Viewing Benchmark Results

Benchmark results are uploaded as artifacts in GitHub Actions:

1. Go to Actions tab
2. Select the workflow run
3. Download "benchmark-results" artifact
4. View `benchmark_results.json`

## Continuous Improvement

### Adding New Tests

1. Add test cases to [`tests/test_voxel_downsampling.cpp`](tests/test_voxel_downsampling.cpp)
2. Use appropriate tags: `[voxel][your-category]`
3. Run locally: `./tests/voxel_downsampling_tests '[your-category]'`
4. Ensure CI includes your test category

### Adding New Benchmarks

1. Add benchmarks to [`benchmarks/benchmark_voxel_downsampling.cpp`](benchmarks/benchmark_voxel_downsampling.cpp)
2. Register with Google Benchmark
3. Test locally: `./benchmarks/benchmark_voxel_downsampling`
4. CI will automatically include new benchmarks

### Modifying CI Workflows

When modifying workflows:
1. Test changes on a branch first
2. Use `workflow_dispatch` for manual testing
3. Check syntax with GitHub Actions validator
4. Ensure all matrix combinations work

## CI/CD Checklist

Before pushing voxel downsampling changes:

- [ ] All unit tests pass locally
- [ ] Edge case tests pass
- [ ] Stress tests complete successfully
- [ ] Scalar and RVV produce identical results
- [ ] Benchmarks show expected performance
- [ ] Code passes cppcheck
- [ ] Standalone executables work correctly
- [ ] `validate_ci.sh` passes completely
- [ ] Documentation updated if needed

## Additional Resources

- [GitHub Actions Documentation](https://docs.github.com/en/actions)
- [Catch2 Documentation](https://github.com/catchorg/Catch2)
- [Google Benchmark Guide](https://github.com/google/benchmark)
- [Testing Guide](TESTING_GUIDE.md)
- [Quickstart Guide](QUICKSTART.md)

## Support

If CI issues persist:
1. Review CI logs in GitHub Actions
2. Check this guide for common issues
3. Run validation script locally
4. Open an issue with CI logs attached
