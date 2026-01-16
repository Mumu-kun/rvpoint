# Pre-Push Testing Guide

## Quick Start - Test Before Pushing

Before pushing any changes to the repository, run this command to validate everything locally:

```bash
export VCPKG_ROOT=/opt/vcpkg  # Set your vcpkg path
./scripts/pre_push_validation.sh
```

This will run **all the same tests** that CI/CD runs, catching failures before you push.

---

## What Gets Tested

The validation script runs these critical checks:

### ✅ Code Quality (What Was Failing)
```bash
cppcheck --enable=all \
  --suppress=missingIncludeSystem \
  --suppress=unusedFunction \
  --suppress=useStlAlgorithm \
  --suppress=unmatchedSuppression \
  --inline-suppr \
  --error-exitcode=1 \
  -I include \
  src/voxel_downsampling_scalar.cpp \
  src/voxel_downsampling_rvv.cpp \
  tests/test_voxel_downsampling.cpp \
  benchmarks/benchmark_voxel_downsampling.cpp
```

**This was the main CI failure** - ODR violation with `Point3D` defined in multiple files.
✅ **FIXED**: Now using shared header `include/rvpoint/point3d.hpp`

### ✅ Build Tests
- Clean Release build
- Clean Debug build
- All unit tests
- Voxel downsampling specific tests
- Edge case tests
- Scalar vs RVV comparison tests
- Stress tests (1M points)
- Standalone executables

### ✅ Optional Tests (warnings only)
- Code formatting (clang-format)
- Performance benchmarks
- Memory leak check (if valgrind available)

---

## Step-by-Step: Fix All Tests Before Pushing

### Step 1: Set Environment
```bash
export VCPKG_ROOT=/opt/vcpkg  # Or wherever your vcpkg is installed
```

### Step 2: Run Validation
```bash
./scripts/pre_push_validation.sh
```

### Step 3: Fix Any Failures

If tests fail, the script will show exactly what failed. Common issues:

#### cppcheck Errors
```
error: The one definition rule is violated
```
**Fix**: Use shared headers, don't duplicate struct definitions

#### Test Failures
```
REQUIRE( result_scalar.size() == 1 )
with expansion:
  2 == 1
```
**Fix**: Check test expectations match actual voxel boundary behavior

#### Build Failures
```
error: 'Point3D' was not declared in this scope
```
**Fix**: Add `#include "rvpoint/point3d.hpp"` to your file

### Step 4: Push When Green
```bash
✓✓✓ ALL CRITICAL TESTS PASSED! ✓✓✓
✓ Ready to push to CI/CD
```

Now you can safely push:
```bash
git add -A
git commit -m "Your message"
git push origin dev
```

---

##  Manual Testing (Alternative)

If you don't want to run the full script, here are the critical commands:

### 1. Check for ODR Violations
```bash
# Should NOT find Point3D definitions in these files (except with BUILD_STANDALONE)
grep -n 'struct Point3D' \
  src/voxel_downsampling_scalar.cpp \
  src/voxel_downsampling_rvv.cpp \
  tests/test_voxel_downsampling.cpp \
  benchmarks/benchmark_voxel_downsampling.cpp
```

### 2. Run cppcheck
```bash
cppcheck --enable=all \
  --suppress=missingIncludeSystem \
  --suppress=unusedFunction \
  --suppress=useStlAlgorithm \
  --suppress=unmatchedSuppression \
  --error-exitcode=1 \
  -I include \
  src/voxel_downsampling_scalar.cpp \
  src/voxel_downsampling_rvv.cpp \
  tests/test_voxel_downsampling.cpp
```

### 3. Build and Test
```bash
./scripts/build.sh --clean
cd build/native
ctest --output-on-failure
```

### 4. Test Specific Categories
```bash
cd build/native

# Edge cases
./tests/voxel_downsampling_tests '[voxel][edge]'

# Scalar vs RVV comparison
./tests/voxel_downsampling_tests '[voxel][comparison]'

# Stress test
./tests/voxel_downsampling_tests '[voxel][stress]'
```

---

## Understanding Test Failures

### Example: Voxel Boundary Test

**Failed Test:**
```cpp
std::vector<Point3D> input = {
    Point3D(0.0f, 0.0f, 0.0f),
    Point3D(5.0f, 5.0f, 5.0f),
    Point3D(10.0f, 10.0f, 10.0f)  // This is in a DIFFERENT voxel!
};
float leaf_size = 10.0f;
REQUIRE(result.size() == 1);  // WRONG - expects all in same voxel
```

**Why It Fails:**
- Voxel for 0.0: floor(0.0/10.0) = 0 → voxel [0,10)
- Voxel for 5.0: floor(5.0/10.0) = 0 → voxel [0,10)
- Voxel for 10.0: floor(10.0/10.0) = 1 → voxel [10,20) ← Different!

**Fixed Test:**
```cpp
std::vector<Point3D> input = {
    Point3D(0.0f, 0.0f, 0.0f),
    Point3D(5.0f, 5.0f, 5.0f),
    Point3D(9.9f, 9.9f, 9.9f)  // Now in same voxel
};
float leaf_size = 10.0f;
REQUIRE(result.size() == 1);  // CORRECT
```

---

## What CI/CD Runs

When you push, GitHub Actions runs these workflows:

### Main CI (`ci.yml`)
- **native-build**: x86_64 builds (Release + Debug)
- **riscv-build**: RISC-V cross-compilation (4 configs)
- **voxel-downsampling-tests**: 8 matrix jobs
- **performance-tests**: Benchmarks
- **code-quality**: cppcheck + clang-format

### Voxel CI (`voxel_ci.yml`)
- **quick-check**: File validation
- **voxel-tests**: Comprehensive (4 configs)
- **performance**: Benchmarks
- **quality**: Code quality
- **ci-success**: Summary

**Total: ~21 jobs**

---

## CI/CD Issues Fixed

### Issue #1: ODR Violation ✅
**Error:**
```
error: The one definition rule is violated, different classes/structs
have the same name 'Point3D' [ctuOneDefinitionRuleViolation]
```

**Cause:** `Point3D` defined identically in 4 files

**Fix:**
- Created `include/rvpoint/point3d.hpp`
- All files now use `#include "rvpoint/point3d.hpp"`
- Standalone builds use `BUILD_STANDALONE` guard

**Files Changed:**
- ✅ `src/voxel_downsampling_scalar.cpp`
- ✅ `src/voxel_downsampling_rvv.cpp`
- ✅ `tests/test_voxel_downsampling.cpp`
- ✅ `benchmarks/benchmark_voxel_downsampling.cpp`

### Issue #2: Test Expectations Wrong ✅
**Error:**
```
REQUIRE( result_scalar.size() == 1 )
with expansion:
  2 == 1
```

**Cause:** Tests didn't account for voxel boundaries correctly

**Fix:** Adjusted test values to stay within voxel boundaries

### Issue #3: VCPKG_ROOT Not Available ✅
**Error:** Build couldn't find vcpkg dependencies

**Fix:** Added `env: VCPKG_ROOT: /opt/vcpkg` to CI workflow steps

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `./scripts/pre_push_validation.sh` | Run all CI tests locally |
| `./scripts/build.sh --clean` | Clean Release build |
| `./scripts/build.sh --debug --clean` | Clean Debug build |
| `cd build/native && ctest` | Run all tests |
| `./tests/voxel_downsampling_tests '[tag]'` | Run specific test category |
| `cppcheck ... src/*.cpp` | Check code quality |

---

## Success Criteria

Before pushing, ensure:

- [ ] All critical tests pass in validation script
- [ ] No cppcheck errors
- [ ] No ODR violations (shared headers used)
- [ ] All unit tests pass
- [ ] Edge case tests pass
- [ ] Scalar vs RVV produce identical results
- [ ] Standalone executables work

---

## Need Help?

- **CI/CD Guide**: See [CI_CD_GUIDE.md](CI_CD_GUIDE.md)
- **Testing Guide**: See [TESTING_GUIDE.md](TESTING_GUIDE.md)
- **Quick Start**: See [QUICKSTART.md](QUICKSTART.md)
- **Fix Summary**: See [CI_FIX_SUMMARY.md](CI_FIX_SUMMARY.md)

---

**Last Updated:** 2026-01-16
**Status:** All CI/CD issues fixed ✅
**Latest Commits:** `ec91014`, `ae58aea`, `9963bc5`
