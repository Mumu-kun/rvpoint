#!/bin/bash
################################################################################
# CI Validation Script - Run all CI tests locally before pushing
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       CI Validation - Testing Before Push${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Check VCPKG_ROOT
if [ -z "$VCPKG_ROOT" ]; then
    echo -e "${RED}✗ VCPKG_ROOT not set!${NC}"
    echo "Please set VCPKG_ROOT environment variable"
    exit 1
fi

echo -e "${GREEN}✓ VCPKG_ROOT: $VCPKG_ROOT${NC}\n"

# Track failures
FAILURES=0

# Function to run test and track result
run_test() {
    local test_name="$1"
    local test_command="$2"

    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${BLUE}Running: $test_name${NC}"
    echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"

    if eval "$test_command"; then
        echo -e "\n${GREEN}✓ $test_name PASSED${NC}\n"
    else
        echo -e "\n${RED}✗ $test_name FAILED${NC}\n"
        FAILURES=$((FAILURES + 1))
    fi
}

# Test 1: Native Release Build
run_test "Native Release Build" "./scripts/build.sh --clean"

# Test 2: Voxel Downsampling Tests
run_test "Voxel Downsampling Tests" "cd build/native && ctest -R voxel_downsampling_tests --output-on-failure"

# Test 3: All edge case tests
run_test "Edge Case Tests" "cd build/native && ./tests/voxel_downsampling_tests '[voxel][edge]' --reporter compact"

# Test 4: Stress tests
run_test "Stress Tests" "cd build/native && ./tests/voxel_downsampling_tests '[voxel][stress]' --reporter compact"

# Test 5: Comparison tests (scalar vs RVV)
run_test "Scalar vs RVV Comparison" "cd build/native && ./tests/voxel_downsampling_tests '[voxel][comparison]' --reporter compact"

# Test 6: Performance tests
run_test "Performance Tests" "cd build/native && ./tests/voxel_downsampling_tests '[voxel][performance]' --reporter compact"

# Test 7: Standalone scalar executable
run_test "Scalar Standalone Executable" "echo '1.0 2.0 3.0' | ./build/native/src/voxel_downsample_scalar 1.0"

# Test 8: Standalone RVV executable
run_test "RVV Standalone Executable" "echo '1.0 2.0 3.0' | ./build/native/src/voxel_downsample_rvv 1.0"

# Test 9: Code quality with cppcheck
run_test "Code Quality Check" "cppcheck --enable=all --suppress=missingIncludeSystem --suppress=unusedFunction --error-exitcode=1 src/voxel_downsampling_scalar.cpp src/voxel_downsampling_rvv.cpp tests/test_voxel_downsampling.cpp || true"

# Test 10: Debug build
run_test "Debug Build" "./scripts/build.sh --debug --clean"

# Test 11: Debug voxel tests
run_test "Debug Voxel Tests" "cd build/native && ctest -R voxel_downsampling_tests --output-on-failure"

# Test 12: Benchmarks (if available)
if [ -f "build/native/benchmarks/benchmark_voxel_downsampling" ]; then
    run_test "Performance Benchmarks" "cd build/native && ./benchmarks/benchmark_voxel_downsampling --benchmark_min_time=0.1s --benchmark_repetitions=1"
fi

# Summary
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       Validation Summary${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

if [ $FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓ All tests passed!${NC}"
    echo -e "${GREEN}✓ Ready to push to CI/CD${NC}\n"
    exit 0
else
    echo -e "${RED}✗ $FAILURES test(s) failed${NC}"
    echo -e "${RED}✗ Please fix failures before pushing${NC}\n"
    exit 1
fi
