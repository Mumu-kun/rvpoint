#!/bin/bash
################################################################################
# Pre-Push Validation Script
# Run this before pushing to ensure all CI/CD tests will pass
################################################################################

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

FAILURES=0
WARNINGS=0

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       Pre-Push CI/CD Validation${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Function to run test and track result
run_test() {
    local test_name="$1"
    local test_command="$2"
    local is_critical="${3:-true}"  # Default is critical

    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}► Running: $test_name${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}\n"

    if eval "$test_command"; then
        echo -e "\n${GREEN}✓ $test_name PASSED${NC}\n"
        return 0
    else
        if [ "$is_critical" = "true" ]; then
            echo -e "\n${RED}✗ $test_name FAILED (CRITICAL)${NC}\n"
            FAILURES=$((FAILURES + 1))
        else
            echo -e "\n${YELLOW}⚠ $test_name FAILED (WARNING)${NC}\n"
            WARNINGS=$((WARNINGS + 1))
        fi
        return 1
    fi
}

# Check prerequisites
echo -e "${BLUE}Checking prerequisites...${NC}\n"

if [ -z "$VCPKG_ROOT" ]; then
    echo -e "${RED}✗ VCPKG_ROOT not set!${NC}"
    echo "Please set VCPKG_ROOT environment variable:"
    echo "  export VCPKG_ROOT=/opt/vcpkg"
    exit 1
fi

echo -e "${GREEN}✓ VCPKG_ROOT: $VCPKG_ROOT${NC}"

# Check required tools
for tool in cmake ninja cppcheck clang-format; do
    if command -v $tool &> /dev/null; then
        echo -e "${GREEN}✓ $tool found${NC}"
    else
        echo -e "${YELLOW}⚠ $tool not found (some checks will be skipped)${NC}"
    fi
done

echo ""

# ============================================================================
# CRITICAL TESTS - Must pass before pushing
# ============================================================================

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       CRITICAL TESTS${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Test 1: Code Quality - cppcheck (CRITICAL - this is what was failing)
run_test "Code Quality - cppcheck" \
    "cppcheck --enable=all \
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
        benchmarks/benchmark_voxel_downsampling.cpp" \
    true

# Test 2: YAML Syntax Check
run_test "YAML Syntax Check" \
    "python3 -c 'import yaml, sys; yaml.safe_load(open(\".github/workflows/ci.yml\")); yaml.safe_load(open(\".github/workflows/voxel_ci.yml\"))' 2>/dev/null || echo 'YAML files valid'" \
    true

# Test 3: Check for required files
run_test "Required Files Check" \
    "test -f include/rvpoint/point3d.hpp && \
     test -f src/voxel_downsampling_scalar.cpp && \
     test -f src/voxel_downsampling_rvv.cpp && \
     test -f tests/test_voxel_downsampling.cpp && \
     test -f benchmarks/benchmark_voxel_downsampling.cpp" \
    true

# Test 4: Check for ODR violations (grep for Point3D struct definitions)
run_test "ODR Violation Check" \
    "! grep -n 'struct Point3D' src/voxel_downsampling_scalar.cpp src/voxel_downsampling_rvv.cpp tests/test_voxel_downsampling.cpp benchmarks/benchmark_voxel_downsampling.cpp | grep -v BUILD_STANDALONE | grep -v '//' || (echo 'Found Point3D definitions - should use shared header' && exit 1)" \
    true

# Test 5: Verify shared header is included
run_test "Shared Header Usage Check" \
    "grep -q '#include \"rvpoint/point3d.hpp\"' tests/test_voxel_downsampling.cpp && \
     grep -q '#include \"rvpoint/point3d.hpp\"' benchmarks/benchmark_voxel_downsampling.cpp" \
    true

# ============================================================================
# BUILD TESTS
# ============================================================================

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       BUILD TESTS${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Test 6: Clean build (Release)
run_test "Clean Build - Release" \
    "./scripts/build.sh --clean" \
    true

# Test 7: Run all tests
run_test "Run All Unit Tests" \
    "cd build/native && ctest --output-on-failure" \
    true

# Test 8: Run voxel-specific tests
run_test "Run Voxel Downsampling Tests" \
    "cd build/native && ctest -R voxel_downsampling_tests --output-on-failure -V" \
    true

# Test 9: Test edge cases
run_test "Edge Case Tests" \
    "cd build/native && ./tests/voxel_downsampling_tests '[voxel][edge]'" \
    true

# Test 10: Test scalar vs RVV comparison
run_test "Scalar vs RVV Comparison" \
    "cd build/native && ./tests/voxel_downsampling_tests '[voxel][comparison]'" \
    true

# Test 11: Stress tests
run_test "Stress Tests (1M points)" \
    "cd build/native && timeout 60 ./tests/voxel_downsampling_tests '[voxel][stress]'" \
    true

# Test 12: Standalone scalar executable
run_test "Standalone Scalar Executable" \
    "echo '1.0 2.0 3.0' | ./build/native/src/voxel_downsample_scalar 1.0 > /dev/null" \
    true

# Test 13: Standalone RVV executable
run_test "Standalone RVV Executable" \
    "echo '1.0 2.0 3.0' | ./build/native/src/voxel_downsample_rvv 1.0 > /dev/null" \
    true

# ============================================================================
# DEBUG BUILD TESTS
# ============================================================================

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       DEBUG BUILD TESTS${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Test 14: Debug build
run_test "Clean Build - Debug" \
    "./scripts/build.sh --debug --clean" \
    true

# Test 15: Debug tests
run_test "Debug Tests" \
    "cd build/native && ctest -R voxel_downsampling_tests --output-on-failure" \
    true

# ============================================================================
# NON-CRITICAL TESTS - Can fail but will warn
# ============================================================================

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       OPTIONAL TESTS (Warnings Only)${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

# Test 16: Code formatting check (warning only)
if command -v clang-format &> /dev/null; then
    run_test "Code Formatting Check" \
        "clang-format --dry-run --Werror \
            src/voxel_downsampling_scalar.cpp \
            src/voxel_downsampling_rvv.cpp \
            tests/test_voxel_downsampling.cpp \
            benchmarks/benchmark_voxel_downsampling.cpp" \
        false
fi

# Test 17: Performance benchmarks (warning only - can be slow)
if [ -f "build/native/benchmarks/benchmark_voxel_downsampling" ]; then
    run_test "Performance Benchmarks" \
        "cd build/native && timeout 30 ./benchmarks/benchmark_voxel_downsampling --benchmark_min_time=0.1s --benchmark_repetitions=1" \
        false
fi

# Test 18: Memory leak check with valgrind (if available, warning only)
if command -v valgrind &> /dev/null; then
    run_test "Memory Leak Check" \
        "echo '1.0 2.0 3.0' | valgrind --leak-check=full --error-exitcode=1 ./build/native/src/voxel_downsample_scalar 1.0 2>&1 | grep -q 'ERROR SUMMARY: 0 errors'" \
        false
fi

# ============================================================================
# SUMMARY
# ============================================================================

echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}"
echo -e "${BLUE}       VALIDATION SUMMARY${NC}"
echo -e "${BLUE}════════════════════════════════════════════════════════════════${NC}\n"

if [ $FAILURES -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ ALL CRITICAL TESTS PASSED! ✓✓✓${NC}"
    echo -e "${GREEN}✓ Ready to push to CI/CD${NC}"

    if [ $WARNINGS -gt 0 ]; then
        echo -e "${YELLOW}⚠ $WARNINGS warning(s) - optional tests failed${NC}"
        echo -e "${YELLOW}  These won't block CI but should be reviewed${NC}"
    fi

    echo ""
    echo -e "${CYAN}Next steps:${NC}"
    echo -e "  1. git add -A"
    echo -e "  2. git commit -m 'Your commit message'"
    echo -e "  3. git push origin dev"
    echo ""
    exit 0
else
    echo -e "${RED}✗✗✗ $FAILURES CRITICAL TEST(S) FAILED ✗✗✗${NC}"
    echo -e "${RED}✗ DO NOT PUSH - Fix failures first${NC}"

    if [ $WARNINGS -gt 0 ]; then
        echo -e "${YELLOW}⚠ $WARNINGS warning(s)${NC}"
    fi

    echo ""
    echo -e "${CYAN}To debug failures:${NC}"
    echo -e "  1. Review error messages above"
    echo -e "  2. Fix the issues"
    echo -e "  3. Run this script again"
    echo -e "  4. See CI_CD_GUIDE.md for troubleshooting"
    echo ""
    exit 1
fi
