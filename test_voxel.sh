#!/bin/bash

################################################################################
# Voxel Downsampling Test Script
# Automated testing and benchmarking for voxel grid downsampling implementations
################################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
BUILD_DIR="build"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "\n${BLUE}======================================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}======================================================================${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

################################################################################
# Build Function
################################################################################

build_project() {
    print_header "Building Project"

    if [ ! -d "$BUILD_DIR" ]; then
        print_info "Creating build directory..."
        mkdir -p "$BUILD_DIR"
    fi

    cd "$BUILD_DIR"

    print_info "Configuring CMake..."
    cmake .. -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake \
        > /dev/null 2>&1

    print_info "Building (this may take a moment)..."
    cmake --build . -j4 > /dev/null 2>&1

    cd ..
    print_success "Build completed successfully"
}

################################################################################
# Unit Tests
################################################################################

run_unit_tests() {
    print_header "Running Unit Tests"

    cd "$BUILD_DIR"

    if [ ! -f "tests/voxel_downsampling_tests" ]; then
        print_error "Test executable not found. Please build first."
        exit 1
    fi

    print_info "Running all test cases..."
    if ./tests/voxel_downsampling_tests; then
        print_success "All unit tests passed!"
    else
        print_error "Some unit tests failed!"
        exit 1
    fi

    cd ..
}

################################################################################
# CTest Integration
################################################################################

run_ctest() {
    print_header "Running CTest Suite"

    cd "$BUILD_DIR"

    print_info "Running CTest..."
    if ctest --output-on-failure; then
        print_success "All CTest tests passed!"
    else
        print_error "Some CTest tests failed!"
        exit 1
    fi

    cd ..
}

################################################################################
# Standalone Executable Tests
################################################################################

test_standalone_executables() {
    print_header "Testing Standalone Executables"

    cd "$BUILD_DIR"

    # Test configurations
    SMALL_TEST="10000 1.0 3"
    MEDIUM_TEST="100000 2.0 3"

    print_info "Testing scalar implementation (10K points)..."
    if ./src/voxel_downsample_scalar $SMALL_TEST > /tmp/scalar_small.log 2>&1; then
        print_success "Scalar implementation works (10K points)"
        grep "Throughput:" /tmp/scalar_small.log
    else
        print_error "Scalar implementation failed"
        cat /tmp/scalar_small.log
        exit 1
    fi

    print_info "Testing RVV implementation (10K points)..."
    if ./src/voxel_downsample_rvv $SMALL_TEST > /tmp/rvv_small.log 2>&1; then
        print_success "RVV implementation works (10K points)"
        grep "Throughput:" /tmp/rvv_small.log
    else
        print_error "RVV implementation failed"
        cat /tmp/rvv_small.log
        exit 1
    fi

    print_info "Testing with larger dataset (100K points)..."
    if ./src/voxel_downsample_scalar $MEDIUM_TEST > /tmp/scalar_medium.log 2>&1; then
        print_success "Scalar handles 100K points"
    else
        print_error "Scalar failed with 100K points"
        exit 1
    fi

    if ./src/voxel_downsample_rvv $MEDIUM_TEST > /tmp/rvv_medium.log 2>&1; then
        print_success "RVV handles 100K points"
    else
        print_error "RVV failed with 100K points"
        exit 1
    fi

    cd ..
}

################################################################################
# Benchmarks
################################################################################

run_benchmarks() {
    print_header "Running Performance Benchmarks"

    cd "$BUILD_DIR"

    if [ ! -f "benchmarks/benchmark_voxel_downsampling" ]; then
        print_error "Benchmark executable not found."
        exit 1
    fi

    print_info "Running quick benchmarks (1K and 10K points)..."
    ./benchmarks/benchmark_voxel_downsampling --benchmark_filter=".*/1000$|.*/10000$"

    cd ..
}

################################################################################
# Performance Comparison
################################################################################

compare_performance() {
    print_header "Performance Comparison: Scalar vs RVV"

    cd "$BUILD_DIR"

    TEST_POINTS="100000"
    LEAF_SIZE="1.0"
    ITERATIONS="5"

    print_info "Testing Scalar implementation..."
    ./src/voxel_downsample_scalar $TEST_POINTS $LEAF_SIZE $ITERATIONS > /tmp/scalar_perf.log 2>&1
    SCALAR_TIME=$(grep "Time:" /tmp/scalar_perf.log | awk '{print $2}')
    SCALAR_THROUGHPUT=$(grep "Throughput:" /tmp/scalar_perf.log | awk '{print $2}')

    print_info "Testing RVV implementation..."
    ./src/voxel_downsample_rvv $TEST_POINTS $LEAF_SIZE $ITERATIONS > /tmp/rvv_perf.log 2>&1
    RVV_TIME=$(grep "Time:" /tmp/rvv_perf.log | awk '{print $2}')
    RVV_THROUGHPUT=$(grep "Throughput:" /tmp/rvv_perf.log | awk '{print $2}')

    echo ""
    echo "Configuration: $TEST_POINTS points, leaf size=$LEAF_SIZE, $ITERATIONS iterations"
    echo "----------------------------------------------------------------"
    printf "%-20s %-15s %-20s\n" "Implementation" "Time (ms)" "Throughput (Mpts/s)"
    echo "----------------------------------------------------------------"
    printf "%-20s %-15s %-20s\n" "Scalar" "$SCALAR_TIME" "$SCALAR_THROUGHPUT"
    printf "%-20s %-15s %-20s\n" "RVV" "$RVV_TIME" "$RVV_THROUGHPUT"
    echo "----------------------------------------------------------------"

    # Calculate speedup (using awk instead of bc for portability)
    SPEEDUP=$(awk "BEGIN {printf \"%.2f\", $SCALAR_TIME / $RVV_TIME}")
    echo -e "${GREEN}Speedup: ${SPEEDUP}x${NC}"

    cd ..
}

################################################################################
# Correctness Verification
################################################################################

verify_correctness() {
    print_header "Verifying Correctness"

    cd "$BUILD_DIR"

    print_info "Running specific correctness tests..."

    # Run specific test cases
    ./tests/voxel_downsampling_tests -c "Voxel downsampling - random cloud comparison" > /tmp/correctness.log 2>&1

    if grep -q "All tests passed" /tmp/correctness.log; then
        print_success "Scalar and RVV produce identical results"
    else
        print_error "Results mismatch between implementations!"
        cat /tmp/correctness.log
        exit 1
    fi

    cd ..
}

################################################################################
# Stress Test
################################################################################

stress_test() {
    print_header "Stress Testing (1M points)"

    cd "$BUILD_DIR"

    print_info "Running stress test with 1 million points..."

    if timeout 120 ./tests/voxel_downsampling_tests -c "Voxel downsampling - stress test" > /tmp/stress.log 2>&1; then
        print_success "Stress test passed (1M points handled successfully)"
    else
        print_error "Stress test failed or timed out"
        tail -20 /tmp/stress.log
        exit 1
    fi

    cd ..
}

################################################################################
# Full Test Suite
################################################################################

run_all_tests() {
    print_header "Running Complete Test Suite"

    build_project
    run_unit_tests
    verify_correctness
    test_standalone_executables
    run_benchmarks
    compare_performance
    stress_test

    print_header "All Tests Completed Successfully! ✓"
    echo -e "${GREEN}All implementations are working correctly and have been benchmarked.${NC}"
}

################################################################################
# Main Script
################################################################################

show_usage() {
    cat << EOF
Usage: $0 [OPTION]

Automated testing script for voxel downsampling implementations.

Options:
    all              Run all tests (default)
    build            Build the project only
    unit             Run unit tests only
    ctest            Run CTest suite
    standalone       Test standalone executables
    benchmark        Run performance benchmarks
    compare          Compare scalar vs RVV performance
    verify           Verify correctness (scalar vs RVV)
    stress           Run stress test (1M points)
    quick            Quick test (build + unit tests + basic benchmark)
    help             Show this help message

Examples:
    $0                  # Run all tests
    $0 quick            # Quick verification
    $0 benchmark        # Run benchmarks only
    $0 compare          # Compare implementations

EOF
}

# Parse command line arguments
case "${1:-all}" in
    all)
        run_all_tests
        ;;
    build)
        build_project
        ;;
    unit)
        run_unit_tests
        ;;
    ctest)
        run_ctest
        ;;
    standalone)
        test_standalone_executables
        ;;
    benchmark)
        run_benchmarks
        ;;
    compare)
        build_project
        compare_performance
        ;;
    verify)
        verify_correctness
        ;;
    stress)
        stress_test
        ;;
    quick)
        print_header "Quick Test Mode"
        build_project
        run_unit_tests
        test_standalone_executables
        print_success "Quick tests completed successfully!"
        ;;
    help|--help|-h)
        show_usage
        exit 0
        ;;
    *)
        print_error "Unknown option: $1"
        show_usage
        exit 1
        ;;
esac

exit 0
