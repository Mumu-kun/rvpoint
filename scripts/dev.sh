#!/bin/bash
# Development script for building and running PCL-RISC-V with smart rebuild detection and ccache support

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
BUILD_TYPE="Release"
ENABLE_RVV="OFF"
IS_RISCV=false
BUILD_DIR="build/native"
TOOLCHAIN_FILE=""
VCPKG_TRIPLET=""
EMULATOR="qemu"
CLEAN=false
FORCE_BUILD=false
SKIP_BUILD=false
RUN_TARGET=""
RUN_ARGS=""
RUN_FILES=()
EXECUTABLES=()

# Function to check if rebuild is needed
needs_rebuild() {
    local build_dir="$1"

    # If build directory doesn't exist, rebuild needed
    if [ ! -d "$build_dir" ]; then
        return 0
    fi

    # If CMakeCache.txt doesn't exist, rebuild needed
    if [ ! -f "$build_dir/CMakeCache.txt" ]; then
        return 0
    fi

    # Check if CMake configuration has changed
    local cached_rvv=$(grep "ENABLE_RVV:BOOL=" "$build_dir/CMakeCache.txt" 2>/dev/null | cut -d'=' -f2)
    if [ "$cached_rvv" != "$ENABLE_RVV" ]; then
        echo -e "${YELLOW}CMake configuration changed (RVV: $cached_rvv -> $ENABLE_RVV), rebuild needed${NC}"
        return 0
    fi

    # Check if any source files are newer than build artifacts
    local newest_source=$(find src include tests examples benchmarks CMakeLists.txt -name "*.cpp" -o -name "*.hpp" -o -name "*.h" -o -name "CMakeLists.txt" 2>/dev/null | xargs ls -t 2>/dev/null | head -1)
    local newest_build=$(find "$build_dir" -name "*.o" -o -name "*.a" -o -name "*rvpoint*" -type f 2>/dev/null | xargs ls -t 2>/dev/null | head -1)

    if [ -z "$newest_source" ]; then
        return 1  # No source files found, assume no rebuild needed
    fi

    if [ -z "$newest_build" ]; then
        return 0  # No build artifacts, rebuild needed
    fi

    # Compare timestamps
    if [ "$newest_source" -nt "$newest_build" ]; then
        return 0  # Source is newer, rebuild needed
    else
        return 1  # No rebuild needed
    fi
}

# Function to setup ccache
setup_ccache() {
    if command -v ccache >/dev/null 2>&1; then
        echo -e "${BLUE}Setting up ccache...${NC}"
        export CC="ccache gcc"
        export CXX="ccache g++"

        # For RISC-V cross-compilation, also setup ccache for cross compilers
        if [ -n "$TOOLCHAIN_FILE" ]; then
            export RISCV_CC="ccache riscv64-linux-gnu-gcc"
            export RISCV_CXX="ccache riscv64-linux-gnu-g++"
        fi

        # Show ccache stats
        echo -e "${BLUE}ccache statistics:${NC}"
        ccache -s | head -10
        echo ""
    else
        echo -e "${YELLOW}ccache not found, building without compiler caching${NC}"
    fi
}

# Function to build project
build_project() {
    local build_dir="$1"

    echo -e "${GREEN}Configuring PCL-RISC-V...${NC}"
    echo "  Build Type: $BUILD_TYPE"
    echo "  RVV Enabled: $ENABLE_RVV"
    echo "  Build Dir: $build_dir"
    if [ -n "$VCPKG_TRIPLET" ]; then
        echo "  Target: RISC-V 64-bit"
        echo "  Triplet: riscv64-linux"
    fi

    # Clean if requested
    if [ "$CLEAN" = true ]; then
        echo -e "${YELLOW}Cleaning build directory...${NC}"
        rm -rf "$build_dir"
    fi

    # If not using RISC-V cross-compilation, use regular vcpkg toolchain
    if [ -z "$VCPKG_TRIPLET" ]; then
        TOOLCHAIN_FILE="-DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
    fi

    # Configure with cmake
    cmake -B "$build_dir" -G Ninja \
        $TOOLCHAIN_FILE \
        $VCPKG_TRIPLET \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DENABLE_RVV="$ENABLE_RVV" \
        -DUSE_SPIKE_EMULATOR="$( [ "$EMULATOR" = "spike" ] && echo "ON" || echo "OFF" )" \
        -DBUILD_TESTS=ON \
        -DBUILD_EXAMPLES=ON \
        -DBUILD_BENCHMARKS=ON

    # Build
    echo -e "${GREEN}Building...${NC}"
    cmake --build "$build_dir" --parallel

    # Run tests
    echo -e "${GREEN}Running tests...${NC}"
    cd "$build_dir"
    ctest --output-on-failure
    cd - >/dev/null

    echo -e "${GREEN}Build completed successfully!${NC}"
}

# Function to run target
run_target() {
    local target="$1"
    local arch="$2"

    case "$target" in
        tests)
            echo -e "${GREEN}Running $arch tests...${NC}"
            ./scripts/run.sh --$arch --tests $RUN_ARGS
            ;;
        examples)
            echo -e "${GREEN}Running $arch examples...${NC}"
            ./scripts/run.sh --$arch --examples $RUN_ARGS
            ;;
        benchmarks)
            echo -e "${GREEN}Running $arch benchmarks...${NC}"
            ./scripts/run.sh --$arch --benchmarks $RUN_ARGS
            ;;
        *)
            echo -e "${RED}Unknown run target: $target${NC}"
            echo "Available targets: tests, examples, benchmarks"
            exit 1
            ;;
    esac
}

# Function to run specific files
run_files() {
    local arch="$1"
    shift
    local files=("$@")

    echo -e "${GREEN}Running specific $arch files...${NC}"

    for file in "${files[@]}"; do
        echo -e "${YELLOW}Running $file...${NC}"
        ./scripts/run.sh --$arch "$file" $RUN_ARGS
        echo ""
    done

    echo -e "${GREEN}All specified files completed!${NC}"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --debug)
            BUILD_TYPE="Debug"
            shift
            ;;
        --rvv)
            ENABLE_RVV="ON"
            shift
            ;;
        --riscv)
            IS_RISCV=true
            TOOLCHAIN_FILE="-DCMAKE_TOOLCHAIN_FILE=${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
            VCPKG_TRIPLET="-DVCPKG_TARGET_TRIPLET=riscv64-linux -DVCPKG_CHAINLOAD_TOOLCHAIN_FILE=/opt/vcpkg/cmake/riscv64-linux-gnu.cmake"
            shift
            ;;
        --spike)
            EMULATOR="spike"
            shift
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --force)
            FORCE_BUILD=true
            shift
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --run-tests)
            RUN_TARGET="tests"
            shift
            ;;
        --run-examples)
            RUN_TARGET="examples"
            shift
            ;;
        --run-benchmarks)
            RUN_TARGET="benchmarks"
            shift
            ;;
        --run-file)
            RUN_FILES+=("$2")
            shift 2
            ;;
        --run-args)
            RUN_ARGS="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [build options] [run options] [executables...]"
            echo ""
            echo -e "${YELLOW}Build Options:${NC}"
            echo "  --debug         Build in Debug mode (default: Release)"
            echo "  --rvv           Enable RISC-V Vector extension (uses build/riscv-rvv)"
            echo "  --riscv         Cross-compile for RISC-V (uses build/riscv or build/riscv-rvv)"
            echo "  --spike         Use Spike instead of QEMU for testing"
            echo "  --clean         Clean build directory before building"
            echo "  --force         Force rebuild even if not needed"
            echo "  --skip-build    Skip build step, only run"
            echo ""
            echo -e "${YELLOW}Run Options:${NC}"
            echo "  --run-tests     Run tests after building"
            echo "  --run-examples  Run examples after building"
            echo "  --run-benchmarks Run benchmarks after building"
            echo "  --run-file FILE Run specific executable file after building"
            echo "  --run-args ARGS Pass additional arguments to run script"
            echo ""
            echo -e "${YELLOW}Examples:${NC}"
            echo "  $0 --run-tests                    # Build and run native tests"
            echo "  $0 --riscv --run-examples         # Build RISC-V and run examples"
            echo "  $0 --force --run-benchmarks       # Force rebuild and run benchmarks"
            echo "  $0 --skip-build --run-tests       # Only run tests (no build)"
            echo "  $0 --run-file rvpoint_tests       # Build and run specific test"
            echo "  $0 rvpoint_tests                  # Build and run specific executable"
            echo "  $0 --riscv error_handling_example # Build RISC-V and run specific example"
            exit 0
            ;;
        *)
            # Treat unrecognized arguments as executable names
            EXECUTABLES+=("$1")
            shift
            ;;
    esac
done

# Determine build directory based on architecture and RVV settings
if [ "$IS_RISCV" = true ]; then
    if [ "$ENABLE_RVV" = "ON" ]; then
        BUILD_DIR="build/riscv-rvv"
    else
        BUILD_DIR="build/riscv"
    fi
fi

# Setup ccache
setup_ccache

# Determine architecture for run script
ARCH="native"
if [ "$IS_RISCV" = true ]; then
    ARCH="riscv"
fi

# Build if needed
if [ "$SKIP_BUILD" = false ]; then
    if [ "$FORCE_BUILD" = true ] || needs_rebuild "$BUILD_DIR"; then
        echo -e "${YELLOW}Build needed, starting build process...${NC}"
        build_project "$BUILD_DIR"
    else
        echo -e "${GREEN}Build is up to date, skipping build step${NC}"
    fi
else
    echo -e "${YELLOW}Skipping build step as requested${NC}"
fi

# Run if target specified
if [ -n "$RUN_TARGET" ]; then
    run_target "$RUN_TARGET" "$ARCH"
elif [ ${#RUN_FILES[@]} -gt 0 ] || [ ${#EXECUTABLES[@]} -gt 0 ]; then
    # Combine RUN_FILES and EXECUTABLES
    all_files=("${RUN_FILES[@]}" "${EXECUTABLES[@]}")
    run_files "$ARCH" "${all_files[@]}"
fi

# # Show ccache stats after build
# if command -v ccache >/dev/null 2>&1; then
#     echo ""
#     echo -e "${BLUE}Final ccache statistics:${NC}"
#     ccache -s | head -10
# fi
