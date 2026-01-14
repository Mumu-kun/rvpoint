# Build script for easy compilation

#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
BUILD_TYPE="Release"
ENABLE_RVV="OFF"
BUILD_DIR="build"
TOOLCHAIN=""
EMULATOR="qemu"
CLEAN=false

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
            TOOLCHAIN="-DCMAKE_TOOLCHAIN_FILE=cmake/riscv64-linux-gnu.cmake"
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
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --debug       Build in Debug mode (default: Release)"
            echo "  --rvv         Enable RISC-V Vector extension"
            echo "  --riscv       Cross-compile for RISC-V"
            echo "  --spike       Use Spike instead of QEMU for testing"
            echo "  --clean       Clean build directory before building"
            echo "  --help        Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$BUILD_DIR"
fi

# Configure
echo -e "${GREEN}Configuring PCL-RISC-V...${NC}"
echo "  Build Type: $BUILD_TYPE"
echo "  RVV Enabled: $ENABLE_RVV"
echo "  Emulator: $EMULATOR"

cmake -B "$BUILD_DIR" -G Ninja \
    $TOOLCHAIN \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DENABLE_RVV="$ENABLE_RVV" \
    -DUSE_SPIKE_EMULATOR=$( [ "$EMULATOR" = "spike" ] && echo "ON" || echo "OFF" ) \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_BENCHMARKS=ON

# Build
echo -e "${GREEN}Building...${NC}"
cmake --build "$BUILD_DIR" --parallel

# Test
echo -e "${GREEN}Running tests...${NC}"
cd "$BUILD_DIR"
ctest --output-on-failure

echo -e "${GREEN}Build completed successfully!${NC}"
