#!/bin/bash
# Unified run script for PCL-RISC-V executables

# set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
ARCH="native"
BUILD_DIR="build/$ARCH"
NATIVE_BUILD_DIR="build/native"
RISCV_BUILD_DIR="build/riscv"
EMULATOR="qemu"

# Function to show available executables
show_executables() {
    # Check RISC-V RVV status
    local rvv_status="Scalar (rv64gc)"
    if [ -f "$RISCV_BUILD_DIR/CMakeCache.txt" ]; then
        if grep -q "ENABLE_RVV:BOOL=ON" "$RISCV_BUILD_DIR/CMakeCache.txt" 2>/dev/null; then
            rvv_status="Vector (rv64gcv)"
        fi
    fi

    echo "Available Executables:"
    echo ""

    # Native (x86-64) Architecture
    echo -e "${YELLOW}Native (x86-64):${NC}"

    echo "  Tests:"
    local native_test_files=$(find "$NATIVE_BUILD_DIR/tests" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$native_test_files" ]; then
        echo "$native_test_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi

    echo "  Examples:"
    local native_example_files=$(find "$NATIVE_BUILD_DIR/examples" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$native_example_files" ]; then
        echo "$native_example_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi

    echo "  Benchmarks:"
    local native_benchmark_files=$(find "$NATIVE_BUILD_DIR/benchmarks" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$native_benchmark_files" ]; then
        echo "$native_benchmark_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi

    echo ""

    # RISC-V Architecture
    echo -e "${YELLOW}RISC-V ($rvv_status):${NC}"

    echo "  Tests:"
    local riscv_test_files=$(find "$RISCV_BUILD_DIR/tests" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$riscv_test_files" ]; then
        echo "$riscv_test_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi

    echo "  Examples:"
    local riscv_example_files=$(find "$RISCV_BUILD_DIR/examples" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$riscv_example_files" ]; then
        echo "$riscv_example_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi

    echo "  Benchmarks:"
    local riscv_benchmark_files=$(find "$RISCV_BUILD_DIR/benchmarks" -type f -executable 2>/dev/null | \
        grep -v '\.so' | grep -v '\.a' | grep -v '^a\.out$' | sort)
    if [ -n "$riscv_benchmark_files" ]; then
        echo "$riscv_benchmark_files" | xargs -n1 basename | sed 's/^/    • /'
    else
        echo "    (none found)"
    fi
}

# Function to run executable
run_executable() {
    local exe_path="$1"
    local exe_name="$(basename "$exe_path")"

    if [ "$ARCH" = "riscv" ]; then
        echo -e "${GREEN}Running RISC-V executable '$exe_name' with $EMULATOR...${NC}"
        if [ "$EMULATOR" = "qemu" ]; then
            qemu-riscv64 "$exe_path" $ARGS
        elif [ "$EMULATOR" = "spike" ]; then
            spike "$exe_path" $ARGS
        fi
    else
        echo -e "${GREEN}Running native executable '$exe_name'...${NC}"
        "$exe_path" $ARGS
    fi
}

# Function to run all in a category
run_category() {
    local category="$1"
    local dir="$BUILD_DIR/$category"

    echo -e "${GREEN}Running $ARCH $category with $EMULATOR...${NC}"

    local executables=$(find "$dir" -type f -executable 2>/dev/null | \
        grep -v '\.so' | \
        grep -v '\.a' | \
        grep -v '^a\.out$' | \
        sort)

    if [ -z "$executables" ]; then
        echo -e "${RED}Error: No $category executables found in $dir${NC}"
        echo -e "${YELLOW}Try building with the appropriate architecture first.${NC}"
        return 1
    fi

    for exe in $executables; do
        echo ""
        echo -e "${YELLOW}Running $(basename "$exe")...${NC}"
        run_executable "$exe"
    done

    echo ""
    echo -e "${GREEN}All $ARCH $category completed!${NC}"
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --native)
            ARCH="native"
            BUILD_DIR="build/native"
            shift
            ;;
        --riscv)
            ARCH="riscv"
            BUILD_DIR="build/riscv"
            shift
            ;;
        --qemu)
            EMULATOR="qemu"
            shift
            ;;
        --spike)
            EMULATOR="spike"
            shift
            ;;
        --tests)
            MODE="tests"
            shift
            ;;
        --examples)
            MODE="examples"
            shift
            ;;
        --benchmarks)
            MODE="benchmarks"
            shift
            ;;
        --list)
            show_executables
            exit 0
            ;;
        --help)
            echo -e "${BLUE}PCL-RISC-V Run Script${NC}"
            echo ""
            echo "Unified script to run native and RISC-V executables."
            echo ""
            echo -e "${YELLOW}Architecture options:${NC}"
            echo "  --native      Run native (x86_64) executables (default)"
            echo "  --riscv       Run RISC-V executables with QEMU/Spike"
            echo ""
            echo -e "${YELLOW}Emulator options (RISC-V only):${NC}"
            echo "  --qemu        Use QEMU for RISC-V execution (default)"
            echo "  --spike       Use Spike for RISC-V execution"
            echo ""
            echo -e "${YELLOW}Mode options:${NC}"
            echo "  --tests       Run all test executables"
            echo "  --examples    Run all example executables"
            echo "  --benchmarks  Run all benchmark executables"
            echo "  --list        List all available executables"
            echo ""
            echo -e "${YELLOW}Other options:${NC}"
            echo "  --help        Show this help message"
            echo ""
            echo -e "${YELLOW}Examples:${NC}"
            echo "  run --tests                           # Run native tests"
            echo "  run --riscv --tests                   # Run RISC-V tests with QEMU"
            echo "  run --riscv --spike --examples        # Run RISC-V examples with Spike"
            echo "  run rvpoint_tests                     # Run specific executable"
            echo "  run --riscv error_handling_example    # Run RISC-V executable"
            echo ""
            show_executables
            exit 0
            ;;
        *)
            if [ -z "$EXECUTABLE" ]; then
                EXECUTABLE="$1"
            else
                ARGS="$ARGS $1"
            fi
            shift
            ;;
    esac
done

# Handle different modes
if [ -n "$MODE" ]; then
    # Run category mode
    run_category "$MODE"
    exit 0
fi

# Handle specific executable mode
if [ -z "$EXECUTABLE" ]; then
    echo -e "${RED}Error: No executable specified${NC}"
    echo ""
    echo "Usage: run [options] <executable> [args...]"
    echo "Use --list to see available executables"
    exit 1
fi

# Find the executable
EXE_PATH="$BUILD_DIR/$EXECUTABLE"
if [ ! -f "$EXE_PATH" ]; then
    # Try to find it with find
    EXE_PATH=$(find "$BUILD_DIR" -type f -executable -name "$EXECUTABLE" | \
        grep -v '\.so' | \
        grep -v '\.a' | \
        grep -v '^a\.out$' | \
        grep -v '\.o$' | \
        grep -v '\.cmake$' | \
        grep -v 'CMakeFiles' | \
        grep -v 'cmake_install\.cmake' | \
        grep -v 'CTestTestfile\.cmake' | \
        grep -v 'compile_commands\.json' | \
        grep -v '\.ninja' | \
        head -1)
    if [ -z "$EXE_PATH" ]; then
        echo -e "${RED}Error: Executable '$EXECUTABLE' not found in $BUILD_DIR${NC}"
        echo -e "${YELLOW}Try building with the appropriate architecture first.${NC}"
        echo ""
        show_executables
        exit 1
    fi
fi

# Check if executable is actually executable
if [ ! -x "$EXE_PATH" ]; then
    echo -e "${RED}Error: '$EXE_PATH' is not executable${NC}"
    exit 1
fi

# Run the executable
run_executable "$EXE_PATH"
