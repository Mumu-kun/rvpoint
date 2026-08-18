#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Check if --host flag is present
RUN_HOST=false
FILTERED_ARGS=()

for arg in "$@"; do
    if [ "$arg" == "--host" ] || [ "$arg" == "--native" ]; then
        RUN_HOST=true
    else
        FILTERED_ARGS+=("$arg")
    fi
done

if [ "$RUN_HOST" = true ]; then
    # Verify libpcl-dev installation for host
    if ! pkg-config --exists pcl_common 2>/dev/null; then
        echo "==> PCL (Point Cloud Library) not found. Installing libpcl-dev..."
        apt-get update && apt-get install -y libpcl-dev
    fi

    BUILD_DIR="${PROJECT_ROOT}/build/pcl"
    BIN_DIR="${BUILD_DIR}/bin"
    mkdir -p "$BIN_DIR"

    SOURCE_FILE="${PROJECT_ROOT}/src/tools/pcl_pipeline_benchmark.cpp"
    TARGET_BIN="${BIN_DIR}/pcl_pipeline_benchmark"

    if [ ! -f "$TARGET_BIN" ] || [ "$SOURCE_FILE" -nt "$TARGET_BIN" ]; then
        echo "==> Compiling native host PCL benchmark binary (O3)..."
        PCL_CFLAGS=$(pkg-config --cflags pcl_common pcl_io pcl_filters pcl_features pcl_kdtree pcl_segmentation)
        PCL_LIBS=$(pkg-config --libs pcl_common pcl_io pcl_filters pcl_features pcl_kdtree pcl_segmentation)

        g++ -O3 -std=c++17 -march=native \
            -I/usr/include/eigen3 \
            $PCL_CFLAGS \
            "$SOURCE_FILE" \
            -o "$TARGET_BIN" \
            $PCL_LIBS \
            -lpthread
        echo "==> Host compilation complete: $TARGET_BIN"
        echo ""
    fi

    echo "==> Running Official PCL Pipeline Benchmark on Host PC..."
    "$TARGET_BIN" "${FILTERED_ARGS[@]}"
else
    BUILD_DIR="${PROJECT_ROOT}/build/pcl"
    BIN_DIR="${BUILD_DIR}/bin"
    mkdir -p "$BIN_DIR"

    SOURCE_FILE="${PROJECT_ROOT}/src/tools/pcl_pipeline_benchmark.cpp"
    TARGET_BIN="${BIN_DIR}/pcl_pipeline_benchmark_riscv"

    if [ ! -f "$TARGET_BIN" ] || [ "$SOURCE_FILE" -nt "$TARGET_BIN" ]; then
        echo "==> Compiling Official PCL benchmark binary for RISC-V (O2)..."
        riscv64-linux-gnu-g++ -O2 -std=c++17 \
            -I/usr/include/eigen3 -I/usr/include/pcl-1.14 \
            "$SOURCE_FILE" \
            -L/usr/lib/riscv64-linux-gnu \
            -lpcl_common -lpcl_io -lpcl_filters -lpcl_features -lpcl_kdtree -lpcl_segmentation -lpcl_search \
            -lboost_system -lboost_filesystem \
            -lpthread \
            -o "$TARGET_BIN"
        echo "==> RISC-V Compilation complete: $TARGET_BIN"
        echo ""
    fi

    echo "==> Running Official Point Cloud Library (PCL 1.14) under QEMU RISC-V..."
    qemu-riscv64 -L /usr/riscv64-linux-gnu -E LD_LIBRARY_PATH=/usr/lib/riscv64-linux-gnu:/lib/riscv64-linux-gnu \
        "$TARGET_BIN" "${FILTERED_ARGS[@]}"
fi
