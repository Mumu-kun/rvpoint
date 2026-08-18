#!/bin/bash
# =============================================================================
#  run_pointer_octree_bench.sh — Builds and runs the pointer-based octree bench
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "==> Building benchmark_pointer_octree_real..."
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT"
cmake --build "$BUILD_DIR" --target benchmark_pointer_octree_real -j$(nproc)

if [ -f "$BUILD_DIR/bin/rvv/benchmark_pointer_octree_real" ]; then
    TARGET_BIN="$BUILD_DIR/bin/rvv/benchmark_pointer_octree_real"
elif [ -f "$BUILD_DIR/bin/benchmark_pointer_octree_real" ]; then
    TARGET_BIN="$BUILD_DIR/bin/benchmark_pointer_octree_real"
else
    TARGET_BIN="$BIN_DIR/benchmark_pointer_octree_real"
fi

echo ""
echo "==> Running benchmark_pointer_octree_real with living_room.pcd..."

# Find QEMU binary
if command -v qemu-riscv64 &> /dev/null; then
    QEMU="qemu-riscv64"
elif [ -f "/opt/qemu/bin/qemu-riscv64" ]; then
    QEMU="/opt/qemu/bin/qemu-riscv64"
else
    echo "Error: qemu-riscv64 not found!" >&2
    exit 1
fi

# Run the benchmark
"$QEMU" -cpu "rv64,v=true,vlen=128" -L "/opt/riscv/sysroot" "$TARGET_BIN" data/0000000010.pcd
