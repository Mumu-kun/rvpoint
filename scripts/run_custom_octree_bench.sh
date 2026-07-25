#!/bin/bash
# =============================================================================
#  run_custom_octree_bench.sh — Builds and runs the custom octree fallback
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "==> Building benchmark_custom_octree..."
rm -rf "$BUILD_DIR" # Force clear cache to override previous -DUSE_CUSTOM_RVV setting
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT" -DCMAKE_CXX_FLAGS=""
cmake --build "$BUILD_DIR" --target benchmark_custom_octree -j$(nproc)

echo ""
echo "==> Running benchmark_custom_octree under QEMU..."

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
"$QEMU" -cpu "rv64,v=true,vlen=128" -L "/opt/riscv/sysroot" "$BIN_DIR/benchmark_custom_octree" data/bunny.pcd
