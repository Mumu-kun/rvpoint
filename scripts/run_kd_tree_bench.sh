#!/bin/bash
# =============================================================================
#  run_kd_tree_bench.sh — Builds and runs the K-d tree benchmark
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "==> Building benchmark_kd_tree..."
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT"
cmake --build "$BUILD_DIR" --target benchmark_kd_tree -j$(nproc)

echo ""
echo "==> Running benchmark_kd_tree under QEMU..."

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
"$QEMU" -cpu "rv64,v=true,vlen=128" -L "/opt/riscv/sysroot" "$BIN_DIR/benchmark_kd_tree" data/bunny.pcd
