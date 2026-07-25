#!/bin/bash
# =============================================================================
#  run_packet_pointer_octree_bench.sh
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_DIR="${PROJECT_ROOT}/build"

echo "==> Building benchmark_packet_pointer_octree..."
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT"
cmake --build "$BUILD_DIR" --target benchmark_packet_pointer_octree -j$(nproc)

echo ""
echo "==> Running benchmark_packet_pointer_octree with living_room.pcd..."

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
"$QEMU" -cpu "rv64,v=true,vlen=128" -L "/opt/riscv/sysroot" "$BIN_DIR/benchmark_packet_pointer_octree" data/0000000010.pcd
