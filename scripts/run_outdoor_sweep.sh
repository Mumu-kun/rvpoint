#!/bin/bash
# =============================================================================
#  run_outdoor_sweep.sh — Builds and runs the outdoor hyperparameter sweep benchmark
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BIN_DIR="${PROJECT_ROOT}/bin"
BUILD_DIR="${PROJECT_ROOT}/build"
RESULTS_DIR="${PROJECT_ROOT}/results"

mkdir -p "$RESULTS_DIR"

echo "==> Building benchmark_outdoor_sweep..."
# Clean build cache to ensure configuration is fresh
rm -rf "$BUILD_DIR"
cmake -B "$BUILD_DIR" -S "$PROJECT_ROOT" -DCMAKE_CXX_FLAGS=""
cmake --build "$BUILD_DIR" --target benchmark_outdoor_sweep -j$(nproc)

echo ""
echo "==> Running benchmark_outdoor_sweep under QEMU..."

# Find QEMU binary
if command -v qemu-riscv64 &> /dev/null; then
    QEMU="qemu-riscv64"
elif [ -f "/opt/qemu/bin/qemu-riscv64" ]; then
    QEMU="/opt/qemu/bin/qemu-riscv64"
else
    echo "Error: qemu-riscv64 not found!" >&2
    exit 1
fi

REPORT_FILE="${RESULTS_DIR}/outdoor_sweep_report.txt"

# Run benchmark and write to console and log file
"$QEMU" -cpu "rv64,v=true,vlen=128" -L "/opt/riscv/sysroot" "$BIN_DIR/benchmark_outdoor_sweep" data/table_scene_lms400.pcd

echo ""
echo "[SUCCESS] Outdoor hyperparameter sweep completed. Report saved to: ${REPORT_FILE}"
