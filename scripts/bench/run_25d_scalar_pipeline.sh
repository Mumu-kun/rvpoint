#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

BUILD_DIR="${PROJECT_ROOT}/build/scalar_25d"
BIN_DIR="${BUILD_DIR}/bin"
mkdir -p "$BIN_DIR"

SOURCE_FILE="${PROJECT_ROOT}/src/tools/scalar_25d_baseline.cpp"
TARGET_BIN="${BIN_DIR}/scalar_25d_baseline_riscv"

if [ ! -f "$TARGET_BIN" ] || [ "$SOURCE_FILE" -nt "$TARGET_BIN" ]; then
    echo "==> Compiling Reference Open-Source Scalar 2.5D Baseline for RISC-V (-O3, No RVV)..."
    riscv64-linux-gnu-g++ -O3 -std=c++17 -march=rv64gc -mabi=lp64d \
        -I"${PROJECT_ROOT}/src/include" \
        "$SOURCE_FILE" \
        -lpthread \
        -o "$TARGET_BIN"
    echo "==> Compilation complete: $TARGET_BIN"
    echo ""
fi

echo "==> Running Official Reference Open-Source Scalar 2.5D Baseline under QEMU RISC-V..."
exec qemu-riscv64 -L /usr/riscv64-linux-gnu -cpu rv64,v=false \
    "$TARGET_BIN" "$@"
