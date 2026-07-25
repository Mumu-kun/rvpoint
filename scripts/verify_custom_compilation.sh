#!/bin/bash
# =============================================================================
#  verify_custom_compilation.sh — Verifies that the custom instructions code
#                                  compiles under the RISC-V GCC toolchain.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "==> Testing compilation of custom octree with -DUSE_CUSTOM_RVV..."

# RISC-V GCC compiler
CXX="riscv64-unknown-linux-gnu-g++"
if ! command -v "$CXX" &> /dev/null; then
    # Fallback to standard cross compiler path if in specific container setup
    CXX="/opt/riscv/bin/riscv64-unknown-linux-gnu-g++"
fi

if [ ! -f "$CXX" ] && ! command -v riscv64-unknown-linux-gnu-g++ &> /dev/null; then
    echo "Error: riscv64-unknown-linux-gnu-g++ compiler not found!" >&2
    exit 1
fi

# Build custom test executable manually with compile-time custom flag enabled
"$CXX" -march=rv64gcv -mabi=lp64d -O3 -DUSE_CUSTOM_RVV \
    -I "${PROJECT_ROOT}/src" \
    -I "${PROJECT_ROOT}/src/include" \
    "${PROJECT_ROOT}/src/custom_rvv_octree/custom_pointer_octree.cpp" \
    "${PROJECT_ROOT}/tests/benchmark_custom_octree.cpp" \
    -L "${PROJECT_ROOT}/build" -lrvv_radius_search \
    -o "${PROJECT_ROOT}/build/benchmark_custom_octree_custom_mode"

echo "[SUCCESS] Custom instruction inline assembly successfully compiled by the toolchain!"
rm -f "${PROJECT_ROOT}/build/benchmark_custom_octree_custom_mode"
