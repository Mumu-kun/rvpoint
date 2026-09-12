#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/build.sh
# PURPOSE: Unified cross-compilation driver for RVPoint static library (librvpoint.a),
#          standalone perception pipelines, benchmarks, and test suites.
#
# AGENT INVOCATION CONTRACT:
#   - Execution Context: WSL2 'rvpoint' distro or Native Linux container
#   - Toolchain Required: GCC 14.2+ with RVV 1.0 intrinsics in /opt/riscv
#   - Exit Codes: 
#       0 = Build successful
#       1 = CMake configuration or compilation failure
#       2 = Invalid command-line usage or unknown option
#   - Output Artifacts:
#       Libraries: build/<backend>/lib/librvpoint.a
#       Binaries:  build/<backend>/bin/<backend>/<target_name>
#       IDE DB:    build/compile_commands.json
#
# EXAMPLES:
#   ./scripts/build.sh                                  # Build complete RVV target suite
#   ./scripts/build.sh --target pipeline_3d_ultimate    # Build specific perception pipeline
#   ./scripts/build.sh --backend scalar --clean         # Clean and rebuild scalar baseline
#   ./scripts/build.sh --gem5                           # Build with gem5 simulation fallbacks
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/lib/common.sh"
wsl_bootstrap "scripts/build.sh" "$@"

BUILD_DIR="$(get_build_dir)"
source "${PROJECT_ROOT}/env/activate.sh"
export RISCV_PATH="${RISCV:-${RISCV_ROOT:-/opt/riscv}}"

# Defaults (can be overridden by command line)
TOOLCHAIN="linux"
BACKEND="rvv"
CLEAN=false
TARGET=""
GEM5_BUILD=false

usage() {
    cat <<'EOF'
RVPoint Cross-Compilation Build System

Usage:
  ./scripts/build.sh [options]

Options:
  --backend <rvv|scalar|all>  Target architecture backend (default: rvv)
                                rvv    -> -march=rv64gcv (RVV 1.0 Vector enabled)
                                scalar -> -march=rv64gc  (Scalar reference)
                                all    -> Builds both rvv and scalar backends
  --target <target_name>      Build specific target only (e.g., pipeline_3d_ultimate, test_voxel_grid)
  --toolchain <linux|elf>     CMake toolchain profile (default: linux)
  --gem5                      Enable gem5 simulation compatibility fallbacks (-DGEM5_BUILD=1)
  --clean                     Wipe build directory before building
  --help, -h                  Show this help documentation

Agent Guidelines:
  - All targets in eval/pipelines/, eval/benchmarks/, and eval/tests/ are auto-registered by CMake.
  - Binaries are output to: build/<backend>/bin/<backend>/<target_name>
  - compile_commands.json is automatically synced to build/compile_commands.json for clangd.

Examples:
  ./scripts/build.sh --backend rvv
  ./scripts/build.sh --target pipeline_3d_ultimate
  ./scripts/build.sh --clean --target test_euclidean_clustering
  ./scripts/build.sh --gem5 --target pipeline_3d_ultimate
EOF
    exit 0
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --toolchain)
            TOOLCHAIN="$2"
            shift 2
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --target)
            TARGET="$2"
            shift 2
            ;;
        --gem5)
            GEM5_BUILD=true
            shift
            ;;
        --help|-h)
            usage
            ;;
        -*)
            echo "Error: Unknown build option '$1'" >&2
            echo "Run './scripts/build.sh --help' for available options." >&2
            exit 2
            ;;
        *)
            echo "Error: Unexpected positional argument '$1'" >&2
            echo "Run './scripts/build.sh --help' for available options." >&2
            exit 2
            ;;
    esac
done

# Helper function to build a single backend (scalar or rvv)
build_backend() {
    local b_name="$1"
    local riscv_arch=""
    local rvv_cmake=""

    case "$b_name" in
        rvv)
            riscv_arch="rv64gcv"
            rvv_cmake="ON"
            ;;
        scalar|riscv|normal)
            b_name="scalar"
            riscv_arch="rv64gc"
            rvv_cmake="OFF"
            ;;
        *)
            echo "Error: Unknown backend '$b_name'. Valid choices: 'rvv', 'scalar', 'all'." >&2
            exit 2
            ;;
    esac

    local build_profile="${b_name}"
    if [ "$GEM5_BUILD" = true ]; then
        build_profile="${b_name}_gem5"
    fi

    local b_dir="${BUILD_DIR}/${build_profile}"

    # Map toolchain name to cmake file
    local toolchain_file=""
    case "$TOOLCHAIN" in
        linux)
            toolchain_file="${PROJECT_ROOT}/env/cmake/riscv_linux.cmake"
            ;;
        elf)
            toolchain_file="${PROJECT_ROOT}/env/cmake/riscv.cmake"
            ;;
        *)
            echo "Error: Unknown toolchain '$TOOLCHAIN'. Valid choices: 'linux', 'elf'." >&2
            exit 2
            ;;
    esac

    if [ ! -f "$toolchain_file" ]; then
        echo "Error: Toolchain file not found: $toolchain_file" >&2
        exit 1
    fi

    # Clean if requested
    if [ "$CLEAN" = true ]; then
        echo "==> Cleaning build directory for $b_name: $b_dir"
        rm -rf "$b_dir"
    fi

    # Auto-clean on toolchain mismatch
    local marker="$b_dir/.toolchain"
    if [ -f "$marker" ] && [ "$(cat "$marker")" != "$TOOLCHAIN" ]; then
        echo "==> Toolchain changed ($(cat "$marker") -> $TOOLCHAIN), reconfiguring $b_name..."
        rm -rf "$b_dir"
    fi

    # Auto-clean if CMake cache is pinned to a non-RISC-V compiler or different build configuration
    local cache_file="$b_dir/CMakeCache.txt"
    if [ -f "$cache_file" ]; then
        local cxx_compiler="$(sed -n 's/^CMAKE_CXX_COMPILER:FILEPATH=//p' "$cache_file" | head -n 1)"
        local cache_arch="$(sed -n 's/^RISCV_ARCH:STRING=//p' "$cache_file" | head -n 1)"
        local cache_rvv="$(sed -n 's/^RVV_PCL_USE_RVV:BOOL=//p' "$cache_file" | head -n 1)"
        local cache_gem5="$(sed -n 's/^GEM5_BUILD:BOOL=//p' "$cache_file" | head -n 1)"
        local cache_build_type="$(sed -n 's/^CMAKE_BUILD_TYPE:STRING=//p' "$cache_file" | head -n 1)"
        local expected_gem5="$([ "$GEM5_BUILD" = true ] && echo ON || echo OFF)"
        local cache_toolchain="$(sed -n 's/^CMAKE_TOOLCHAIN_FILE:[^=]*=//p' "$cache_file" | head -n 1)"
        if [ -n "$cache_toolchain" ] && ([ "$cache_toolchain" != "$toolchain_file" ] || [ ! -f "$cache_toolchain" ]); then
            echo "==> Detected stale or invalid toolchain file in CMake cache ($cache_toolchain -> $toolchain_file), reconfiguring $b_name..."
            rm -rf "$b_dir"
        elif [ -n "$cxx_compiler" ] && [[ "$cxx_compiler" != *riscv64* ]]; then
            echo "==> Detected stale host compiler in CMake cache ($cxx_compiler), reconfiguring $b_name..."
            rm -rf "$b_dir"
        elif [ -n "$cache_arch" ] && [ "$cache_arch" != "$riscv_arch" ]; then
            echo "==> Detected stale ISA in CMake cache ($cache_arch -> $riscv_arch), reconfiguring $b_name..."
            rm -rf "$b_dir"
        elif [ -n "$cache_rvv" ] && [ "$cache_rvv" != "$rvv_cmake" ]; then
            echo "==> Detected stale RVV setting in CMake cache ($cache_rvv -> $rvv_cmake), reconfiguring $b_name..."
            rm -rf "$b_dir"
        elif [ -n "$cache_gem5" ] && [ "$cache_gem5" != "$expected_gem5" ]; then
            echo "==> Detected stale GEM5 setting in CMake cache ($cache_gem5 -> $expected_gem5), reconfiguring $b_name..."
            rm -rf "$b_dir"
        elif [ -z "$cache_build_type" ] || [ "$cache_build_type" != "Release" ]; then
            echo "==> Detected non-Release build type in CMake cache ($cache_build_type), reconfiguring $b_name..."
            rm -rf "$b_dir"
        fi
    fi

    # Configure if needed
    if [ ! -f "$b_dir/CMakeCache.txt" ]; then
        local rvv_gem5_arg="$([ "$GEM5_BUILD" = true ] && echo ON || echo OFF)"
        echo "==> Configuring CMake (toolchain: $TOOLCHAIN, backend: $b_name, gem5: $rvv_gem5_arg)..."
        cmake -S "$PROJECT_ROOT" -B "$b_dir" \
            -DCMAKE_TOOLCHAIN_FILE="$toolchain_file" \
            -DCMAKE_BUILD_TYPE=Release \
            -DRISCV_ARCH="$riscv_arch" \
            -DRISCV_ABI="lp64d" \
            -DRVPOINT_USE_RVV="$rvv_cmake" \
            -DRVV_PCL_USE_RVV="$rvv_cmake" \
            -DGEM5_BUILD="$rvv_gem5_arg"
        echo "$TOOLCHAIN" > "$marker"
    fi

    # Build
    if [ -n "$TARGET" ]; then
        echo "==> Building target '$TARGET' [$b_name]..."
        cmake --build "$b_dir" --target "$TARGET" -j"${NPROC:-2}"
    else
        echo "==> Building full target suite [$b_name]..."
        cmake --build "$b_dir" -j"${NPROC:-2}"
    fi

    echo "==> Build complete for ${build_profile} backend. Binaries located in: ${b_dir}/bin/${b_name}/"

    # Sync compile_commands.json for host IDE (e.g., Windows clangd / IntelliSense)
    if [ -f "${b_dir}/compile_commands.json" ]; then
        mkdir -p "${PROJECT_ROOT}/build"
        local wsl_root="${PROJECT_ROOT}"
        local host_root=""
        if command -v wslpath &>/dev/null; then
            host_root="$(wslpath -m "${wsl_root}" 2>/dev/null || echo "")"
        fi

        if [ -n "$host_root" ] && [ -n "$wsl_root" ]; then
            sed -e "s|${wsl_root}|${host_root}|g" -e "s|\"directory\": \"[^\"]*\"|\"directory\": \"${host_root}\"|g" "${b_dir}/compile_commands.json" > "${PROJECT_ROOT}/build/compile_commands.json"
        else
            cp "${b_dir}/compile_commands.json" "${PROJECT_ROOT}/build/compile_commands.json"
        fi
        echo "==> Exported IntelliSense compilation database to: ${PROJECT_ROOT}/build/compile_commands.json"
    fi
}

case "$BACKEND" in
    all|both)
        build_backend "scalar"
        build_backend "rvv"
        ;;
    *)
        build_backend "$BACKEND"
        ;;
esac