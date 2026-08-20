#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/lib/common.sh"
wsl_bootstrap "scripts/run.sh" "$@"

BUILD_DIR="$(get_build_dir)"
source "${PROJECT_ROOT}/env/activate.sh"

TOOLCHAIN="linux"
BACKEND="rvv"
TARGET_RAW=""
CPP_ARGS=()

usage() {
    cat <<'EOF'
Usage: run.sh [options] <target|cpp_file> [cpp_args...]

Options:
  --backend <rvv|scalar>   Backend to run (default: rvv)
  --toolchain <elf|linux> Toolchain for build (default: linux)
  --help, -h               Show this help

Examples:
  ./run.sh test_voxel_grid
  ./run.sh voxel_grid
  ./run.sh pipeline_export --progress --skip-sor ./data/indoor_scene.pcd ./output/results/indoor/
  ./run.sh --backend scalar src/tools/pipeline_export.cpp --progress ./data/indoor_scene.pcd
  ./run.sh benchmark
  ./run.sh pointer_octree data/0000000010.pcd
EOF
    exit 0
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --toolchain)
            TOOLCHAIN="$2"
            shift 2
            ;;
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --help|-h)
            usage
            ;;
        --)
            shift
            TARGET_RAW="$1"
            shift
            CPP_ARGS=("$@")
            break
            ;;
        -*)
            echo "Error: Unknown script option '$1'"
            usage
            ;;
        *)
            TARGET_RAW="$1"
            shift
            CPP_ARGS=("$@")
            break
            ;;
    esac
done

if [ -z "$TARGET_RAW" ]; then
    echo "Error: No target or C++ file specified."
    echo ""
    usage
fi

# --- Target Normalization ---
TARGET_NAME="$(basename "$TARGET_RAW")"
TARGET_NAME="${TARGET_NAME%.cpp}"
TARGET_NAME="${TARGET_NAME%.c}"
TARGET_NAME="${TARGET_NAME%.cc}"

if [[ "$TARGET_NAME" == "pointer_octree" || "$TARGET_NAME" == "pointer_octree_bench" || "$TARGET_NAME" == "pointer_octree_real" ]]; then
    TARGET_NAME="benchmark_pointer_octree_real"
elif [[ "$TARGET_NAME" != test_* && "$TARGET_NAME" != rvv_test && "$TARGET_NAME" != benchmark && "$TARGET_NAME" != benchmark_* && "$TARGET_NAME" != pipeline_export && "$TARGET_NAME" != pipeline_fast_export && "$TARGET_NAME" != pipeline_rvv_ultra_fast && "$TARGET_NAME" != pipeline_rvv_turbo && "$TARGET_NAME" != pipeline_3d_turbo && "$TARGET_NAME" != pipeline_3d_ultra && "$TARGET_NAME" != pipeline_3d_ultimate && "$TARGET_NAME" != scalar_25d_baseline && "$TARGET_NAME" != ablation_bench && "$TARGET_NAME" != neighbor_search_sor_bench && "$TARGET_NAME" != pipeline_compare_bench && "$TARGET_NAME" != pcl_standalone_pipeline ]]; then
    TARGET_NAME="test_$TARGET_NAME"
fi

# --- Build target and dependencies ---
echo "==> Building target '$TARGET_NAME' (toolchain: $TOOLCHAIN, backend: $BACKEND)..."
"$SCRIPT_DIR/build.sh" --toolchain "$TOOLCHAIN" --backend "$BACKEND" --target "$TARGET_NAME"
echo ""

# --- Run target in QEMU ---
QEMU_BIN="$(find_qemu)"
QEMU_FLAGS=(${QEMU_SYSROOT_FLAGS:-})
if [ "$BACKEND" = "rvv" ]; then
    QEMU_FLAGS+=("-cpu" "rv64,v=true,vlen=128")
else
    QEMU_FLAGS+=("-cpu" "rv64")
fi

BIN_DIR="${BUILD_DIR}/${BACKEND}/bin/${BACKEND}"
TARGET_BIN="$BIN_DIR/$TARGET_NAME"
if [ ! -f "$TARGET_BIN" ]; then
    TARGET_BIN="${BUILD_DIR}/${BACKEND}/bin/$TARGET_NAME"
fi
if [ ! -f "$TARGET_BIN" ]; then
    TARGET_BIN="${PROJECT_ROOT}/build/bin/${BACKEND}/$TARGET_NAME"
fi

if [ ! -f "$TARGET_BIN" ]; then
    echo "Error: Executable '$TARGET_NAME' not found in $BIN_DIR"
    exit 1
fi

echo "==> Running '$TARGET_NAME' under QEMU [$BACKEND]..."
if [ ${#CPP_ARGS[@]} -gt 0 ]; then
    echo "    Binary args: ${CPP_ARGS[*]}"
fi
echo "------------------------------------------------------------"

exec "$QEMU_BIN" "${QEMU_FLAGS[@]}" "$TARGET_BIN" "${CPP_ARGS[@]}"