#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/run.sh
# PURPOSE: Universal runner for RVPoint targets (pipelines, benchmarks, tests)
#          under QEMU user-mode emulation with RVV 1.0 support.
#
# AGENT INVOCATION CONTRACT:
#   - Execution Context: WSL2 'rvpoint' distro or Native Linux container
#   - Emulator: qemu-riscv64 with -cpu rv64,v=true,vlen=128
#   - Exit Codes:
#       0 = Successful run
#       1 = Execution failure or missing binary
#       2 = Invalid script arguments
#   - Target Discovery:
#       Automatically maps <target_name> to:
#         1. eval/pipelines/<target>.cpp
#         2. eval/benchmarks/<target>.cpp
#         3. eval/tests/fast/<target>.cpp (or test_<target>.cpp)
#         4. eval/tests/experimental/<target>.cpp (or test_<target>.cpp)
#
# EXAMPLES:
#   ./scripts/run.sh test_voxel_grid
#   ./scripts/run.sh pipeline_3d_ultimate data/bunny.pcd --progress --no-write
#   ./scripts/run.sh --backend scalar pcl_standalone_pipeline data/bunny.pcd
#   ./scripts/run.sh ablation_bench
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/lib/common.sh"
wsl_bootstrap "scripts/run.sh" "$@"

BUILD_DIR="$(get_build_dir)"
source "${PROJECT_ROOT}/env/activate.sh"

TOOLCHAIN="linux"
BACKEND="rvv"
TIER=""
TARGET_POINTS=""
TARGET_RAW=""
CPP_ARGS=()

usage() {
    cat <<'EOF'
RVPoint Target Runner (QEMU Emulation)

Usage:
  ./scripts/run.sh [runner_options] <target|cpp_file> [cpp_args...]

Downsampling Budget Options:
  --sanity, --tier sanity   Target ~150 points (Instant smoke test)
  --dev,    --tier dev      Target ~1,000 points (Fast dev loop)
  --eval,   --tier eval     Target ~5,000 points (Standard evaluation benchmark)
  --stress, --tier stress   Target ~10,000 points (Stress benchmark)
  --full,   --tier full     Full point cloud (100% original dataset, no downsampling)
  --pts <N>, --target-points <N>  Arbitrary exact point count target

Runner Options:
  --backend <rvv|scalar>    Backend to run (default: rvv)
                              rvv    -> Vector-accelerated binary (-march=rv64gcv)
                              scalar -> Scalar baseline binary (-march=rv64gc)
  --toolchain <linux|elf>   Toolchain profile used for auto-build (default: linux)
  --help, -h                Show this help documentation

Target Resolution:
  Pass either the bare target name or the path to a .cpp file.
  Targets are auto-discovered from:
    - eval/pipelines/        (e.g., pipeline_3d_ultimate, pcl_standalone_pipeline)
    - eval/benchmarks/       (e.g., ablation_bench, neighbor_search_sor_bench)
    - eval/tests/fast/       (e.g., test_voxel_grid, test_euclidean_clustering)
    - eval/tests/experimental/

CLI Argument Forwarding:
  All trailing arguments after <target> are passed directly to the target executable.

Examples:
  ./scripts/run.sh test_voxel_grid
  ./scripts/run.sh --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write
  ./scripts/run.sh --pts 2000 pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write
  ./scripts/run.sh --backend scalar pcl_standalone_pipeline data/01_table_scene_lms400.pcd --no-write
  ./scripts/run.sh ablation_bench --trials 5
EOF
    exit 0
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --sanity)
            TIER="sanity"
            shift
            ;;
        --dev)
            TIER="dev"
            shift
            ;;
        --eval)
            TIER="eval"
            shift
            ;;
        --stress)
            TIER="stress"
            shift
            ;;
        --full)
            TIER="full"
            shift
            ;;
        --tier)
            TIER="$2"
            shift 2
            ;;
        --pts|--target-points)
            TARGET_POINTS="$2"
            shift 2
            ;;
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
            if [ $# -gt 0 ]; then
                TARGET_RAW="$1"
                shift
                CPP_ARGS=("$@")
            fi
            break
            ;;
        -*)
            echo "Error: Unknown runner option '$1'" >&2
            echo "Run './scripts/run.sh --help' for available options." >&2
            exit 2
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
    echo "Error: No target or C++ file specified." >&2
    echo "Run './scripts/run.sh --help' for usage instructions." >&2
    exit 2
fi

# --- Target Normalization ---
TARGET_NAME="$(basename "$TARGET_RAW")"
TARGET_NAME="${TARGET_NAME%.cpp}"
TARGET_NAME="${TARGET_NAME%.c}"
TARGET_NAME="${TARGET_NAME%.cc}"

if [[ -f "${PROJECT_ROOT}/eval/pipelines/${TARGET_NAME}.cpp" ]]; then
    :
elif [[ -f "${PROJECT_ROOT}/eval/benchmarks/${TARGET_NAME}.cpp" ]]; then
    :
elif [[ -f "${PROJECT_ROOT}/eval/tests/fast/${TARGET_NAME}.cpp" || -f "${PROJECT_ROOT}/eval/tests/fast/${TARGET_NAME}.c" ]]; then
    :
elif [[ -f "${PROJECT_ROOT}/eval/tests/experimental/${TARGET_NAME}.cpp" ]]; then
    :
elif [[ -f "${PROJECT_ROOT}/eval/tests/fast/test_${TARGET_NAME}.cpp" ]]; then
    TARGET_NAME="test_${TARGET_NAME}"
elif [[ -f "${PROJECT_ROOT}/eval/tests/experimental/test_${TARGET_NAME}.cpp" ]]; then
    TARGET_NAME="test_${TARGET_NAME}"
fi

# --- PCD Detection & Pre-Execution Downsampling ---
PCD_INDEX=-1
ORIGINAL_PCD=""

for i in "${!CPP_ARGS[@]}"; do
    arg="${CPP_ARGS[$i]}"
    if [[ "$arg" == *.pcd || "$arg" == *.PCD ]] || [[ -f "$arg" && "$(head -n 1 "$arg" 2>/dev/null)" =~ ^#[[:space:]]*\.PCD ]]; then
        PCD_INDEX=$i
        ORIGINAL_PCD="$arg"
        break
    fi
done

if [[ $PCD_INDEX -ge 0 ]]; then
    if [ -n "$TARGET_POINTS" ]; then
        mkdir -p "$PROJECT_ROOT/output"
        PCD_BASE="$(basename "$ORIGINAL_PCD" .pcd)"
        DOWNSAMPLED_PCD="$PROJECT_ROOT/output/.downsampled_${TARGET_POINTS}pts_${PCD_BASE}.pcd"
        
        echo "==> Applying adaptive voxel decimation (--pts $TARGET_POINTS)..."
        python3 "$PROJECT_ROOT/scripts/lib/downsample_pcd.py" \
            --input "$ORIGINAL_PCD" \
            --target-points "$TARGET_POINTS" \
            --output "$DOWNSAMPLED_PCD"
        
        CPP_ARGS[$PCD_INDEX]="$DOWNSAMPLED_PCD"
        echo ""
    elif [[ -n "$TIER" && "$TIER" != "full" ]]; then
        mkdir -p "$PROJECT_ROOT/output"
        PCD_BASE="$(basename "$ORIGINAL_PCD" .pcd)"
        DOWNSAMPLED_PCD="$PROJECT_ROOT/output/.downsampled_${TIER}_${PCD_BASE}.pcd"
        
        echo "==> Applying adaptive voxel decimation (--tier $TIER)..."
        python3 "$PROJECT_ROOT/scripts/lib/downsample_pcd.py" \
            --input "$ORIGINAL_PCD" \
            --tier "$TIER" \
            --output "$DOWNSAMPLED_PCD"
        
        CPP_ARGS[$PCD_INDEX]="$DOWNSAMPLED_PCD"
        echo ""
    fi
fi

# --- Build target and dependencies ---
echo "==> Building target '$TARGET_NAME' (toolchain: $TOOLCHAIN, backend: $BACKEND)..."
"$SCRIPT_DIR/build.sh" --toolchain "$TOOLCHAIN" --backend "$BACKEND" --target "$TARGET_NAME"
echo ""

# --- Run target in QEMU ---
QEMU_BIN="$(find_qemu)"
QEMU_FLAGS=(${QEMU_SYSROOT_FLAGS:-})
if [ -d "/usr/riscv64-linux-gnu" ]; then
    QEMU_FLAGS+=("-L" "/usr/riscv64-linux-gnu")
fi
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
    echo "Error: Executable '$TARGET_NAME' not found in $BIN_DIR" >&2
    exit 1
fi

echo "==> Executing '$TARGET_NAME' under QEMU [$BACKEND]..."
if [ ${#CPP_ARGS[@]} -gt 0 ]; then
    echo "    Arguments: ${CPP_ARGS[*]}"
fi
echo "────────────────────────────────────────────────────────────"

exec "$QEMU_BIN" "${QEMU_FLAGS[@]}" "$TARGET_BIN" "${CPP_ARGS[@]}"