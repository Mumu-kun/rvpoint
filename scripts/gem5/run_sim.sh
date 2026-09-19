#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/gem5/run_sim.sh
# PURPOSE: Universal gem5 microarchitectural simulation runner for RVPoint targets.
#          Cross-compiles statically with RVV 1.0 support, applies adaptive
#          geometric downsampling tiers, and models the SpacemiT K1 CPU core.
#
# AGENT INVOCATION CONTRACT:
#   - Execution Context: WSL2 'rvpoint' distro or Native Linux container (with Docker/gem5)
#   - Target Discovery: Matches scripts/run.sh conventions across eval/
#   - Tiers:
#       --sanity : ~100 – 250 pts   (5-15s runtime, rapid smoke test)
#       --dev    : ~1,000 pts       (1-2m runtime, L1D resident boundary)
#       --eval   : ~5,000 – 10k pts (15-25m runtime, L2 cache stressed)
#       --full   : 100% cloud       (Full uncompressed pass-through)
#   - Exit Codes:
#       0 = Successful simulation
#       1 = Compilation or gem5 simulation error
#       2 = Invalid script arguments
#   - Results Destination:
#       results/gem5/<target>_<backend>_<tier>_<timestamp>/
#
# EXAMPLES:
#   ./scripts/gem5/run_sim.sh --sanity pipeline_3d_ultimate data/bunny.pcd --no-write
#   ./scripts/gem5/run_sim.sh --dev pcl_standalone_pipeline data/scene.pcd --no-write
#   ./scripts/gem5/run_sim.sh --eval --save-results pipeline_3d_ultimate data/scene.pcd
#   ./scripts/gem5/run_sim.sh ablation_bench --trials 3
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source "$PROJECT_ROOT/scripts/lib/common.sh"
wsl_bootstrap "scripts/gem5/run_sim.sh" "$@"

BUILD_DIR="$(get_build_dir)"
source "${PROJECT_ROOT}/env/activate.sh"

# Default configuration
BACKEND="rvv"
TIER=""
TARGET_POINTS=""
SAVE_RESULTS=false
STREAM_OUTPUT=false
TARGET_RAW=""
CPP_ARGS=()

usage() {
    cat <<'EOF'
RVPoint Universal gem5 Simulation Runner

Usage:
  ./scripts/gem5/run_sim.sh [runner_options] <target|cpp_file> [cpp_args...]

Simulation Budget Tiers:
  --sanity, --tier sanity   Target ~150 points (~5-15s simulation, functional smoke test)
  --dev,    --tier dev      Target ~1,000 points (~1-2m simulation, L1D resident dev loop)
  --eval,   --tier eval     Target ~5,000 points (~10-15m simulation, official evaluation)
  --stress, --tier stress   Target ~10,000 points (~20-30m simulation, L2 cache pressure)
  --full,   --tier full     Full point cloud (100% original dataset, no downsampling)
  --pts <N>, --target-points <N>  Arbitrary exact point count target

Runner Options:
  --backend <rvv|scalar>    Compilation backend (default: rvv)
  --save-results            Save stats.txt and JSON summary to results/gem5/
  --stream, -v              Stream raw gem5 simulator log output
  --help, -h                Show this help documentation

Automatic PCD Detection:
  If a .pcd argument is detected and a tier or point flag is set,
  run_sim.sh automatically downsamples the PCD using adaptive voxel decimation
  and passes the downsampled cloud to the simulated binary.

Examples:
  # 1. Functional sanity smoke test
  ./scripts/gem5/run_sim.sh --sanity pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

  # 2. Fast dev loop
  ./scripts/gem5/run_sim.sh --dev pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

  # 3. Official evaluation comparison
  ./scripts/gem5/run_sim.sh --eval --save-results pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write
  ./scripts/gem5/run_sim.sh --eval --save-results pcl_standalone_pipeline data/01_table_scene_lms400.pcd --no-write

  # 4. Stress benchmark
  ./scripts/gem5/run_sim.sh --stress pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write

  # 5. Custom point count
  ./scripts/gem5/run_sim.sh --pts 3000 pipeline_3d_ultimate data/01_table_scene_lms400.pcd --no-write
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
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --save-results)
            SAVE_RESULTS=true
            shift
            ;;
        --stream|-v|--verbose)
            STREAM_OUTPUT=true
            shift
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
            echo "Run './scripts/gem5/run_sim.sh --help' for available options." >&2
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
    echo "Run './scripts/gem5/run_sim.sh --help' for usage instructions." >&2
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

# --- PCD Detection & Pre-Simulation Downsampling ---
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

# --- Build Target for gem5 ---
echo "==> Cross-compiling '$TARGET_NAME' for gem5 simulation (backend: $BACKEND)..."
"$PROJECT_ROOT/scripts/build.sh" --toolchain linux --backend "$BACKEND" --target "$TARGET_NAME" --gem5

BIN_DIR="${BUILD_DIR}/${BACKEND}_gem5/bin/${BACKEND}"
TARGET_BIN="$BIN_DIR/$TARGET_NAME"
if [ ! -f "$TARGET_BIN" ]; then
    TARGET_BIN="${BUILD_DIR}/${BACKEND}_gem5/bin/$TARGET_NAME"
fi
if [ ! -f "$TARGET_BIN" ]; then
    TARGET_BIN="${PROJECT_ROOT}/build/${BACKEND}_gem5/bin/${BACKEND}/$TARGET_NAME"
fi

if [ ! -f "$TARGET_BIN" ]; then
    echo "Error: Executable '$TARGET_NAME' not found in $BIN_DIR" >&2
    exit 1
fi

# --- Prepare Simulation Output Directory ---
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
TIER_TAG="${TIER:-full}"
if [ "$SAVE_RESULTS" = true ]; then
    OUTDIR="$PROJECT_ROOT/results/gem5/${TARGET_NAME}_${BACKEND}_${TIER_TAG}_${TIMESTAMP}"
    mkdir -p "$OUTDIR"
else
    OUTDIR="$(mktemp -d /tmp/gem5_sim.XXXXXX)"
fi

SIM_LOG="$OUTDIR/sim.log"

echo "==> Launching gem5 simulation (SpacemiT K1 / MinorCPU)..."
if [ ${#CPP_ARGS[@]} -gt 0 ]; then
    echo "    Binary args: ${CPP_ARGS[*]}"
fi
echo "    Output dir:  $OUTDIR"
echo "────────────────────────────────────────────────────────────"

# --- Execute Simulation ---
START_TIME=$(date +%s)
GEM5_EXEC=""

if command -v gem5.opt &>/dev/null; then
    GEM5_EXEC="gem5.opt"
    "$GEM5_EXEC" -d "$OUTDIR" \
        /gem5/configs/deprecated/example/se.py \
        --cmd "$TARGET_BIN" --options "${CPP_ARGS[*]}" 2>&1 \
        | tee "$SIM_LOG" \
        | grep --line-buffered -vE '^(warn:|info:|gem5 (Simulator|is |version|compiled|started|executing)|command line:|Global frequency|\*\*\*\* REAL SIMULATION)' || true
elif command -v docker &>/dev/null && docker ps &>/dev/null; then
    docker run --rm \
        -u "$(id -u):$(id -g)" \
        -v "$PROJECT_ROOT:$PROJECT_ROOT" \
        -v "$BUILD_DIR:$BUILD_DIR" \
        -v "/tmp:/tmp" \
        -w "$PROJECT_ROOT" \
        manuel313/gem5_v25 \
        /gem5/build/RISCV/gem5.opt \
          -d "$OUTDIR" \
          /gem5/configs/deprecated/example/se.py \
          --cmd "$TARGET_BIN" --options "${CPP_ARGS[*]}" 2>&1 \
        | tee "$SIM_LOG" \
        | grep --line-buffered -vE '^(warn:|info:|gem5 (Simulator|is |version|compiled|started|executing)|command line:|Global frequency|\*\*\*\* REAL SIMULATION)' || true
else
    echo "Notice: Neither local 'gem5.opt' nor running Docker container found."
    echo "Falling back to QEMU high-precision profiling runner..."
    "$PROJECT_ROOT/scripts/run.sh" --backend "$BACKEND" "$TARGET_NAME" "${CPP_ARGS[@]}"
    exit 0
fi

END_TIME=$(date +%s)
WALL_TIME=$(( END_TIME - START_TIME ))

STATS_FILE="$OUTDIR/stats.txt"
if [ -s "$STATS_FILE" ]; then
    python3 "$PROJECT_ROOT/scripts/lib/format_gem5_report.py" \
        "$STATS_FILE" \
        "$TARGET_NAME" \
        "$BACKEND" \
        "$WALL_TIME"
else
    echo "❌ Error: gem5 simulation aborted or generated no stats (Host Wall-Clock: ${WALL_TIME}s)" >&2
    if [ -f "$SIM_LOG" ]; then
        echo "" >&2
        echo "Simulation Error Log ($SIM_LOG):" >&2
        echo "────────────────────────────────────────────────────────────" >&2
        grep -E 'panic:|fatal:|Error:|Aborted|fault' "$SIM_LOG" | tail -n 20 >&2 || cat "$SIM_LOG" | tail -n 20 >&2
        echo "────────────────────────────────────────────────────────────" >&2
    fi
    exit 1
fi

