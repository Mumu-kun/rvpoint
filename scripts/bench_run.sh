#!/bin/bash
# =============================================================================
#  bench_run.sh — Configurable per-algorithm benchmark runner for RVPoint
#
#  Runs one algorithm at a time (or all), for scalar / RVV / both modes.
#  Each run gets its own timestamped subdirectory under results/:
#
#    results/<algo>_<mode>_N<n>_<timestamp>/
#      results.csv                 ← per-stage instruction counts
#      01_voxel_<mode>/output.pcd  ← intermediate point cloud after Voxel
#      02_ransac_extract_<mode>/output.pcd
#      03_sor_<mode>/output.pcd
#      04_normals_<mode>/output.pcd
#
#  Usage:
#    bash scripts/bench_run.sh [OPTIONS]
#
#  Options:
#    --algo     <name>    voxel | ransac | sor | normal | pipeline | all
#                         (default: pipeline)
#    --mode     <m>       sc | rvv | both  (default: both)
#    --n        <N>       Synthetic cloud size, ignored when a real dataset is used
#                         (default: 1024)
#    --dataset  <name>    Dataset to use:
#                           synthetic          — generated random cloud (default)
#                           bunny              — data/bunny.pcd
#                           table              — data/table_scene_lms400.pcd
#                           <path>             — any .pcd file path
#    --pcd      <file>    Alias for --dataset <path>  (kept for backward compat)
#    -h | --help
#
#  Examples:
#    bash scripts/bench_run.sh --algo pipeline --mode rvv --n 1024
#    bash scripts/bench_run.sh --algo pipeline --mode sc  --dataset table
#    bash scripts/bench_run.sh --algo pipeline --mode both --dataset bunny
#    bash scripts/bench_run.sh --algo pipeline --mode rvv --dataset /path/to/cloud.pcd
#    bash scripts/bench_run.sh --algo all --mode both --n 4096
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RESULTS_DIR="${PROJECT_ROOT}/results"
BIN_DIR="${PROJECT_ROOT}/bin"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
info()    { echo -e "${CYAN}==>${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
error()   { echo -e "${RED}[ERR]${RESET} $*" >&2; }

# ── Defaults ──────────────────────────────────────────────────────────────────
ALGO="pipeline"
MODE="both"
N_PTS=1024
DATASET="synthetic"   # synthetic | bunny | table | <path>

# ── Dataset name → file path resolver ─────────────────────────────────────────
resolve_dataset() {
    local name="$1"
    case "$name" in
        synthetic)  echo "" ;;
        bunny)      echo "${PROJECT_ROOT}/data/bunny.pcd" ;;
        table)      echo "${PROJECT_ROOT}/data/table_scene_lms400.pcd" ;;
        *)
            # Treat as a literal path; check it exists
            if [ ! -f "$name" ]; then
                error "Dataset file not found: $name"
                exit 1
            fi
            echo "$name" ;;
    esac
}

# ── Argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --algo)    ALGO="$2";    shift 2 ;;
        --mode)    MODE="$2";    shift 2 ;;
        --n)       N_PTS="$2";  shift 2 ;;
        --dataset) DATASET="$2"; shift 2 ;;
        --pcd)     DATASET="$2"; shift 2 ;;   # backward-compat alias
        -h|--help)
            sed -n '3,38p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) error "Unknown option: $1"; exit 1 ;;
    esac
done

PCD_FILE="$(resolve_dataset "$DATASET")"

# Validate mode
if [[ "$MODE" != "sc" && "$MODE" != "rvv" && "$MODE" != "both" ]]; then
    error "--mode must be sc | rvv | both"
    exit 1
fi

# Validate algo
VALID_ALGOS=(voxel ransac sor normal pipeline all)
if ! printf '%s\n' "${VALID_ALGOS[@]}" | grep -qx "$ALGO"; then
    error "--algo must be: ${VALID_ALGOS[*]}"
    exit 1
fi

# ── Resolve QEMU ──────────────────────────────────────────────────────────────
# Support running from host (via docker exec) or directly inside container
if [ -n "${IN_RVPOINT_CONTAINER:-}" ]; then
    # Inside container
    QEMU="$(command -v qemu-riscv64 2>/dev/null || echo /opt/qemu/bin/qemu-riscv64)"
    QEMU_FLAGS=(-cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot)
    PIPELINE_BIN="${BIN_DIR}/benchmark_pipeline"
    _run_in_container() { bash -c "$*"; }
else
    # On host — delegate everything into the container
    CONTAINER_NAME="${RVPOINT_CONTAINER:-rvpoint-dev}"
    if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
        error "Container '$CONTAINER_NAME' is not running. Start it via rvpoint.sh first."
        exit 1
    fi
    # Re-run this script inside the container
    info "Delegating to container '$CONTAINER_NAME'..."
    docker exec -it "$CONTAINER_NAME" bash \
        -c "cd /workspace && IN_RVPOINT_CONTAINER=1 bash scripts/bench_run.sh \
            --algo ${ALGO} --mode ${MODE} --n ${N_PTS} \
            $([ -n \"$PCD_FILE\" ] && echo \"--pcd ${PCD_FILE}\" || true)"
    exit $?
fi

# ── Build benchmark_pipeline if stale ─────────────────────────────────────────
info "Building benchmark_pipeline..."
cmake -B "${PROJECT_ROOT}/build" -S "${PROJECT_ROOT}" \
      -DCMAKE_TOOLCHAIN_FILE="${PROJECT_ROOT}/toolchain-riscv.cmake" \
      -DCMAKE_BUILD_TYPE=Release -Wno-dev > /dev/null 2>&1
cmake --build "${PROJECT_ROOT}/build" --target benchmark_pipeline -- -j"$(nproc)" 2>&1 \
    | grep -E 'Building|Linking|Error|error' || true
if [ ! -x "$PIPELINE_BIN" ]; then
    error "benchmark_pipeline binary not found at $PIPELINE_BIN"
    exit 1
fi
success "Binary ready: $PIPELINE_BIN"

# ── CSV header writer ─────────────────────────────────────────────────────────
write_csv_header() {
    local csv="$1"
    echo "timestamp,algo,mode,stage,n_in,n_out,instructions" > "$csv"
}

# ── Append BENCH_STAGE lines from binary stdout to CSV ───────────────────────
# Input lines look like:
#   BENCH_STAGE pipeline rvv VoxelGrid 1024 512 748381440
#   BENCH_TOTAL pipeline rvv 1024 301  54704576992
append_csv_from_stdout() {
    local csv="$1"
    local ts="$2"
    # stdin is the filtered stdout from the benchmark binary
    while IFS= read -r line; do
        read -r tag algo mode stage n_in n_out ins <<< "$line"
        if [ "$tag" = "BENCH_STAGE" ] || [ "$tag" = "BENCH_TOTAL" ]; then
            echo "${ts},${algo},${mode},${stage},${n_in},${n_out},${ins}" >> "$csv"
        fi
    done
}

# ── Run one (algo, mode) pair ─────────────────────────────────────────────────
run_one() {
    local algo="$1"
    local mode="$2"        # sc | rvv
    local n_pts="$3"
    local pcd="${4:-}"
    local dataset_name="${5:-synthetic}"   # human label for directory name

    local ts
    ts=$(date +%Y%m%d_%H%M%S)
    local label
    if [ -n "$pcd" ]; then
        label="${algo}_${mode}_${dataset_name}_${ts}"
    else
        label="${algo}_${mode}_N${n_pts}_${ts}"
    fi
    local run_dir="${RESULTS_DIR}/${label}"
    mkdir -p "$run_dir"

    local csv="${run_dir}/results.csv"
    write_csv_header "$csv"

    info "Running ${algo} [${mode}]  →  ${run_dir}"

    # Build argument list for the benchmark binary
    local bin_args=(--mode "$mode" --n "$n_pts" --save-dir "$run_dir")
    if [ -n "$pcd" ]; then
        bin_args+=("$pcd")   # positional PCD path
    fi

    # Run under QEMU; capture stdout to parse BENCH_ lines
    local stdout_log="${run_dir}/stdout.txt"
    set +e
    "${QEMU}" "${QEMU_FLAGS[@]}" "$PIPELINE_BIN" "${bin_args[@]}" \
        2>"${run_dir}/stderr.txt" | tee "$stdout_log"
    local exit_code=${PIPESTATUS[0]}
    set -e

    if [ "$exit_code" -ne 0 ]; then
        warn "Binary exited with code ${exit_code} — check ${run_dir}/stderr.txt"
    fi

    # Parse BENCH_ lines into CSV
    grep -E '^BENCH_(STAGE|TOTAL)' "$stdout_log" \
        | append_csv_from_stdout "$csv" "$(date -Iseconds)"

    local n_rows
    n_rows=$(wc -l < "$csv")
    success "CSV: ${csv}  (${n_rows} rows)"

    # List saved PCD files
    local pcds
    pcds=$(find "$run_dir" -name "*.pcd" 2>/dev/null | sort || true)
    if [ -n "$pcds" ]; then
        info "Saved PCD files:"
        echo "$pcds" | sed 's/^/    /'
    fi
}

# ── Expand "all" algo into each individual algorithm ─────────────────────────
if [ "$ALGO" = "all" ]; then
    RUN_ALGOS=(voxel ransac sor normal pipeline)
else
    RUN_ALGOS=("$ALGO")
fi

# ── Note: single-stage algos (voxel, ransac, sor, normal) currently run ──────
# through benchmark_pipeline which always runs the full pipeline. To run a
# single-stage algo in isolation, they re-use the pipeline binary but only the
# relevant BENCH_STAGE rows will be non-zero. A dedicated single-stage binary
# would give more accurate isolated counts; this is deferred for now.

# ── Main loop ─────────────────────────────────────────────────────────────────
mkdir -p "$RESULTS_DIR"

for algo in "${RUN_ALGOS[@]}"; do
    if [ "$MODE" = "both" ]; then
        run_one "$algo" "rvv" "$N_PTS" "$PCD_FILE" "$DATASET"
        run_one "$algo" "sc"  "$N_PTS" "$PCD_FILE" "$DATASET"
    else
        run_one "$algo" "$MODE" "$N_PTS" "$PCD_FILE" "$DATASET"
    fi
done

echo ""
success "All runs complete. Results in: ${RESULTS_DIR}/"
