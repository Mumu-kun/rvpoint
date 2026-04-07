#!/bin/bash
# =============================================================================
#  gem5_bench_run.sh — Configurable gem5 cycle-accurate benchmark for RVPoint
#
#  Run one algorithm at a time, for scalar / RVV / both, on synthetic data or
#  a real LiDAR PCD file. Results go in a timestamped subdirectory under
#  results/ as a CSV of gem5 cycle counts.
#
#  Usage:
#    bash scripts/gem5_bench_run.sh [OPTIONS]
#
#  Options:
#    --algo     <name>   voxel|ransac|sor|normal|pipeline  (default: pipeline)
#    --mode     <m>      sc|rvv|both  (default: both)
#    --dataset  <name>   synthetic|bunny|table|<path>  (default: synthetic)
#                        Note: real PCD datasets only apply to --algo pipeline.
#                        Other algos always use synthetic data.
#    --n        <N>      Synthetic cloud size OR PCD subsample size (default: 512)
#    --cpu      <model>  o3|minor|timing  (default: o3)
#    -h | --help
#
#  Output directory:
#    results/gem5_<algo>_<mode>_<dataset>_N<n>_<ts>/
#      results.csv        ← gem5 cycle counts + sim_insts per run
#      <algo>_sc/         ← gem5 output dir (stats.txt, gem5_stdout.txt, ...)
#      <algo>_rvv/
#
#  Examples:
#    bash scripts/gem5_bench_run.sh --algo pipeline --mode rvv --dataset table --n 512
#    bash scripts/gem5_bench_run.sh --algo pipeline --mode both --dataset synthetic --n 1024
#    bash scripts/gem5_bench_run.sh --algo normal --mode both --n 512
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RESULTS_DIR="${PROJECT_ROOT}/results"
GEM5_CONFIG="${SCRIPT_DIR}/gem5_se.py"

# Prefer gem5 v25 (has vcompress.vm support)
GEM5_DIR="/opt/gem5"
[ -f "/opt/gem5-25/build/RISCV/gem5.opt" ] && GEM5_DIR="/opt/gem5-25"
GEM5_BIN="${GEM5_DIR}/build/RISCV/gem5.opt"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
info()    { echo -e "${CYAN}==>${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
error()   { echo -e "${RED}[ERR]${RESET} $*" >&2; }

# ── Defaults ──────────────────────────────────────────────────────────────────
ALGO="pipeline"
MODE="both"
DATASET="synthetic"
N_PTS=512
CPU_MODEL="o3"

# ── Dataset resolver ──────────────────────────────────────────────────────────
resolve_dataset() {
    case "$1" in
        synthetic) echo "" ;;
        bunny)     echo "${PROJECT_ROOT}/data/bunny.pcd" ;;
        table)     echo "${PROJECT_ROOT}/data/table_scene_lms400.pcd" ;;
        *)
            [ -f "$1" ] || { error "Dataset file not found: $1"; exit 1; }
            echo "$1" ;;
    esac
}

# ── Argument parsing ──────────────────────────────────────────────────────────
while [[ $# -gt 0 ]]; do
    case "$1" in
        --algo)    ALGO="$2";    shift 2 ;;
        --mode)    MODE="$2";    shift 2 ;;
        --dataset) DATASET="$2"; shift 2 ;;
        --n)       N_PTS="$2";  shift 2 ;;
        --cpu)     CPU_MODEL="$2"; shift 2 ;;
        -h|--help)
            sed -n '3,28p' "$0" | sed 's/^# \?//'
            exit 0 ;;
        *) error "Unknown option: $1"; exit 1 ;;
    esac
done

# Validate
for v in sc rvv both; do [ "$MODE" = "$v" ] && break; done
[[ "$MODE" =~ ^(sc|rvv|both)$ ]] || { error "--mode must be sc|rvv|both"; exit 1; }
[[ "$ALGO" =~ ^(voxel|ransac|sor|normal|pipeline)$ ]] || {
    error "--algo must be: voxel ransac sor normal pipeline"; exit 1; }
[[ "$CPU_MODEL" =~ ^(o3|minor|timing)$ ]] || {
    error "--cpu must be o3|minor|timing"; exit 1; }

PCD_FILE="$(resolve_dataset "$DATASET")"

# Warn if real dataset requested for non-pipeline algo
if [ -n "$PCD_FILE" ] && [ "$ALGO" != "pipeline" ]; then
    warn "Real PCD datasets only apply to --algo pipeline."
    warn "Running ${ALGO} with synthetic data (N=${N_PTS}) instead."
    PCD_FILE=""
    DATASET="synthetic"
fi

# ── Sanity check: gem5 ────────────────────────────────────────────────────────
if [ ! -f "$GEM5_BIN" ]; then
    error "gem5 not found at ${GEM5_BIN}"
    error "Run option 7 (gem5 Setup) first."
    exit 1
fi

# ── Build static binaries ─────────────────────────────────────────────────────
ELF_GCC="$(command -v riscv64-unknown-elf-g++ 2>/dev/null || true)"
[ -z "$ELF_GCC" ] && ELF_GCC="${RISCV:-/opt/riscv}/bin/riscv64-unknown-elf-g++"

LINUX_GCC="${RISCV:-/opt/riscv}/bin/riscv64-unknown-linux-gnu-g++"
[ ! -x "$LINUX_GCC" ] && LINUX_GCC="$(command -v riscv64-unknown-linux-gnu-g++ 2>/dev/null || true)"

SRCS=(
    "${PROJECT_ROOT}/src/voxel_grid_downsamp.cpp"
    "${PROJECT_ROOT}/src/rvv_common.cpp"
    "${PROJECT_ROOT}/src/statistical_outlier_removal.cpp"
    "${PROJECT_ROOT}/src/normal_estimation.cpp"
    "${PROJECT_ROOT}/src/radius_search.cpp"
    "${PROJECT_ROOT}/src/ransac_plane.cpp"
    "${PROJECT_ROOT}/src/octree.cpp"
    "${PROJECT_ROOT}/src/spatial_hashing.cpp"
    "${PROJECT_ROOT}/tests/benchmark_gem5.cpp"
)
mkdir -p "${PROJECT_ROOT}/bin"

# ELF binary — synthetic algos (always built)
ELF_BIN="${PROJECT_ROOT}/bin/benchmark_gem5_static"
info "Building benchmark_gem5_static (ELF, synthetic)..."
"$ELF_GCC" -march=rv64gcv -mabi=lp64d -O2 -std=c++17 -static \
    -DGEM5_BUILD -I"${PROJECT_ROOT}/src/include" \
    "${SRCS[@]}" -o "$ELF_BIN" -lstdc++ -lm -lc
success "Built: ${ELF_BIN}"

# Linux-gnu binary — PCD pipeline (only if a real dataset is selected)
PCD_BIN="${PROJECT_ROOT}/bin/benchmark_gem5_pcd_static"
HAVE_PCD_BIN=0
if [ -n "$PCD_FILE" ]; then
    if [ -x "$LINUX_GCC" ]; then
        info "Building benchmark_gem5_pcd_static (linux-gnu, real PCD file I/O)..."
        "$LINUX_GCC" -march=rv64gcv -mabi=lp64d -O2 -std=c++17 -static \
            -DGEM5_BUILD -I"${PROJECT_ROOT}/src/include" \
            "${SRCS[@]}" -o "$PCD_BIN"
        success "Built: ${PCD_BIN}"
        HAVE_PCD_BIN=1
    else
        warn "riscv64-unknown-linux-gnu-g++ not found — cannot run PCD pipeline in gem5."
        warn "Falling back to synthetic data."
        PCD_FILE=""
        DATASET="synthetic"
    fi
fi

# ── gem5 stat helpers ─────────────────────────────────────────────────────────
extract_stat() {
    local file="$1/stats.txt" key="$2"
    grep -m1 "^${key}" "$file" 2>/dev/null | awk '{print $2}' || echo "N/A"
}
extract_cycles() {
    local dir="$1"
    [ -f "${dir}/.failed" ] && echo "FAILED" && return
    local v
    v=$(extract_stat "$dir" "system.cpu.numCycles")
    [ "$v" != "N/A" ] && [ "$v" != "0" ] && echo "$v" && return
    local insts ipc
    insts=$(extract_stat "$dir" "sim_insts")
    ipc=$(extract_stat "$dir" "system.cpu.ipc")
    if [ "$insts" != "N/A" ] && [ "$ipc" != "N/A" ] && [ "$ipc" != "0" ]; then
        python3 -c "print(int(${insts}/${ipc}))" 2>/dev/null || echo "N/A"
        return
    fi
    echo "N/A"
}
extract_ns() {
    local sec
    sec=$(extract_stat "$1" "simSeconds")
    [ "$sec" = "N/A" ] && echo "N/A" && return
    python3 -c "print(f'{float(\"${sec}\")*1e9:.1f}')" 2>/dev/null || echo "N/A"
}
extract_insts() {
    local v
    v=$(extract_stat "$1" "simInsts")
    [ "$v" != "N/A" ] && echo "$v" && return
    extract_stat "$1" "sim_insts"
}

# ── Run one gem5 simulation ───────────────────────────────────────────────────
run_gem5_once() {
    local bin="$1" algo_arg="$2" n="$3" outdir="$4" pcd="${5:-}"
    mkdir -p "$outdir"

    local options="${algo_arg} ${n}"
    [ -n "$pcd" ] && options="${algo_arg} ${n} ${pcd}"

    "$GEM5_BIN" \
        --outdir="$outdir" \
        "${GEM5_CONFIG}" \
        --cmd="$bin" \
        --options="$options" \
        --cpu="$CPU_MODEL" \
        > "${outdir}/gem5_stdout.txt" 2>&1 || true

    if grep -q "IllegalInstFault\|illegal instruction\|panic:" \
            "${outdir}/gem5_stdout.txt" 2>/dev/null; then
        warn "  ${algo_arg}: unsupported instruction — marking as FAILED"
        echo "FAILED" > "${outdir}/.failed"
    elif ! grep -q "gem5_bench:" "${outdir}/gem5_stdout.txt" 2>/dev/null; then
        warn "  ${algo_arg}: binary may not have completed — check ${outdir}/gem5_stdout.txt"
    fi
}

# ── CSV helpers ───────────────────────────────────────────────────────────────
write_csv_header() {
    echo "timestamp,algo,mode,dataset,n,cpu_model,gem5_cycles,gem5_ns,sim_insts" > "$1"
}
append_csv_row() {
    local csv="$1" algo="$2" mode="$3" dataset="$4" n="$5" \
          cycles="$6" ns="$7" insts="$8"
    echo "$(date -Iseconds),${algo},${mode},${dataset},${n},${CPU_MODEL},${cycles},${ns},${insts}" \
        >> "$csv"
}

# ── Main ──────────────────────────────────────────────────────────────────────
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
RUN_DIR="${RESULTS_DIR}/gem5_${ALGO}_${MODE}_${DATASET}_N${N_PTS}_${TIMESTAMP}"
mkdir -p "$RUN_DIR"

CSV="${RUN_DIR}/results.csv"
write_csv_header "$CSV"

echo ""
echo -e "${BOLD}======================================================${RESET}"
echo -e "${BOLD}  gem5 Configurable Benchmark${RESET}"
echo    "  algo=${ALGO}  mode=${MODE}  dataset=${DATASET}  N=${N_PTS}  cpu=${CPU_MODEL}"
echo    "  Output: ${RUN_DIR}"
echo -e "${BOLD}======================================================${RESET}"
echo ""
echo    "  Note: gem5 simulates ~1-10 MIPS — each run takes 2-10 min."
echo ""

# Determine modes to run
MODES=()
[ "$MODE" = "sc"   ] && MODES=(sc)
[ "$MODE" = "rvv"  ] && MODES=(rvv)
[ "$MODE" = "both" ] && MODES=(sc rvv)

SC_CYCLES=""; RVV_CYCLES=""

for mode in "${MODES[@]}"; do
    # Choose algo name for benchmark_gem5
    if [ -n "$PCD_FILE" ]; then
        # bunny uses centimetre-scale params; other real PCDs use metre-scale
        if [ "$DATASET" = "bunny" ]; then
            algo_arg="pipeline_pcd_bunny_${mode}"
        else
            algo_arg="pipeline_pcd_${mode}"
        fi
        bin="$PCD_BIN"
        pcd_arg="$PCD_FILE"
    else
        algo_arg="${ALGO}_${mode}"
        bin="$ELF_BIN"
        pcd_arg=""
    fi

    outdir="${RUN_DIR}/${algo_arg}"
    info "Running ${algo_arg}  (N=${N_PTS}, cpu=${CPU_MODEL})..."

    run_gem5_once "$bin" "$algo_arg" "$N_PTS" "$outdir" "$pcd_arg"

    cycles=$(extract_cycles "$outdir")
    ns=$(extract_ns "$outdir")
    insts=$(extract_insts "$outdir")

    append_csv_row "$CSV" "$ALGO" "$mode" "$DATASET" "$N_PTS" \
                   "$cycles" "$ns" "$insts"

    [ "$mode" = "sc"  ] && SC_CYCLES="$cycles"
    [ "$mode" = "rvv" ] && RVV_CYCLES="$cycles"

    success "${algo_arg}: ${cycles} cycles  |  ${ns} ns  |  ${insts} sim_insts"
done

# Compute speedup if both modes ran
echo ""
echo -e "${BOLD}======================================================${RESET}"
if [ -n "$SC_CYCLES" ] && [ -n "$RVV_CYCLES" ] \
   && [ "$SC_CYCLES" != "N/A" ] && [ "$RVV_CYCLES" != "N/A" ] \
   && [ "$SC_CYCLES" != "FAILED" ] && [ "$RVV_CYCLES" != "FAILED" ]; then
    speedup=$(python3 -c \
        "print(f'Speedup (sc/rvv): {int(\"${SC_CYCLES}\")/int(\"${RVV_CYCLES}\"):.2f}x')" \
        2>/dev/null || echo "Speedup: N/A")
    echo -e "  ${GREEN}${speedup}${RESET}"
fi
echo -e "${BOLD}======================================================${RESET}"
echo ""
success "CSV saved: ${CSV}"
echo ""
echo "  gem5 stats per run:"
for mode in "${MODES[@]}"; do
    if [ -n "$PCD_FILE" ]; then
        algo_arg="pipeline_pcd_${mode}"
    else
        algo_arg="${ALGO}_${mode}"
    fi
    echo "    ${RUN_DIR}/${algo_arg}/stats.txt"
done
