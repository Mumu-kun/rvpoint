#!/bin/bash
# =============================================================================
#  gem5 Cycle-Accurate Benchmark for RVPoint
#
#  Runs scalar vs RVV algorithms through gem5 SE mode, extracts cycle counts
#  from stats.txt, and generates a hardware-accurate comparison report.
#
#  Prerequisites:
#    - gem5 built at /opt/gem5/build/RISCV/gem5.opt  (run gem5_setup.sh first)
#    - benchmark_gem5_static binary built (done automatically by this script)
#
#  Output: results/gem5_report_YYYYMMDD_HHMMSS.txt
#
#  Model: RISC-V O3CPU @ 1 GHz, 32KB L1, 256KB L2, DDR4-2400
#  N: 512 (gem5 is ~1000x slower than real HW; N=512 keeps runtime ~20 min)
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Prefer v25+ (has vcompress.vm support) over v24 if available
GEM5_DIR="/opt/gem5"
[ -f "/opt/gem5-25/build/RISCV/gem5.opt" ] && GEM5_DIR="/opt/gem5-25"
GEM5_BIN="${GEM5_DIR}/build/RISCV/gem5.opt"
STATIC_BIN="${PROJECT_ROOT}/bin/benchmark_gem5_static"
PCD_STATIC_BIN="${PROJECT_ROOT}/bin/benchmark_gem5_pcd_static"
RESULTS_DIR="${PROJECT_ROOT}/results"
GEM5_CONFIG="${SCRIPT_DIR}/gem5_se.py"

# Benchmark N — keep small for gem5 (each run ~ 2-5 min at N=512)
N="${GEM5_N:-512}"
CPU_MODEL="${GEM5_CPU:-o3}"   # o3 | minor | timing

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'
info()    { echo -e "${CYAN}==>${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
error()   { echo -e "${RED}[ERR]${RESET} $*" >&2; }

# ── Sanity checks ─────────────────────────────────────────────────────────────
if [ ! -f "$GEM5_BIN" ]; then
    error "gem5 not found at ${GEM5_BIN}"
    error "Run: ./scripts/rvpoint.sh  → option 6 (gem5 Setup)"
    exit 1
fi

# ── Build static binary (elf toolchain — no sysroot needed for gem5 SE) ───────
ELF_GCC="$(command -v riscv64-unknown-elf-g++ 2>/dev/null || true)"
if [ -z "$ELF_GCC" ]; then
    ELF_GCC="${RISCV:-/opt/riscv}/bin/riscv64-unknown-elf-g++"
fi
if [ ! -x "$ELF_GCC" ]; then
    error "riscv64-unknown-elf-g++ not found. Is the container running?"
    exit 1
fi

info "Building benchmark_gem5_static (ELF toolchain, static)..."
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
"$ELF_GCC" -march=rv64gcv -mabi=lp64d -O2 -std=c++17 -static \
    -DGEM5_BUILD \
    -I"${PROJECT_ROOT}/src/include" \
    "${SRCS[@]}" \
    -o "$STATIC_BIN" \
    -lstdc++ -lm -lc
success "Built: ${STATIC_BIN}"

# ── Build PCD static binary (linux-gnu toolchain — glibc for real file I/O) ───
# The ELF/newlib toolchain stubs out open() for bare-metal; the linux-gnu
# toolchain uses glibc which emits real Linux ecall instructions that gem5 SE
# intercepts, allowing std::ifstream to open PCD files inside the simulation.
LINUX_GCC="${RISCV:-/opt/riscv}/bin/riscv64-unknown-linux-gnu-g++"
if [ ! -x "$LINUX_GCC" ]; then
    LINUX_GCC="$(command -v riscv64-unknown-linux-gnu-g++ 2>/dev/null || true)"
fi

if [ -x "$LINUX_GCC" ]; then
    info "Building benchmark_gem5_pcd_static (linux-gnu toolchain, static + glibc)..."
    "$LINUX_GCC" -march=rv64gcv -mabi=lp64d -O2 -std=c++17 -static \
        -DGEM5_BUILD \
        -I"${PROJECT_ROOT}/src/include" \
        "${SRCS[@]}" \
        -o "$PCD_STATIC_BIN"
    success "Built: ${PCD_STATIC_BIN}"
    HAVE_PCD_BIN=1
else
    warn "riscv64-unknown-linux-gnu-g++ not found — PCD benchmark will be skipped."
    HAVE_PCD_BIN=0
fi

# ── Stat extraction helpers ────────────────────────────────────────────────────
# gem5 stats.txt format: "stat.name    value    # description"
extract_stat() {
    local file="$1/stats.txt"
    local key="$2"
    grep -m1 "^${key}" "$file" 2>/dev/null | awk '{print $2}' || echo "N/A"
}

# Try multiple stat name patterns — gem5 renames stats between versions
extract_cycles() {
    local dir="$1"
    [ -f "${dir}/.failed" ] && echo "UNSUPPORTED" && return
    local v
    # gem5 v23+: system.cpu.numCycles
    v=$(extract_stat "$dir" "system.cpu.numCycles")
    [ "$v" != "N/A" ] && [ "$v" != "0" ] && echo "$v" && return
    # gem5 older: system.cpu.cpi / sim_insts
    local insts ipc
    insts=$(extract_stat "$dir" "sim_insts")
    ipc=$(extract_stat "$dir" "system.cpu.ipc")
    if [ "$insts" != "N/A" ] && [ "$ipc" != "N/A" ] && [ "$ipc" != "0" ]; then
        # cycles = insts / IPC
        python3 -c "print(int(${insts} / ${ipc}))" 2>/dev/null || echo "N/A"
        return
    fi
    echo "N/A"
}

extract_seconds() { extract_stat "$1" "simSeconds"; }
# gem5 v24 renamed sim_insts → simInsts
extract_insts() {
    local v
    v=$(extract_stat "$1" "simInsts")
    [ "$v" != "N/A" ] && echo "$v" && return
    extract_stat "$1" "sim_insts"   # fallback for older gem5
}

# ── Run one gem5 simulation ───────────────────────────────────────────────────
run_gem5() {
    local algo="$1"
    local outdir="$2"
    mkdir -p "$outdir"

    # Run gem5 — allow non-zero exit (gem5 aborts on unsupported instructions)
    "$GEM5_BIN" \
        --outdir="$outdir" \
        "${GEM5_CONFIG}" \
        --cmd="$STATIC_BIN" \
        --options="${algo} ${N}" \
        --cpu="$CPU_MODEL" \
        > "${outdir}/gem5_stdout.txt" 2>&1 || true

    # Check for illegal instruction fault (unsupported RVV opcode in gem5 v24)
    if grep -q "IllegalInstFault\|illegal instruction\|panic:" "${outdir}/gem5_stdout.txt" 2>/dev/null; then
        warn "  ${algo}: unsupported RVV instruction in gem5 v24 — result will show N/A"
        echo "ILLEGAL_INST" > "${outdir}/.failed"
    elif ! grep -q "gem5_bench:" "${outdir}/gem5_stdout.txt" 2>/dev/null; then
        warn "  ${algo}: binary may not have completed — check ${outdir}/gem5_stdout.txt"
    fi
}

# ── Main benchmark loop ───────────────────────────────────────────────────────
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
mkdir -p "$RESULTS_DIR"
REPORT="${RESULTS_DIR}/gem5_report_${TIMESTAMP}.txt"
GEM5_OUTBASE="${RESULTS_DIR}/gem5_runs_${TIMESTAMP}"

# Algorithms to benchmark (sc/rvv pairs)
ALGOS=(voxel ransac sor normal pipeline)

# Print header to screen and file simultaneously
tee_out() { echo "$@" | tee -a "$REPORT"; }

tee_out "Saving report to: ${REPORT}"
tee_out ""
tee_out "========================================================================"
tee_out "  RVPoint gem5 Cycle-Accurate Benchmark"
tee_out "  Model: RISC-V ${CPU_MODEL^^}CPU @ 1 GHz | L1 32KB | L2 256KB | DDR4-2400 | 1GB RAM"
tee_out "  N=${N}  (use GEM5_N=<val> to override)"
tee_out "  Note: gem5 simulates ~1-10 MIPS; each run takes 2-5 minutes."
tee_out "  RVV support: basic vector instructions (v1.0 subset in gem5 v24)"
tee_out "========================================================================"
tee_out ""
tee_out "Algorithm    | Scalar cycles | RVV cycles   | Speedup | Scalar ns  | RVV ns"
tee_out "-------------|---------------|--------------|---------|------------|----------"

declare -A SC_CYCLES RVV_CYCLES SC_NS RVV_NS

for algo in "${ALGOS[@]}"; do
    sc_algo="${algo}_sc"
    rv_algo="${algo}_rvv"
    sc_dir="${GEM5_OUTBASE}/${sc_algo}"
    rv_dir="${GEM5_OUTBASE}/${rv_algo}"

    info "Running ${sc_algo} (N=${N})..."
    run_gem5 "$sc_algo" "$sc_dir"
    sc_cyc=$(extract_cycles "$sc_dir")
    sc_sec=$(extract_seconds "$sc_dir")

    info "Running ${rv_algo} (N=${N})..."
    run_gem5 "$rv_algo" "$rv_dir"
    rv_cyc=$(extract_cycles "$rv_dir")
    rv_sec=$(extract_seconds "$rv_dir")

    # Convert simSeconds to nanoseconds (1GHz clock → 1 cycle = 1ns)
    sc_ns="N/A"; rv_ns="N/A"; speedup="N/A"
    if [ "$sc_sec" != "N/A" ]; then
        sc_ns=$(python3 -c "print(f'{float(\"${sc_sec}\")*1e9:.1f}')" 2>/dev/null || echo "N/A")
    fi
    if [ "$rv_sec" != "N/A" ]; then
        rv_ns=$(python3 -c "print(f'{float(\"${rv_sec}\")*1e9:.1f}')" 2>/dev/null || echo "N/A")
    fi
    if [ "$sc_cyc" != "N/A" ] && [ "$rv_cyc" != "N/A" ] && [ "$rv_cyc" != "0" ] \
       && [ "$sc_cyc" != "UNSUPPORTED" ] && [ "$rv_cyc" != "UNSUPPORTED" ]; then
        speedup=$(python3 -c "print(f'{int(\"${sc_cyc}\")/int(\"${rv_cyc}\"):.2f}x')" 2>/dev/null || echo "N/A")
    fi

    SC_CYCLES[$algo]=$sc_cyc
    RVV_CYCLES[$algo]=$rv_cyc
    SC_NS[$algo]=$sc_ns
    RVV_NS[$algo]=$rv_ns

    printf "%-13s| %-14s| %-13s| %-8s| %-11s| %s\n" \
        "$algo" "$sc_cyc" "$rv_cyc" "$speedup" "$sc_ns" "$rv_ns" \
        | tee -a "$REPORT"
done

tee_out "========================================================================"
tee_out ""
tee_out "Columns:"
tee_out "  Scalar/RVV cycles : actual simulated CPU cycles (not instruction count)"
tee_out "  Speedup           : scalar_cycles / rvv_cycles  (real cycle-level ratio)"
tee_out "  Scalar/RVV ns     : simulated nanoseconds at 1 GHz (= cycles at 1 GHz)"
tee_out ""
tee_out "Cross-validation (rdinstret instruction count vs gem5 sim_insts):"
tee_out "  If sim_insts ≈ rdinstret, the gem5 run executed the same code path."
tee_out ""
tee_out "Detailed stats per run: ${GEM5_OUTBASE}/<algo>/stats.txt"
tee_out ""

# Cross-validation table
tee_out "Algorithm      | gem5 sim_insts (sc) | gem5 sim_insts (rvv)"
tee_out "---------------|---------------------|---------------------"
for algo in "${ALGOS[@]}"; do
    sc_dir="${GEM5_OUTBASE}/${algo}_sc"
    rv_dir="${GEM5_OUTBASE}/${algo}_rvv"
    sc_i=$(extract_insts "$sc_dir")
    rv_i=$(extract_insts "$rv_dir")
    printf "%-15s| %-20s| %s\n" "$algo" "$sc_i" "$rv_i" | tee -a "$REPORT"
done

tee_out "========================================================================"

# ── Real LiDAR PCD benchmark ──────────────────────────────────────────────────
# Runs the full pipeline on a real PCD file, subsampled to GEM5_PCD_N points.
# Skip if no PCD file is configured.
PCD_FILE="${GEM5_PCD:-${PROJECT_ROOT}/data/table_scene_lms400.pcd}"
PCD_N="${GEM5_PCD_N:-512}"

if [ "${HAVE_PCD_BIN:-0}" -eq 1 ] && [ -f "$PCD_FILE" ]; then
    tee_out ""
    tee_out "========================================================================"
    tee_out "  Real LiDAR PCD Benchmark (gem5 cycle-accurate)"
    tee_out "  File: ${PCD_FILE}"
    tee_out "  Subsampled to N=${PCD_N} pts  (use GEM5_PCD_N=<val> to override)"
    tee_out "  Params: leaf=0.05m  ransac_thresh=0.05m  norm_r=0.1m"
    tee_out "========================================================================"
    tee_out ""
    tee_out "Mode             | Scalar cycles | RVV cycles   | Speedup"
    tee_out "-----------------|---------------|--------------|--------"

    pcd_sc_dir="${GEM5_OUTBASE}/pipeline_pcd_sc"
    pcd_rv_dir="${GEM5_OUTBASE}/pipeline_pcd_rvv"

    info "Running pipeline_pcd_sc (N=${PCD_N}, real LiDAR)..."
    run_gem5_pcd() {
        local mode="$1"
        local outdir="$2"
        mkdir -p "$outdir"
        "$GEM5_BIN" \
            --outdir="$outdir" \
            "${GEM5_CONFIG}" \
            --cmd="$PCD_STATIC_BIN" \
            --options="${mode} ${PCD_N} ${PCD_FILE}" \
            --cpu="$CPU_MODEL" \
            > "${outdir}/gem5_stdout.txt" 2>&1 || true

        if grep -q "IllegalInstFault\|illegal instruction\|panic:" "${outdir}/gem5_stdout.txt" 2>/dev/null; then
            warn "  ${mode}: unsupported instruction in gem5 — result will show N/A"
            echo "ILLEGAL_INST" > "${outdir}/.failed"
        elif ! grep -q "gem5_bench:" "${outdir}/gem5_stdout.txt" 2>/dev/null; then
            warn "  ${mode}: binary may not have completed — check ${outdir}/gem5_stdout.txt"
        fi
    }

    run_gem5_pcd "pipeline_pcd_sc" "$pcd_sc_dir"
    pcd_sc_cyc=$(extract_cycles "$pcd_sc_dir")
    pcd_sc_sec=$(extract_seconds "$pcd_sc_dir")

    info "Running pipeline_pcd_rvv (N=${PCD_N}, real LiDAR)..."
    run_gem5_pcd "pipeline_pcd_rvv" "$pcd_rv_dir"
    pcd_rv_cyc=$(extract_cycles "$pcd_rv_dir")
    pcd_rv_sec=$(extract_seconds "$pcd_rv_dir")

    pcd_speedup="N/A"
    if [ "$pcd_sc_cyc" != "N/A" ] && [ "$pcd_rv_cyc" != "N/A" ] \
       && [ "$pcd_rv_cyc" != "0" ] \
       && [ "$pcd_sc_cyc" != "UNSUPPORTED" ] && [ "$pcd_rv_cyc" != "UNSUPPORTED" ]; then
        pcd_speedup=$(python3 -c "print(f'{int(\"${pcd_sc_cyc}\")/int(\"${pcd_rv_cyc}\"):.2f}x')" 2>/dev/null || echo "N/A")
    fi

    printf "%-17s| %-14s| %-13s| %s\n" \
        "pipeline_pcd" "$pcd_sc_cyc" "$pcd_rv_cyc" "$pcd_speedup" \
        | tee -a "$REPORT"

    tee_out "========================================================================"
    tee_out ""
    tee_out "PCD pipeline stdout (rdinstret stage breakdown):"
    tee_out "  Scalar: ${pcd_sc_dir}/gem5_stdout.txt"
    tee_out "  RVV:    ${pcd_rv_dir}/gem5_stdout.txt"
    tee_out ""
    tee_out "Note: rdinstret counts inside the binary output are also cycle-validated"
    tee_out "      by gem5 — they show per-stage instruction counts on real LiDAR data."
    tee_out "========================================================================"
else
    tee_out ""
    if [ "${HAVE_PCD_BIN:-0}" -eq 0 ]; then
        tee_out "Skipping PCD benchmark: linux-gnu toolchain not found (needed for file I/O)."
    else
        tee_out "Skipping PCD benchmark: ${PCD_FILE} not found."
        tee_out "  Set GEM5_PCD=/path/to/cloud.pcd to enable."
    fi
fi

success "Report saved to: ${REPORT}"
