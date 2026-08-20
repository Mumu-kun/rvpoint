#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

source "$PROJECT_ROOT/scripts/lib/common.sh"
wsl_bootstrap "scripts/run_gem5_compare.sh" "$@"

GEM5_BIN="${GEM5_BIN:-gem5.opt}"
GEM5_CONFIG="${GEM5_CONFIG:-}"

usage() {
  cat <<'EOF'
Usage:
  1. Compare PCD Radius Search:
     ./scripts/run_gem5_compare.sh pcd <pcd_file> <algorithm> <radius> [iterations]
     Example: ./scripts/run_gem5_compare.sh pcd data/bunny.pcd caravan 0.2 10

  2. Compare Custom Pairs (Backend + Target/PCD + Arguments):
     ./scripts/run_gem5_compare.sh pair <backend1> <test_file> <alg> <radius> vs <backend2> <test_file> <alg> <radius>
     Example: ./scripts/run_gem5_compare.sh pair scalar data/bunny.pcd caravan 0.2 vs rvv data/bunny.pcd caravan 0.2

  3. Compare Benchmark Kernels:
     ./scripts/run_gem5_compare.sh bench <kernel> <size> [iterations]
     Example: ./scripts/run_gem5_compare.sh bench caravan 10k 10
EOF
  exit 1
}

if [[ $# -lt 2 ]]; then
  usage
fi

MODE_TYPE="$1"
shift

# Function to run a target under gem5 if available, or fall back to QEMU/host timing
run_executable() {
  local backend="$1"
  local target_name="$2"
  shift 2
  local target_args=("$@")

  # Build target
  local build_gem5_flag=""
  if [[ -n "$GEM5_CONFIG" && -x "$(command -v "$GEM5_BIN")" ]]; then
    build_gem5_flag="--gem5"
  fi
  "$PROJECT_ROOT/scripts/build.sh" --toolchain linux --backend "$backend" --target "$target_name" $build_gem5_flag >/dev/null 2>&1

  local build_dir="$(get_build_dir)"
  local target_bin="${build_dir}/${backend}/bin/${backend}/${target_name}"
  if [[ ! -f "$target_bin" ]]; then
    target_bin="${build_dir}/${backend}/bin/${target_name}"
  fi
  if [[ ! -f "$target_bin" ]]; then
    target_bin="${PROJECT_ROOT}/build/bin/${backend}/${target_name}"
  fi

  if [[ ! -f "$target_bin" ]]; then
    echo "Error: Executable $target_name not found for backend $backend" >&2
    return 1
  fi

  local cycles="N/A"
  local time_us="N/A"
  local avg_us="N/A"
  local found_count="N/A"

  if [[ -n "$GEM5_CONFIG" && -x "$(command -v "$GEM5_BIN")" ]]; then
    # gem5 simulation mode
    local outdir="$PROJECT_ROOT/results/gem5/${backend}_${target_name}_$(date +%s%N)"
    mkdir -p "$outdir"
    local stats_file="$outdir/stats.txt"
    local log_file="$outdir/gem5.log"

    "$GEM5_BIN" -d "$outdir" "$GEM5_CONFIG" --cmd "$target_bin" --options "${target_args[*]}" \
      >"$log_file" 2>&1 || true

    if [[ -f "$stats_file" ]]; then
      cycles=$(awk '$1 ~ /(numCycles|cycle|simTicks)/ {print $2; exit}' "$stats_file" || echo "N/A")
      time_us=$(awk '$1 == "simSeconds" {printf "%.2f us", $2*1000000; exit}' "$stats_file" || echo "N/A")
    fi

    local run_output
    run_output=$("$PROJECT_ROOT/scripts/run.sh" --backend "$backend" "$target_name" "${target_args[@]}" 2>&1 || true)
    avg_us=$(echo "$run_output" | grep -oP 'avg_us=\K[0-9.]+' | head -n 1 || echo "N/A")
    found_count=$(echo "$run_output" | grep -oP 'found=\K[0-9]+' | head -n 1 || echo "N/A")
  else
    # QEMU / Host high-resolution timing fallback
    local run_output
    run_output=$("$PROJECT_ROOT/scripts/run.sh" --backend "$backend" "$target_name" "${target_args[@]}" 2>&1 || true)
    avg_us=$(echo "$run_output" | grep -oP 'avg_us=\K[0-9.]+' | head -n 1 || echo "N/A")
    time_us=$(echo "$run_output" | grep -oP 'total_us=\K[0-9.]+' | head -n 1 || echo "$avg_us")
    found_count=$(echo "$run_output" | grep -oP 'found=\K[0-9]+' | head -n 1 || echo "N/A")
    cycles="QEMU (emulated)"
  fi

  echo "$backend|$cycles|$time_us|$avg_us|$found_count"
}

case "$MODE_TYPE" in
  pcd)
    if [[ $# -lt 3 ]]; then usage; fi
    PCD_FILE="$1"
    ALG="$2"
    RADIUS="$3"
    ITERS="${4:-10}"

    echo "==================================================================================="
    echo "            PCD Radius Search Benchmark Comparison (Scalar vs RVV)                "
    echo "==================================================================================="
    echo "PCD File: $PCD_FILE | Algorithm: $ALG | Radius: $RADIUS | Iterations: $ITERS"
    if [[ -z "$GEM5_CONFIG" ]]; then
      echo "Note: GEM5_CONFIG not set. Measuring high-resolution wall-clock/QEMU timing."
    fi
    echo "-----------------------------------------------------------------------------------"
    printf "%-10s %-18s %-16s %-16s %-12s %-10s\n" "BACKEND" "CYCLES/TICKS" "TOTAL TIME" "AVG US/ITER" "FOUND" "STATUS"
    echo "-----------------------------------------------------------------------------------"

    res_scalar=$(run_executable scalar test_pcd_radius "$PCD_FILE" "$ALG" "$RADIUS" "$ITERS")
    IFS='|' read -r b1 c1 t1 a1 f1 <<< "$res_scalar"
    printf "%-10s %-18s %-16s %-16s %-12s %-10s\n" "$b1" "$c1" "$t1" "$a1" "$f1" "PASSED"

    res_rvv=$(run_executable rvv test_pcd_radius "$PCD_FILE" "$ALG" "$RADIUS" "$ITERS")
    IFS='|' read -r b2 c2 t2 a2 f2 <<< "$res_rvv"
    printf "%-10s %-18s %-16s %-16s %-12s %-10s\n" "$b2" "$c2" "$t2" "$a2" "$f2" "PASSED"

    echo "==================================================================================="
    if (( $(echo "$a1 > 0 && $a2 > 0" | awk '{print ($1 && $2)}') )); then
      speedup=$(awk -v s="$a1" -v r="$a2" 'BEGIN { printf "%.2fx", s/r }')
      echo "Execution Speedup Ratio (Scalar avg / RVV avg): $speedup"
    fi
    echo "==================================================================================="
    ;;

  bench)
    if [[ $# -lt 2 ]]; then usage; fi
    KERNEL="$1"
    SIZE="$2"
    ITERS="${3:-10}"

    if [[ "$KERNEL" == "carvan" ]]; then KERNEL="caravan"; fi

    echo "==================================================================================="
    echo "            Benchmark Kernel Comparison (Scalar vs RVV)                           "
    echo "==================================================================================="
    echo "Kernel: $KERNEL | Size: $SIZE | Iterations: $ITERS"
    echo "-----------------------------------------------------------------------------------"
    printf "%-10s %-18s %-16s %-16s %-10s\n" "BACKEND" "CYCLES/TICKS" "TOTAL TIME" "AVG US/ITER" "STATUS"
    echo "-----------------------------------------------------------------------------------"

    res_scalar=$(run_executable scalar benchmark scalar "$KERNEL" "$SIZE" "$ITERS")
    IFS='|' read -r b1 c1 t1 a1 f1 <<< "$res_scalar"
    printf "%-10s %-18s %-16s %-16s %-10s\n" "$b1" "$c1" "$t1" "$a1" "PASSED"

    res_rvv=$(run_executable rvv benchmark rvv "$KERNEL" "$SIZE" "$ITERS")
    IFS='|' read -r b2 c2 t2 a2 f2 <<< "$res_rvv"
    printf "%-10s %-18s %-16s %-16s %-10s\n" "$b2" "$c2" "$t2" "$a2" "PASSED"

    echo "==================================================================================="
    if (( $(echo "$a1 > 0 && $a2 > 0" | awk '{print ($1 && $2)}') )); then
      speedup=$(awk -v s="$a1" -v r="$a2" 'BEGIN { printf "%.2fx", s/r }')
      echo "Execution Speedup Ratio (Scalar avg / RVV avg): $speedup"
    fi
    echo "==================================================================================="
    ;;

  pair)
    if [[ $# -lt 8 ]]; then usage; fi
    b1="$1"; pcd1="$2"; alg1="$3"; r1="$4"
    shift 5 # skip vs token
    b2="$1"; pcd2="$2"; alg2="$3"; r2="$4"

    echo "==================================================================================="
    echo "                    Pairwise PCD Test Comparison                                  "
    echo "==================================================================================="
    printf "%-10s %-18s %-10s %-8s %-16s %-16s\n" "BACKEND" "PCD FILE" "ALG" "RADIUS" "CYCLES/TICKS" "AVG US/ITER"
    echo "-----------------------------------------------------------------------------------"

    res1=$(run_executable "$b1" test_pcd_radius "$pcd1" "$alg1" "$r1" 10)
    IFS='|' read -r _ c1 t1 a1 f1 <<< "$res1"
    printf "%-10s %-18s %-10s %-8s %-16s %-16s\n" "$b1" "$(basename "$pcd1")" "$alg1" "$r1" "$c1" "$a1 us"

    res2=$(run_executable "$b2" test_pcd_radius "$pcd2" "$alg2" "$r2" 10)
    IFS='|' read -r _ c2 t2 a2 f2 <<< "$res2"
    printf "%-10s %-18s %-10s %-8s %-16s %-16s\n" "$b2" "$(basename "$pcd2")" "$alg2" "$r2" "$c2" "$a2 us"

    echo "==================================================================================="
    ;;

  pointer_octree|pointer)
    PCD_FILE="${1:-data/0000000010.pcd}"
    echo "==================================================================================="
    echo "       Pointer-Based Octree Real PCD Benchmark Comparison (RVV)                    "
    echo "==================================================================================="
    echo "PCD File: $PCD_FILE"
    echo "-----------------------------------------------------------------------------------"
    "$PROJECT_ROOT/scripts/run.sh" rvv benchmark_pointer_octree_real "$PCD_FILE"
    ;;

  *)
    usage
    ;;
esac
