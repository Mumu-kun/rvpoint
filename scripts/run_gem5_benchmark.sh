#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BENCH_BIN="$PROJECT_ROOT/bin/benchmark"

if [[ $# -ne 3 ]]; then
  echo "Usage: $0 <mode> <kernel> <size>" >&2
  exit 1
fi

MODE="$1"
KERNEL="$2"
SIZE="$3"

GEM5_BIN="${GEM5_BIN:-gem5.opt}"
GEM5_CONFIG="${GEM5_CONFIG:-}"
if [[ -z "$GEM5_CONFIG" ]]; then
  echo "Set GEM5_CONFIG to the gem5 config script for SE benchmarking." >&2
  echo "Example: export GEM5_CONFIG=/path/to/gem5/configs/example/se.py" >&2
  exit 2
fi

OUTDIR="$PROJECT_ROOT/results/gem5/${MODE}_${KERNEL}_${SIZE}"
mkdir -p "$OUTDIR"

LOG_FILE="$OUTDIR/gem5.log"
STATS_FILE="$OUTDIR/stats.txt"

"$GEM5_BIN" -d "$OUTDIR" "$GEM5_CONFIG" --cmd "$BENCH_BIN" --options "$MODE $KERNEL $SIZE" \
  >"$LOG_FILE" 2>&1

if [[ ! -f "$STATS_FILE" ]]; then
  echo "gem5 stats file not found: $STATS_FILE" >&2
  echo "See log: $LOG_FILE" >&2
  exit 3
fi

parse_cycles() {
  local file="$1"
  local key value
  for key in \
    system.cpu.numCycles \
    system.cpu.cycle \
    system.cpus.numCycles \
    system.cpus.cycle \
    simTicks; do
    value=$(awk -v key="$key" '$1 == key {print $2; exit}' "$file")
    if [[ -n "$value" ]]; then
      echo "$value"
      return 0
    fi
  done

  value=$(awk '$1 ~ /(numCycles|cycle|simTicks)/ {print $2; exit}' "$file")
  if [[ -n "$value" ]]; then
    echo "$value"
    return 0
  fi

  return 1
}

if ! CYCLES="$(parse_cycles "$STATS_FILE")"; then
  echo "Unable to parse cycle count from $STATS_FILE" >&2
  exit 4
fi

echo "$MODE $KERNEL $SIZE $CYCLES"