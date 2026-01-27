#!/bin/bash
# scripts/run_benchmarks.sh

set -e

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
REPORT_FILE="results/benchmark_report_${TIMESTAMP}.txt"
QEMU_BIN="/usr/bin/qemu-riscv64"
QEMU_FLAGS="-L /opt/riscv/sysroot -cpu rv64,v=true"
BENCH_BIN="bin/benchmark"

mkdir -p results

echo "========================================" | tee -a "$REPORT_FILE"
echo "   RVPoint Performance Benchmark Report " | tee -a "$REPORT_FILE"
echo "   Timestamp: $TIMESTAMP"              | tee -a "$REPORT_FILE"
echo "========================================" | tee -a "$REPORT_FILE"

ALGOS=("voxel" "sor" "normal" "radius" "ransac")

for ALGO in "${ALGOS[@]}"; do
    echo "[RUNNING] $ALGO..." | tee -a "$REPORT_FILE"
    
    # Run Scalar
    SC_OUT=$($QEMU_BIN $QEMU_FLAGS $BENCH_BIN "$ALGO" "sc" 2>&1)
    
    # Run RVV
    RVV_OUT=$($QEMU_BIN $QEMU_FLAGS $BENCH_BIN "$ALGO" "rvv" 2>&1)
    
    echo "$SC_OUT" >> "$REPORT_FILE"
    echo "$RVV_OUT" >> "$REPORT_FILE"
    echo "----------------------------------------" >> "$REPORT_FILE"
done

echo "[SUCCESS] Benchmark report saved to $REPORT_FILE"
