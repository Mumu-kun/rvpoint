#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Build the benchmark first
echo "Building benchmark..."
"$SCRIPT_DIR/build.sh" --toolchain linux > /dev/null

# Define algorithms
ALGOS=("voxel" "sor" "normal" "radius" "ransac")

mkdir -p "$PROJECT_ROOT/results"
echo "============================================================" > "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
echo "  RVPoint Benchmark (Instruction Counts) - QEMU Trace       " >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
echo "  *Measured by counting QEMU TB executions (-one-insn-per-tb)*" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
echo "============================================================" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
printf "%-20s | %-12s | %-12s | %s\n" "Algorithm" "Scalar(ins)" "RVV(ins)" "Ratio" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
echo "------------------------------------------------------------" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"

for algo in "${ALGOS[@]}"; do
    echo "Running $algo..."
    
    # Run Setup (Baseline)
    SETUP_COUNT=$(qemu-riscv64 -one-insn-per-tb -d exec,nochain "$PROJECT_ROOT/bin/benchmark" "$algo" setup 2>&1 | grep -c "Trace")
    
    # Run Scalar
    SC_RAW=$(qemu-riscv64 -one-insn-per-tb -d exec,nochain "$PROJECT_ROOT/bin/benchmark" "$algo" sc 2>&1 | grep -c "Trace")
    SC_COUNT=$((SC_RAW - SETUP_COUNT))
    
    # Run RVV
    RVV_RAW=$(qemu-riscv64 -one-insn-per-tb -d exec,nochain "$PROJECT_ROOT/bin/benchmark" "$algo" rvv 2>&1 | grep -c "Trace")
    RVV_COUNT=$((RVV_RAW - SETUP_COUNT))
    
    # Calculate Ratio
    if [ $RVV_COUNT -gt 0 ]; then
        RATIO=$(echo "scale=2; $SC_COUNT / $RVV_COUNT" | bc)
    else
        RATIO="N/A"
    fi
    
    printf "%-20s | %-12s | %-12s | %sx\n" "$algo" "$SC_COUNT" "$RVV_COUNT" "$RATIO" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
done

echo "============================================================" >> "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
cat "$PROJECT_ROOT/results/report_sc_rvv_qemu.txt"
