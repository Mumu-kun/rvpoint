#!/bin/bash
set -e

# Build the benchmark first
echo "Building benchmark..."
rm -rf build_cmake
mkdir -p build_cmake
cd build_cmake
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv.cmake > /dev/null
make benchmark -j$(nproc) > /dev/null
cd ..

# Define algorithms
ALGOS=("voxel" "sor" "normal" "radius" "ransac")

echo "============================================================" > results/report_sc_rvv_qemu.txt
echo "  RVPoint Benchmark (Instruction Counts) - QEMU Trace       " >> results/report_sc_rvv_qemu.txt
echo "  *Measured by counting QEMU TB executions (-one-insn-per-tb)*" >> results/report_sc_rvv_qemu.txt
echo "============================================================" >> results/report_sc_rvv_qemu.txt
printf "%-20s | %-12s | %-12s | %s\n" "Algorithm" "Scalar(ins)" "RVV(ins)" "Ratio" >> results/report_sc_rvv_qemu.txt
echo "------------------------------------------------------------" >> results/report_sc_rvv_qemu.txt

for algo in "${ALGOS[@]}"; do
    echo "Running $algo..."
    
    # Run Setup (Baseline)
    SETUP_COUNT=$(/opt/riscv/bin/qemu-riscv64 -one-insn-per-tb -d exec,nochain build_cmake/benchmark $algo setup 2>&1 | grep -c "Trace")
    
    # Run Scalar
    SC_RAW=$(/opt/riscv/bin/qemu-riscv64 -one-insn-per-tb -d exec,nochain build_cmake/benchmark $algo sc 2>&1 | grep -c "Trace")
    SC_COUNT=$((SC_RAW - SETUP_COUNT))
    
    # Run RVV
    RVV_RAW=$(/opt/riscv/bin/qemu-riscv64 -one-insn-per-tb -d exec,nochain build_cmake/benchmark $algo rvv 2>&1 | grep -c "Trace")
    RVV_COUNT=$((RVV_RAW - SETUP_COUNT))
    
    # Calculate Ratio
    if [ $RVV_COUNT -gt 0 ]; then
        RATIO=$(echo "scale=2; $SC_COUNT / $RVV_COUNT" | bc)
    else
        RATIO="N/A"
    fi
    
    printf "%-20s | %-12s | %-12s | %sx\n" "$algo" "$SC_COUNT" "$RVV_COUNT" "$RATIO" >> results/report_sc_rvv_qemu.txt
done

echo "============================================================" >> results/report_sc_rvv_qemu.txt
cat results/report_sc_rvv_qemu.txt
