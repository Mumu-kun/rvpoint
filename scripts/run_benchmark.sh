#!/bin/bash
set -e
# Colors
BLUE="\033[1;34m"
RESET="\033[0m"

echo -e "${BLUE}Building Benchmarks...${RESET}"
mkdir -p build_cmake
cd build_cmake || exit 1
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv.cmake > /dev/null
make benchmark -j$(nproc) > /dev/null
cd ..

echo -e "${BLUE}Running Benchmark on QEMU...${RESET}"
/opt/riscv/bin/qemu-riscv64 -cpu max build_cmake/benchmark
