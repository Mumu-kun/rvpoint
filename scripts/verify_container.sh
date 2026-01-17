#!/bin/bash
set -e

# Get the root directory of the project
PROJECT_ROOT="$(dirname "$(dirname "$(readlink -f "$0")")")"
echo "Running verification from ${PROJECT_ROOT}..."
cd "${PROJECT_ROOT}"

echo "0. Running Workflow Checks..."
if [ -f "scripts/check_format.sh" ]; then
    bash scripts/check_format.sh
fi

if [ -f "scripts/lint.sh" ]; then
    if command -v cppcheck &> /dev/null; then
        bash scripts/lint.sh
    else
        echo "[WARNING] cppcheck not found. Skipping."
    fi
fi

echo "1. Configuring CMake..."
export CC=/opt/riscv/bin/riscv64-unknown-elf-gcc
export CXX=/opt/riscv/bin/riscv64-unknown-elf-g++

rm -rf build_cmake
mkdir -p build_cmake
cd build_cmake

cmake .. -G "Unix Makefiles"

echo "2. Building Project..."
make -j$(nproc)

echo "3. Running Scalar Test..."
/opt/riscv/bin/qemu-riscv64 -cpu max ./test_scalar

echo "4. Running Vector Test..."
/opt/riscv/bin/qemu-riscv64 -cpu max ./test_vector

echo "5. Running Voxel Grid Verification (Scalar vs RVV check)..."
/opt/riscv/bin/qemu-riscv64 -cpu max ./test_voxel_grid

echo "Verification Complete!"
