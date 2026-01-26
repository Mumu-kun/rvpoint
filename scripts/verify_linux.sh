#!/bin/bash
set -e

# Compiler and Flags
RISCV_PATH=${RISCV_PATH:-"/opt/riscv"}
CXX="${RISCV_PATH}/bin/riscv64-unknown-linux-gnu-g++"
FLAGS="-march=rv64gcv -mabi=lp64d -I src/include -static" 
# Note: -static is often useful for qemu-user verification to avoid library path issues, 
# though we are installing glibc so dynamic linking should work if paths are set correctly.
# Let's try dynamic first, if it fails we can switch to static or set QEMU_LD_PREFIX.

# Use qemu-riscv64 from PATH (typically /usr/bin/qemu-riscv64)
QEMU="qemu-riscv64 -cpu rv64,v=true,vlen=128 -L ${RISCV_PATH}/sysroot"

echo "========================================"
echo "Verifying Voxel Grid (Linux Toolchain)..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/voxel_grid_downsamp.cpp tests/test_voxel_grid.cpp -o test_voxel_rvv_linux
$QEMU ./test_voxel_rvv_linux

echo ""
echo "========================================"
echo "Verifying RANSAC Plane (Linux Toolchain)..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/ransac_plane.cpp tests/test_ransac.cpp -o test_ransac_rvv_linux
$QEMU ./test_ransac_rvv_linux

echo ""
echo "========================================"
echo "Verifying Radius Search (Linux Toolchain)..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/radius_search.cpp tests/test_radius.cpp -o test_radius_rvv_linux
$QEMU ./test_radius_rvv_linux

echo ""
echo "========================================"
echo "Verifying Statistical Outlier Removal (Linux Toolchain)..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/statistical_outlier_removal.cpp tests/test_sor.cpp -o test_sor_rvv_linux
$QEMU ./test_sor_rvv_linux

echo ""
echo "All tests passed with Linux Toolchain!"
