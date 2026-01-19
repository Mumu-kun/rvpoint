#!/bin/bash
set -e

# Compiler and Flags
CXX="/opt/riscv/bin/riscv64-unknown-elf-g++"
FLAGS="-march=rv64gcv -mabi=lp64d -I src/include"
QEMU="qemu-riscv64 -cpu rv64,v=true,vlen=128"

echo "========================================"
echo "Verifying Voxel Grid..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/voxel_grid_downsamp.cpp tests/test_voxel_grid.cpp -o test_voxel_rvv_manual
$QEMU ./test_voxel_rvv_manual

echo ""
echo "========================================"
echo "Verifying RANSAC Plane..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/ransac_plane.cpp tests/test_ransac.cpp -o test_ransac_rvv_manual
$QEMU ./test_ransac_rvv_manual

echo ""
echo "========================================"
echo "Verifying Radius Search..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/radius_search.cpp tests/test_radius.cpp -o test_radius_rvv_manual
$QEMU ./test_radius_rvv_manual

echo ""
echo "========================================"
echo "Verifying Statistical Outlier Removal..."
echo "========================================"
$CXX $FLAGS src/rvv_common.cpp src/statistical_outlier_removal.cpp tests/test_sor.cpp -o test_sor_rvv_manual
$QEMU ./test_sor_rvv_manual

echo ""
echo "All tests passed!"
