# RVV Point Cloud Pipeline Optimization Demo (Member: Arian)

This branch `arian_exps` contains the optimized RISC-V implementation of the PCL pipeline, specifically featuring:
1.  **Octree-based Normal Estimation**: Reduced complexity from O(N^2) to O(N log N).
2.  **Fused RVV Gather-Filter Kernel**: Optimized `vluxei32` implementation for high-performance radius search on RVV 1.0 hardware/emulator.

## Prerequisites
- RISC-V Toolchain (gcc 10+ with vector support)
- QEMU (`qemu-riscv64` with `v=true` support)
- CMake
- Python 3 + Matplotlib (for visualization)

## Quick Start

### 1. Build
```bash
mkdir -p build_cmake
cd build_cmake
cmake ..
make test_pipeline_walkthrough
```

### 2. Run Verification (Bunny Dataset)
Run the pipeline on the small `bunny.pcd` dataset for instant verification:
```bash
/usr/bin/qemu-riscv64 -cpu rv64,v=true -s 8192000 ./test_pipeline_walkthrough bunny.pcd
```
**Expected Output:**
- `Neighbors found within r=0.05: 81`
- `[PASS]` checks.

### 3. Run Stress Test (Table Scene)
Run on the larger dataset (this may take a few minutes on QEMU):
```bash
/usr/bin/qemu-riscv64 -cpu rv64,v=true -s 8192000 ./test_pipeline_walkthrough table_scene_lms400.pcd
```

### 4. Visualize Results
Use the python script to generate a 3D plot of the voxelized output:
```bash
python3 ../scripts/visualize_result.py bunny_voxelized.pcd bunny_view.png
```
(Outputs `bunny_view.png`)

## Key Files
- `src/rvv_common.cpp`: Contains the fused `get_inds_in_radius_rvv` kernel.
- `src/normal_estimation.cpp`: Contains the Octree-optimized normal estimation logic.
- `tests/test_pipeline_walkthrough.cpp`: The main test harness.
