# RVV Point Cloud Pipeline Demo (Branch: arian_exps)

This branch contains the optimized RISC-V Vector (RVV 1.0) implementation of the PCL pipeline.

## Key Optimizations
1.  **Octree-based Spatial Search**: Reduced neighbor search complexity from O(N²) to O(N log N).
2.  **Fused RVV Gather-Filter Kernel**: High-performance `vluxei32` implementation with byte-offset correction.
3.  **Explicit Pipeline Steps**: Clear separation of Octree Build → Normal Estimation → Verification.

## Project Structure
```
.
├── bin/                    # Compiled executables (gitignored)
├── build_cmake/            # CMake build directory (gitignored)
├── data/                   # Input datasets
│   ├── bunny.pcd           # Small test dataset (397 points)
│   └── table_scene_lms400.pcd  # Large dataset (460k points)
├── results/                # Timestamped output files
│   └── bunny_YYYYMMDD_HHMMSS_voxelized.pcd/.png
├── scripts/
│   └── visualize_result.py # Visualization script
├── src/
│   ├── include/rvv_pcl.h   # Public API
│   ├── rvv_common.cpp      # Fused gather-filter kernel
│   ├── octree.cpp          # Octree implementation
│   └── normal_estimation.cpp  # Step-by-step normal estimation
└── tests/
    └── test_pipeline_walkthrough.cpp  # Main test harness
```

## Prerequisites
- **GCC 14** RISC-V Toolchain (`/opt/riscv/bin/riscv64-unknown-linux-gnu-g++`)
- **QEMU** with RVV support (`qemu-riscv64 -cpu rv64,v=true`)
- **Python 3** + Matplotlib (for visualization)

## Quick Start

### 1. Build
```bash
mkdir -p build_cmake && cd build_cmake
cmake -DCMAKE_C_COMPILER=/opt/riscv/bin/riscv64-unknown-linux-gnu-gcc \
      -DCMAKE_CXX_COMPILER=/opt/riscv/bin/riscv64-unknown-linux-gnu-g++ ..
make test_pipeline_walkthrough
```

### 2. Run Pipeline
```bash
cd bin
/usr/bin/qemu-riscv64 -L /opt/riscv/sysroot -cpu rv64,v=true ./test_pipeline_walkthrough bunny.pcd
```

### 3. Expected Output
```
[Step 1] Loading bunny.pcd... Loaded 397 points.
[Step 2] Voxel Grid Downsampling... Filtered: 294 points.
[Step 3] Building Octree... Success.
[Step 4] Estimating Normals... [PASS]
[Step 5] Radius Search Verification... [PASS]
[SUCCESS] Visualization saved to results/bunny_YYYYMMDD_HHMMSS_voxelized.png
```

Results are saved to `results/` with timestamps for serialization.

## Key Files
| File | Description |
|------|-------------|
| `src/rvv_common.cpp` | Fused `get_inds_in_radius_rvv` kernel |
| `src/octree.cpp` | Octree spatial index |
| `src/spatial_hashing.cpp` | Spatial Hash Grid implementation |
| `src/normal_estimation.cpp` | Octree-based normal estimation |
| `tests/test_pipeline_walkthrough.cpp` | Main pipeline test |
| `tests/test_spatial_hash_comparison.cpp` | Benchmark: Octree vs Spatial Hash |
