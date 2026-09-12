> **Physical Hardware Reference**: For standalone board commands targeting the Orange Pi RV2 (SpacemiT K1 Octa-Core RV64GCV), see [BOARD_BENCHMARK_GUIDE.md](BOARD_BENCHMARK_GUIDE.md).

---

### 1. True 3D Pipelines (100% PCL Backbone & Full 3D Geometry)

#### A. Ultimate True 3D RVPoint Pipeline (`pipeline_3d_ultimate`) - RECOMMENDED FASTEST TRUE 3D
*Runs the streamlined, zero-overhead True 3D engine (Flat Direct 3D Spatial Grid + Fast RVV SOR + RVV SPRT RANSAC + Flat Array Union-Find Clustering).*
```bash
# Standard run with PCD export:
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Zero-Disk I/O Compute Benchmark:
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### B. Full 10-Stage True 3D Benchmark Pipeline (`pipeline_3d_ultra`)
*Runs the 10-stage evaluation pipeline supporting full Cardano surface normal estimation for official PCL comparison, or `--no-normals` for fast evaluation.*
```bash
# Standard 10-stage run (with surface normal estimation & PCD export):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Fast mode (skipping redundant normal estimation):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-normals

# Pure compute benchmark (no disk I/O & no normals):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write --no-normals
```

#### C. Accelerated True 3D RVPoint Baseline (`pipeline_3d_turbo`)
*Runs the earlier accelerated True 3D engine (Flat Direct 3D Spatial Grid + Cardano Normals + RVV RANSAC).*
```bash
./scripts/run.sh pipeline_3d_turbo data/pcd_compressed/0000000090.pcd \
    --progress --json --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### D. Baseline True 3D PointerOctree Pipeline (`pipeline_export`)
*Runs the hierarchical `PointerOctree` baseline pipeline.*
```bash
./scripts/run.sh pipeline_export data/pcd_compressed/0000000090.pcd \
    --progress --json --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### E. Official Debian PCL 1.14 Pipeline (`official_pcl_pipeline`)
*Runs genuine Debian Point Cloud Library 1.14 (`libpcl-dev` shared libraries: `pcl::VoxelGrid`, `pcl::StatisticalOutlierRemoval` / `pcl::RadiusOutlierRemoval`, `pcl::NormalEstimation`, `pcl::SACSegmentation`, `pcl::EuclideanClusterExtraction`).*
```bash
# Standard SOR baseline:
./scripts/run.sh official_pcl_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Radius Outlier Removal (ROR) mode:
./scripts/run.sh official_pcl_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --use-ror --ror-radius 0.25 --ror-min-pts 2 --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### F. Standalone Scalar PCL Baseline Replicate (`pcl_standalone_pipeline`)
*Runs zero-dependency standalone scalar C++ implementation replicating standard PCL 1.14 algorithms.*
```bash
./scripts/run.sh --backend scalar pcl_standalone_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### G. Slotted Register-File Perception Pipeline Prototype (`pipeline_prototype`)
*Demonstrates the zero-heap, zero-vtable slotted register file pipeline engine with topological DAG scheduling and dynamic parameter reconfiguration (ADR-0012).*
```bash
# RVV 1.0 vectorized backend:
./scripts/run.sh pipeline_prototype data/pcd_compressed/0000000090.pcd

# Scalar backend:
./scripts/run.sh --backend scalar pipeline_prototype data/pcd_compressed/0000000090.pcd
```

---

### 2. 2.5D Real-Time Autonomous Driving Pipelines

#### A. Ultra-Fast 2.5D RVV 1.0 Pipeline (`pipeline_rvv_ultra_fast`)
*Runs the vectorized 2.5D elevation grid engine ($< 4.5\,\text{ms}$ on physical 8-core hardware).*
```bash
./scripts/run.sh pipeline_rvv_ultra_fast data/pcd_compressed/0000000080.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### B. Official Open-Source Scalar 2.5D Baseline (`scalar_25d_baseline`)
*Runs the open-source pure C++ scalar 2.5D baseline (`-march=rv64gc -O3`).*
```bash
./scripts/run.sh --backend scalar scalar_25d_baseline data/pcd_compressed/0000000080.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

---

### 3. Architecture & Algorithmic Benchmarks

#### A. Pure In-Memory Compute Benchmark (Zero Disk I/O)
*Direct compute timing across all architectures on identical in-memory frames:*
```bash
./scripts/run.sh test_pure_compute_benchmark
```

#### B. 3D Spatial Search & Branching Vectorization Benchmark
*Compares PointerOctree vs. Linear Morton Octree vs. Compact 3D Hash Grid:*
```bash
./scripts/run.sh test_all_3d_branching_approaches data/pcd_compressed/0000000080.pcd
```

#### C. Fused $O(1)$ Hash Branching + RVV Contiguous Leaf Vectors Benchmark
*Compares standard pointer jumping vs. $O(1)$ Hash-to-Leaf vector burst reads:*
```bash
./scripts/run.sh test_fused_hash_leaf_rvv data/pcd_compressed/0000000080.pcd
```

#### D. Fused RVV 1.0 vs. Scalar 2.5D Baseline Benchmark
*Measures RVV vectorization speedup over scalar execution:*
```bash
./scripts/run.sh test_fused_turbo_vs_scalar
```

#### E. Multi-Core OpenMP Parallel Scaling Test (1, 2, 4, 8 Cores)
*Measures hardware multi-threading scaling for the Orange Pi RV2 SoC:*
```bash
./scripts/run.sh test_openmp_rvv_scaling
```

#### F. Multi-Frame Accuracy & Recall Validation Sweep
*Sweeps across all frames and computes clustering precision/recall metrics:*
```bash
./scripts/run.sh test_multi_frame_sweep
```

---

### 4. Build and Testing Commands

#### A. Build All Targets (Toolchain: RISC-V RVV 1.0)
```bash
./scripts/build.sh
```

#### B. Build Specific Executable
```bash
./scripts/build.sh --target pipeline_3d_ultimate
./scripts/build.sh --target pipeline_3d_ultra
```

#### C. Run All Unit Tests
```bash
# Run default RVV 1.0 unit test suite (13 tests):
./scripts/test.sh

# Run Scalar unit test suite (12 tests):
./scripts/test.sh --backend scalar
```
