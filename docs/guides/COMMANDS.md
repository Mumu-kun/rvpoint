> **Physical Hardware Reference**: For standalone board commands targeting the Orange Pi RV2 (SpacemiT K1 Octa-Core RV64GCV), see [BOARD_BENCHMARK_GUIDE.md](file:///workspace/docs/guides/BOARD_BENCHMARK_GUIDE.md).

---

### 1. True 3D Pipelines (100% PCL Backbone & Full 3D Geometry)

#### A. High-Performance Inverted Slab Pipeline (`pipeline_3d_intra`) - FLAGSHIP INVERTED 8-CORE PIPELINE
*Runs the inverted perception pipeline (Parallel Radix Sort Downsample $\rightarrow$ RANSAC Ground Removal $\rightarrow$ 8-Core Spatial Slab Grid Decomposition $\rightarrow$ Vectorized Slab ROR $\rightarrow$ Slab Union-Find Clustering & Cross-Border Stitching).*
```bash
# Standard run with PCD export:
./scripts/run.sh pipeline_3d_intra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 3 \
    --ransac-dist 0.22 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Zero-Disk I/O Compute Benchmark:
./scripts/run.sh pipeline_3d_intra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 3 \
    --ransac-dist 0.22 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Multi-Frame Batch Directory Sweep (e.g. data/data_2 or data/pcd_compressed):
./scripts/run.sh pipeline_3d_intra data/data_2/ \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 3 \
    --ransac-dist 0.22 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Handheld / Unconstrained Sensor Mode (Arbitrary Pitch/Roll):
./scripts/run.sh pipeline_3d_intra <frame.pcd> \
    --progress --no-ground-prior --ransac-dist 0.22 --ransac-iters 100 --no-write
```

#### B. Full 10-Stage True 3D Benchmark Pipeline (`pipeline_3d_ultra`)
*Runs the 10-stage evaluation pipeline supporting full Cardano surface normal estimation for official PCL comparison, or `--no-normals` for fast evaluation.*
```bash
# Standard 10-stage run (with surface normal estimation & PCD export):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --compute-normals --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 250 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Fast mode (skipping redundant normal estimation):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-normals

# Pure compute benchmark (no disk I/O & no normals):
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write --no-normals
```

#### C. Single-Frame RVV Clustering Pipeline (`pipeline_3d_rvv_clust`)
*Runs the streamlined single-frame hardware RVV 1.0 clustering engine.*
```bash
# Pure compute benchmark (zero disk I/O):
./scripts/run.sh pipeline_3d_rvv_clust data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Export clustered obstacles:
./scripts/run.sh pipeline_3d_rvv_clust data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### D. Ultimate True 3D RVPoint Pipeline (`pipeline_3d_ultimate`)
*Runs the streamlined, zero-overhead True 3D engine (Flat Direct 3D Spatial Grid + Fast RVV SOR + RVV SPRT RANSAC + Flat Array Union-Find Clustering).*
```bash
# Standard run with PCD export:
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000

# Zero-Disk I/O Compute Benchmark:
./scripts/run.sh pipeline_3d_ultimate data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### E. Accelerated True 3D RVPoint Baseline (`pipeline_3d_turbo`)
*Runs the earlier accelerated True 3D engine (Flat Direct 3D Spatial Grid + Cardano Normals + RVV RANSAC).*
```bash
./scripts/run.sh pipeline_3d_turbo data/pcd_compressed/0000000090.pcd \
    --progress --json --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### F. Baseline True 3D PointerOctree Pipeline (`pipeline_export`)
*Runs the hierarchical `PointerOctree` baseline pipeline.*
```bash
./scripts/run.sh pipeline_export data/pcd_compressed/0000000090.pcd \
    --progress --json --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000
```

#### G. Official Debian PCL 1.14 Pipeline (`official_pcl_pipeline`)
*Runs genuine Debian Point Cloud Library 1.14 (`libpcl-dev` shared libraries: `pcl::VoxelGrid`, `pcl::StatisticalOutlierRemoval` / `pcl::RadiusOutlierRemoval`, `pcl::NormalEstimation`, `pcl::SACSegmentation`, `pcl::EuclideanClusterExtraction`).*
```bash
# 1:1 Parity Mode (ROR + no normals):
./scripts/run.sh official_pcl_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --use-ror --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 --no-normals \
    --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write

# Standard SOR baseline:
./scripts/run.sh official_pcl_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --use-sor --leaf-size 0.10 \
    --ransac-dist 0.20 --ransac-iters 100 --no-normals \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### H. Standalone Scalar PCL Baseline Replicate (`pcl_standalone_pipeline`)
*Runs zero-dependency standalone scalar C++ implementation replicating standard PCL 1.14 algorithms.*
```bash
./scripts/run.sh --backend scalar pcl_standalone_pipeline data/pcd_compressed/0000000090.pcd \
    --progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

---

### 2. Continuous Multi-Core Streaming Pipelines (KITTI Dataset)

#### A. RVPoint Continuous Multi-Threaded Stream (`pipeline_3d_stream_rvv_clust`)
*Asynchronous inter-frame worker pool across all 8 CPU cores.*
```bash
# Automotive Perception Rate: 131-Frame Full Dataset Stream (31.81 FPS Sustained):
./scripts/run.sh pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
    --mode inter --threads 8 --max-frames 131 \
    --leaf-size 0.15 --ror-radius 0.25 --ror-min-pts 3 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.20 --min-cluster 30 --max-cluster 100000 --no-write

# High-Density Perception: 20-Frame Stream (23.90 FPS Sustained, 0.10m leaf):
./scripts/run.sh pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
    --mode inter --threads 8 --max-frames 20 \
    --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

#### B. Official PCL 1.14 Multi-Threaded Stream (`official_pcl_stream`)
```bash
# 20-Frame Baseline Stream (4.61 FPS):
./scripts/run.sh official_pcl_stream data/pcd_compressed/ \
    --threads 8 --max-frames 20 \
    --leaf-size 0.10 --ror-radius 0.25 --ror-min-pts 2 \
    --ransac-dist 0.20 --ransac-iters 100 \
    --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write
```

---

### 3. 2.5D Real-Time Autonomous Driving Pipelines

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

### 4. Architecture & Algorithmic Benchmarks

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

### 5. Comprehensive Parameter Quick-Reference Table

| Parameter Flag | CLI Aliases | Type | Default | Description |
| :--- | :--- | :--- | :--- | :--- |
| `--leaf-size <val>` | -- | Float | `0.10` m | Downsampling voxel grid resolution in meters |
| `--ror-radius <val>` | -- | Float | `0.25` m | Radius Outlier Removal neighbor search sphere |
| `--ror-min-pts <val>` | `--ror-min-neighbors` | Integer | `3` (intra) / `2` (ultra) | Minimum neighbor count inside `ror-radius` |
| `--skip-ror` | `--no-ror`, `--skip-sor`, `--no-sor` | Flag | `false` | Completely bypass outlier removal stage |
| `--use-ror` | `--ror` | Flag | `true` | Enable Radius Outlier Removal |
| `--use-sor` | `--sor` | Flag | `false` | Enable Statistical Outlier Removal |
| `--ransac-dist <val>` | `--ransac-distance-threshold`, `--ransac-thresh`, `--ransac-threshold` | Float | `0.22` m (intra) / `0.20` m (ultra) | RANSAC ground plane inlier distance threshold |
| `--ransac-iters <val>` | -- | Integer | `250` (or `100`) | Max hypothesis iterations for RANSAC |
| `--ground-angle-thresh <deg>` | -- | Float | `45.0` deg | Max angle cone from vertical ground prior |
| `--no-ground-prior` | `--unconstrained-plane` | Flag | `false` | Allow arbitrary tilt/roll for handheld testing |
| `--optical-frame` | -- | Flag | `false` | Use $+Y$ ground prior for RGB-D optical cameras |
| `--seed <val>` | -- | Integer | `42` | FastPRNG random seed for deterministic sampling |
| `--cluster-tolerance <val>` | -- | Float | `0.15` m | Euclidean cluster extraction distance threshold |
| `--min-cluster <val>` | -- | Integer | `50` | Minimum point threshold for valid clusters |
| `--max-cluster <val>` | -- | Integer | `100000` | Maximum point threshold for valid clusters |
| `--threads <N>` | `--cores`, `--num-slabs` | Integer | `8` | Worker thread / OpenMP thread count |
| `--max-frames <N>` | `--frames` | Integer | `-1` (all) / `20` | Max frames to process in directory batch/stream |
| `--mode <mode>` | `--stream-mode`, `--inter`, `--intra` | String | `inter` | Stream parallelism mode (`inter` or `intra`) |
| `--no-normals` | `--skip-normals`, `--no-normal`, `--skip-normal` | Flag | `true` (ultra) | Skip PCA surface normal covariance estimation |
| `--compute-normals` | `--with-normals`, `--normals` | Flag | `false` | Force Cardano PCA surface normal calculation |
| `--no-write` | `--disable-disk` | Flag | `false` | Skip saving PCD files for pure compute timing |
| `--progress` | -- | Flag | `false` | Print live per-stage timing and metrics |
| `--json` | `--json-metrics` | Flag | `false` | Output telemetry metrics to structured JSON |

---

### 6. Build and Testing Commands

#### A. Build All Targets (Toolchain: RISC-V RVV 1.0)
```bash
./scripts/build.sh
```

#### B. Build Specific Executables
```bash
./scripts/build.sh --target pipeline_3d_intra
./scripts/build.sh --target pipeline_3d_ultra
./scripts/build.sh --target pipeline_3d_stream_rvv_clust
./scripts/build.sh --target pipeline_3d_ultimate
```

#### C. Run All Unit Tests
```bash
./scripts/test.sh
```