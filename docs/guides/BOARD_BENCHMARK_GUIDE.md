# Orange Pi RV2 (SpacemiT K1) Benchmark & Execution Runbook

> **Complete reference guide for build, execution, profiling, and synchronization commands used on the physical Orange Pi RV2 (RV64GCV Octa-Core) testbed.**

---

## 1. Remote Access & Environment

### Connect to Board via SSH
```bash
ssh orangepi@100.94.165.126
# Password: orangepi
# Working directory on board:
cd ~/projects/rvpoint
```

### Inspect Hardware & Vector ISA Support
```bash
# Verify 8 physical cores and RVV 1.0 support:
lscpu
cat /proc/cpuinfo | grep -E "isa|model name" | head -n 4

# Check active CPU core clock frequencies (SpacemiT K1 @ ~1.6 GHz):
cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq
```

---

## 2. Dual-Backend Compilation on Board

The SpacemiT K1 runs Ubuntu 24.04 with GCC 13/14 native toolchain and RVV 1.0 support.  
RVPoint supports **two independent coexisting build targets** on the board:
* **RVV Backend (`build/rvv`)**: Target architecture `rv64gcv` with hardware vector intrinsics enabled.
* **Scalar Backend (`build/scalar`)**: Target architecture `rv64gc` with pure scalar C++ fallbacks (zero vector instructions) for direct speedup comparison.

```bash
# Enter project directory:
cd ~/projects/rvpoint

# ==============================================================================
# A. Configure & Build RVV Backend (-march=rv64gcv, RVV enabled)
# ==============================================================================
# Configure build/rvv (run once):
cmake -S . -B build/rvv \
  -DCMAKE_BUILD_TYPE=Release \
  -DRISCV_ARCH=rv64gcv \
  -DRISCV_ABI=lp64d \
  -DRVPOINT_USE_RVV=ON \
  -DRVV_PCL_USE_RVV=ON

# Compile specific targets under build/rvv:
cmake --build build/rvv -j8 --target pipeline_3d_intra
cmake --build build/rvv -j8 --target pipeline_3d_ultra
cmake --build build/rvv -j8 --target pipeline_3d_rvv_clust
cmake --build build/rvv -j8 --target pipeline_3d_stream_rvv_clust
cmake --build build/rvv -j8 --target official_pcl_pipeline
cmake --build build/rvv -j8 --target official_pcl_stream

# Or build all RVV targets:
cmake --build build/rvv -j8


# ==============================================================================
# B. Configure & Build Scalar Backend (-march=rv64gc, RVV disabled)
# ==============================================================================
# Configure build/scalar (run once):
cmake -S . -B build/scalar \
  -DCMAKE_BUILD_TYPE=Release \
  -DRISCV_ARCH=rv64gc \
  -DRISCV_ABI=lp64d \
  -DRVPOINT_USE_RVV=OFF \
  -DRVV_PCL_USE_RVV=OFF

# Compile specific targets under build/scalar:
cmake --build build/scalar -j8 --target pipeline_3d_intra
cmake --build build/scalar -j8 --target pipeline_3d_ultra

# Or build all scalar targets:
cmake --build build/scalar -j8
```

### Binary Output Directory Hierarchy
| Backend | Build Directory | Binary Output Path | Target Architecture |
| :--- | :--- | :--- | :--- |
| **Hardware RVV 1.0** | `build/rvv/` | `./build/rvv/bin/rvv/<target_name>` | `rv64gcv` (RVV enabled) |
| **Pure Scalar** | `build/scalar/` | `./build/scalar/bin/scalar/<target_name>` | `rv64gc` (RVV disabled) |

---

## 3. High-Performance Intra-Frame Pipeline (`pipeline_3d_intra`)

**`pipeline_3d_intra`** is the state-of-the-art inverted 8-core single-frame perception pipeline.

### Architectural Innovations:
1. **Parallel Radix Sort Downsampling**: Replaces $O(N \log N)$ `std::sort` with parallel $O(N)$ radix sort.
2. **Pipeline Inversion**: RANSAC ground removal executes on the downsampled cloud **before** spatial grid construction. This eliminates ~30% of ground points, radically reducing work for spatial indexing, ROR, and clustering.
3. **8-Core Spatial Slab Decomposition**: Divides the point cloud into balanced spatial slabs processed in parallel across all 8 cores, then seamlessly stitches boundary clusters.

### Supported Parameters:
* `--leaf-size <val>`: Voxel leaf size in meters (default: `0.10`)
* `--ror-radius <val>`: ROR search sphere radius in meters (default: `0.25`)
* `--ror-min-pts <val>`: Minimum neighbors within ROR sphere (default: `2` or `3`)
* `--skip-ror` / `--skip-sor`: Bypass outlier removal stage entirely
* `--ransac-iters <val>`: Max hypothesis iterations for SPRT RANSAC (default: `250` or `100`)
* `--ransac-dist <val>`: RANSAC inlier distance threshold in meters (default: `0.22`)
* `--cluster-tolerance <val>`: Euclidean clustering distance threshold (default: `0.15`)
* `--min-cluster <val>` / `--max-cluster <val>`: Min/max cluster size filter (default: `50` / `100000`)
* `--threads <N>`: Worker thread count (default: all `8` cores)
* `--no-write`: Disable writing PCD files to disk for pure compute benchmarking
* `--progress`: Print live per-stage timing breakdown
* `--json`: Export telemetry metrics to JSON

```bash
# ------------------------------------------------------------------------------
# A. Run RVV Hardware-Accelerated Intra-Frame Pipeline:
# ------------------------------------------------------------------------------
./build/rvv/bin/rvv/pipeline_3d_intra data/pcd_compressed/0000000000.pcd \
  --progress \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 3 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# ------------------------------------------------------------------------------
# B. Run Pure Scalar Intra-Frame Pipeline (Direct Comparison):
# ------------------------------------------------------------------------------
./build/scalar/bin/scalar/pipeline_3d_intra data/pcd_compressed/0000000000.pcd \
  --progress \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 3 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# ------------------------------------------------------------------------------
# C. Multi-Frame Batch Directory Sweep with pipeline_3d_intra:
# ------------------------------------------------------------------------------
./build/rvv/bin/rvv/pipeline_3d_intra data/pcd_compressed/ \
  --progress \
  --max-frames 20 \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 3 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

---

## 4. End-to-End 10-Stage Perception Pipeline (`pipeline_3d_ultra`)

**`pipeline_3d_ultra`** is the full 10-stage baseline perception pipeline matching standard robotics pipelines stage-by-stage.

### Key Options:
* `--use-ror` / `--ror`: Enable Radius Outlier Removal (Default: `true`)
* `--use-sor` / `--sor`: Enable Statistical Outlier Removal
* `--ror-radius <val>`: ROR search radius (default: `0.25`)
* `--ror-min-pts <val>`: Min points in radius (default: `2`)
* `--no-normals`: Skip surface normal estimation stage
* `--leaf-size <val>`, `--cluster-tolerance <val>`, `--min-cluster <val>`, `--max-cluster <val>`
* `--no-write`: Measure pure compute without disk write overhead

```bash
# ------------------------------------------------------------------------------
# A. Standard ROR + No Normals (Matches Automotive LiDAR pipeline):
# ------------------------------------------------------------------------------
./build/rvv/bin/rvv/pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
  --progress \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-normals \
  --no-write

# ------------------------------------------------------------------------------
# B. Statistical Outlier Removal (SOR) mode:
# ------------------------------------------------------------------------------
./build/rvv/bin/rvv/pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
  --progress \
  --use-sor \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-normals \
  --no-write
```

---

## 5. Single-Frame RVV Clustering Pipeline (`pipeline_3d_rvv_clust`)

```bash
# Standard in-memory compute benchmark (Zero Disk I/O):
./build/rvv/bin/rvv/pipeline_3d_rvv_clust data/pcd_compressed/0000000010.pcd \
  --progress \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# Export clustered 3D obstacles (writes output/<stem>_pipeline_rvv_clust/06_clusters.pcd):
./build/rvv/bin/rvv/pipeline_3d_rvv_clust data/pcd_compressed/0000000010.pcd \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000
```

---

## 6. Official Debian PCL 1.14 Baseline (`official_pcl_pipeline`)

*Genuine upstream Point Cloud Library 1.14 dynamically linked against Debian's `/usr/lib/riscv64-linux-gnu/libpcl_*.so` packages.*

> [!NOTE]
> **Normals vs Stage Equivalence:**  
> By default, PCL executes Stage 7 (`pcl::NormalEstimation`), adding ~516 ms of compute and ~74 ms of KdTree rebuild time. Pass `--no-normals` for a strict 1:1 stage-by-stage comparison with RVPoint.

```bash
# 1:1 Algorithmic Parity Benchmark (ROR + RANSAC + Euclidean Clustering, no normals):
./build/rvv/bin/rvv/official_pcl_pipeline data/pcd_compressed/0000000010.pcd \
  --progress \
  --use-ror \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --no-normals \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# Baseline with Statistical Outlier Removal (SOR):
./build/rvv/bin/rvv/official_pcl_pipeline data/pcd_compressed/0000000010.pcd \
  --progress \
  --use-sor \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-normals \
  --no-write
```

---

## 7. Multi-Core Continuous Streaming Benchmarks (KITTI Dataset)

### A. RVPoint Continuous Multi-Threaded Stream (`pipeline_3d_stream_rvv_clust`)
*Asynchronous inter-frame worker pool across all 8 physical CPU cores.*

```bash
# Automotive Perception Rate: 131-Frame Full Dataset Stream (31.81 FPS Sustained):
./build/rvv/bin/rvv/pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
  --mode inter \
  --threads 8 \
  --max-frames 131 \
  --leaf-size 0.15 \
  --cluster-tolerance 0.20 \
  --min-cluster 30 \
  --max-cluster 100000 \
  --ror-min-pts 3 \
  --no-write

# High-Density Perception: 20-Frame Stream (23.90 FPS Sustained, 0.10m leaf):
./build/rvv/bin/rvv/pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
  --mode inter \
  --threads 8 \
  --max-frames 20 \
  --leaf-size 0.10 \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --ransac-iters 100 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

### B. Official PCL 1.14 Multi-Threaded Stream (`official_pcl_stream`)
```bash
# 20-Frame Baseline Stream (4.61 FPS):
./build/rvv/bin/rvv/official_pcl_stream data/pcd_compressed/ \
  --threads 8 \
  --max-frames 20 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

---

## 8. Host-to-Board Deployment & Data Synchronization

Run these commands from your development environment (x86_64 Host / Dev Container / WSL):

### A. Deploy Modified Sources to Board
```bash
# Push modified sources and build configurations to the board:
scp eval/pipelines/pipeline_3d_intra.cpp \
    eval/pipelines/pipeline_3d_ultra.cpp \
    eval/pipelines/pipeline_3d_rvv_clust.cpp \
    eval/pipelines/pipeline_3d_stream_rvv_clust.cpp \
    eval/pipelines/official_pcl_pipeline.cpp \
    eval/pipelines/official_pcl_stream.cpp \
    orangepi@100.94.165.126:~/projects/rvpoint/eval/pipelines/

scp CMakeLists.txt scripts/build.sh \
    orangepi@100.94.165.126:~/projects/rvpoint/
```

### B. Fetch Output Clouds and Benchmark Metrics
```bash
# Fast incremental synchronization of clustered outputs:
rsync -avzP orangepi@100.94.165.126:~/projects/rvpoint/output/stream_clusters/ ./board_result/
```

---

## 9. Empirical Performance Summary Matrix

Measurements collected directly on **Orange Pi RV2 (SpacemiT K1 Octa-Core RV64GCV @ 1.6 GHz, Ubuntu 24.04)**:

### A. Single-Frame Stage Breakdown (`data/pcd_compressed/0000000010.pcd`: 116,412 raw pts $\rightarrow$ 51,655 voxel pts)

| Perception Stage | Official PCL 1.14 | RVPoint Hardware RVV 1.0 (`pipeline_3d_rvv_clust`) | RVPoint Inverted Slab (`pipeline_3d_intra`) | Absolute Speedup vs PCL |
| :--- | :--- | :--- | :--- | :--- |
| **[1] Disk PCD Load** | 60.8 ms | 26.0 ms | **26.6 ms** | 2.3× |
| **[2] Voxel Downsampling** | 51.9 ms | 46.8 ms | **~19 ms (Radix RVV)** | 2.7× |
| **[3] RANSAC Ground Fit** | 354.1 ms (`pcl::SAC`) | 1.9 ms (SPRT) | **6.8 ms (Inverted)** | 🚀 **52× – 186×** |
| **[4] Spatial Search Index** | 86.4 ms (KdTree) | 16.3 ms (Grid) | **5.4 ms (Obstacle-only)** | ⚡ **16.0×** |
| **[5] Radius Outlier Removal** | 405.7 ms (`pcl::ROR`) | 9.9 ms (RVV ROR) | **4.6 ms (Slab ROR)** | 🚀 **88.2×** |
| **[6] Euclidean Clustering** | 422.4 ms (`pcl::ECE`) | 34.5 ms (RVV UF) | **13.5 ms (Slab + Merge)** | ⚡ **31.3×** |
| **Total In-Memory Compute** | 1318.1 ms (w/o normals) | 109.4 ms | **~50 ms** | ⚡ **26.4× faster** |
| **Total Wall-Clock Latency** | 1973.5 ms (0.51 FPS) | 147.7 ms (6.77 FPS) | **~75–90 ms (>11 FPS)** | ⚡ **22× faster** |

---

### B. Multi-Core Continuous Streaming (KITTI Dataset, 8 Threads)

| Benchmark Scenario | Official PCL 1.14 Stream | RVPoint RVV Stream Pool | Throughput Advantage | Autonomous Driving Feasibility |
| :--- | :--- | :--- | :--- | :--- |
| **High-Density Stream (0.10m, 20 frames)** | 4.61 FPS (4,337 ms total) | **23.90 FPS** (836 ms total) | 🚀 **5.18× higher throughput** | Real-Time capable |
| **Per-Frame Compute Latency (0.10m)** | 1397.8 ms / frame | **241.4 ms / frame** | ⚡ **5.79× lower latency** | Parallel asynchronous pool |
| **Automotive Stream (0.15m, 131 frames)** | ~4.2 FPS | **31.81 FPS sustained** | 🚀 **7.57× higher throughput** | **Exceeds 30 Hz sensor rate** |
| **Smoke Stream (0.15m, 50 frames)** | ~4.2 FPS | **32.40 FPS sustained** | 🚀 **7.71× higher throughput** | **Real-Time 32+ FPS** |
