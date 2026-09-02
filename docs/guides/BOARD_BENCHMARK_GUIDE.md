# Orange Pi RV2 (SpacemiT K1) Benchmark & Execution Runbook

> **Complete reference guide for all build, execution, profiling, and synchronization commands used on the Orange Pi RV2 (RV64GCV Octa-Core) testbed.**

---

## 1. Remote Access & Environment

### Connect to Board via SSH
```bash
ssh orangepi@100.94.165.126
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

## 2. Compilation Commands on Board

The SpacemiT K1 runs Ubuntu 24.04 with GCC 13/14 native toolchain and RVV 1.0 support.

```bash
# Enter project directory
cd ~/projects/rvpoint

# 1. Compile the Hardware RVV 1.0 Single-Frame Pipeline:
cmake --build build/rvv -j8 --target pipeline_3d_rvv_clust

# 2. Compile the Multi-Core Asynchronous Stream Pipeline:
cmake --build build/rvv -j8 --target pipeline_3d_stream_rvv_clust

# 3. Compile Official Debian PCL 1.14 Single-Frame Pipeline:
cmake --build build/rvv -j8 --target official_pcl_pipeline

# 4. Compile Official Debian PCL 1.14 Multi-Threaded Stream Pipeline:
cmake --build build/rvv -j8 --target official_pcl_stream

# (Optional) Build all targets simultaneously:
cmake --build build/rvv -j8
```

---

## 3. Single-Frame Benchmarks (`data/0000000000.pcd` & `0000000090.pcd`)

### A. RVPoint Hardware RVV 1.0 Single Pipeline (`pipeline_3d_rvv_clust`)
*Uses intra-frame parallelism across all 8 cores (parallel ROR + parallel cell-level 13-forward clustering). Latency: ~120 ms (8.9 FPS).*

```bash
# Pure in-memory compute benchmark (Zero Disk I/O):
./build/rvv/bin/rvv/pipeline_3d_rvv_clust data/0000000090.pcd \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# Export clustered 3D obstacles (writes output/<stem>_pipeline_rvv_clust/06_clusters.pcd):
./build/rvv/bin/rvv/pipeline_3d_rvv_clust data/0000000090.pcd \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000
```

### B. Official Debian PCL 1.14 Pipeline (`official_pcl_pipeline`)
*Standard robotics baseline using `pcl::VoxelGrid`, `pcl::RadiusOutlierRemoval`/`pcl::StatisticalOutlierRemoval`, `pcl::SACSegmentation`, and `pcl::EuclideanClusterExtraction`. Latency: ~2000 ms (0.5 FPS).*

```bash
# Standard PCL with Statistical Outlier Removal (SOR):
./build/rvv/bin/rvv/official_pcl_pipeline data/0000000090.pcd \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# Fast PCL with Radius Outlier Removal (ROR) for 1:1 algorithmic parity:
./build/rvv/bin/rvv/official_pcl_pipeline data/0000000005.pcd \
  --use-ror \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

---

## 4. Multi-Core Continuous Streaming Benchmarks (KITTI Dataset)

### A. RVPoint Hardware RVV Continuous Stream (`pipeline_3d_stream_rvv_clust`)
*Asynchronous inter-frame worker pool across 8 physical cores. Throughput: **26.18 FPS** across 131 frames.*

```bash
# 1. 131-Frame Pure Compute Throughput Benchmark (Zero Disk I/O):
./build/rvv/bin/rvv/pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
  --mode inter \
  --threads 8 \
  --max-frames 131 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# 2. 131-Frame Production Run with Cluster Export:
# (Saves only 3D obstacle clusters into output/stream_clusters/frame_XXXXXX_clusters.pcd)
./build/rvv/bin/rvv/pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
  --mode inter \
  --threads 8 \
  --max-frames 131 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000

# 3. Quick 10-Frame Smoke Test:
./build/rvv/bin/rvv/pipeline_3d_stream_rvv_clust data/pcd_compressed/ \
  --mode inter \
  --threads 8 \
  --max-frames 10 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

### B. Official PCL 1.14 Multi-Threaded Continuous Stream (`official_pcl_stream`)
*Parallel OpenMP frame pool using official PCL 1.14. Throughput: **3.55 FPS**.*

```bash
# Run 10-frame baseline stream:
./build/rvv/bin/rvv/official_pcl_stream data/pcd_compressed/ \
  --threads 8 \
  --max-frames 10 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write

# Run 50-frame baseline stream:
./build/rvv/bin/rvv/official_pcl_stream data/pcd_compressed/ \
  --threads 8 \
  --max-frames 50 \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000 \
  --no-write
```

---

## 5. Host-to-Board Deployment & Data Synchronization

Run these commands from your development machine (Windows PowerShell or Git Bash):

### A. Deploy Modified Sources to Board
```bash
# Push modified pipeline sources to the board:
scp eval/pipelines/pipeline_3d_rvv_clust.cpp \
    eval/pipelines/pipeline_3d_stream_rvv_clust.cpp \
    eval/pipelines/official_pcl_stream.cpp \
    orangepi@100.94.165.126:~/projects/rvpoint/eval/pipelines/
```

### B. Fetch Clustered Results to Local Machine
```bash
# Fetch exported 3D clusters (131 frames) from board to local board_result/:
scp -r orangepi@100.94.165.126:~/projects/rvpoint/output/stream_clusters/* ./board_result/

# Fast incremental sync via rsync (Git Bash / WSL):
rsync -avzP orangepi@100.94.165.126:~/projects/rvpoint/output/stream_clusters/ ./board_result/
```

---

## 6. Empirical Performance Summary Matrix

Measurements collected on **Orange Pi RV2 (SpacemiT K1 Octa-Core RV64GCV @ 1.6 GHz)**:

| Perception Benchmark | Official PCL 1.14 | RVPoint Hardware RVV 1.0 | Absolute Speedup | Output Parity |
| :--- | :--- | :--- | :--- | :--- |
| **Voxel Downsampling** | 54.7 ms | **49.2 ms** | 1.11× | Exact point-for-point match |
| **Radius Outlier Removal** | 411.2 ms | **11.1 ms** | 🚀 **37.0×** | Exact parity |
| **RANSAC Ground Fit** | 117.7 ms | **2.6 ms** | 🚀 **45.3×** | Road segmented |
| **Euclidean Clustering** | 332.6 ms | **28.8 ms** | ⚡ **11.5×** | **73 clusters vs 73 clusters** |
| **Single-Frame Latency** | 2042.2 ms (0.49 FPS) | **149.9 ms (6.67 FPS)** | ⚡ **13.6× faster** | 100% Exact Parity |
| **Stream Throughput (131 frames)** | 3.55 FPS | **26.18 FPS** | 🚀 **7.37× higher** | 100% Real-Time (>25 Hz) |
