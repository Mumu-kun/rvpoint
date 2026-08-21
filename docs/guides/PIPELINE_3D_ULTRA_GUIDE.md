# Pipeline 3D Ultra — Parameters & CLI Command Guide

Comprehensive reference guide for all command-line parameters, options, default values, and operational use-cases for [`pipeline_3d_ultra`](file:///workspace/src/tools/pipeline_3d_ultra.cpp).

---

## 1. Quick Syntax & Overview

```bash
./scripts/run.sh pipeline_3d_ultra <input.pcd> [output_dir] [OPTIONS]
```

### Complete Example Command
```bash
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000003.pcd results/demo_run \
  --progress \
  --json \
  --leaf-size 0.10 \
  --use-ror \
  --ror-radius 0.25 \
  --ror-min-pts 2 \
  --ransac-iters 250 \
  --no-ground-prior \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000
```

---

## 2. Detailed Parameter Reference

### A. Input / Output & Diagnostics

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `<input.pcd>` | Path (Positional) | *Required* | Path to input PCD file. Supports **ASCII**, **Binary**, and **Binary Compressed** formats. Automatically resolves inside `data/` and `data/pcd_compressed/`. |
| `[output_dir]` | Path (Positional) | `results/<stem>_pipeline_ultra` | Target directory where intermediate `.pcd` stages, colored cluster clouds, and `metrics.json` will be saved. |
| `--progress` | Flag | `false` | Prints live stage-by-stage execution times (`[1/10]` to `[10/10]`), point counts, and final latency breakdown. |
| `--json` / `--json-metrics` | Flag | `false` | Saves a structured `metrics.json` containing perception status, stage latencies, inlier/outlier counts, and cluster statistics. |
| `--no-write` / `--disable-disk` | Flag | `false` | Disables disk file saving. Measures pure in-memory compute latency without filesystem overhead. |

---

### B. Stage 3: Voxel Grid Downsampling (RVV)

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `--leaf-size <val>` | Float (meters) | `0.10` (10 cm) | Spatial resolution of the downsampling grid. <br>• `0.05`–`0.08`: Higher spatial density, finer geometry. <br>• `0.10`: Optimal balance between accuracy and real-time speed. <br>• `0.15`–`0.20`: Aggressive downsampling for long-range sparse clouds. |

---

### C. Stage 5: Outlier Removal (RVV)

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `--use-ror` / `--ror` | Flag | `true` (Default) | Activates **True Radius Outlier Removal (ROR)** with dynamic spatial grid and self-cell fastpath acceleration. |
| `--ror-radius <val>` | Float (meters) | `0.25` (25 cm) | Euclidean search radius sphere. Points with fewer than `--ror-min-pts` neighbors inside this radius are filtered out. |
| `--ror-min-pts <val>` | Integer | `2` | Minimum neighbor points required within `ror-radius`. Set to `2` to preserve thin peripheral objects (lampposts, pedestrians). |
| `--use-sor` / `--sor` | Flag | `false` | Enables **Statistical Outlier Removal (SOR)** using true nearest neighbor distance sorting and standard deviation filtering. |
| `--skip-sor` / `--no-sor` | Flag | `false` | Bypasses outlier filtering entirely for maximum throughput. |

---

### D. Stage 8: RANSAC Ground Plane Extraction

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `--ransac-iters <val>` | Integer | `250` | Maximum RANSAC hypothesis iterations. Adaptive early-termination stops iterations automatically when confidence reaches 99%. |
| `--ground-angle-thresh <deg>` | Float (degrees) | `45.0` | Angular tolerance cone ($\theta_{\text{max}}$) between plane normal and vertical reference prior ($+Z$). Accepts sloped terrain up to $\pm 45^\circ$. |
| `--no-ground-prior` / `--unconstrained-plane` | Flag | `false` | **Critical for handheld testing.** Removes orientation constraints, fitting the largest dominant plane at **any arbitrary pitch/roll tilt**. |
| `--optical-frame` | Flag | `false` | Sets ground normal prior to $+Y$ (`0, 1, 0`) for RGB-D cameras (Intel RealSense, OAK-D, Azure Kinect) where $+Y$ points down. |
| `--seed <val>` | Unsigned Int | `42` | 64-bit seed for stateful `FastPRNG` for deterministic, repeatable RANSAC sampling across frames. |

---

### E. Stage 9: Euclidean Clustering (Union-Find)

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `--cluster-tolerance <val>` | Float (meters) | `0.15` (15 cm) | Maximum distance threshold between points to be grouped into the same cluster. |
| `--min-cluster <val>` | Integer | `50` | Minimum point threshold for a cluster. Clusters with fewer points are discarded as sensor noise. |
| `--max-cluster <val>` | Integer | `100000` | Maximum point threshold. Clusters larger than this value are discarded (prevents large background blobs). |

---

### F. Stage 7: Surface Normal Estimation

| Parameter | Type | Default | Description & Recommended Usage |
| :--- | :--- | :--- | :--- |
| `--no-normals` / `--skip-normals` | Flag | `false` | Skips PCA surface normal covariance estimation. Sets default vertical normals $(0, 0, 1)$ to eliminate computation overhead. |

---

## 3. Common Command Profiles & Recipes

### 1. Standard Autonomous Driving / LiDAR Benchmark (KITTI / nuScenes)
```bash
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000045.pcd \
  --progress \
  --leaf-size 0.10 \
  --cluster-tolerance 0.15 \
  --min-cluster 50 \
  --max-cluster 100000
```

### 2. Handheld Sensor / Arbitrary Angle Real Hardware Testing
*Removes plane angle constraints so the floor/ground is extracted regardless of how the camera is tilted.*
```bash
./scripts/run.sh pipeline_3d_ultra <frame.pcd> \
  --no-ground-prior \
  --progress \
  --no-write
```

### 3. Intel RealSense / RGB-D Camera (Optical Frame $+Y$ Down)
```bash
./scripts/run.sh pipeline_3d_ultra camera_frame.pcd \
  --optical-frame \
  --progress \
  --leaf-size 0.05 \
  --cluster-tolerance 0.08
```

### 4. Ultra-High-Speed Real-Time Perception (Pure Compute Benchmark)
*Disables disk write and skips surface normal estimation for lowest latency.*
```bash
./scripts/run.sh pipeline_3d_ultra data/pcd_compressed/0000000090.pcd \
  --no-write \
  --no-normals \
  --progress
```

---

## 4. Pipeline Stages & Output Files

When disk output is enabled (`--no-write` is not set), the pipeline saves the following files in the output directory:

| Stage # | Stage Name | Output File | Description |
| :---: | :--- | :--- | :--- |
| **01** | Load Input | `00_input.pcd` | Original point cloud loaded from disk. |
| **03** | Downsampling | `01_downsampled.pcd` | Uniformly downsampled point cloud. |
| **05** | Outlier Filter | `02_sor_filtered.pcd` | Cloud after Radius/Statistical outlier removal. |
| **08** | Ground Inliers | `04_ransac_inliers.pcd` | Extracted ground plane points. |
| **08** | Obstacle Cloud | `05_ground_plane_removed.pcd` | Non-ground obstacle points ready for clustering. |
| **10** | Clustering | `06_clusters.pcd` | Segmented obstacle clusters with distinct RGB colors. |
| **--** | JSON Metrics | `metrics.json` | Complete benchmark timings and point metrics. |
