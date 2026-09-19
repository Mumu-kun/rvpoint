# Walkthrough: Pipeline Execution, MCAP Generation, and Foxglove Visualization

This guide provides instructions for running the **RVPoint** point cloud processing pipeline, converting stage PCD outputs into a Foxglove `.mcap` timeline archive, and visualizing the dataset offline in **Foxglove Studio** using local file loading.

---

## Step 1: Run the Point Cloud Processing Pipeline

### Option A: Serial Batch Processing (All LiDAR Frames)
To process all compressed PCD dataset frames in `data/pcd_compressed/` and generate per-stage outputs:

```bash
./scripts/run_batch_pcd_compressed.sh --backend rvv --skip-sor --output-dir output/pcd_compressed_pipeline
```

* **Generated Output Directories**:
  * [output/pcd_compressed_pipeline/by_frame/](output/pcd_compressed_pipeline/by_frame/): Per-frame stage PCD files (`00_input.pcd`, `01_downsampled.pcd`, `04_ransac_inliers.pcd`, `05_ground_plane_removed.pcd`).
  * [output/pcd_compressed_pipeline/by_stage/](output/pcd_compressed_pipeline/by_stage/): Stage-grouped PCD files across frames.

### Option B: Single Frame Pipeline Export
To execute the pipeline on a single point cloud frame:

```bash
./scripts/run.sh --backend rvv pipeline_export --skip-sor data/pcd_compressed/0000000010.pcd output/single_frame/by_frame/0000000010
```

---

## Step 2: Convert Pipeline Outputs to MCAP File

Use [`scripts/viz/export_mcap.py`](../../scripts/viz/export_mcap.py) to package the pipeline stage `.pcd` files into a single `.mcap` timeline file.

### 1. Export All Frames & Stages (10 FPS Playback)
```bash
python3 scripts/viz/export_mcap.py output/pcd_pipeline --output output/rvpoint_pipeline.mcap --fps 10
```

### 2. Export Specific Stages Only
To reduce file size and focus on key processing milestones:
```bash
python3 scripts/viz/export_mcap.py output/pcd_pipeline --output output/rvpoint_pipeline_filtered.mcap --stages 00_input,01_downsampled,05_ground_plane_removed
```

### 3. Interactive Wizard Mode
To interactively select stages and frames:
```bash
python3 scripts/viz/export_mcap.py output/pcd_pipeline --interactive
```

* **Output Artifact**: `output/rvpoint_pipeline.mcap`

---

## Step 3: View in Foxglove Studio (Offline File Loading)

No live WebSocket or network server is required. Foxglove Studio loads `.mcap` files directly from local storage.

### 1. Open Foxglove Studio
* Launch **Foxglove Studio** (Desktop app or web version at [app.foxglove.dev](https://app.foxglove.dev)).

### 2. Load the `.mcap` File
1. Click **"Open local file..."** (or press `Ctrl+O` / `Cmd+O`).
2. Select your generated file:
   [output/rvpoint_pipeline.mcap](file:///output/rvpoint_pipeline.mcap)

### 3. Configure 3D Point Cloud Panel
1. Click **"Add panel"** and select **3D**.
2. In the left panel settings, expand **Topics**.
3. Toggle visibility for the pipeline topics:
   * `/3d_points/00_input` (Original point cloud)
   * `/3d_points/01_downsampled` (Voxel Grid downsampled cloud)
   * `/3d_points/04_ransac_inliers` (Detected ground plane inliers)
   * `/3d_points/05_ground_plane_removed` (Filtered obstacle cloud)
4. Customize visualization:
   * **Color Mode**: Set to **"Cost / Height"** or **"Z-Axis"** for elevation coloring.
   * **Point Size**: Set to `2.0` pixels.

### 4. Playback Controls
* Use the timeline bar at the bottom to **Play**, **Pause**, step through individual frames, or scrub across the timeline.

---

## Summary Command Reference

```bash
# 1. Run pipeline and generate stage PCDs
./scripts/run.sh pipeline_3d_ultimate data/01_table_scene_lms400.pcd

# 2. Convert to Foxglove MCAP
python3 scripts/viz/export_mcap.py output --output output/rvpoint_pipeline.mcap --fps 10

# 3. Open Foxglove Studio and load output/rvpoint_pipeline.mcap via Ctrl+O
```
