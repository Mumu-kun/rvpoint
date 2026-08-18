# RVPoint Tracking Mode — Test & Benchmark Guide

This guide explains how to build, test, and benchmark the Tracking Mode pipeline.

---

## Prerequisites

### Environment

You need the RVPoint development environment set up:

1. **WSL2** (Windows Subsystem for Linux) with the `rvpoint` distribution
2. **RISC-V GCC Toolchain** (`riscv64-unknown-linux-gnu-g++`) with RVV support
3. **QEMU** (user-mode, `qemu-riscv64`) for running RISC-V binaries on x86

If the environment is not set up yet:

```bash
# From the project root (in Git Bash or WSL)
./env/setup.sh
```

This installs the RISC-V cross-compiler and QEMU automatically.

### Verify Environment

```bash
# In WSL or the container
source env/activate.sh

# Check compiler
riscv64-unknown-linux-gnu-g++ --version

# Check QEMU
qemu-riscv64 --version
```

---

## Building

### Build Everything (RVV Backend)

```bash
./scripts/build.sh --backend rvv
```

This builds:
- The `rvvpcl` library (including the new `registration/` module)
- All test executables
- All benchmark executables
- The `tracking_mode_bench` benchmark tool

### Build Only Tracking Mode Tests

```bash
./scripts/build.sh --backend rvv --target test_tracking_registration
./scripts/build.sh --backend rvv --target test_tracking_pipeline
```

### Build Only the Benchmark

```bash
./scripts/build.sh --backend rvv --target tracking_mode_bench
```

### Build Both Backends (for comparison)

```bash
./scripts/build.sh --backend all
```

---

a
## Running Tests

### Test 1: ICP Registration Unit Test

Tests the ICP core (Stages 2–5) with synthetic point clouds and known transforms:

```bash
./scripts/run.sh test_tracking_registration
```

**What it tests:**
- Pure translation recovery (0.1m, 0.05m, 0m)
- Small rotation (5° about Z) + translation
- Identity (no motion) — should converge in 1–2 iterations
- SE3Transform math operations

**Expected output:**
```
=============================================
   ICP Registration Unit Test
=============================================

--- Test 1: Pure Translation (0.1, 0.05, 0.0) ---
  ICP iterations: 5
  Final error:    0.0012
  Translation error: 0.0023 m (threshold: 0.05 m)
  Rotation error:    0.1234 deg (threshold: 5.0 deg)
  PASS

--- Test 2: Small Rotation (5 deg about Z) + Translation ---
  PASS

--- Test 3: Identity Transform (no motion) ---
  PASS

--- Test 4: SE3Transform operations ---
  PASS

  Results: 4 PASSED, 0 FAILED
```

### Test 2: Tracking Pipeline End-to-End Test

Tests the full 9-stage pipeline with synthetic scene data:

```bash
./scripts/run.sh test_tracking_pipeline
```

**What it tests:**
- Keyframe initialization (full pipeline: voxel downsample → normal estimation → RANSAC → clustering)
- Single tracking frame processing (all 9 stages)
- Multi-frame tracking (5 consecutive frames)
- Point conservation (confirmed + residual = total)
- RVV `propagate_transform_rvv` kernel correctness

---

## Running Benchmarks

### Tracking Mode Benchmark

This is the main benchmark that compares **full pipeline on every frame** vs **tracking mode**:

#### With synthetic data (no PCD file needed):

```bash
./scripts/run.sh tracking_mode_bench
```

#### With a real PCD file:

```bash
./scripts/run.sh tracking_mode_bench data/0000000000.pcd benchmarks/ 10
```

Arguments:
1. `input.pcd` — Input point cloud file (optional; synthetic if omitted)
2. `output_dir` — Directory for JSON results (default: `benchmarks/`)
3. `n_frames` — Number of tracking frames (default: 10)

#### With more tracking frames:

```bash
./scripts/run.sh tracking_mode_bench data/0000000000.pcd benchmarks/ 20
```

### Understanding Benchmark Output

The benchmark outputs:

```
================================================================
   RVPoint Tracking Mode Benchmark
   Full Pipeline vs Tracking Mode — Speedup Comparison
================================================================

=== Benchmark 1: Full Pipeline on Every Frame ===
  Keyframe (full pipeline): 156.42 ms
  Average per-frame (full): 148.31 ms
  Total (full pipeline):    1631.44 ms

=== Benchmark 2: Tracking Mode ===
  Keyframe init: 162.15 ms
  Frame  1: tracking=23.45ms  full=152.31ms  speedup=6.5x ...
  Frame  2: tracking=21.87ms  full=149.22ms  speedup=6.8x ...
  ...

================================================================
                         RESULTS SUMMARY
================================================================
  Overall speedup:        4.8x
  Avg full pipeline/frame: 148.31 ms
  Avg tracking/frame:      22.56 ms
  Avg per-frame speedup:   6.6x

  Average Per-Stage Timing:
    Stage 1 (Voxel Downsample):     3.21 ms
    Stage 2 (Correspondence):       8.45 ms
    Stage 3+4 (Residual+Reduction): 4.12 ms
    Stage 5 (Solve 6x6):            0.01 ms
    Stage 6 (Propagate):            0.23 ms
    Stage 7 (Verify):               1.56 ms
    Stage 8 (Split):                0.89 ms
    Stage 9 (Update Index):         2.34 ms
    Residual Full Pipeline:         1.75 ms
================================================================
```

**Key metrics:**
- **Overall speedup**: Total time saved across all frames
- **Per-frame speedup**: How much faster each tracking frame is vs full pipeline
- **Per-stage timing**: Identifies bottlenecks in the tracking pipeline
- **Residual ratio**: What percentage of points needed full pipeline processing

### JSON Output

Results are saved to `benchmarks/tracking_benchmark_results.json`:

```json
{
  "benchmark": "tracking_mode_vs_full_pipeline",
  "n_raw_points": 120000,
  "n_frames": 10,
  "total_full_pipeline_ms": 1631.44,
  "total_tracking_mode_ms": 340.12,
  "overall_speedup": 4.80,
  "frames": [
    {
      "frame_id": 1,
      "full_pipeline_ms": 152.31,
      "tracking_total_ms": 23.45,
      "speedup": 6.5,
      "stage_timing_ms": {
        "voxel_downsample": 3.21,
        "correspondence": 8.45,
        ...
      }
    },
    ...
  ]
}
```

---

## Running on Real Point Cloud Data

### Using Existing Data

The repo includes `data/0000000000.pcd`:

```bash
./scripts/run.sh tracking_mode_bench data/0000000000.pcd benchmarks/ 10
```

### Using Compressed PCD Sequences

If you have multiple PCD files (frame sequences):

```bash
# Extract compressed PCDs
cd data/pcd_compressed
unzip ../pcd_compressed.zip

# Run benchmark on first frame
./scripts/run.sh tracking_mode_bench data/pcd_compressed/0000000000.pcd benchmarks/ 15
```

---

## Troubleshooting

### Build Fails with "RVV support is mandatory"

Make sure you're building with the RVV backend:

```bash
./scripts/build.sh --backend rvv
```

### QEMU "Illegal instruction" Error

This usually means the QEMU CPU model doesn't have vector extension enabled. The `run.sh` script adds `-cpu rv64,v=true,vlen=128` automatically, but verify:

```bash
qemu-riscv64 -cpu rv64,v=true,vlen=128 <binary>
```

### Test Fails with "Not enough correspondences"

This can happen with very small point clouds or very large transforms. The synthetic tests use small transforms (1-5°) which should always converge. If testing with real data, ensure:
- Point cloud has > 100 points after downsampling
- The applied transform between frames is small (< 10° rotation, < 0.5m translation)

### Environment Not Found

```bash
# Re-run setup
./env/setup.sh

# Or manually source the environment
source env/activate.sh
```

---

## File Locations

| File | Description |
|---|---|
| `src/registration/tracking_types.h` | Data structures (SE3Transform, PlaneModel, etc.) |
| `src/registration/icp_registration.h/cpp` | ICP core (Stages 2–5) |
| `src/registration/tracking_pipeline.h/cpp` | Full 9-stage pipeline orchestrator |
| `src/tools/tracking_pcd_compare.cpp` | Real-PCD comparison tool (full vs tracking mode) |
| `tests/unit/test_tracking_registration.cpp` | ICP unit tests |
| `tests/unit/test_tracking_pipeline.cpp` | Pipeline end-to-end tests |
| `tests/benchmark/tracking_mode_bench.cpp` | Speedup benchmark |
| `benchmarks/` | JSON benchmark results (generated) |
| `documentation/tracking_mode_design.md` | Technical design document |

---

## Real PCD Comparison: Full Pipeline vs Tracking Mode

This section explains how to process the sequential PCD frames from `data/pcd_compressed/` through both the full pipeline and the tracking mode, then visually compare the outputs.

### Overview

The `tracking_pcd_compare` tool:

1. **Loads** N sequential PCD frames from a directory
2. **Mode A (Full Pipeline)**: Processes each frame independently through the complete pipeline (Downsample → SOR → Normal Estimation → RANSAC → Ground Removal)
3. **Mode B (Tracking Mode)**: Runs the full pipeline only on **keyframes** (every K-th frame), and the lightweight 9-stage tracking pipeline on all other frames
4. **Exports** processed PCD files to separate directories for side-by-side visual comparison
5. **Reports** per-frame timing and overall speedup

### Running the Comparison

#### Basic usage (first 10 frames):

```bash
./scripts/run.sh tracking_pcd_compare data/pcd_compressed data/pcd_processed 10
```

#### Full 131-frame sequence:

```bash
./scripts/run.sh tracking_pcd_compare data/pcd_compressed data/pcd_processed 0
```

> Setting `max_frames` to `0` processes all available PCD files.

#### Custom keyframe interval:

```bash
# Keyframe every 10 frames (more tracking, bigger speedup, potentially less accurate)
./scripts/run.sh tracking_pcd_compare data/pcd_compressed data/pcd_processed 50 10

# Keyframe every 3 frames (more keyframes, less speedup, closer to full pipeline)
./scripts/run.sh tracking_pcd_compare data/pcd_compressed data/pcd_processed 50 3
```

#### Arguments:

| Argument | Description | Default |
|---|---|---|
| `pcd_dir` | Directory containing sequential `.pcd` files | (required) |
| `output_dir` | Output directory for processed PCDs | (required) |
| `max_frames` | Number of frames to process (`0` = all) | (required) |
| `keyframe_interval` | Full pipeline runs every N-th frame | `5` |

### Output Directory Structure

After running, the output directory will contain:

```
data/pcd_processed/
├── full_pipeline/            # Mode A: Every frame processed independently
│   ├── 0000000000.pcd
│   ├── 0000000001.pcd
│   ├── ...
│   └── 0000000049.pcd
├── tracking_mode/            # Mode B: Keyframes + tracking frames
│   ├── 0000000000.pcd        # [KEYFRAME] Full pipeline
│   ├── 0000000001.pcd        # [TRACKING] Tracking mode
│   ├── 0000000002.pcd        # [TRACKING]
│   ├── ...
│   ├── 0000000005.pcd        # [KEYFRAME] Full pipeline
│   └── ...
└── comparison_results.json   # Per-frame timing comparison
```

### Understanding the Output

- **`full_pipeline/`**: Each PCD is the **ground-removed** point cloud (RANSAC outliers) — this is the baseline "correct" output.
- **`tracking_mode/`**: Keyframe PCDs match `full_pipeline/` exactly. Tracking frame PCDs contain the **residual points** — points not matched to known geometric models. These should look similar to `full_pipeline/` if tracking is working correctly.

### Console Output Example

```
================================================================
   RVPoint — Tracking Mode PCD Comparison
================================================================
  PCD directory:      data/pcd_compressed
  Max frames:         10
  Keyframe interval:  5
================================================================

=== Mode A: Full Pipeline (every frame independently) ===
  Frame    0: 1542.3 ms, 28451 output points
  ...

=== Mode B: Tracking Mode (keyframe every 5 frames) ===
  [KEYFRAME] Frame    0: full pipeline + tracking init (1612.5 ms), 28451 pts
  [TRACKING] Frame    1: tracking mode (245.3 ms), confirmed=25122 residual=3891 ...
  [TRACKING] Frame    2: tracking mode (238.1 ms), ...
  ...
  [KEYFRAME] Frame    5: full pipeline + tracking init (1598.7 ms), 28103 pts
  ...

================================================================
                      COMPARISON SUMMARY
================================================================
  Total full pipeline:    15234.1 ms
  Total tracking mode:     5891.2 ms
  Overall speedup:        2.59x
================================================================
```

---

## Visual Comparison Guide

To verify that tracking mode produces visually similar results to the full pipeline, open matching frame PCD files from both output directories in a 3D point cloud viewer.

### Option 1: CloudCompare (Recommended — GUI)

CloudCompare is a free, cross-platform 3D point cloud viewer.

1. **Install**: Download from [cloudcompare.org](https://www.cloudcompare.org/release/index.html)
2. **Open both files side by side**:
   - `File → Open` → select `data/pcd_processed/full_pipeline/0000000001.pcd`
   - `File → Open` → select `data/pcd_processed/tracking_mode/0000000001.pcd`
3. **Color-code** each cloud differently (click the cloud in the DB Tree → Properties → change color)
4. **Toggle visibility** of each cloud to spot differences
5. **Use Cloud-to-Cloud Distance** (`Tools → Distances → Cloud/Cloud Dist`) for quantitative comparison

### Option 2: Open3D (Python)

```python
import open3d as o3d

full = o3d.io.read_point_cloud("data/pcd_processed/full_pipeline/0000000001.pcd")
track = o3d.io.read_point_cloud("data/pcd_processed/tracking_mode/0000000001.pcd")

full.paint_uniform_color([0.2, 0.6, 1.0])   # Blue = full pipeline
track.paint_uniform_color([1.0, 0.3, 0.2])  # Red  = tracking mode

o3d.visualization.draw_geometries([full, track],
    window_name="Full Pipeline (blue) vs Tracking Mode (red)")
```

### Option 3: pcl_viewer (if PCL is installed)

```bash
# View full pipeline output
pcl_viewer data/pcd_processed/full_pipeline/0000000005.pcd

# View tracking mode output
pcl_viewer data/pcd_processed/tracking_mode/0000000005.pcd
```

### What to Check

| Check | What to Look For |
|---|---|
| **Overall shape** | Both clouds should have the same general 3D structure |
| **Ground removal** | The ground plane should be absent in both outputs |
| **Point density** | Tracking mode may have slightly fewer points (residual-only on non-keyframes) |
| **Keyframe frames** | Frames 0, 5, 10, ... should be **identical** between both modes |
| **Tracking frames** | Frames 1-4, 6-9, ... may have minor differences but overall shape should match |
| **Drift** | Check later frames (e.g., frame 40+) — if tracking drifts, the keyframe resets will correct it |

### JSON Results

The `comparison_results.json` file contains per-frame timing for quantitative analysis:

```json
{
  "comparison": "full_pipeline_vs_tracking_mode",
  "n_frames": 50,
  "keyframe_interval": 5,
  "total_full_pipeline_ms": 75234.12,
  "total_tracking_mode_ms": 29891.23,
  "overall_speedup": 2.52,
  "frames": [
    {"frame": 0, "full_ms": 1542.3, "tracking_ms": 1612.5, "speedup": 0.96},
    {"frame": 1, "full_ms": 1501.2, "tracking_ms": 245.3, "speedup": 6.12},
    ...
  ]
}
```
