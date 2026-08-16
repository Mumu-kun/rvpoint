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
| `tests/unit/test_tracking_registration.cpp` | ICP unit tests |
| `tests/unit/test_tracking_pipeline.cpp` | Pipeline end-to-end tests |
| `tests/benchmark/tracking_mode_bench.cpp` | Speedup benchmark |
| `benchmarks/` | JSON benchmark results (generated) |
| `documentation/tracking_mode_design.md` | Technical design document |
