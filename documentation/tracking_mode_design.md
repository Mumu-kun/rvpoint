# RVPoint Tracking Mode — Technical Design Document

## Overview

Tracking Mode is a fast inter-frame processing pipeline that avoids running the expensive full pipeline (voxel downsample → SOR → RANSAC → normal estimation → clustering) on every frame. Instead, it runs the full pipeline only on **keyframes** and uses a lightweight ICP-based registration + model propagation pipeline for intermediate frames.

**Key insight**: Between consecutive frames from a moving sensor, the scene structure barely changes — only the viewpoint moves. Instead of re-detecting everything from scratch, we can *predict* where existing planes and clusters moved to, *verify* the predictions, and only run full detection on the small set of points that don't match any prediction.

---

## Architecture

```
Frame N (keyframe)                    Frame N+1 (tracking)
┌──────────────────┐                  ┌──────────────────┐
│  Full Pipeline   │                  │   Tracking Mode  │
│                  │                  │                  │
│  Voxel Downsamp  │                  │  [1] Voxel Down  │
│  SOR Filter      │   stored state   │  [2] Correspond. │
│  Normal Estim.   │ ──────────────►  │  [3] Resid+Jacob │
│  RANSAC Planes   │                  │  [4] Reduce 6×6  │
│  Clustering      │                  │  [5] Solve       │
│                  │                  │  [6] Propagate   │
└──────────────────┘                  │  [7] Verify      │
                                      │  [8] Split       │
                                      │  [9] Update Idx  │
                                      │                  │
                                      │  Residual ──► Full Pipeline
                                      │             (on small subset)
                                      └──────────────────┘
```

---

## 9-Stage Pipeline Detail

### Stage 1: Voxel Downsampling

**Reuses**: `voxel_grid_downsamp_rvv_v2()` — unchanged from existing codebase.

**RVV**: LMUL=m4. Vectorized bbox (vfmin/vfmax), voxel key computation (vfmul + vfloor), indexed gather centroid reduction (vluxei32 + vfredosum).

**Purpose**: Reduce raw sensor density to uniform spacing, making all subsequent stages faster.

### Stage 2: Correspondence Search

**New**: `ICPRegistration::findCorrespondences()`

For each downsampled new-frame point:
1. Transform it by the current pose estimate
2. Query the SpatialHash for nearby reference points
3. Use RVV-accelerated distance evaluation to find the nearest neighbor

**RVV**: LMUL=m8 for distance computation. Same `vfsub_vf → vfmul_vv → vfadd_vv` pattern as `get_dist_sq_rvv()` in `rvv_common.cpp`. GEM5_BUILD fallback uses scalar gather + RVV distances.

### Stage 3: Residual + Jacobian Computation

**New**: `residual_jacobian_rvv()`

For each correspondence pair (source_i, target_i):

```
diff_i = T * source_i - target_i
residual_i = normal_i · diff_i          (point-to-plane distance)
J_i = [cross(T*source_i, normal_i), normal_i]  (1×6 Jacobian row)
```

**RVV**: LMUL=m8. Fully vectorized per-pair computation:
- `vfmul_vv`, `vfmacc_vv` for dot products and cross products
- `vfnmsac_vv` for negated multiply-subtract in cross product
- All N pairs processed as contiguous SoA arrays through vector registers

### Stage 4: Reduction to 6×6 System

**Fused with Stage 3** in `residual_jacobian_rvv()`.

Accumulates:
- **JᵀJ**: 6×6 symmetric matrix (21 upper-triangle values) = `sum_i(J_i^T * J_i)`
- **Jᵀr**: 6×1 vector = `sum_i(J_i^T * r_i)`

**RVV**: `vfredusum_vs_f32m8_f32m1` — tree-sum reduction, same pattern as SOR's global mean/variance computation. Each of the 27 accumulations (21 JtJ + 6 Jtr) processes the full N-element column via vfredusum.

### Stage 5: Solve for Transform

**New**: `ICPRegistration::solve6x6()` — **Intentionally scalar.**

Cholesky decomposition of the 6×6 JᵀJ matrix, followed by forward/backward substitution. This is a fixed-size problem (6×6 = 36 elements, 21 unique) — too small for vectorization to provide any benefit.

Returns 6-DOF incremental update `[α, β, γ, tx, ty, tz]` which is converted to an SE3Transform via axis-angle construction.

**Convergence**: Stages 2–5 loop up to `icp_max_iterations` times. Convergence is detected when the update norm² drops below `icp_convergence_thresh`.

### Stage 6: Propagate

**New**: `propagate_transform_rvv()`

Applies the solved SE3Transform to all stored reference data:
- **Plane equations**: `n' = R * n`, `d' = d - n' · t` (normals transform by rotation only)
- **Cluster centroids**: Full SE3 application `c' = R * c + t`
- **Reference normals**: Rotation-only transform `n' = R * n` (bulk RVV processing)

**RVV**: LMUL=m8. Batched matrix-vector multiply:
```
out_x = R00*in_x + R01*in_y + R02*in_z + tx  (via vfmacc_vf)
out_y = R10*in_x + R11*in_y + R12*in_z + ty
out_z = R20*in_x + R21*in_y + R22*in_z + tz
```

### Stage 7: Verify

**New**: `verify_plane_rvv()`, `verify_cluster_rvv()`

**Plane verification**: For each new-frame point, compute `|a*x + b*y + c*z + d|` and check `≤ threshold`. Count inliers. If inlier ratio ≥ `plane_verify_ratio`, the plane is confirmed.

**RVV**: Identical to RANSAC inlier counting — `vmfle_vf + vmfge_vf + vmand + vcpop`.

**Cluster verification**: For each new-frame point, compute squared distance to centroid. If nearby-point ratio ≥ `cluster_verify_ratio`, the cluster is confirmed.

**RVV**: Same distance kernel as SOR — `vfsub_vf → vfmul_vv → vfadd_vv → vmfle_vf → vcpop`.

### Stage 8: Split

**New**: `compact_points_rvv()`

Partitions new-frame points into:
- **Confirmed**: matched an existing plane or cluster (done — no more processing needed)
- **Residual**: didn't match anything (needs full pipeline detection)

**RVV**: Uses `vcompress_vm` + `vsse32` (strided store to AoS) — the same compaction pattern as RANSAC's `extract_plane_inliers_rvv()`.

### Stage 9: Update Spatial Index

**Modified**: `SpatialHash::insertPoints()`, `SpatialHash::removePoints()`

Incrementally updates the spatial hash grid:
- Insert new/residual points (O(1) per point — hash and append)
- Remove disappeared points (O(cells × points_per_cell) worst case)

**RVV**: The "which cell does this point belong to" computation (hash key) can be vectorized using the same voxel keying pattern as Stage 1. The actual hash insertion remains scalar (hash table mutation is inherently sequential).

---

## RVV Parallelism Summary

| Stage | Parallelism | LMUL | Key RVV Intrinsics |
|---|---|---|---|
| 1. Downsample | 🟢 Fully parallel | m4 | vfmin, vfmax, vfmul, vfloor, vluxei32, vfredosum |
| 2. Correspondence | 🟢 Parallel per query | m8/m2 | vfsub_vf, vfmul_vv, vfadd_vv, vluxei32 |
| 3. Residual+Jacobian | 🟢 Fully parallel | m8 | vfmul_vv, vfmacc_vv, vfnmsac_vv |
| 4. Reduction | 🟡 Tree reduction | m8 | vfredusum_vs |
| 5. Solve | 🔴 Scalar (6×6) | — | Plain C++ Cholesky |
| 6. Propagate | 🟢 Fully parallel | m8 | vfmacc_vf, vfmv_v_f |
| 7. Verify | 🟢 Fully parallel | m8 | vmfle_vf, vmfge_vf, vmand_mm, vcpop_m |
| 8. Split | 🟡 Parallel + compact | m8 | vcompress_vm, vsse32 |
| 9. Index Update | 🟡 Parallel hash | m4 | Same as Stage 1 keying |

---

## Data Structures

### SE3Transform

4×4 row-major rigid body transformation. Supports:
- `apply(PointXYZ)` — transform a single point
- `compose(SE3Transform)` — chain two transforms
- `fromAxisAngle(ax, ay, az, tx, ty, tz)` — construct from rotation + translation

### TrackingState

Persistent state across tracking frames:
- Reference cloud (SoA: `ref_x`, `ref_y`, `ref_z`)
- Reference normals (SoA: `ref_nx`, `ref_ny`, `ref_nz`)
- Known planes (`vector<PlaneModel>`)
- Known clusters (`vector<ClusterModel>`)
- Accumulated transform from keyframe origin
- SpatialHash for fast neighbor search

### TrackingResult

Per-frame output:
- Recovered SE3Transform
- Confirmed/new planes and clusters
- Residual points
- Per-stage timing breakdown (milliseconds)

---

## Integration with Existing Pipeline

Tracking mode **does not modify** any existing full-pipeline code. It calls the existing functions on the residual point set:

- `voxel_grid_downsamp_rvv_v2()` — Stage 1
- `normal_estimation_rvv()` — for keyframe initialization
- `ransac_plane_rvv()` — for residual plane detection
- `EuclideanClustering::extract()` — for residual cluster detection

This ensures that existing benchmarks, tests, and ablations continue to work unchanged.

---

## Benchmarking Methodology

The benchmark (`tracking_mode_bench.cpp`) measures:

1. **Full pipeline baseline**: Run the full pipeline (downsample → normal estimation → RANSAC → clustering) independently on every frame
2. **Tracking mode**: Run full pipeline on keyframe (frame 0), then tracking mode on frames 1..N
3. **Per-frame speedup** = full_pipeline_time / tracking_frame_time
4. **Overall speedup** = total_full_time / total_tracking_time
5. **Per-stage breakdown** for identifying bottlenecks
6. **Power proxy**: Estimated instruction savings based on residual ratio

Frame sequences are generated synthetically by applying small incremental rigid transforms (1° rotation + 1cm translation per frame) to simulate real sensor motion.
