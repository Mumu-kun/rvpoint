# RVPoint End-to-End Vector Perception & Tracking Pipeline: Future Engineering Roadmap

**Date:** September 18, 2026  
**Document ID:** `docs/plans/2026-09-18-rvpoint-end-to-end-perception-roadmap.md`  
**Target Codebase:** `src/core/`, `src/filters/`, `src/segmentation/`, `src/tracking/`, `eval/pipelines/`  
**Status:** Architecture Proposal & Implementation Roadmap  

---

## 1. Executive Summary & Strategic Objective

The **RVPoint Ultra** perception pipeline has demonstrated an end-to-end **15.6× speedup** over standard PCL 1.14 on RISC-V vector architectures (`rv64gcv`) by substituting heap-allocated Pointer structures and iterative solvers with:
1. Pure **Structure-of-Arrays (SoA)** 64-byte aligned memory layouts,
2. Hardware unit-stride streaming (`vle32.v`) and vector fused multiply-accumulate (`vfmacc.vf`, `vfmacc.vv`),
3. Wald Sequential Probability Ratio Test (SPRT) early-rejection RANSAC, and
4. Symmetric 13-forward neighbor spatial grid clustering with two-pass Compressed Sparse Row (CSR) reduction.

### The Objective of This Roadmap
To evolve RVPoint from an accelerated point cloud filtering/segmentation library into a **complete, production-grade, zero-copy autonomous vehicle (AV) perception engine**. This roadmap defines the architectural formalization, kernel designs, vector intrinsics, and implementation milestones across five unified perception stages:

```text
       [ Raw LiDAR PCD (FIELDS x y z) ]
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│ 1. Spatial Conditioning, Indexing & Normalization           │
│    • rigid_transform_kernel()      [IMU/Odom Gravity Align] │
│    • passthrough_roi_filter()      [Vectorized Spatial Crop]│
│    • voxel_grid_downsample_rvv()   [Centroid Downsampling]  │
│    • Fast3DSpatialGrid::build()    [O(1) Spatial Hash Grid] │
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ 2. Fast Filtering & Feature Geometry Processing             │
│    • execute_voxel_ror_rvv()       [Vector Radius Outlier]  │
│    • normal_estimation_cardano()   [Closed-Form Cardano/PCA]│
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ 3. High-Throughput Vector RANSAC Ground Engine              │
│    • compute_plane_coeffs()        [3-Point Cross Product]  │
│    • upright_prior_gate()          [O(1) Vertical Gate]     │
│    • vector_wald_sprt_score()      [2048-Sample + vcpop.m]  │
│    • extract_inliers_outliers_soa()[vcompress.vm Mask Comp] │
│    • covariance_minor_refine()     [Closed-Form 3x3 Minors] │
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ 4. Obstacle Topology & Union-Find Clustering                │
│    • 13_forward_neighbor_lookup()  [O13 Symmetry (50% Cut)] │
│    • vector_distance_vfmacc()      [vfmacc.vv Euclidean]    │
│    • flat_union_find_compress()    [O(alpha(N)) Disjoint Set│
│    • two_pass_csr_reduce()         [Zero-Allocation Grouping│
└─────────────────────────────┬───────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ 5. Downstream AV Perception & Tracking Stack (New Phase)    │
│    • l_shape_box_fit()             [RVV Oriented 3D BBoxes] │
│    • kalman_filter_track()         [Hungarian MOT + EKF]    │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. Technical Specifications by Pipeline Stage

### Stage 1: Spatial Conditioning, Indexing & Normalization

#### 1.1 `rigid_transform_kernel()` (New RVV Primitive)
* **Mathematical Purpose:** Transform points from sensor coordinates $\mathbf{p}_{\text{sensor}}$ to world/vehicle coordinates $\mathbf{p}_{\text{world}}$ using a $3 \times 3$ rotation matrix $\mathbf{R} = [r_{ij}]$ and translation $\mathbf{T} = [t_x, t_y, t_z]^T$:
  $$\begin{bmatrix} x_{\text{world}} \\ y_{\text{world}} \\ z_{\text{world}} \end{bmatrix} = \begin{bmatrix} r_{00} & r_{01} & r_{02} \\ r_{10} & r_{11} & r_{12} \\ r_{20} & r_{21} & r_{22} \end{bmatrix} \begin{bmatrix} x \\ y \\ z \end{bmatrix} + \begin{bmatrix} t_x \\ t_y \\ t_z \end{bmatrix}$$
* **Vector Implementation Strategy:**
  Stream contiguous SoA arrays $\mathbf{X}, \mathbf{Y}, \mathbf{Z}$ in blocks of $V_L$ floats (LMUL=8):
  ```cpp
  vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
  vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
  vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);

  // x_out = r00*x + r01*y + r02*z + tx
  vfloat32m8_t wx = __riscv_vfmul_vf_f32m8(vx, r00, vl);
  wx = __riscv_vfmacc_vf_f32m8(wx, r01, vy, vl);
  wx = __riscv_vfmacc_vf_f32m8(wx, r02, vz, vl);
  wx = __riscv_vfadd_vf_f32m8(wx, tx, vl);
  __riscv_vse32_v_f32m8(&out.x[i], wx, vl);
  ```
* **Performance Target:** $< 0.15\,\text{ms}$ on 100k points (memory bandwidth saturated).

#### 1.2 `passthrough_roi_filter()` (New RVV Primitive)
* **Mathematical Purpose:** Vectorized spatial pruning against a 3D bounding box $[x_{\min}, x_{\max}] \times [y_{\min}, y_{\max}] \times [z_{\min}, z_{\max}]$:
  $$\mathcal{M}_{\text{roi}}(i) = (x_i \ge x_{\min}) \land (x_i \le x_{\max}) \land (y_i \ge y_{\min}) \land (y_i \le y_{\max}) \land (z_i \ge z_{\min}) \land (z_i \le z_{\max})$$
* **Vector Implementation Strategy:**
  Evaluate 6 float inequalities per lane, reduce to a single vector boolean mask `vbool4_t`, and compress via `__riscv_vcompress_vm_f32m8`.
* **Deliverable:** `src/filters/passthrough_roi.h`.

#### 1.3 `voxel_grid_downsample_rvv()` & `Fast3DSpatialGrid::build()`
* **Current Status:** Implemented in `src/filters/voxel_grid.h` and `src/search/fast_3d_spatial_grid.h`.
* **Action Item:** Expose a standardized interface accepting `PointCloudSoA` inputs with zero temporary memory allocations across frame streams.

---

### Stage 2: Fast Filtering & Feature Geometry Processing

#### 2.1 `execute_voxel_ror_rvv()`
* **Current Status:** Implemented in `eval/pipelines/pipeline_3d_ultra.cpp`.
* **Action Item:** Promote from pipeline utility into `src/filters/radius_outlier_removal_rvv.h` as a public library module. Incorporate self-cell fastpath logic:
  - If a cell contains $\ge K_{\min}$ points, all points in that cell are immediately marked valid without cross-cell distance checks.

#### 2.2 `normal_estimation_cardano()`
* **Current Status:** Analytical cubic solver formulated in `src/features/normal_estimation.h` and `docs/paper/rvpoint_mathematical_formulation.tex`.
* **Action Item:** Standardize SoA outputs (`nx, ny, nz` buffers) and verify IEEE-754 precision parity against PCL's Jacobi SVD.

---

### Stage 3: High-Throughput Vector RANSAC Ground Engine

Promote all RANSAC components from `eval/pipelines/pipeline_3d_ultra.cpp` into `src/segmentation/ransac_ground_ultra.h`:

1. **`compute_plane_coeffs()`**: Fast 3-point cross-product solver with degenerate collinearity checks.
2. **`upright_prior_gate()`**: $\mathcal{O}(1)$ normal angle gate $|c| \ge \cos(\theta_{\max}) \approx 0.707$ for gravity vertical alignment (bypassed via `--no-ground-prior`).
3. **`vector_wald_sprt_score()`**: 2,048-point strided uniform sample scoring with `vfmacc.vf` distance accumulation, bit-count inlier extraction via `vcpop.m`, and dynamic Wald SPRT iteration reduction.
4. **`extract_inliers_outliers_direct_soa()`**: Zero-copy hardware vector compression via `__riscv_vcompress_vm_f32m8`.
5. **`covariance_minor_refine()`**: Closed-form $3 \times 3$ covariance minor cross products:
   $$r_x = C_{01}C_{12} - C_{02}C_{11}, \quad r_y = C_{01}C_{02} - C_{00}C_{12}, \quad r_z = C_{00}C_{11} - C_{01}^2$$

---

### Stage 4: Obstacle Topology & Union-Find Clustering

Promote the complete clustering engine from `eval/pipelines/pipeline_3d_ultra.cpp` into `src/segmentation/euclidean_clustering_ultra.h`:

1. **`13_forward_neighbor_lookup()`**: 13-offset forward hemisphere search eliminating half of all spatial queries.
2. **`vector_distance_vfmacc()`**: Vectorized Euclidean metric lane evaluation via `vfmacc.vv`.
3. **`flat_union_find_compress()`**: Array-backed disjoint set with two-pass path compression in $\mathcal{O}(\alpha(N))$.
4. **`two_pass_csr_reduce()`**: Zero-allocation static two-pass CSR cluster grouping.

---

### Stage 5: Downstream AV Perception & Tracking Stack (New Architecture)

To bridge RVPoint with autonomous driving controllers, implement two foundational perception modules:

#### 5.1 `l_shape_box_fit()` (Oriented 3D Bounding Box Fitting)
* **Objective:** For each segmented obstacle cluster $\mathcal{C}_k = \{\mathbf{p}_i\}$, compute the minimal bounding volume with yaw angle $\psi \in [-\pi/2, \pi/2)$:
  $$\mathcal{B}_k = \{x_c, y_c, z_c, L, W, H, \psi\}$$
* **Vector Acceleration Strategy:**
  1. Project cluster points onto the $XY$ ground plane: $\mathbf{p}'_i = (x_i, y_i)$.
  2. Compute 2D convex hull via Graham scan or Monotone Chain.
  3. Search over orientation angles $\theta \in [0, \pi/2)$ with angular step $\Delta \theta = 5^\circ$ (18 candidate orientations).
  4. For each angle $\theta$, evaluate directional projections concurrently across vector lanes using `vfmacc.vf`:
     $$u_i = x_i \cos\theta + y_i \sin\theta, \quad v_i = -x_i \sin\theta + y_i \cos\theta$$
  5. Use `vfredmax` and `vfredmin` to extract $[u_{\min}, u_{\max}]$ and $[v_{\min}, v_{\max}]$.
  6. Minimize the search criterion (Area criterion $A(\theta) = \Delta u \cdot \Delta v$ or Closeness criterion to LiDAR ray).
* **Deliverable:** `src/tracking/l_shape_box_fit.h`.

#### 5.2 `kalman_filter_track()` (Multi-Object Tracking - MOT)
* **Objective:** Maintain persistent object tracks across consecutive LiDAR frames, estimate velocity vectors $(\vec{v}_x, \vec{v}_y)$, and handle occlusion.
* **State Vector & Motion Model:**
  $$\mathbf{x}_t = \begin{bmatrix} x & y & z & \psi & v_x & v_y & \dot{\psi} \end{bmatrix}^T$$
  Using a Constant Velocity (CV) or Constant Turn Rate and Velocity (CTRV) motion model.
* **Data Association:**
  - 3D Bounding Box Generalized Intersection-over-Union (GIoU) or Mahalanobis distance metric.
  - Linear Sum Assignment via Jonker-Volgenant or greedy cost gating.
* **Track State Machine:**
  `Tentative (N frames)` $\rightarrow$ `Confirmed` $\rightarrow$ `Coasted (Occluded)` $\rightarrow$ `Deleted`.
* **Deliverable:** `src/tracking/kalman_tracker.h`.

---

## 3. Implementation Phases & Milestones

| Phase | Module | Target File Path | Dependencies | Estimated Complexity |
| :---: | :--- | :--- | :--- | :---: |
| **Phase 1** | Rigid Transform Kernel | `src/core/rigid_transform_rvv.h` | `core/point_types.h`, RVV 1.0 | Low (1-2 days) |
| **Phase 1** | Passthrough ROI Filter | `src/filters/passthrough_roi_rvv.h` | `core/point_types.h`, RVV 1.0 | Low (1-2 days) |
| **Phase 2** | Library Ground RANSAC | `src/segmentation/ransac_ground_ultra.h` | `ransac_plane_sprt_rvv` extraction | Medium (2-3 days) |
| **Phase 2** | Library Union-Find Clust | `src/segmentation/euclidean_clustering_ultra.h` | `execute_union_find` extraction | Medium (2-3 days) |
| **Phase 3** | L-Shape Bounding Box Fit | `src/tracking/l_shape_box_fit.h` | `ClusterResult`, RVV 1.0 | High (4-5 days) |
| **Phase 4** | Kalman Multi-Object Track | `src/tracking/kalman_tracker.h` | `BoundingBox3D`, Eigen / RVV | High (5-7 days) |
| **Phase 5** | End-to-End Tracking Pipeline | `eval/pipelines/pipeline_3d_tracker.cpp` | All modules | Medium (3-4 days) |

---

## 4. Verification & Validation Protocol

1. **Unit Testing (`eval/tests/fast/`)**:
   - `test_rigid_transform.cpp`: Verify identity, rotation, and translation against pure math oracle.
   - `test_passthrough_roi.cpp`: Verify point filtering and mask compression parity with scalar loops.
   - `test_l_shape_fit.cpp`: Verify bounding box dimensional accuracy on synthetic vehicle point clouds.
2. **End-to-End Pipeline Evaluation**:
   - Run on KITTI tracking benchmark and nuScenes LiDAR sweeps.
   - Verify that Stage 1–4 latency remains $\le 4.5\,\text{ms}$ on RVV hardware, and Stage 5 overhead is $\le 1.0\,\text{ms}$ per frame (total $\le 5.5\,\text{ms}$, achieving $> 180\,\text{Hz}$ real-time operation).
3. **Build System Compliance**:
   - Comply with the 6-folder root workspace hierarchy and compile clean with `./scripts/build.sh`.
