# Research Report: Kernel Composition, Dependency Injection, and the "Leaf-Kernel Invariant" in RVPoint

> **Investigation into Kernel Ownership, Scratchpad Allocations, and Dependency Injection across the RVPoint Perception Stack.**
> **Status:** Approved Recommendation
> **Date:** September 13, 2026

---

## 1. Executive Summary

The architectural rule proposed—**"Kernels shouldn't own kernels; embedded dependencies like `RansacPlane` in `GroundFilter` should be dependency-injected or decomposed into discrete pipeline DAG nodes"**—is **technically essential, microarchitecturally advantageous, and directly enforces RVPoint's core Architectural Decision Records (ADRs)**.

Embedding `RansacPlane ransac_;` as a private member inside `GroundFilter`:
1. **Violates the Zero-Heap Hot-Path Invariant ([ADR-0010](../adr/0010-zero-heap-hot-path-invariant.md))**: `GroundFilter::process()` introduces stack-allocated fallbacks (`PointCloud local_body;`) that dynamically reallocate heap memory per frame if `body_cloud_out` is `nullptr`. Furthermore, it permanently bakes $\sim 274\,\text{KB}$ of `RansacPlane` scratch buffers inside `GroundFilter`, polluting memory when RANSAC is dormant 99% of the time ([ADR-0008](../adr/0008-static-elevation-slicing.md)).
2. **Defeats Slotted Pipeline Observability ([ADR-0012](../adr/0012-slotted-register-file-pipeline-manager.md))**: Encapsulating `RansacPlane` conceals the fitted ground equation (`PlaneModel`), ground inlier count, and intermediate leveled point cloud (`body_cloud`) from the `PipelineManager` declarative probe system (`pm.add_probe<T>()`) and MCAP visualizers.
3. **Impedes Multi-Core Scaling ([ADR-0013](../adr/0013-multi-core-execution-modes-and-slot-retention.md))**: In worker-pool execution modes, each worker cloning `GroundFilter` duplicates dormant RANSAC scratchpad allocations across all 8 SpacemiT K1 cores, evicting the 32 KB L1D / 512 KB L2 cache working sets.
4. **Blurs Architectural Taxonomy**: `RansacPlane` is an **Atomic Compute Kernel (Leaf)**, whereas the current `GroundFilter` is a **Compound Pipeline Stage**. Conflating the two creates monolithic black boxes that cannot be independently scheduled, profiled, or reused.

---

## 2. Primary Source & Architectural Invariant Investigation

### 2.1 Alignment with RVPoint Invariants

| Reference | Mandated Invariant | Current `GroundFilter` Violation |
|---|---|---|
| **ADR-0010** *(Zero-Heap Hot-Path)* | Prohibits dynamic heap allocations (`new`, `resize`, `malloc`) inside the 30–50 Hz per-frame perception loop on the SpacemiT K1 SoC. Caller-owned buffers must be passed by reference. | In `GroundFilter::process()`: if caller passes `body_cloud_out = nullptr`, a local `PointCloud local_body` is allocated on the stack and resized via `body_cloud.resize(n)` on every frame. |
| **ADR-0011** *(Non-Virtual Deep Class)* | Core algorithms are stateful, zero-vtable C++ classes with value semantics that manage scratch workspaces. | ADR-0011 was designed for **leaf mathematical algorithms** (`VoxelGrid`, `RansacPlane`, `NormalEstimation`). Embedding one deep class inside another creates nested scratch workspaces and hidden secondary state machines. |
| **ADR-0012** *(Slotted Register-File & DAG)* | Stages communicate via typed, pre-bound slot pointers (`in<T>`, `out<T>`, `param<T>`). Kernels are pure callables taking standard C++ references. | `GroundFilter` acts as a monolithic black box. `PipelineManager` cannot observe or bind to `PlaneModel` or intermediate `body_cloud` because RANSAC is executed privately inside `GroundFilter::calibrate_ground()`. |
| **ADR-0008** *(Static Elevation Slicing over Per-Frame RANSAC)* | Mandates running RANSAC *only once at startup calibration or when stationary*, freezing the floor height for nominal driving to achieve $<0.05\,\text{ms}$ latency ($30\times$ speedup). | Embedding `RansacPlane` inside `GroundFilter` forces `GroundFilter` to maintain an internal state flag (`bool calibrated_`), mixing one-shot calibration logic with 50 Hz streaming elevation slicing. In a DAG, the scheduler should bypass Stage 2 during nominal driving. |
| **ADR-0013** *(Slot Retention & Worker Pools)* | Each worker thread clones the prototype `FrameContext` and node state. | Every worker clones an entire `RansacPlane` instance and its internal scratch capacity, multiplying dormant memory across 8 cores. |

### 2.2 Peer Systems & Industry Standards

- **Halide & TVM / MLIR**: Strict separation of leaf compute primitives (micro-kernels) from scheduling and memory planning. Micro-kernels never instantiate or embed other micro-kernels; memory buffers are owned and allocated by the top-level pipeline scheduler.
- **Ceres Solver & Eigen**: Cost functions and residual blocks (`CostFunction`) are pure leaf evaluators. Solvers (`TrustRegionMinimizer`) and preconditioners are never owned by individual cost functions. Workspaces are supplied via execution contexts.
- **Autoware.Universe & Apollo**: The perception stack decomposes sensor processing into distinct, publish-subscribe nodelets:
  `crop_box_filter_nodelet` $\rightarrow$ `ground_segmentation_nodelet` $\rightarrow$ `voxel_grid_filter_nodelet`.
  Ground plane parameters are published as explicit messages (`/perception/ground_plane`) so downstream obstacle clustering, costmaps, and safety monitors observe the exact calibration state.
- **Game Engine ECS (Unreal Niagara, Unity DOTS)**: Systems and compute jobs are stateless operators over SoA buffers. "Systems do not own Systems; Kernels do not own Kernels." Temporary scratchpad memory is drawn from a linear frame arena or context.

---

## 3. Microarchitectural Analysis: Cache Footprint on SpacemiT K1

On the **SpacemiT K1 SoC**, each of the 8 SpacemiT X60 cores possesses a **32 KB private L1 Data Cache** (64-byte cache line, 8,192 single-precision floats).

In `src/segmentation/ransac_plane/ransac_plane.h`, `RansacPlane` owns five persistent scratch vectors to fulfill ADR-0010:
```cpp
std::vector<int> inlier_indices_scratch_;   // Up to N * 4 bytes (e.g., 50k pts = 200 KB)
std::vector<uint8_t> inlier_mask_scratch_;  // Up to N bytes (50 KB)
std::vector<float> sample_x_;              // 2048 * 4 bytes = 8 KB
std::vector<float> sample_y_;              // 8 KB
std::vector<float> sample_z_;              // 8 KB
```
Total scratch capacity per `RansacPlane` instance exceeds **$274\,\text{KB}$**—more than **$8.5\times$ the entire L1D cache**!

When `GroundFilter` owns `RansacPlane`:
1. During nominal driving (ADR-0008), RANSAC is **never called**; points are purely streamed through `transform_to_body` and `filter_ground`.
2. Despite being completely idle, the $274\,\text{KB}$ scratch allocation remains pinned in memory inside `GroundFilter`.
3. Under ADR-0013, if an 8-worker thread pool is used, **$8 \times 274\,\text{KB} \approx 2.2\,\text{MB}$** of scratch buffers are duplicated across cores, causing L2 cache eviction and cache line thrashing.
4. If instead `RansacPlane` is owned by `PipelineManager` / `FrameContext` as a dedicated node or injected workspace, this memory is only allocated where and when needed, or shared across mutually exclusive stages.

---

## 4. Architectural Taxonomy

```text
┌────────────────────────────────────────────────────────────────────────┐
│                        TIER 3: PIPELINE ENGINE                         │
│   PipelineManager, FrameContext, RegisterFile, StreamResequencer       │
│   - Owns DAG topological scheduler, Slot memory, Probes, Telemetry     │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Orchestrates
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                   TIER 2: COMPOUND PIPELINE STAGE                      │
│   GroundSegmentationStage, ObstacleClusterTrackingStage                │
│   - Composes multiple Tier 1 kernels to achieve a domain pipeline task │
│   - INVARIANT: Must NEVER privately instantiate Tier 1 kernels.        │
│   - Receives child kernels via Dependency Injection (Refs/Functors)    │
└───────────────────────────────────┬────────────────────────────────────┘
                                    │ Executes
                                    ▼
┌────────────────────────────────────────────────────────────────────────┐
│                     TIER 1: ATOMIC COMPUTE KERNEL                      │
│   VoxelGrid, RansacPlane, NormalEstimation, BodyElevationFilter        │
│   - Pure mathematical transformation on PointCloudSoA / arrays         │
│   - INVARIANT: Leaf node. Must NEVER own or instantiate other kernels. │
│   - Zero vtable, stateful value semantics (ADR-0011), zero-heap (ADR-10)│
└────────────────────────────────────────────────────────────────────────┘
```

- **Atomic Compute Kernel (Tier 1 / Leaf)**: An indivisible mathematical operation (e.g., `RansacPlane`, `VoxelGrid`, `PassThroughFilter`). It must never own, instantiate, or invoke another kernel.
- **Compound Pipeline Stage (Tier 2 / Composite)**: A multi-step perception subsystem. It coordinates atomic kernels either via declarative DAG construction in `PipelineManager` or via Dependency Injection.
- **Pipeline Engine (Tier 3 / Orchestrator)**: Manages buffer allocation, slot lifecycle, and execution sequencing.

---

## 5. Architectural Recommendation

1. **Adopt the Leaf-Kernel Invariant as a Official Standard**:
   > *"Atomic compute kernels in `src/` must be pure leaf operators and must NEVER embed, instantiate, or own other compute kernels. All dependent kernels or scratchpads must be dependency-injected by the caller or orchestrated via `PipelineManager` DAG slots."*
2. **Refactor `GroundFilter`**:
   - Make `GroundFilter` accept `RansacPlane&` via Dependency Injection for `calibrate_ground(body_cloud, ransac_kernel)`.
   - In `PipelineManager` (ADR-0012), split the perception pipeline into explicit discrete nodes:
     1. `extrinsics_transform`: `cam_cloud` $\rightarrow$ `body_cloud` (using gravity)
     2. `ransac_ground`: `body_cloud` $\rightarrow$ `ground_plane` (`PlaneModel`) [Executed during startup / calibration]
     3. `ground_elevation_slice`: `body_cloud` + `ground_plane` $\rightarrow$ `obstacles` + `ground`
   - Eliminate heap reallocation in `process()` by requiring caller-owned workspace buffers.

