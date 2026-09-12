# ADR 0012: Slotted Register-File PipelineContext & Dynamic PipelineManager

- **Status**: accepted
- **Deciders**: Fahad, Antigravity
- **Date**: 2026-09-11

## Context & Decision

Perception pipelines in `eval/pipelines/` (such as `pipeline_3d_intra.cpp`) suffered from monolithic, hardcoded wiring (>1,700 lines), duplicating stage orchestration, memory management, and telemetry export. We needed an extensible, composable pipeline architecture that supports non-linear DAG flows, live parameter tuning, and intermediate data probing without introducing runtime string lookups, mutex contention, or heap allocations in the 50 Hz perception loop.

We decided to adopt a **Slotted Heterogeneous Register-File PipelineContext & ConfigStore** orchestrated by a **PipelineManager**:
1. **Heterogeneous Slotted Data Register File (`PipelineContext`)**: Stores arbitrary pre-allocated C++ representations (`PointCloudSoA`, `Fast3DSpatialGrid`, `PointerOctree`, `NormalsSoA`, `PlaneModel`, cluster vectors) via strongly-typed, lightweight `SlotId` integer handles. Slot names are user-specified per node instance (with sensible defaults) to allow multiple instances of the same kernel in DAG topologies. String names are resolved **once during setup**; runtime access is a direct $O(1)$ array dereference (1 CPU cycle on RISC-V).
2. **Pure Reference Passing for Multi-Input / Multi-Output Kernels**: Algorithmic kernels are pure C++ functions/lambdas that take standard C++ references (`const In1&, const In2&, Out1&, Out2&`). The kernel has zero knowledge of `PipelineContext` or `SlotId`. The `PipelineManager` retrieves references directly from the store and passes them to the kernel at zero-copy overhead.
3. **Symmetrical Parameter Store (`ConfigStore`) & Reconfiguration Hooks**: Mirrors the data register file for pipeline configuration parameters (floats, ints, strings). Numerical thresholds pass directly to kernels per frame. For structural parameters that require index reinitialization (e.g. `grid_cell_size`, `octree_max_depth`), nodes can register an `on_param_change` hook that reinitializes internal hash buckets or lookup tables before the next frame, maintaining zero overhead in steady state.
4. **Tagged Positional Node Registration**: Developers do not write manual `PipelineStage` boilerplate subclasses for every algorithm. Instead, nodes are registered via a unified `.bind(in<T>("name"), out<T>("name"), param<T>("name", default_val)).kernel(callable)` API. This eliminates implicit lambda deduction traps, guarantees 1:1 positional and semantic alignment, supports generic lambdas and direct free functions (`&voxel_grid_downsamp_rvv_v2`), and produces clean 1-line `static_assert` error messages on signature mismatch.
5. **PipelineManager**: Handles node sequencing, slot dependency resolution, dynamic parameter entry points, and lifecycle management.
6. **Data & Lifecycle Hooks**: Telemetry, MCAP streaming, and intermediate PCD probing are implemented simply as post-execution hooks attached to nodes or slots, eliminating the need for a separate probe manager subsystem.


## Consequences

- Full DAG / branching composability: stages are loosely coupled to slots, allowing arbitrary pipeline topologies with multi-input/multi-output nodes (e.g. RANSAC writing planes and splitting clouds, NormalEstimation taking clouds and spatial grids).
- Pure algorithm separation: compute kernels remain 100% standard C++ taking normal references, fully unit-testable outside of any pipeline manager.
- Zero boilerplate: adding a new algorithm or kernel to the pipeline is a 3-line node declaration rather than writing custom stage classes.
- Zero-heap invariant is preserved across frames: slots pre-allocate buffer capacities at setup and reset logical lengths without freeing memory.
- Dynamic parameter tuning: parameters can be tweaked live with negligible overhead (<1 microsecond).
- `pipeline_3d_intra.cpp` shrinks from 1,798 lines to a clean declarative script.


