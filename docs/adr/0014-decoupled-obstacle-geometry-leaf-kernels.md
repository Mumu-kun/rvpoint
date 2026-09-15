# Decoupled Obstacle Geometry: Pure Leaf Kernels & DTO Elimination

Decompose obstacle geometric feature extraction into three independent, zero-heap Tier 1 leaf kernels (`ConvexHull2D`, `BoundingBoxExtractor`, and `BoundingDiscExtractor`), eliminating monolithic coupled DTOs (`ObstacleGeometry`) and cross-kernel dependencies.

## Status
Accepted

## Context
Ticket 06 originally grouped bounding disc extraction, convex hull computation, and oriented bounding box (OBB) generation into a monolithic class (`ObstacleGeometryExtractor`) producing a coupled data structure:
```cpp
struct ObstacleGeometry {
    BoundingDisc disc;
    OrientedBoundingBox obb;
};
```
When refactored, `BoundingBoxExtractor` retained an overload computing `BoundingDisc`, directly violating the **Leaf-Kernel Invariant** (`AGENTS.md` §3.A) and forcing callers to load 96-byte AoS structs even when only high-rate 12-byte disc clearances were needed by the reactive collision avoidance loop. Furthermore, candidate scratch allocations inside per-cluster loops violated ADR-0010 (Zero-Heap Invariant), and missing runtime backend selectors violated ADR-0011 (Non-Virtual Deep Class Pattern).

## Decision
1. **Strict Leaf-Kernel Invariant**:
   - `ConvexHull2D` (`src/features/convex_hull/`): Pure leaf operator computing CCW `PointCloud2D` polygons from clusters using RVV Akl-Toussaint extrema pre-filtering and Andrew's Monotone Chain ($O(K \log K)$), Vectorized Jarvis March ($O(M \cdot K)$), or Angular Binning ($O(K)$).
   - `BoundingBoxExtractor` (`src/features/bounding_box/`): Pure leaf operator computing `OrientedBoundingBox` from `PointCloud` and `PointCloud2D` hull. Has zero reference to `BoundingDisc`.
   - `BoundingDiscExtractor` (`src/features/bounding_disc/`): Pure leaf operator computing `BoundingDisc` via either RVV point-centroid reduction (`compute_from_points`) or branchless circumscribing disc (`compute_concentric`).
2. **Four Unconditional Bounding Box Strategies**:
   - Leaf kernels must execute deterministically without internal heuristic fallback branches. `BoundingBoxExtractor` supports 4 unconditional strategies:
     * `MIN_AREA`: Freeman-Shapira Rotating Calipers hull edge sweep minimizing 2D area ($O(M)$).
     * `L_SHAPE_ALIGN`: Zhang et al. (2017) RVV Truncated Closeness with midpoint sign-injection (`vfsgnjx.vv`), 3-candidate streaming, and lane-local accumulation.
     * `EDGE_ALIGN`: Hull Edge-Perimeter Alignment (EPA) scoring edge collinearity ($S_{\text{EPA}} = \sum L_k \max(|\mathbf{d}_k \cdot \mathbf{u}|, |\mathbf{d}_k \cdot \mathbf{v}|)^4$), sub-degree exact on vehicle sheet metal in $O(C \cdot M)$ time ($23\times$ faster than point closeness).
     * `WIREFRAME_PCA`: Closed-form $O(M)$ 2D covariance eigendecomposition of hull boundary edges.
   - Any adaptive fallback logic or heuristic gating (e.g., aspect ratio $> 1.3$, $N \ge 20$) is the exclusive responsibility of pipeline orchestrators (`pipeline_obstacles.cpp` with `--strategy adaptive`), not leaf operators.
3. **DTO Elimination**:
   - Eliminate `struct ObstacleGeometry`. Downstream pipelines and `PipelineManager` (ADR-0012) manage independent slots `SlotId::BoundingDiscs` (`std::vector<BoundingDisc>`) and `SlotId::ObstacleBoxes` (`std::vector<OrientedBoundingBox>`).
4. **Stage-Owned Zero-Heap Scratch Workspaces (ADR-0010)**:
   - Pre-allocate all candidate buffers (`CandidateBox candidates_[64]`), index arrays, and decimation buffers (`scratch_dec_x_`, `scratch_dec_y_`) during construction/`reserve()`. Zero dynamic `malloc`, `resize`, or `push_back` on the hot per-cluster execution path.
5. **Backend Dispatch (ADR-0011)**:
   - Provide `enum class Backend { Auto, RVV, Scalar }` across all three feature classes to enable runtime dispatch and validation testing, protected by `#if defined(__riscv_vector)` for compiler portability.
6. **Synchronized Heading Extents & Corners**:
   - Calculate `Point2D corners[4]` strictly after final yaw normalization to prevent heading/extent desynchronization.

## Consequences
- **Positive**:
  - Eliminates $87.5\%$ cache pollution for reactive control loops querying only bounding discs.
  - Zero heap allocations in steady-state cluster processing.
  - Full microarchitectural parity testing between scalar reference and RVV SIMD.
- **Negative**:
  - Upstream orchestrators must invoke both extractors if both representations are required for telemetry.

