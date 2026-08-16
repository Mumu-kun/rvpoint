# RVPoint Pipeline Workflow

## HIGHER LEVEL OVERVIEW

RVPoint is a point cloud processing library designed to maximize throughput on RISC-V hardware using the RISC-V Vector Extension (RVV). The overall pipeline is a sequence of point cloud processing stages that take a raw point cloud input and produce filtered, segmented, and fitted output data.

### Core Pipeline Stages

1. **Input Loading**
   - Read a PCD file into memory.
   - Convert data into the project’s internal `PointCloudSoA` representation when necessary.

2. **Voxel Grid Downsampling**
   - Reduce point density using a uniform 3D voxel grid.
   - The pipeline supports both scalar and RVV-optimized implementations.

3. **Spatial Index Construction**
   - Build a neighbor-search structure for the downsampled cloud.
   - Available structures include `Octree`, `PointerOctree`, and `SpatialHash`.

4. **Statistical Outlier Removal (SOR)**
   - Remove noisy points based on local mean distances to neighbors.
   - This is the most expensive stage in the current pipeline when brute-force methods are used.

5. **Filtered Cloud Re-Indexing**
   - Rebuild the search index on the inlier cloud after SOR.
   - This ensures subsequent stages have an up-to-date spatial structure.

6. **Normal Estimation**
   - Estimate surface normals using local neighbor covariance analysis.
   - RVV-optimized normal estimation is available around octree or spatial hash searches.

7. **Plane Fitting with RANSAC**
   - Fit dominant planes to the cloud using random sample consensus.
   - RVV acceleration is used for inlier counting and distance evaluation.

8. **Euclidean Clustering**
   - Extract clusters using BFS-based radius connectivity.
   - The clustering stage can use pure RVV point scanning or neighbor-search structures.

9. **Output Generation**
   - Save stage outputs and final clusters as PCD files.
   - Optionally export JSON metrics and stage timings.

### Pipeline Purpose

The pipeline is built for:
- performance comparison between scalar and RVV paths,
- rapid experimentation with spatial indexes,
- page-by-page profiling of pipeline stages,
- validating real point cloud workloads on RISC-V emulation and hardware.

## DETAILED OVERVIEW

### Data Representations and Why They Matter

#### `PointXYZ`
- Simple 3D point type.
- Used mainly in scalar code and for final AoS output.

#### `PointCloudSoA`
- Stores `x`, `y`, and `z` arrays separately.
- This enables unit-stride vector loads with RVV.
- Core RVV kernels assume this layout for maximum throughput.

### Stage 1: Input Loading

- The pipeline accepts a PCD file path.
- The file is read and converted to the project’s internal point cloud types.
- Input validation and path resolution are handled before processing begins.

### Stage 2: Voxel Grid Downsampling

#### Scalar implementation
- Groups points into voxels using `std::map` keyed by voxel coordinates.
- Computes centroid of each voxel by summing coordinates.

#### RVV innovation
- `voxel_grid_downsamp_rvv_v2` introduces a fully vectorized, sort-based algorithm.
- Innovations include:
  - RVV bounding box computation with vector reductions.
  - Vectorized voxel-key generation using RVV floor-based integer conversion.
  - Sort-based grouping to avoid expensive `std::map` operations.
  - Vectorized centroid reduction with indexed gathers and reductions.

#### Why this matters
- This stage reduces input size for later expensive operations.
- A faster downsample stage means less data enters SOR, normal estimation, and clustering.

### Stage 3: Spatial Index Construction

The pipeline can choose between multiple index structures. Each has a different performance tradeoff.

#### `Octree`
- Recursive space partitioning into up to 8 children.
- Good for moderately sized clouds when queries are distributed.

#### `SpatialHash`
- Partitions space into uniform cells.
- Ideal for uniform point distributions and fixed-radius queries.
- Innovation: RVV-accelerated hash cell construction and RVV-enabled radius query filtering.

#### `PointerOctree`
- Stores contiguous leaf coordinate arrays inside leaf nodes.
- Reduces indirection during radius search and SOR.
- Innovation: combines tree pruning with contiguous leaf arrays for efficient RVV leaf queries.

### Stage 4: Statistical Outlier Removal (SOR)

#### Scalar baseline
- Brute-force mean K-NN distance search using full pairwise distance computation.
- Complexity: $O(N^2)$ in the number of points.

#### RVV optimized path
- Uses a vectorized distance kernel for the distance matrix.
- Maintains priority-queue based K-selection to reduce sorting cost.
- Compresses filtered output using RVV masks and stores results directly into AoS output.

#### Index-accelerated innovations
- `sor_octree`, `sor_spatial_hash`, `sor_pointer_octree` use pre-built indexes to avoid brute-force nearest neighbor search.
- `PointerOctree`-based SOR is the most effective ablation in the repository, delivering large speedups relative to naive $O(N^2)$.

### Stage 5: Rebuild Search Index

- After SOR, the inlier cloud is smaller.
- The pipeline rebuilds the chosen search structure on the filtered point cloud.
- This enables faster normal estimation and clustering on cleaner data.

### Stage 6: Normal Estimation

#### Scalar implementation
- Brute-force neighbor search
- Computes local centroid and covariance matrix
- Solves for smallest eigenvector via Jacobi iteration
- Flips normal toward the viewpoint

#### RVV implementation
- Uses RVV-friendly octree or spatial hash queries to find neighbors faster.
- Computes covariance per neighbor set using a mixed scalar/RVV approach.
- Innovation: pre-built spatial index overloads allow normal estimation to reuse tree construction work.

### Stage 7: RANSAC Plane Fitting

#### Scalar baseline
- Repeatedly select 3 random points
- Fit a candidate plane
- Count inliers with a scalar loop

#### RVV innovation
- `ransac_plane_rvv` vectorizes the inlier counting step.
- Performs batch distance evaluation with RVV: `vfmacc`, mask compares, and popcount.
- Supports both direct inlier counting and efficient extraction of inliers/outliers.
- Innovation: the RVV path avoids scalar `abs()` per point by using twin bounds checks and mask logic.

### Stage 8: Euclidean Clustering

#### Core algorithm
- BFS flood-fill growth from unvisited seed points
- Queries all unvisited points within a tolerance radius
- Marks points visited and builds clusters

#### RVV acceleration
- The inner radius query uses RVV vector scans to compute squared distances for many candidates at once.
- A bitmask is materialized using RVV mask store instructions, then a scalar scan extracts matching indices.
- This pattern preserves correctness while giving a large per-query speedup.

#### Optionally accelerated by neighbor search
- If a searcher is configured, clustering can use `Octree` radius queries instead of a global scan.
- This is useful when the point cloud is large and spatial locality is strong.

### Stage 9: Output Generation

- Final clusters and intermediate stage outputs are written as PCD files.
- The pipeline may also emit JSON metrics and a timing breakdown.
- Output files are organized by stage to support debugging and visualization.

## INNOVATIONS AND KEY CONTRIBUTIONS

The codebase contains several practical innovations designed for RISC-V vector workloads:

1. **SoA-first API design**
   - The project favors `PointCloudSoA` for RVV paths.
   - This is a core design decision that enables unit-stride vector loads.

2. **RVV sort-based voxel downsampling**
   - Replaces `std::map` grouping with a fully vectorized sorting and reduction pipeline.
   - This reduces overhead and improves scalability.

3. **Index-accelerated SOR variants**
   - Adds `sor_octree`, `sor_spatial_hash`, and `sor_pointer_octree`.
   - These variants dramatically reduce the runtime of one of the pipeline’s heaviest stages.

4. **PointerOctree leaf vectorization**
   - Stores contiguous `leaf_x`, `leaf_y`, `leaf_z` arrays inside each leaf node.
   - Reduces pointer chasing during RVV leaf evaluation.

5. **RVV RANSAC inlier counting**
   - Uses vectorized distance evaluation, masked comparisons, and lane counts.
   - This is a strong example of applying RVV to a randomized consensus algorithm.

6. **Zero-overhead profiling framework**
   - `src/include/profiler.h` provides compile-time switchable instrumentation.
   - When disabled, profiling code compiles away entirely.

7. **Pipeline stage timing and JSON export**
   - `src/tools/pipeline_export.cpp` supports progress printing, stage timing, and structured JSON output.
   - This enables detailed analysis without modifying the core algorithm.

## HOW TO USE THIS WORKFLOW

The pipeline is intended as both:
- an experimental framework for performance evaluation,
- a practical processing flow for point cloud filtering and segmentation.

A typical research workflow is:

1. choose the input PCD file,
2. select the target index structure,
3. choose between scalar or RVV implementations,
4. run the pipeline with profiling enabled,
5. compare stage timings,
6. swap in the faster algorithms discovered through ablation.

## RECOMMENDED READING ORDER

1. `src/include/rvv_pcl.h`
2. `src/voxel_grid_downsamp.cpp`
3. `src/neighbor_search/octree.cpp`
4. `src/neighbor_search/spatial_hashing.cpp`
5. `src/pointer_octree/pointer_octree.h`
6. `src/statistical_outlier_removal.cpp`
7. `src/ransac_plane.cpp`
8. `src/euclidean_clustering.cpp`
9. `src/tools/pipeline_export.cpp`

## NOTES

- The pipeline is not a single monolithic algorithm; it is designed as a stage-by-stage research workflow.
- The goal is to make it easy to swap implementations and measure the effect of each change.
- The innovations are centered on reducing the cost of neighbor search and applying RVV to data-parallel math.
