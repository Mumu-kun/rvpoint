# Pipeline 3D Ultra: Correctness, Hardening, and Deployment Plan

**Date:** 2026-08-20  
**Target:** `src/tools/pipeline_3d_ultra.cpp` and its loader/downsampling dependencies  
**Status:** Proposed implementation plan  
**Scope:** Current worktree version, including the new default voxel-ROR path

## 1. Objective

Make the pipeline deterministic, bounded, diagnosable, and geometrically correct before it is used for robotics or autonomous-perception workloads. The implementation must fail closed on invalid input and uncertain perception results rather than returning successful empty or stale output.

The work should be implemented in dependency order. Do not optimize the existing kernels further until the scalar/reference behavior is defined and covered by tests.

## 2. Priority Summary

| Priority | Area | Main outcome |
| --- | --- | --- |
| P0 | Clustering, filtering, failure signaling | No silent loss of valid geometry or silent success on failed perception |
| P0 | Input and parameter validation | No undefined behavior or unbounded allocation from malformed data |
| P0 | Bounded data structures | No infinite probing or unbounded per-frame latency |
| P1 | RANSAC and numerical robustness | Stable model selection with explicit confidence and quality metrics |
| P1 | Output publication and observability | Consumers never see partial or stale results as current data |
| P2 | Performance and RVV optimization | Optimize only after reference equivalence is established |
| P2 | Robotics deployment integration | Add frame, timing, health, and fail-safe contracts outside this utility |

## 3. Implementation Work Packages

### WP1: Establish a reference contract

Before changing optimized code:

- Define exact semantics for ROR and SOR:
  - Whether the query point counts as a neighbor.
  - Whether `min_neighbors` includes or excludes the query point.
  - Whether SOR uses the nearest `k` points or all points in the radius.
- Define coordinate units, coordinate frame, valid finite range, and expected ground-plane orientation.
- Define result states: `success`, `no_ground_plane`, `invalid_input`, `resource_limit`, and `output_failure`.
- Extract filtering, RANSAC, clustering, and output publication behind testable functions or small modules.
- Keep the existing executable as a compatibility wrapper during the transition.

### WP2: Fix clustering completeness

Replace the current half-space plus point-index condition with one of these correct strategies:

1. Visit all 27 neighboring cells and retain `curr > i`; or
2. Visit only the 14 forward cells and do not apply `curr > i`, with duplicate suppression based on cell/index ordering.

The first option is simpler and should be the initial correctness implementation. For the performance implementation, prefer the 14-cell traversal with a deterministic pair-owner rule based on cell ordering and point ordering; do not use point-index filtering alone. Use `size_t` for point indices, while enforcing an explicit maximum if the union-find representation remains 32-bit.

Add tests for:

- Two points in adjacent cells where the lower point index is in the higher cell.
- Points on every cell-boundary direction.
- Same-cell and diagonal-cell neighbors.
- Input-order permutation invariance.
- A chain of points that should form one connected component.

### WP3: Make spatial indexing bounded and safe

Replace fixed open-addressing tables with a bounded implementation:

- Track occupied entries and stop probing after capacity.
- Return a typed `capacity_exceeded` error instead of looping.
- Size capacity from the expected number of occupied cells with a load-factor limit, subject to a hard memory budget.
- Use a collision-safe key type, preferably a checked 3D integer-cell tuple or 64-bit packed/hash key.
- Use `size_t` or checked 64-bit arithmetic for voxel linearization.
- Reject dimensions, products, and point counts that exceed configured limits.
- Ensure lookup of an absent key terminates even when the table is full.

The same validation must be applied to the voxel downsampler, not only the local grid.

### WP4: Validate input and CLI parameters

Add a single validation layer before allocating or processing data:

- Require finite coordinates and reject or explicitly filter invalid points.
- Require finite, positive `leaf_size`, ROR radius, SOR radius, and cluster tolerance.
- Require positive neighbor counts and iteration counts.
- Require `0 < min_cluster_size <= max_cluster_size`.
- Reject extra positional arguments and unknown options.
- Catch `stof`/`stoi` exceptions and report the offending option.
- Enforce maximum points, maximum spatial extent, maximum compressed payload, and maximum output size.
- Check all integer multiplications and float-to-integer conversions before execution.
- Treat an empty cloud as a defined result, not as an accidental loader failure.

### WP5: Correct ROR and SOR behavior

For ROR:

- Use the spatial grid only as a candidate generator.
- Compute and check actual squared Euclidean distance for every candidate.
- Define self-neighbor behavior explicitly and implement it consistently.
- Reject or report a radius larger than the grid cell size unless the candidate search range is expanded correctly.

For SOR:

- Do not assume hash-chain order is distance order.
- Either collect, distance-select, and sort the nearest neighbors, or use a bounded selection algorithm for the nearest `k` values.
- Exclude the query point by index rather than skipping an arbitrary `d2[0]` entry.
- Use the configured `normal_k`; do not hard-code 16.
- If normals are not consumed or exported, remove their computation from this executable. Otherwise, export them with validity and confidence fields.

Add equivalence tests against a simple brute-force reference implementation.

### WP6: Harden RANSAC

- Replace `std::rand` with a local, explicit PRNG and configurable seed.
- Sample from the full cloud without modulo bias.
- Randomize or stratify the pre-check sample instead of always using the first 512 points.
- Make the normal prior configurable and express it in the sensor/world frame.
- Validate the distance threshold and enforce a minimum inlier ratio.
- Use stable arithmetic for plane fitting; reject non-finite or poorly conditioned models.
- Return the model, inlier count, inlier ratio, residual statistics, and confidence.
- Return an explicit failure status when no model meets quality criteria.
- Ensure dynamic iteration count is clamped to valid bounds.

Test horizontal, tilted, vertical, absent, multiple, degenerate, highly noisy, and ordered clouds.

### WP7: Numerical robustness

- Use checked finite predicates at every public processing boundary.
- Use double precision for bounding-box products, covariance accumulation, and RANSAC residual calculations where practical.
- Guard squared-distance and norm calculations against overflow.
- Normalize or translate coordinates before covariance and plane fitting when coordinate magnitudes are large.
- Define tolerances relative to physical units and document them.
- Validate normal magnitude and orientation; never use a fabricated normal as if it were measured.
- Ensure JSON serialization cannot emit non-standard `nan` or `inf` values.

### WP8: Resource, latency, and concurrency controls

- Add hard limits for points, cells, clusters, memory, and processing time.
- Add cancellation/deadline checks between pipeline stages and inside long loops.
- Avoid retaining raw AoS, multiple SoA copies, filtered copies, and output copies simultaneously where possible.
- Reuse buffers with explicit capacity limits.
- Make RNG and scratch storage local to the invocation; do not use process-global mutable state.
- Document single-thread assumptions and make concurrent invocation safe.
- Report actual worst-case and percentile latency, not only average stage time.

### WP9: Output integrity and observability

- Make save functions return success/error status and propagate failures.
- Write each run into a unique temporary directory, fsync/close successfully, then atomically publish the completed result directory or manifest.
- Include input identity, timestamp/frame ID, configuration, model quality, result status, and point counts in the manifest.
- Never reuse stale artifacts after a failed stage.
- Make `--no-write` disable all disk output, including metrics, or provide a separate explicit metrics destination.
- Fix stage timing labels so work is timed where it actually occurs.
- Emit structured diagnostics suitable for a supervisor, not only human-readable stdout.

### WP10: RVV portability and validation

- Compile RVV code only under `__riscv_vector`, not merely `__riscv`.
- Remove the unconditional RVV header requirement from shared interfaces if scalar builds are expected, or mark this target explicitly RVV-only.
- Size mask-storage buffers from the active VLEN or avoid storing masks to memory.
- Add scalar and RVV reference-comparison tests for filtering, voxelization, RANSAC distances, and extraction.
- Run tests across the supported VLEN configurations, not only QEMU `vlen=128`.
- Keep optimized kernels behind the same validated semantics as the scalar reference.

### WP11: Complete parser, serialization, and invocation edge cases

Explicitly cover the remaining boundary cases that are easy to miss in a general validation layer:

- Validate PCD `FIELDS`, `SIZE`, `TYPE`, and `COUNT` lengths and values together.
- Correctly handle or reject multi-component fields in ASCII PCD data; do not assume one scalar per field when `COUNT > 1`.
- Check every `SIZE * COUNT * POINTS` and field-offset multiplication before allocation or pointer arithmetic.
- Reject unsupported numeric field types and document the little-endian binary assumption, or add endian conversion.
- Verify that the declared point count matches the payload policy for ASCII, binary, and compressed files.
- Bound and validate compressed and decompressed sizes before allocation and decompression.
- Prevent `size_t` to `int` narrowing in loader return values and downstream point counts.
- Reject trailing positional arguments and distinguish option typos from input paths.
- Make `--no-write` suppress metrics and every other file operation, or require an explicit metrics path.
- Escape or structurally serialize all metadata; never emit invalid JSON for non-finite values.
- Define output-path policy for existing directories, symlinks, permissions, and stale artifacts.
- Make repeated and concurrent invocations independent: no global RNG state, shared scratch buffers, or output-name collisions.
- Verify that all configured values are represented in metrics, including ROR/SOR mode, radii, neighbor counts, thresholds, seed, frame identity, and status.

## 4. Suggested Delivery Sequence

1. Add result/status types, validation, and brute-force reference tests.
2. Fix clustering completeness and add permutation/boundary tests.
3. Replace or harden hash tables and add saturation/resource tests.
4. Correct ROR distance semantics and SOR nearest-neighbor selection.
5. Harden RANSAC and add model-quality failure handling.
6. Add checked output publication and structured metrics.
7. Reduce memory copies and add deadline/resource controls.
8. Reintroduce or tune RVV optimizations using differential tests.
9. Add robotics integration contracts: frame/timestamp validation, watchdog, health state, and fail-safe consumer behavior.

Do not reorder steps 1 through 5 for performance work.

## 5. Verification Matrix

### Unit tests

- Invalid, NaN, infinity, extreme, and empty point clouds.
- Zero, negative, NaN, and infinite CLI parameters.
- Hash-table saturation and absent-key lookup at capacity.
- Voxel-key overflow and checked dimension products.
- ROR against brute-force radius search.
- SOR against sorted nearest-neighbor reference.
- Clustering against brute-force graph connectivity and shuffled input order.
- Plane fitting for degenerate and tilted inputs.
- RVV versus scalar output equivalence.

### Integration tests

- Missing input, truncated PCD, malformed header, huge declared point count, and corrupt compressed payload.
- Output directory permission failure, full disk simulation, interrupted write, and rerun into an existing directory.
- No-plane and low-quality-plane behavior.
- `--no-write` behavior and metrics consistency.
- Deadline/cancellation behavior.
- Multiple sequential frames with changing pose and timestamps.

### Performance tests

- Normal, sparse, dense, adversarial-collision, and widely spread clouds.
- Peak RSS, allocation count, probe length, and processing deadline.
- Multiple RVV VLEN settings and scalar reference.
- Worst-case rather than only median latency.

### Fault-injection tests

- Allocation failure and memory-budget exhaustion.
- Hash-table capacity exhaustion and pathological collision patterns.
- Read-only, full, disconnected, and symlinked output paths.
- Short reads, truncated compressed streams, malformed LZF back-references, and extra payload bytes.
- Non-finite coordinates, extreme magnitudes, duplicate points, all-identical points, and coordinate-boundary values.
- RANSAC with fewer than three points, all-collinear points, no acceptable plane, multiple competing planes, and invalid thresholds.
- Empty filtered clouds, all-points-filtered clouds, zero clusters, more clusters than the color table policy allows, and clusters at min/max size boundaries.
- Cancellation or deadline expiry during every long-running stage.

## 6. Solution Accuracy and Performance Review

The current QEMU RVV baseline for `0000000045.pcd` is approximately 435 ms total with `--no-write`: load 80 ms, downsampling 69 ms, default voxel-ROR 60 ms, RANSAC 120 ms, and clustering 69 ms. These numbers are a reference for regression tracking, not hardware performance claims.

| Work package | Correct for the identified problem? | Expected performance impact |
| --- | --- | --- |
| WP2 clustering | Yes, provided pair ownership is corrected. All 27 cells with `curr > i` is correct but adds roughly 2x cell probes versus the current 14-cell loop. A correct 14-cell owner rule should preserve most throughput. | All-27 version likely regresses clustering; owner-based 14-cell version should be near-neutral. |
| WP3 spatial tables | Yes. Bounded probing and checked keys eliminate hangs and overflow. A lower load factor may use more memory but usually reduces probe cost. | Neutral to positive on valid inputs; bounded failure is the primary benefit. |
| WP4 validation | Yes. It directly prevents the reproduced `nan`, zero-radius, negative-iteration, and inverted-bound cases. | Negligible per-frame cost compared with loading and processing. |
| WP5 ROR/SOR | Yes. Exact ROR distance checks fix voxel false neighbors. A fixed-size nearest-`k` heap is preferable to sorting all candidates. | ROR adds distance arithmetic and may cost more than the current approximation; nearest-`k` selection can be faster than a full sort. Benchmark both. |
| WP6 RANSAC | Yes. Randomized sampling and explicit model quality address prefix bias and silent bad models. | Similar evaluation cost; random sampling adds small overhead. Double-precision fitting may cost more, so keep the hot RVV distance scan in float where numerically safe. |
| WP7 numerical hardening | Yes, but precision must be applied selectively. | Double precision in reductions/model fitting can regress latency; coordinate translation and narrow double-precision regions reduce that cost. |
| WP8 resource controls | Yes. Limits and deadlines prevent hangs and unbounded latency. | Checks at stage/chunk boundaries should be low cost; memory reuse can reduce peak RSS and allocation time. |
| WP9 output integrity | Yes. Atomic publication prevents stale or partial consumer results. | Extra filesystem work affects write-enabled runs, not `--no-write`; keep publication outside the perception deadline where system architecture permits. |
| WP10 RVV validation | Yes. It addresses build and VLEN portability, not algorithmic correctness by itself. | Scalar fallback may be slower; VLEN-aware code should be neutral on supported hardware. Differential tests add CI time only. |
| WP11 parser and invocation checks | Yes. It closes malformed-PCD, allocation, serialization, and repeated-invocation gaps. | Validation overhead is negligible; bounded decompression may reject workloads that previously consumed excessive memory. |

Performance claims must be verified with before/after measurements for peak RSS, total latency, each stage, probe length, candidate counts, and output time. No optimization is accepted based only on an average-speed result if worst-case latency or correctness regresses.

## 7. Acceptance Gates

The pipeline is not deployment-ready until all of the following are true:

- No unbounded loops or unchecked allocation paths remain for externally supplied data.
- Scalar reference and RVV implementation agree within documented tolerances.
- Clustering is invariant to input permutation.
- ROR and SOR pass brute-force equivalence tests.
- Invalid input and no-plane cases produce explicit non-success statuses.
- Output publication is atomic and cannot expose stale or partial results.
- Worst-case processing time and memory stay within the system budget.
- A supervisor can distinguish valid perception, degraded perception, and failed perception.
- Robotics/system-level review confirms frame, timestamp, watchdog, and fail-safe behavior.

## 8. Files Likely to Change

- `src/tools/pipeline_3d_ultra.cpp`
- `src/include/simple_pcd_loader.h`
- `src/voxel_grid_downsamp.cpp`
- `src/include/rvv_pcl.h`
- `tests/unit/` for reference and adversarial cases
- `tests/integration/` for I/O, failure, and multi-frame behavior
- `docs/` for configuration, result status, and deployment contracts

Existing unrelated worktree changes must be preserved while implementing this plan.
