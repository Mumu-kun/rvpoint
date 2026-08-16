# RVPoint — Tracking Mode Pipeline (RVV-First Design)

This document covers **only Tracking Mode** — the fast, per-frame path that runs on every frame *except* keyframes. The goal of this document is to make the pipeline so clear that each stage's job, justification, and parallelization potential is obvious at a glance.

---

## Part 1: The Pipeline, Step by Step

Think of Tracking Mode as a relay race with 8 stages. A new raw frame comes in one end, and out the other end comes: an updated camera/sensor pose, confirmed planes, confirmed clusters, and a small leftover pile of "new/unexplained" points that get handed to the full (slow) pipeline.

```
RAW FRAME
   │
   ▼
[STAGE 1] Voxel Downsampling
   │
   ▼
[STAGE 2] Correspondence Search  ─┐
   │                              │  (this loop, called
   ▼                              │   "ICP iteration",
[STAGE 3] Residual + Jacobian     │   repeats a few times)
   │                              │
   ▼                              │
[STAGE 4] Reduction to 6x6 System │
   │                              │
   ▼                              │
[STAGE 5] Solve for Transform  ───┘
   │
   ▼
[STAGE 6] Propagate (apply transform to old planes/clusters/normals)
   │
   ▼
[STAGE 7] Verify (check which propagated guesses are still correct)
   │
   ▼
[STAGE 8] Split → Confirmed (done) vs Residual (send to full pipeline)
   │
   ▼
[STAGE 9] Update Spatial Index (add/remove points, don't rebuild)
```

That's the entire tracking-mode pipeline. Nine stages. Let's go through each one.

---

## Part 2: What Each Stage Does, Why, and How It Parallelizes

For every stage: **What it does** (plain language) → **Why it exists** (justification) → **Parallelization** (how RVV chews through it).

---

### STAGE 1 — Voxel Downsampling

**What it does:** Takes the raw frame (could be hundreds of thousands of points) and shrinks it to a manageable, evenly-spaced set of points by snapping points into a 3D grid and keeping one representative point per grid cell.

**Why it exists:** Raw sensor data is way denser than you need, and it's denser in some areas than others (close objects have way more points than far ones). Downsampling makes point density uniform and makes every later stage faster because there are simply fewer points to touch.

**Parallelization — 🟢 Fully parallel, no dependencies between points:**
> **THE VOXEL DOWNSAMPLING STAGE CAN BE PARALLELIZED LIKE THIS:** every single point independently computes "which grid cell do I belong to?" — this is pure per-point math (divide coordinates by voxel size, floor it). RVV can process a whole vector register's worth of points (e.g. 4, 8, 16 points depending on VLEN) computing their voxel-cell IDs in one instruction, with zero communication between points. This is the easiest kind of parallelism there is — you already have this implemented in RVPoint (`voxel_grid_downsamp_rvv_v2`), so tracking mode reuses it unchanged.

---

### STAGE 2 — Correspondence Search

**What it does:** For every point in the new downsampled frame, finds its "twin" — the nearest matching point in last frame's reference cloud.

**Why it exists:** To figure out how the camera/sensor moved, you first need to know "this point I see now used to be *that* point I saw a moment ago." Without correspondences, there's nothing to compare positions against, and no way to compute a transform.

**Parallelization — 🟢 Parallel across query points, index lookups happen independently:**
> **THE CORRESPONDENCE SEARCH STAGE CAN BE PARALLELIZED LIKE THIS:** each query point's "find my nearest neighbor" search is completely independent of every other query point's search — point #1's search doesn't need to wait for or share anything with point #2's search. RVV can batch many query points together and evaluate their distance-to-candidate calculations in vector lanes simultaneously (this is the exact same distance-kernel pattern already used in your SOR stage — same math, different purpose). The spatial index traversal itself (walking the Octree/SpatialHash) has some inherent branchiness, but the distance computations at each step are vectorizable, and multiple independent queries can be batched to keep vector lanes full even when individual tree paths diverge.

---

### STAGE 3 — Residual + Jacobian Computation

**What it does:** For every matched pair (new point ↔ old point), computes two numbers: (a) how "wrong" the current guess is (the residual — basically "how far off is this point from the reference surface"), and (b) a small set of numbers describing how that wrongness would change if we nudged the rotation/translation slightly (the Jacobian — think of it as "sensitivity").

**Why it exists:** This is the raw ingredient the next stage needs to actually solve for the correct transform. Every matched pair "votes" on what transform would make it line up better; this stage computes each pair's vote.

**Parallelization — 🟢 Fully parallel, per-pair, identical math for every pair:**
> **THE RESIDUAL + JACOBIAN STAGE CAN BE PARALLELIZED LIKE THIS:** every matched point-pair computes its residual and Jacobian using the exact same formula, with zero dependency on any other pair's result. This is the textbook case for SIMD/RVV — same instruction, different data, across all pairs at once. This is directly analogous to your RANSAC inlier-counting kernel (same operation applied uniformly across many points) — same vectorization shape, new formula.

---

### STAGE 4 — Reduction to 6×6 System

**What it does:** Takes all the individual per-pair "votes" from Stage 3 and sums them up into one small 6×6 matrix and one 6-value vector (6 = 3 for rotation + 3 for translation).

**Why it exists:** You can't solve for one single "best" transform using thousands of separate individual votes directly — you need to combine ("accumulate") them into one compact system first, the same way you'd average many opinions into one final number before acting on it.

**Parallelization — 🟡 Partially parallel — this is a reduction, not a per-item operation:**
> **THE REDUCTION STAGE CAN BE PARALLELIZED LIKE THIS:** summing thousands of small matrices into one is done via a *parallel/tree reduction* — RVV can sum chunks of the data in parallel vector lanes first (e.g. sum every 8th element together across the whole set), then combine those partial sums down to one final result in a few more steps, instead of adding one-at-a-time in a simple loop. This is the same reduction pattern used any time you compute a sum/average across a large point set in the existing codebase — it's parallel, just not "embarrassingly" parallel like Stages 1–3 (there's a final combine step that's inherently sequential, but it's tiny compared to the per-pair work it's summarizing).

---

### STAGE 5 — Solve for Transform

**What it does:** Takes the one small 6×6 system from Stage 4 and solves it to get the actual answer: "here is the rotation and translation that best explains the motion between frames."

**Why it exists:** This is the actual output of registration — everything before this stage was just gathering evidence; this stage is where the evidence becomes an answer.

**Parallelization — 🔴 Not worth parallelizing — and that's fine:**
> **THE SOLVE STAGE STAYS SCALAR:** the system being solved is only 6×6 — tiny. There's no meaningful parallelism to extract from an operation this small, and trying to vectorize it would add complexity for no real speedup. This step runs on ordinary scalar CPU code. This is normal and expected — not every stage needs to be vectorized, and the FPGA/hardware registration papers researched earlier follow exactly this same division: vectorize the big repetitive stages (2, 3, 4), keep the tiny one-shot math (this stage) scalar.

*(Stages 2 → 5 repeat a few times — this loop is called "ICP iteration." Each individual iteration internally parallelizes exactly as described above; the iterations themselves happen one after another because each one refines the previous guess.)*

---

### STAGE 6 — Propagate

**What it does:** Takes the transform computed in Stage 5 and applies it to everything remembered from the last keyframe — the old plane equations, the old cluster center-points, the old point normals — producing an updated *guess* of where all of these things are now.

**Why it exists:** This is the entire point of doing registration in the first place — instead of re-detecting planes and clusters from scratch, you predict where they moved to and only check if the prediction was right (much cheaper than detecting from nothing).

**Parallelization — 🟢 Fully parallel — same transform applied to many independent items:**
> **THE PROPAGATE STAGE CAN BE PARALLELIZED LIKE THIS:** applying one transform to many planes, many cluster centroids, and many normals is the same operation (a matrix-vector multiply) repeated across independent items — RVV processes a batch of these transform applications simultaneously, since none of them affect each other.

---

### STAGE 7 — Verify

**What it does:** Checks whether each propagated guess from Stage 6 actually still matches the new frame's real points. For planes: count how many new points still lie close to the (moved) plane. For clusters: check how many new points land near the (moved) cluster centroid.

**Why it exists:** Registration gives you a *hypothesis*, not a guarantee. A plane might have gone out of view, an object might have moved on its own (not just the camera), or the ICP solve might have had some error. Verification is the safety net that prevents wrong guesses from silently propagating forward.

**Parallelization — 🟢 Fully parallel — per-point checks, independent of each other:**
> **THE VERIFY STAGE CAN BE PARALLELIZED LIKE THIS:** "is this point close to that plane?" and "is this point close to that cluster centroid?" are both simple per-point distance checks, identical in form to your existing RANSAC inlier-counting kernel — every point's check is independent, so RVV evaluates many points' pass/fail status simultaneously.

---

### STAGE 8 — Split (Confirmed vs Residual)

**What it does:** Divides all the new frame's points into two piles: "confirmed" (matched an existing plane/cluster successfully — done, no further work needed) and "residual" (didn't match anything — these are new objects, newly-revealed surfaces, or things that genuinely changed and need full, from-scratch detection).

**Why it exists:** This is what actually saves computation — the expensive full-detection stages (RANSAC, clustering, normal estimation) only need to run on the small "residual" pile instead of the entire frame.

**Parallelization — 🟡 Mostly parallel, with one small sequential step:**
> **THE SPLIT STAGE CAN BE PARALLELIZED LIKE THIS:** deciding "does this point belong to confirmed or residual?" is an independent per-point decision (a pass/fail flag), fully parallel across all points. Physically gathering all the "residual" points into one compact list afterward uses a "prefix sum" / compaction step, which has a small sequential dependency chain but is a well-known, still-substantially-parallelizable pattern (it's not a per-point bottleneck, just not 100% embarrassingly parallel like Stage 1–3).

---

### STAGE 9 — Update Spatial Index

**What it does:** Instead of throwing away and rebuilding the Octree/SpatialHash from zero every frame, it inserts the handful of new/changed points and removes points that disappeared (occluded, left the field of view).

**Why it exists:** Rebuilding a spatial index from scratch every single frame is wasteful when the structure barely changed from last frame — incremental updates are far cheaper.

**Parallelization — 🟡 Partially parallel:**
> **THE INDEX UPDATE STAGE CAN BE PARALLELIZED LIKE THIS:** computing *where* each new point should be inserted (which voxel/octree cell it belongs to) is independent per-point work and vectorizes the same way Stage 1 does. The actual insertion into the shared tree/hash structure has some inherent sequential/synchronization overhead (multiple points can't safely write to the exact same structure node at the exact same instant), but the "figure out where I go" computation — the expensive part — is fully parallel.

---

## Part 3: Quick-Reference Summary Table

| Stage | What | Parallelization |
|---|---|---|
| 1. Voxel Downsampling | Shrink raw frame to uniform density | 🟢 Fully parallel (reused as-is) |
| 2. Correspondence Search | Find "twin" point in last frame | 🟢 Parallel across query points |
| 3. Residual + Jacobian | Compute per-pair "vote" on the transform | 🟢 Fully parallel, per-pair |
| 4. Reduction to 6×6 | Sum all votes into one small system | 🟡 Parallel reduction (tree-sum) |
| 5. Solve for Transform | Solve the tiny 6×6 system | 🔴 Scalar — too small to vectorize |
| 6. Propagate | Apply transform to old planes/clusters/normals | 🟢 Fully parallel |
| 7. Verify | Check if propagated guesses are still right | 🟢 Fully parallel, per-point |
| 8. Split | Separate confirmed points from residual points | 🟡 Parallel flagging + compaction |
| 9. Update Spatial Index | Insert/remove points incrementally | 🟡 Parallel lookup, some serial insertion |

**The takeaway:** 6 of 9 stages are fully or mostly RVV-parallelizable using patterns you've already built (voxel keying, distance kernels, inlier counting, reductions). Only Stage 5 (the tiny 6×6 solve) is intentionally left scalar — and that's correct, not a compromise.

---

## Part 4: Agent Prompt (for Repo-Scanning Implementation)

Copy the block below into an agent session (e.g. Opus 4.6 in Antigravity) pointed at the RVPoint repository.

```
You are implementing Tracking Mode for RVPoint, a RISC-V Vector Extension (RVV)
point cloud processing library written in C++17.

STEP 1 — Repo scan (do this first, before writing any code):
Scan the full repository structure. Specifically identify:
- The existing RVV kernel patterns used in src/statistical_outlier_removal.cpp
  and src/ransac_plane.cpp (these contain the distance-computation and
  inlier-counting vectorization patterns you will reuse).
- The voxel keying implementation in src/voxel_grid_downsamp.cpp
  (voxel_grid_downsamp_rvv_v2), which you will reuse unchanged for
  downsampling in tracking mode.
- The spatial index APIs exposed by src/neighbor_search/ and
  src/pointer_octree/ (Octree, SpatialHash, PointerOctree) — identify
  whether they currently support incremental insert/remove, or only
  full construction, since tracking mode requires incremental updates.
- The public API surface in src/include/rvv_pcl.h, and the data structures
  PointXYZ and PointCloudSoA, to determine the right place and style for
  new registration-related types.
- The profiling/ablation macros in src/tools/, to understand how new
  pipeline stages should report timing so tracking mode fits the existing
  measurement framework.

STEP 2 — Report back before implementing:
Summarize what you found for each item above, and propose:
- Where a new `registration/` module should live and what its minimal
  public interface should look like, consistent with existing module
  conventions in this repo.
- Which existing RVV kernels can be directly reused vs. need a new
  (but pattern-consistent) kernel written for tracking mode.
- Any gaps in the current spatial index APIs that block incremental
  insert/remove, and what minimal API addition would resolve them.

STEP 3 — Implement Tracking Mode following this exact 9-stage pipeline
(do not reorder or merge stages):
1. Voxel Downsampling — reuse existing RVV kernel unchanged.
2. Correspondence Search — nearest-neighbor lookup per new-frame point
   against the stored reference cloud, using the existing spatial index.
   Vectorize the distance evaluation the same way SOR's distance kernel
   is vectorized.
3. Residual + Jacobian — compute per-matched-pair point-to-plane residual
   and Jacobian, fully vectorized per pair, following the RANSAC
   inlier-counting kernel's data-parallel structure.
4. Reduction to 6x6 system — accumulate per-pair contributions into one
   6x6 matrix and 6x1 vector via a parallel/tree reduction.
5. Solve for transform — small 6x6 linear solve; implement as plain
   scalar code, do not attempt to vectorize this step.
6. Propagate — apply the solved transform to the stored reference planes,
   cluster centroids, and point normals; vectorize as a batched
   matrix-vector transform.
7. Verify — check propagated planes/clusters against the new frame's
   actual points (distance-to-plane / distance-to-centroid checks),
   vectorized per-point, same pattern as RANSAC inlier counting.
8. Split — partition new-frame points into confirmed vs. residual sets;
   vectorize the per-point pass/fail flagging, followed by a compaction
   step to produce the residual point list.
9. Update spatial index — incrementally insert new/residual points and
   remove points no longer present, rather than rebuilding the index.
   Vectorize the "which cell does this point belong to" computation;
   the actual tree/hash mutation may remain scalar/sequential.

STEP 4 — Wire residual points from Stage 8 into the EXISTING full-pipeline
stages (RANSAC plane fitting, Euclidean clustering, normal estimation)
unmodified — tracking mode should call these existing functions on the
smaller residual point set, not reimplement them.

STEP 5 — Add profiling hooks for each of the 9 stages consistent with the
existing pipeline profiling/ablation tooling, so keyframe-vs-tracking-mode
cost comparisons are measurable.

Constraints:
- Target ISA remains rv64gcv; all new vectorized code must use
  <riscv_vector.h> intrinsics consistent with existing kernel style
  in this repo.
- Prefer Structure-of-Arrays (PointCloudSoA) for any new data touched
  by RVV kernels, consistent with existing project conventions.
- Do not modify the existing full-pipeline (keyframe mode) stages —
  tracking mode must call them, not replace them.
- After implementation, propose test cases: a synthetic frame-pair with
  a known applied transform, to validate registration correctness before
  relying on real captured data.
```

---

## One-Sentence Recap

Tracking mode is: **downsample → find matches → vote on the motion → sum the votes → solve for the motion (small, stays scalar) → predict → verify → split into "done" vs "needs full detection" → keep the index up to date** — and of those nine steps, six are naturally RVV-parallel using patterns RVPoint already has, one is intentionally scalar because it's too small to bother vectorizing, and two are "mostly parallel with one small sequential glue step."
