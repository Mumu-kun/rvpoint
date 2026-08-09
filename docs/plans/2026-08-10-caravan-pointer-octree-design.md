# Caravan-PointerOctree Hybrid: Architecture & Technical Design Document

**Date:** 2026-08-10  
**Feature:** Integrating Caravan Query-Pack Vectorization into `PointerOctree`  
**Target Codebase:** `src/neighbor_search/pointer_octree.cpp`, `src/pointer_octree/pointer_octree.h`  
**Status:** Approved Design Concept  

---

## 1. Executive Summary & Problem Statement

Currently in `rvpoint`:
- **`PointerOctree`**: Excellent single-query radius search (**0.45 ms** / **2.5x faster than Standard Octree**), but requires $Q$ independent tree traversals when executing batch queries ($Q$ queries = $Q$ tree traversals).
- **`CaravanRadiusSearch`**: Streams points across $VL$ query registers simultaneously ($8x-16x$ memory bandwidth reduction), but lacks spatial bounding-box pruning, leading to $O(N^2 / VL)$ complexity.

**The Solution — Caravan-PointerOctree Hybrid**:
Incorporate Caravan Query-Pack vectorization directly into `PointerOctree` leaf traversal. By tile-grouping $VL$ queries together, $VL$ queries share a **single combined octree traversal** using the query tile's 3D Bounding Box ($\text{TileAABB}$). When reaching leaf nodes, Caravan scalar-broadcast vector registers (`vfsub.vf` + `vfmacc.vv`) compare leaf points against all $VL$ query registers simultaneously.

---

## 2. Architectural Design & Vector Mechanics

### A. Query Tiling & Tile AABB Computation
For a cloud of $Q$ batch queries, partition queries into tiles of width $VL$ (matching RISC-V Vector register length, e.g. $VL=8, 16$):

$$\text{TileAABB}_{min} = \left[ \min_{lane}(qx) - R, \; \min_{lane}(qy) - R, \; \min_{lane}(qz) - R \right]$$
$$\text{TileAABB}_{max} = \left[ \max_{lane}(qx) + R, \; \max_{lane}(qy) + R, \; \max_{lane}(qz) + R \right]$$

### B. Octree Traversal with Shared Tile AABB
Traverse `PointerOctreeNode` tree using `boxOverlapsBox(node_box, TileAABB)`:
- If a sub-tree node does **not** overlap $\text{TileAABB}$, prune the entire sub-tree for all $VL$ queries in **one single test**.
- If a sub-tree node overlaps $\text{TileAABB}$, descend down to leaf nodes.

### C. Caravan Leaf Vector Kernel
When reaching a leaf node containing contiguous float arrays (`leaf_x, leaf_y, leaf_z` of size $N_{leaf}$):

```cpp
// RVV Caravan Leaf Kernel (vfloat32m1_t, VL lanes)
vfloat32m1_t vqx = __riscv_vle32_v_f32m1(tile_qx, vl);
vfloat32m1_t vqy = __riscv_vle32_v_f32m1(tile_qy, vl);
vfloat32m1_t vqz = __riscv_vle32_v_f32m1(tile_qz, vl);

for (size_t p = 0; p < leaf_node->indices.size(); ++p) {
    float px = leaf_node->leaf_x[p];
    float py = leaf_node->leaf_y[p];
    float pz = leaf_node->leaf_z[p];

    // dx = vqx - px (scalar broadcast)
    vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
    vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
    vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

    // d^2 = dx^2 + dy^2 + dz^2
    vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

    vbool32_t mask = __riscv_vmfle_vf_f32m1_b32(d2, r_sq, vl);
    ...
}
```

---

## 3. Key Synergy & Expected Performance Impact

| Performance Dimension | Standard PointerOctree | Pure Caravan Stream | Caravan-PointerOctree Hybrid |
| :--- | :--- | :--- | :--- |
| **Octree Tree Traversals** | $Q$ full traversals | **0** (No tree) | **$\lceil Q / VL \rceil$ tile traversals** ($8\times-16\times$ reduction!) |
| **Spatial Bounding Box Pruning** | Yes ($>95\%$ pruned) | No ($0\%$ pruned) | **Yes ($>95\%$ pruned)** |
| **Leaf SIMD Vectorization** | Single-query unit-stride | Tile-broadcast SIMD | **Tile-broadcast SIMD on Leaf Arrays** |
| **Projected Batch Query Speedup** | Baseline ($1.0\times$) | $0.2\times$ | **$3.5\times - 6.0\times$ faster than PointerOctree** ⚡⚡ |

---

## 4. API Extensions to `PointerOctree`

Add to `src/pointer_octree/pointer_octree.h`:

```cpp
/**
 * @brief Caravan Query-Pack Batch Radius Search integrated directly into PointerOctree.
 * Combines spatial octree tile pruning with RVV scalar-broadcast leaf registers.
 */
void radiusSearchCaravanBatch(const PointCloudSoA &queries, float radius,
                              std::vector<std::vector<int32_t>> &results) const;
```

---

## 5. Next Steps for Implementation

1. Create implementation task plan using `/plan`.
2. Implement `radiusSearchCaravanBatch` in `src/neighbor_search/pointer_octree.cpp`.
3. Add benchmark test case to `src/tools/neighbor_search_sor_bench.cpp`.
