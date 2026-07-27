// caravan_radius_search.cpp
//
// Caravan Query-Pack Radius Search
// ---------------------------------
// Core idea: instead of scanning the cloud once per query (O(N*Q) cloud reads),
// we scan the cloud ONCE and compare each point against a full tile of queries
// simultaneously using RVV vector registers.
//
// RVV loop structure (m1 lmul, VLEN=256 → 8 float32 lanes):
//   for each point p in cloud:                        ← one read of px/py/pz
//       for each query tile of width VL:              ← VL queries at once
//           dx = qx[tile] - px   (vfsub scalar-broadcast)
//           d2 = dx²+dy²+dz²     (vfmul + vfmacc)
//           mask = d2 <= r²      (vmfle)
//           scatter point index into matching result lists
//
// Complexity: O(N × ⌈Q/VL⌉)  cloud reads, vs O(N×Q) in the old code.

#include "include/caravan_radius_search.h"
#include <algorithm>
#include <cmath>
#include <cstdint>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

// ─── helpers ──────────────────────────────────────────────────────────────────

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

// Process one tile of queries (width = vl) against a single point (px, py, pz).
// Returns a bitmask (as uint64_t) of which query lanes are within r_sq.
static void caravanTile(
    const float *qx, const float *qy, const float *qz, // query tile arrays
    std::size_t vl,                                      // tile width
    float px, float py, float pz,                        // the single point
    float r_sq,                                          // radius squared
    std::size_t point_index,                             // index to push
    std::vector<std::vector<int32_t>> &results,          // per-query result lists
    std::size_t q_offset                                 // which query in results[]
) {
    // Load tile of query coordinates
    vfloat32m1_t vqx = __riscv_vle32_v_f32m1(qx, vl);
    vfloat32m1_t vqy = __riscv_vle32_v_f32m1(qy, vl);
    vfloat32m1_t vqz = __riscv_vle32_v_f32m1(qz, vl);

    // dx = qx - px, dy = qy - py, dz = qz - pz
    vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vqx, px, vl);
    vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vqy, py, vl);
    vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vqz, pz, vl);

    // d2 = dx² + dy² + dz²
    vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, vl);
    d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, vl);

    // mask lanes where d2 <= r_sq
    vbool32_t mask = __riscv_vmfle_vf_f32m1_b32(d2, r_sq, vl);

    // Scatter: for each set lane, push point_index into results[q_offset + lane]
    // We materialise the mask as a packed uint via vfirst / viota or scalar read.
    // Simplest correct approach: store mask to a temp bool array.
    // (vcpop can count but can't tell us WHICH lanes; we use a byte-store trick.)
    uint8_t mask_bytes[64]; // max 64 lanes for e32 on any VLEN
    // Store mask as a packed bit-vector then test each bit
    __riscv_vsm_v_b32(mask_bytes, mask, vl);

    for (std::size_t lane = 0; lane < vl; ++lane) {
        // mask_bytes stores 1 bit per lane; byte index = lane/8, bit = lane%8
        if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
            results[q_offset + lane].push_back(static_cast<int32_t>(point_index));
        }
    }
}

#endif // RVV

// ─── batchRadiusSearch (core) ─────────────────────────────────────────────────

void CaravanRadiusSearch::batchRadiusSearch(
    const PointCloudSoA &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
    if (!pointCloud_ || pointCloud_->empty() || queries.empty()) {
        results.clear();
        return;
    }

    const std::size_t num_points  = pointCloud_->size();
    const std::size_t num_queries = queries.size();
    results.assign(num_queries, {});

    const float r_sq = radius * radius;

    const float *const px = pointCloud_->xData();
    const float *const py = pointCloud_->yData();
    const float *const pz = pointCloud_->zData();

    const float *const qx = queries.xData();
    const float *const qy = queries.yData();
    const float *const qz = queries.zData();

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

    // ── True Caravan: one pass over cloud, tile of queries per step ────────────
    for (std::size_t i = 0; i < num_points; ++i) {
        const float cx = px[i];
        const float cy = py[i];
        const float cz = pz[i];

        std::size_t q = 0;
        while (q < num_queries) {
            // vl = how many queries fit in one vector register this iteration
            std::size_t vl = __riscv_vsetvl_e32m1(num_queries - q);
            caravanTile(qx + q, qy + q, qz + q, vl,
                        cx, cy, cz, r_sq,
                        i, results, q);
            q += vl;
        }
    }

#else

    // ── Scalar fallback: per-query linear scan ─────────────────────────────────
    for (std::size_t q = 0; q < num_queries; ++q) {
        const float qcx = qx[q], qcy = qy[q], qcz = qz[q];
        for (std::size_t i = 0; i < num_points; ++i) {
            const float dx = px[i] - qcx;
            const float dy = py[i] - qcy;
            const float dz = pz[i] - qcz;
            if (dx*dx + dy*dy + dz*dz <= r_sq) {
                results[q].push_back(static_cast<int32_t>(i));
            }
        }
    }

#endif
}

// ─── batchRadiusSearch (ptr overload) ─────────────────────────────────────────

void CaravanRadiusSearch::batchRadiusSearch(
    const PointCloudSoAPtr &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
    if (!queries) { results.clear(); return; }
    batchRadiusSearch(*queries, radius, results);
}

// ─── single-point radiusSearch ────────────────────────────────────────────────

void CaravanRadiusSearch::radiusSearch(
    const PointXYZ &query,
    float radius,
    std::vector<int32_t> &indices
) const {
    PointCloudSoA q_cloud;
    q_cloud.push_back(query);
    std::vector<std::vector<int32_t>> batch_res;
    batchRadiusSearch(q_cloud, radius, batch_res);
    if (!batch_res.empty()) indices = std::move(batch_res[0]);
    else indices.clear();
}

// ─── NeighborSearch base interface (by index) ─────────────────────────────────

std::size_t CaravanRadiusSearch::radiusSearch(
    int queryPointIndex,
    std::vector<int> &resultIndices,
    std::vector<float> *resultDistances,
    int maxResults
) const {
    resultIndices.clear();
    if (resultDistances) resultDistances->clear();
    if (!pointCloud_ || queryPointIndex < 0 ||
        static_cast<std::size_t>(queryPointIndex) >= pointCloud_->size()) {
        return 0;
    }

    const PointXYZ q = queryPoint(queryPointIndex);
    const float r    = (searchRadius_ > 0.0f) ? searchRadius_ : 0.0f;
    const float r2   = r * r;

    const std::size_t num_points = pointCloud_->size();
    const float *const px = pointCloud_->xData();
    const float *const py = pointCloud_->yData();
    const float *const pz = pointCloud_->zData();

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
    std::size_t i = 0;
    while (i < num_points) {
        std::size_t vl = __riscv_vsetvl_e32m8(num_points - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(px + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(py + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz + i, vl);
        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, q.x, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, q.y, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, q.z, vl);
        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);
        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);

        uint8_t mask_bytes[64];
        __riscv_vsm_v_b4(mask_bytes, mask, vl);
        for (std::size_t lane = 0; lane < vl; ++lane) {
            if ((mask_bytes[lane >> 3] >> (lane & 7)) & 1u) {
                resultIndices.push_back(static_cast<int>(i + lane));
                if (resultDistances) {
                    // recompute scalar distance for output
                    const float ddx = px[i+lane] - q.x;
                    const float ddy = py[i+lane] - q.y;
                    const float ddz = pz[i+lane] - q.z;
                    resultDistances->push_back(ddx*ddx + ddy*ddy + ddz*ddz);
                }
                if (maxResults > 0 &&
                    resultIndices.size() >= static_cast<std::size_t>(maxResults)) {
                    return resultIndices.size();
                }
            }
        }
        i += vl;
    }
#else
    for (std::size_t i = 0; i < num_points; ++i) {
        const float dx = px[i] - q.x;
        const float dy = py[i] - q.y;
        const float dz = pz[i] - q.z;
        if (dx*dx + dy*dy + dz*dz <= r2) {
            resultIndices.push_back(static_cast<int>(i));
            if (resultDistances) resultDistances->push_back(dx*dx + dy*dy + dz*dz);
            if (maxResults > 0 &&
                resultIndices.size() >= static_cast<std::size_t>(maxResults)) break;
        }
    }
#endif

    return resultIndices.size();
}

} // namespace rvv_pcl
