// euclidean_clustering.cpp
//
// Euclidean Cluster Extraction – RVV-accelerated + scalar fallback
// ----------------------------------------------------------------
// This file implements EuclideanClustering declared in
// include/euclidean_clustering.h.
//
// Key design choices
// ──────────────────
// • The cloud is stored SoA (Structure-of-Arrays) as float* px/py/pz.
//   This layout is ideal for RVV: we can issue three contiguous unit-stride
//   loads (vle32) and process VLEN/32 points per vector instruction cycle.
//
// • Inner loop: radiusQueryUnvisited()
//   For each BFS seed we scan the full point array:
//
//   RVV path (m8 lmul, up to 8×VLEN/32 points per iteration):
//     1. vle32_v_f32m8  – load up to VL px values into a single vector reg
//     2. vfsub_vf_f32m8 – subtract seed.x from every lane (scalar broadcast)
//     3. vfmul / vfmacc – compute dx²+dy²+dz² in-register
//     4. vmfle_vf_f32m8_b4 – compare against tol_sq, produce bit-mask
//     5. vsm_v_b4       – store bit-mask to uint8_t buffer
//     6. Scalar loop over mask bits → push unvisited indices
//
//   Scalar fallback (no RVV):
//     Plain O(N) loop computing (dx²+dy²+dz²) per point.
//
// • BFS (breadth-first) expansion avoids the recursion/stack depth issues of
//   DFS and also gives predictable, cache-friendly queue growth behaviour.
//
// • visited[] is a std::vector<bool> (one-bit-per-entry on most STL
//   implementations) – memory-compact for large clouds and cheap to reset.
//
// Complexity: O(N × C_avg × N) in the worst case (each BFS step scans all N
// points), but the large VLEN and m8 lmul mean the per-point cost is a small
// fraction of a scalar loop.  A spatial index (octree / hash) would reduce
// the per-step cost but is not required for basic correctness.

#include "include/euclidean_clustering.h"
#include "pointer_octree/pointer_octree.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

// ─── Setters ─────────────────────────────────────────────────────────────────

void EuclideanClustering::setInputCloud(const PointCloudSoA &cloud) {
    cloud_ = &cloud;
}

void EuclideanClustering::setClusterTolerance(float tolerance) {
    clusterTolerance_ = tolerance;
}

void EuclideanClustering::setMinClusterSize(int min_size) {
    minClusterSize_ = min_size;
}

void EuclideanClustering::setMaxClusterSize(int max_size) {
    maxClusterSize_ = max_size;
}

void EuclideanClustering::setNeighborSearch(const Octree *search) {
    searcher_ = search;
    ptr_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const PointerOctree *search) {
    ptr_searcher_ = search;
    searcher_ = nullptr;
}

// ─── radiusQueryUnvisited ─────────────────────────────────────────────────────
//
// Find all cloud indices within tol_sq of (qx,qy,qz) that have not yet been
// assigned to a cluster (visited[i] == false).  Matched indices are appended
// to @p out.
//
// Both paths share the same signature; the compiler selects the correct one
// via the preprocessor guard.

void EuclideanClustering::radiusQueryUnvisited(
    float qx, float qy, float qz,
    float tol_sq,
    const std::vector<bool> &visited,
    std::vector<int>        &out
) const {
    if (!cloud_) return;

    const std::size_t  n  = cloud_->n;
    const float *const px = cloud_->x;
    const float *const py = cloud_->y;
    const float *const pz = cloud_->z;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

    // ── RVV path ─────────────────────────────────────────────────────────────
    // Use m8 LMUL so a single vsetvl grants the largest possible VL (up to
    // 8×VLEN/32 elements).  This mirrors the style used in the single-query
    // radiusSearch() in caravan_radius_search.cpp.

    std::size_t i = 0;
    while (i < n) {
        const std::size_t vl = __riscv_vsetvl_e32m8(n - i);

        // Load lane coordinates
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(px + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(py + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz + i, vl);

        // dx = px - qx  (scalar broadcast subtraction)
        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

        // d2 = dx² + dy² + dz²  (fused multiply-accumulate)
        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        // mask: d2 <= tol_sq
        // For m8 lmul, e32 element width → ratio = 4 → b4 mask type.
        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);

        // Materialise mask as packed bits into a byte buffer.
        // Each byte holds 8 lane bits; max 64 lanes for b4 at VLEN=256.
        // ceil(VL / 8) bytes are written.
        uint8_t mask_bytes[64]; // ≥ ceil(VLEN_max_lanes / 8) = 64 bytes
        __riscv_vsm_v_b4(mask_bytes, mask, vl);

        // Scalar scatter: test each bit, skip visited points
        for (std::size_t lane = 0; lane < vl; ++lane) {
            if ((mask_bytes[lane >> 3] >> (lane & 7u)) & 1u) {
                const int idx = static_cast<int>(i + lane);
                if (!visited[static_cast<std::size_t>(idx)]) {
                    out.push_back(idx);
                }
            }
        }

        i += vl;
    }

#else

    // ── Scalar fallback path ──────────────────────────────────────────────────
    for (std::size_t i = 0; i < n; ++i) {
        if (visited[i]) continue;
        const float dx = px[i] - qx;
        const float dy = py[i] - qy;
        const float dz = pz[i] - qz;
        if (dx * dx + dy * dy + dz * dz <= tol_sq) {
            out.push_back(static_cast<int>(i));
        }
    }

#endif // RVV
}

// ─── extract ─────────────────────────────────────────────────────────────────
//
// BFS flood-fill cluster extraction.
//
// For each unvisited seed we:
//   1. Start a new cluster.
//   2. Push the seed into the BFS queue.
//   3. While the queue is non-empty:
//      a. Pop the front index.
//      b. Find all unvisited neighbours within clusterTolerance_.
//      c. Mark each neighbour as visited, add it to the cluster and queue.
//   4. Accept the cluster if minClusterSize_ ≤ |cluster| ≤ maxClusterSize_.

std::vector<ClusterIndices> EuclideanClustering::extract() const {
    std::vector<ClusterIndices> result;

    if (!cloud_ || cloud_->n == 0) {
        return result;
    }

    const std::size_t  n      = cloud_->n;
    const float *const px     = cloud_->x;
    const float *const py     = cloud_->y;
    const float *const pz     = cloud_->z;
    const float        tol_sq = clusterTolerance_ * clusterTolerance_;

    // visited[i] == true  →  point i is already assigned to a cluster
    std::vector<bool> visited(n, false);

    // Temporary buffer reused across BFS steps to avoid repeated allocations
    std::vector<int> neighbors;
    neighbors.reserve(256);

    // BFS queue
    std::queue<int> bfs_queue;

    for (std::size_t seed = 0; seed < n; ++seed) {
        if (visited[seed]) continue;

        // ── New cluster ───────────────────────────────────────────────────────
        ClusterIndices cluster;
        cluster.indices.reserve(64);

        // Mark seed and enqueue
        visited[seed] = true;
        bfs_queue.push(static_cast<int>(seed));

        while (!bfs_queue.empty()) {
            const int current = bfs_queue.front();
            bfs_queue.pop();

            cluster.indices.push_back(current);

            // Find unvisited neighbours of 'current'
            neighbors.clear();
            if (ptr_searcher_) {
                PointXYZ q{px[current], py[current], pz[current]};
                std::vector<int> raw_neighbors;
                std::vector<float> dists;
                ptr_searcher_->radiusSearch(q, clusterTolerance_, raw_neighbors, dists);
                for (const int nb : raw_neighbors) {
                    if (nb >= 0 && static_cast<std::size_t>(nb) < n && !visited[static_cast<std::size_t>(nb)]) {
                        neighbors.push_back(nb);
                    }
                }
            } else if (searcher_) {
                PointXYZ q{px[current], py[current], pz[current]};
                std::vector<int> raw_neighbors;
                std::vector<float> dists;
                searcher_->radiusSearch(q, clusterTolerance_, raw_neighbors, dists);
                for (const int nb : raw_neighbors) {
                    if (nb >= 0 && static_cast<std::size_t>(nb) < n && !visited[static_cast<std::size_t>(nb)]) {
                        neighbors.push_back(nb);
                    }
                }
            } else {
                radiusQueryUnvisited(
                    px[current], py[current], pz[current],
                    tol_sq, visited, neighbors
                );
            }

            // Mark all found neighbours as visited immediately to prevent
            // them from being queued multiple times from different BFS fronts.
            for (const int nb : neighbors) {
                visited[static_cast<std::size_t>(nb)] = true;
                bfs_queue.push(nb);
            }
        }

        // ── Size filter ───────────────────────────────────────────────────────
        const int sz = static_cast<int>(cluster.indices.size());
        if (sz >= minClusterSize_ && sz <= maxClusterSize_) {
            std::sort(cluster.indices.begin(), cluster.indices.end());
            result.push_back(std::move(cluster));
        }
    }

    return result;
}

} // namespace rvv_pcl
