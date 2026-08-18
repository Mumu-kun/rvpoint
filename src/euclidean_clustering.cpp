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
#include "caravan_pointer_octree.h"
#include "caravan_radius_search.h"

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
    pointer_searcher_ = nullptr;
    spatial_searcher_ = nullptr;
    caravan_pointer_searcher_ = nullptr;
    caravan_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const PointerOctree *search) {
    pointer_searcher_ = search;
    searcher_ = nullptr;
    spatial_searcher_ = nullptr;
    caravan_pointer_searcher_ = nullptr;
    caravan_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const SpatialHash *search) {
    spatial_searcher_ = search;
    searcher_ = nullptr;
    pointer_searcher_ = nullptr;
    caravan_pointer_searcher_ = nullptr;
    caravan_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const CaravanPointerOctree *search) {
    caravan_pointer_searcher_ = search;
    searcher_ = nullptr;
    pointer_searcher_ = nullptr;
    spatial_searcher_ = nullptr;
    caravan_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const CaravanRadiusSearch *search) {
    caravan_searcher_ = search;
    searcher_ = nullptr;
    pointer_searcher_ = nullptr;
    spatial_searcher_ = nullptr;
    caravan_pointer_searcher_ = nullptr;
}

// ─── radiusQueryUnvisited ─────────────────────────────────────────────────────
//
// Find all cloud indices within tol_sq of (qx,qy,qz) that have not yet been
// assigned to a cluster (visited[i] == false).  Matched indices are appended
// to @p out.

void EuclideanClustering::radiusQueryUnvisited(
    float qx, float qy, float qz,
    float tol_sq,
    const std::vector<uint8_t> &visited,
    std::vector<int>           &out
) const {
    if (!cloud_) return;

    const std::size_t  n  = cloud_->n;
    const float *const px = cloud_->x;
    const float *const py = cloud_->y;
    const float *const pz = cloud_->z;

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)

    std::size_t i = 0;
    while (i < n) {
        const std::size_t vl = __riscv_vsetvl_e32m8(n - i);

        vfloat32m8_t vx = __riscv_vle32_v_f32m8(px + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(py + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz + i, vl);

        vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
        vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
        vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(dx, dx, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dy, dy, vl);
        d2 = __riscv_vfmacc_vv_f32m8(d2, dz, dz, vl);

        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);

        alignas(16) uint8_t mask_bytes[64];
        __riscv_vsm_v_b4(mask_bytes, mask, vl);

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

    // visited[i] == 1  →  point i is already assigned to a cluster
    std::vector<uint8_t> visited(n, 0);

    // Reusable buffers across BFS steps to eliminate heap allocation bottlenecks
    std::vector<int> neighbors;
    neighbors.reserve(256);

    std::vector<int> raw_neighbors;
    raw_neighbors.reserve(256);

    std::vector<float> dists;
    dists.reserve(256);

    // BFS queue
    std::queue<int> bfs_queue;

    for (std::size_t seed = 0; seed < n; ++seed) {
        if (visited[seed]) continue;

        ClusterIndices cluster;
        cluster.indices.reserve(64);

        visited[seed] = 1;
        bfs_queue.push(static_cast<int>(seed));

        while (!bfs_queue.empty()) {
            const int current = bfs_queue.front();
            bfs_queue.pop();

            cluster.indices.push_back(current);

            neighbors.clear();
            if (pointer_searcher_) {
                raw_neighbors.clear();
                dists.clear();
                PointXYZ q{px[current], py[current], pz[current]};
                pointer_searcher_->radiusSearch(q, clusterTolerance_, raw_neighbors, dists);
                for (const int nb : raw_neighbors) {
                    if (nb >= 0 && static_cast<std::size_t>(nb) < n && !visited[static_cast<std::size_t>(nb)]) {
                        neighbors.push_back(nb);
                    }
                }
            } else if (searcher_) {
                raw_neighbors.clear();
                dists.clear();
                PointXYZ q{px[current], py[current], pz[current]};
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

            for (const int nb : neighbors) {
                visited[static_cast<std::size_t>(nb)] = 1;
                bfs_queue.push(nb);
            }
        }

        const int sz = static_cast<int>(cluster.indices.size());
        if (sz >= minClusterSize_ && sz <= maxClusterSize_) {
            std::sort(cluster.indices.begin(), cluster.indices.end());
            result.push_back(std::move(cluster));
        }
    }

    return result;
}

} // namespace rvv_pcl
