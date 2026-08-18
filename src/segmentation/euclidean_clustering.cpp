#include "segmentation/euclidean_clustering.h"
#include "search/octree.h"
#include "search/pointer_octree.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

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

        uint8_t mask_bytes[64];
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

#endif
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

    std::vector<bool> visited(n, false);
    std::vector<int> neighbors;
    neighbors.reserve(256);

    std::queue<int> bfs_queue;

    for (std::size_t seed = 0; seed < n; ++seed) {
        if (visited[seed]) continue;

        ClusterIndices cluster;
        cluster.indices.reserve(64);

        visited[seed] = true;
        bfs_queue.push(static_cast<int>(seed));

        while (!bfs_queue.empty()) {
            const int current = bfs_queue.front();
            bfs_queue.pop();

            cluster.indices.push_back(current);

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

            for (const int nb : neighbors) {
                visited[static_cast<std::size_t>(nb)] = true;
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

} // namespace rvpoint
