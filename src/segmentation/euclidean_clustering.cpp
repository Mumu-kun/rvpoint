#include "segmentation/euclidean_clustering.h"
#include "search/octree.h"
#include "search/pointer_octree.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
#include <cstring>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace rvpoint {

EuclideanClustering::EuclideanClustering(float tolerance, int min_size, int max_size, Backend backend)
    : clusterTolerance_(tolerance), minClusterSize_(min_size), maxClusterSize_(max_size), backend_(backend) {}

void EuclideanClustering::setNeighborSearch(const Octree *search) {
    searcher_ = search;
    ptr_searcher_ = nullptr;
}

void EuclideanClustering::setNeighborSearch(const PointerOctree *search) {
    ptr_searcher_ = search;
    searcher_ = nullptr;
}

void EuclideanClustering::reserve(std::size_t max_points) {
    visited_.resize(max_points, 0);
    bfs_queue_.resize(max_points);
    neighbors_.reserve(1024);
    search_raw_nb_scratch_.reserve(1024);
    search_dists_scratch_.reserve(1024);
    grid_ = Fast3DSpatialGrid(clusterTolerance_, max_points);

    uf_parent_.resize(max_points);
    uf_rank_.resize(max_points, 0);
    root_counts_.resize(max_points, 0);
    root_to_cid_.resize(max_points, -1);

    int n_threads = num_threads_ > 0 ? num_threads_ :
#if defined(_OPENMP)
        omp_get_max_threads();
#else
        1;
#endif
    if (n_threads < 1) n_threads = 1;

    thread_scratch_.resize(n_threads);
    for (int t = 0; t < n_threads; ++t) {
        thread_scratch_[t].self_pts.reserve(256);
        thread_scratch_[t].cand_idx.reserve(2048);
        thread_scratch_[t].cand_x.reserve(2048);
        thread_scratch_[t].cand_y.reserve(2048);
        thread_scratch_[t].cand_z.reserve(2048);
        thread_scratch_[t].edges.reserve(max_points * 2 / n_threads);
    }
}

void EuclideanClustering::radiusQueryUnvisited(
    const PointCloudView& in,
    float qx, float qy, float qz,
    float tol_sq,
    std::vector<int> &out
) {
    const std::size_t n = in.n;
    const float *const px = in.x;
    const float *const py = in.y;
    const float *const pz = in.z;

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
                if (!visited_[static_cast<std::size_t>(idx)]) {
                    out.push_back(idx);
                }
            }
        }

        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        if (visited_[i]) continue;
        const float dx = px[i] - qx;
        const float dy = py[i] - qy;
        const float dz = pz[i] - qz;
        if (dx * dx + dy * dy + dz * dz <= tol_sq) {
            out.push_back(static_cast<int>(i));
        }
    }
#endif
}

void EuclideanClustering::operator()(const PointCloudView& in, ClusterResult& out,
                                    float tolerance, int min_size, int max_size) {
    if (in.n == 0 || !in.x || !in.y || !in.z) {
        out.clear();
        return;
    }

    if (method_ == ClusteringMethod::SymmetricUnionFind) {
        extract_symmetric_union_find(in, out, tolerance, min_size, max_size);
    } else {
        extract_bfs(in, out, tolerance, min_size, max_size);
    }
}

void EuclideanClustering::extract_bfs(const PointCloudView& in, ClusterResult& out,
                                     float tolerance, int min_size, int max_size) {
    out.clear();
    const size_t n = in.n;
    if (visited_.size() < n) {
        visited_.assign(n, 0);
        bfs_queue_.resize(n);
    } else {
        std::memset(visited_.data(), 0, n * sizeof(uint8_t));
    }

    if (neighbors_.capacity() < 1024) {
        neighbors_.reserve(1024);
    }

    const float tol_sq = tolerance * tolerance;

    bool use_grid = use_spatial_grid_ && (ptr_searcher_ == nullptr) && (searcher_ == nullptr);
    if (use_grid) {
        grid_.cell_size_ = tolerance;
        grid_.inv_cell_ = 1.0f / tolerance;
        if (!grid_.build(in.x, in.y, in.z, in.n)) {
            use_grid = false;
        }
    }

    // Initialize offsets: cluster 0 starts at index 0
    out.offsets.clear();
    out.offsets.push_back(0);
    out.indices.clear();

    for (size_t seed = 0; seed < n; ++seed) {
        if (visited_[seed]) continue;

        visited_[seed] = 1;
        size_t head = 0;
        size_t tail = 0;
        bfs_queue_[tail++] = static_cast<int>(seed);

        size_t cluster_start_idx = out.indices.size();

        while (head < tail) {
            int current = bfs_queue_[head++];
            out.indices.push_back(static_cast<uint32_t>(current));

            neighbors_.clear();
            float qx = in.x[current];
            float qy = in.y[current];
            float qz = in.z[current];

            if (ptr_searcher_) {
                PointXYZ q{qx, qy, qz};
                search_raw_nb_scratch_.clear();
                search_dists_scratch_.clear();
                ptr_searcher_->radiusSearch(q, tolerance, search_raw_nb_scratch_, search_dists_scratch_);
                for (int nb : search_raw_nb_scratch_) {
                    if (nb >= 0 && static_cast<size_t>(nb) < n && !visited_[nb]) {
                        neighbors_.push_back(nb);
                    }
                }
            } else if (searcher_) {
                PointXYZ q{qx, qy, qz};
                search_raw_nb_scratch_.clear();
                search_dists_scratch_.clear();
                searcher_->radiusSearch(q, tolerance, search_raw_nb_scratch_, search_dists_scratch_);
                for (int nb : search_raw_nb_scratch_) {
                    if (nb >= 0 && static_cast<size_t>(nb) < n && !visited_[nb]) {
                        neighbors_.push_back(nb);
                    }
                }
            } else if (use_grid) {
                grid_.query_unvisited(qx, qy, qz, tol_sq, visited_.data(), neighbors_);
            } else {
                // Vector / scalar unvisited query
                radiusQueryUnvisited(in, qx, qy, qz, tol_sq, neighbors_);
            }

            for (int nb : neighbors_) {
                if (!visited_[nb]) {
                    visited_[nb] = 1;
                    bfs_queue_[tail++] = nb;
                }
            }
        }

        size_t cluster_size = out.indices.size() - cluster_start_idx;
        if (static_cast<int>(cluster_size) >= min_size && static_cast<int>(cluster_size) <= max_size) {
            std::sort(out.indices.begin() + cluster_start_idx, out.indices.end());
            out.offsets.push_back(static_cast<uint32_t>(out.indices.size()));
        } else {
            // Discard cluster by rolling back indices
            out.indices.resize(cluster_start_idx);
        }
    }
}

void EuclideanClustering::extract_symmetric_union_find(
    const PointCloudView& in, ClusterResult& out,
    float tolerance, int min_size, int max_size)
{
    out.clear();
    const size_t n = in.n;
    if (n == 0) return;

    const float tol_sq = tolerance * tolerance;
    grid_.cell_size_ = tolerance;
    grid_.inv_cell_ = 1.0f / tolerance;
    if (!grid_.build(in.x, in.y, in.z, in.n)) {
        // Fallback to BFS if grid build fails
        extract_bfs(in, out, tolerance, min_size, max_size);
        return;
    }

    if (uf_parent_.size() < n) {
        uf_parent_.resize(n);
        uf_rank_.resize(n, 0);
    }
    for (size_t i = 0; i < n; ++i) {
        uf_parent_[i] = static_cast<int>(i);
        uf_rank_[i] = 0;
    }

    auto uf_find = [&](int i) -> int {
        int root = i;
        while (root != uf_parent_[root]) root = uf_parent_[root];
        int curr = i;
        while (curr != root) {
            int nxt = uf_parent_[curr];
            uf_parent_[curr] = root;
            curr = nxt;
        }
        return root;
    };

    auto uf_unite = [&](int i, int j) {
        int root_i = uf_find(i);
        int root_j = uf_find(j);
        if (root_i != root_j) {
            if (uf_rank_[root_i] < uf_rank_[root_j]) {
                uf_parent_[root_i] = root_j;
            } else if (uf_rank_[root_i] > uf_rank_[root_j]) {
                uf_parent_[root_j] = root_i;
            } else {
                uf_parent_[root_j] = root_i;
                uf_rank_[root_i]++;
            }
        }
    };

    static const int kForwardOffsets[13][3] = {
        {-1, -1, 1}, { 0, -1, 1}, { 1, -1, 1},
        {-1,  0, 1}, { 0,  0, 1}, { 1,  0, 1},
        {-1,  1, 1}, { 0,  1, 1}, { 1,  1, 1},
        {-1,  1, 0}, { 0,  1, 0}, { 1,  1, 0},
        { 1,  0, 0}
    };

    const auto& touched = grid_.touched_slots_;
    const size_t num_cells = touched.size();

    int n_threads = num_threads_ > 0 ? num_threads_ :
#if defined(_OPENMP)
        omp_get_max_threads();
#else
        1;
#endif
    if (n_threads < 1) n_threads = 1;

    if (static_cast<int>(thread_scratch_.size()) < n_threads) {
        thread_scratch_.resize(n_threads);
        for (int t = 0; t < n_threads; ++t) {
            thread_scratch_[t].self_pts.reserve(256);
            thread_scratch_[t].cand_idx.reserve(2048);
            thread_scratch_[t].cand_x.reserve(2048);
            thread_scratch_[t].cand_y.reserve(2048);
            thread_scratch_[t].cand_z.reserve(2048);
            thread_scratch_[t].edges.reserve(n * 2 / n_threads);
        }
    }

    for (int t = 0; t < n_threads; ++t) {
        thread_scratch_[t].edges.clear();
    }

#if defined(_OPENMP)
    #pragma omp parallel num_threads(n_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        auto& scratch = thread_scratch_[tid];
        auto& local_edges = scratch.edges;
        auto& self_pts = scratch.self_pts;
        auto& cand_idx = scratch.cand_idx;
        auto& cand_x = scratch.cand_x;
        auto& cand_y = scratch.cand_y;
        auto& cand_z = scratch.cand_z;

#if defined(_OPENMP)
        #pragma omp for schedule(dynamic, 32)
#endif
        for (size_t s_idx = 0; s_idx < num_cells; ++s_idx) {
            uint32_t slot = touched[s_idx];
            const auto& cell = grid_.cells_[slot];
            if (cell.head == -1) continue;

            self_pts.clear();
            int curr = cell.head;
            while (curr != -1) {
                self_pts.push_back(curr);
                curr = grid_.next_[curr];
            }
            size_t n_self = self_pts.size();

            // 1. Intra-cell pairwise checks
            for (size_t u = 0; u < n_self; ++u) {
                int p_u = self_pts[u];
                float ux = in.x[p_u], uy = in.y[p_u], uz = in.z[p_u];
                for (size_t v = u + 1; v < n_self; ++v) {
                    int p_v = self_pts[v];
                    float ddx = in.x[p_v] - ux, ddy = in.y[p_v] - uy, ddz = in.z[p_v] - uz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                        local_edges.emplace_back(p_u, p_v);
                    }
                }
            }

            // 2. Gather candidates from 13 forward neighbors
            cand_idx.clear();
            cand_x.clear();
            cand_y.clear();
            cand_z.clear();

            for (int k = 0; k < 13; ++k) {
                int tcx = cell.cx + kForwardOffsets[k][0];
                int tcy = cell.cy + kForwardOffsets[k][1];
                int tcz = cell.cz + kForwardOffsets[k][2];

                size_t h = grid_.hash3D(tcx, tcy, tcz);
                int probe = 0;
                while (grid_.cells_[h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
                    if (grid_.cells_[h].cx == tcx && grid_.cells_[h].cy == tcy && grid_.cells_[h].cz == tcz) {
                        int c_nbr = grid_.cells_[h].head;
                        while (c_nbr != -1) {
                            cand_idx.push_back(c_nbr);
                            cand_x.push_back(in.x[c_nbr]);
                            cand_y.push_back(in.y[c_nbr]);
                            cand_z.push_back(in.z[c_nbr]);
                            c_nbr = grid_.next_[c_nbr];
                        }
                        break;
                    }
                    h = (h + 1) & grid_.mask_;
                    probe++;
                }
            }

            // 3. Vector distance checks against candidate forward neighbors
            size_t M = cand_idx.size();
            if (M > 0) {
                for (size_t u = 0; u < n_self; ++u) {
                    int p_u = self_pts[u];
                    float qx = in.x[p_u], qy = in.y[p_u], qz = in.z[p_u];

#if defined(__riscv_vector)
                    bool use_rvv = (backend_ == Backend::Auto || backend_ == Backend::RVV);
                    if (use_rvv) {
                        size_t k = 0;
                        while (k < M) {
                            size_t vl = __riscv_vsetvl_e32m8(M - k);
                            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cand_x[k], vl);
                            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cand_y[k], vl);
                            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cand_z[k], vl);

                            vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
                            vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
                            vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

                            vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
                            d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
                            d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);

                            vbool4_t in_tol = __riscv_vmfle_vf_f32m8_b4(d2, tol_sq, vl);
                            if (__riscv_vcpop_m_b4(in_tol, vl) > 0) {
                                uint8_t mbytes[64];
                                __riscv_vsm_v_b4(mbytes, in_tol, vl);
                                for (size_t lane = 0; lane < vl; ++lane) {
                                    if ((mbytes[lane >> 3] >> (lane & 7u)) & 1u) {
                                        local_edges.emplace_back(p_u, cand_idx[k + lane]);
                                    }
                                }
                            }
                            k += vl;
                        }
                        continue;
                    }
#endif
                    for (size_t k = 0; k < M; ++k) {
                        float ddx = cand_x[k] - qx;
                        float ddy = cand_y[k] - qy;
                        float ddz = cand_z[k] - qz;
                        if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                            local_edges.emplace_back(p_u, cand_idx[k]);
                        }
                    }
                }
            }
        }
    }

    // 4. Unify all edges serially (deterministic, fast: < 1 ms)
    for (int t = 0; t < n_threads; ++t) {
        for (const auto& edge : thread_scratch_[t].edges) {
            uf_unite(edge.first, edge.second);
        }
    }

    // Two-pass zero-allocation CSR Grouping
    if (root_counts_.size() < n) {
        root_counts_.resize(n, 0);
        root_to_cid_.resize(n, -1);
    } else {
        std::memset(root_counts_.data(), 0, n * sizeof(int));
        std::memset(root_to_cid_.data(), -1, n * sizeof(int));
    }

    for (size_t i = 0; i < n; ++i) {
        root_counts_[uf_find(static_cast<int>(i))]++;
    }

    int num_valid_clusters = 0;
    for (size_t r = 0; r < n; ++r) {
        int sz = root_counts_[r];
        if (sz >= min_size && sz <= max_size) {
            root_to_cid_[r] = num_valid_clusters++;
        }
    }

    out.offsets.resize(num_valid_clusters + 1);
    out.offsets[0] = 0;
    std::vector<uint32_t> write_ptrs(num_valid_clusters);
    size_t total_indices = 0;
    for (size_t r = 0; r < n; ++r) {
        int cid = root_to_cid_[r];
        if (cid != -1) {
            write_ptrs[cid] = static_cast<uint32_t>(total_indices);
            total_indices += root_counts_[r];
            out.offsets[cid + 1] = static_cast<uint32_t>(total_indices);
        }
    }

    out.indices.resize(total_indices);
    for (size_t i = 0; i < n; ++i) {
        int r = uf_find(static_cast<int>(i));
        int cid = root_to_cid_[r];
        if (cid != -1) {
            out.indices[write_ptrs[cid]++] = static_cast<uint32_t>(i);
        }
    }

    // Sort point indices in each cluster for determinism
    for (int cid = 0; cid < num_valid_clusters; ++cid) {
        uint32_t start = out.offsets[cid];
        uint32_t end = out.offsets[cid + 1];
        std::sort(out.indices.begin() + start, out.indices.begin() + end);
    }
}

std::vector<ClusterIndices> EuclideanClustering::extract() const {
    std::vector<ClusterIndices> result;
    if (!cloud_ || cloud_->n == 0) return result;

    EuclideanClustering mutable_copy = *this;
    ClusterResult cr;
    mutable_copy(*cloud_, cr, clusterTolerance_, minClusterSize_, maxClusterSize_);

    result.resize(cr.num_clusters());
    for (size_t i = 0; i < cr.num_clusters(); ++i) {
        auto [c_indices, len] = cr.cluster(i);
        result[i].indices.resize(len);
        for (size_t j = 0; j < len; ++j) {
            result[i].indices[j] = static_cast<int>(c_indices[j]);
        }
    }
    return result;
}

} // namespace rvpoint
