#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "euclidean_clustering.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <unordered_map>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ============================================================================
// 1. Contiguous CSR-style Spatial Grid (Flat Arrays instead of Linked List)
// ============================================================================
class Contiguous3DSpatialGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct Cell {
        int cx = -999999, cy = -999999, cz = -999999;
        uint32_t offset = 0;
        uint16_t count = 0;
    };

    float cell_size_;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> point_indices_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    Contiguous3DSpatialGrid(float cell_size = 0.25f) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
        cells_.resize(kCapacity);
    }

    static inline size_t hash3D(int x, int y, int z) {
        size_t h = (static_cast<size_t>(x) * 73856093) ^
                   (static_cast<size_t>(y) * 19349663) ^
                   (static_cast<size_t>(z) * 83492791);
        return h & kMask;
    }

    void build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z; n_pts_ = n;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i] = Cell();
        point_indices_.resize(n);

        // Temp structures for flat grouping
        std::vector<uint16_t> cell_counts(kCapacity, 0);
        std::vector<size_t> assigned_cell(n);

        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].count > 0 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
                h = (h + 1) & kMask;
            }
            if (cells_[h].count == 0) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
            }
            cells_[h].count++;
            assigned_cell[i] = h;
        }

        // Prefix sum for contiguous offsets
        uint32_t running_offset = 0;
        for (size_t i = 0; i < kCapacity; ++i) {
            if (cells_[i].count > 0) {
                cells_[i].offset = running_offset;
                running_offset += cells_[i].count;
                cell_counts[i] = 0; // reuse as write pointer
            }
        }

        // Pack point indices contiguously per cell
        for (size_t i = 0; i < n; ++i) {
            size_t h = assigned_cell[i];
            uint32_t dest = cells_[h].offset + cell_counts[h]++;
            point_indices_[dest] = static_cast<int>(i);
        }
    }

    inline void radiusSearch(float qx, float qy, float qz, float r2,
                             std::vector<int>& neighbors, std::vector<float>& dists2) const {
        neighbors.clear(); dists2.clear();
        int qcx = static_cast<int>(std::floor(qx * inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = hash3D(tcx, tcy, tcz);

                    while (cells_[h].count > 0) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            uint32_t start = cells_[h].offset;
                            uint16_t count = cells_[h].count;
                            const int* __restrict ptr = &point_indices_[start];

                            for (uint16_t k = 0; k < count; ++k) {
                                int curr = ptr[k];
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    neighbors.push_back(curr);
                                    dists2.push_back(d2);
                                }
                            }
                            break;
                        }
                        h = (h + 1) & kMask;
                    }
                }
            }
        }
    }
};

// ============================================================================
// 2. Symmetric Half-Space Disjoint-Set Clustering (14 Cells vs 27 Cells)
// ============================================================================
struct DisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;
    DisjointSet(int n) : parent(n), rank(n, 0) { std::iota(parent.begin(), parent.end(), 0); }
    inline int find(int i) {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) { int nxt = parent[curr]; parent[curr] = root; curr = nxt; }
        return root;
    }
    inline void unite(int i, int j) {
        int root_i = find(i), root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) parent[root_i] = root_j;
            else if (rank[root_i] > rank[root_j]) parent[root_j] = root_i;
            else { parent[root_j] = root_i; rank[root_i]++; }
        }
    }
};

static std::vector<ClusterIndices> execute_halfspace_clustering(
    const PointCloudSoA& cloud, float cluster_tol, int min_sz, int max_sz)
{
    const size_t n = cloud.n;
    if (n == 0) return {};

    Contiguous3DSpatialGrid grid(cluster_tol);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    DisjointSet ds(static_cast<int>(n));
    float tol_sq = cluster_tol * cluster_tol;

    // Define 14 forward half-space offsets (dz>0, or dz=0&dy>0, or dz=0&dy=0&dx>=0)
    static constexpr int kHalfOffsets[14][3] = {
        {0, 0, 0}, {1, 0, 0},
        {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1},
        {-1, 0, 1},  {0, 0, 1},  {1, 0, 1},
        {-1, 1, 1},  {0, 1, 1},  {1, 1, 1}
    };

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        int qcx = static_cast<int>(std::floor(qx * grid.inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * grid.inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * grid.inv_cell_));

        for (int o = 0; o < 14; ++o) {
            int tcx = qcx + kHalfOffsets[o][0];
            int tcy = qcy + kHalfOffsets[o][1];
            int tcz = qcz + kHalfOffsets[o][2];
            size_t h = Contiguous3DSpatialGrid::hash3D(tcx, tcy, tcz);

            while (grid.cells_[h].count > 0) {
                if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                    uint32_t start = grid.cells_[h].offset;
                    uint16_t count = grid.cells_[h].count;
                    const int* __restrict ptr = &grid.point_indices_[start];

                    for (uint16_t k = 0; k < count; ++k) {
                        int nb = ptr[k];
                        if (nb <= static_cast<int>(i)) continue; // avoid redundant self/backward edge
                        float ddx = grid.px_[nb] - qx, ddy = grid.py_[nb] - qy, ddz = grid.pz_[nb] - qz;
                        float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                        if (d2 <= tol_sq) {
                            ds.unite(static_cast<int>(i), nb);
                        }
                    }
                    break;
                }
                h = (h + 1) & Contiguous3DSpatialGrid::kMask;
            }
        }
    }

    std::unordered_map<int, std::vector<int>> cluster_map;
    for (size_t i = 0; i < n; ++i) {
        cluster_map[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));
    }

    std::vector<ClusterIndices> clusters;
    for (auto& kv : cluster_map) {
        if (kv.second.size() >= static_cast<size_t>(min_sz) && kv.second.size() <= static_cast<size_t>(max_sz)) {
            ClusterIndices ci;
            ci.indices = std::move(kv.second);
            clusters.push_back(std::move(ci));
        }
    }
    return clusters;
}

// ============================================================================
// MAIN BENCHMARK
// ============================================================================
int main(int argc, char** argv) {
    std::string pcd_path = "data/pcd_compressed/0000000090.pcd";
    if (argc > 1) pcd_path = argv[1];

    std::cout << "================================================================" << std::endl;
    std::cout << " SINGLE-CORE DEEP DIVE OPTIMIZATION BENCHMARK" << std::endl;
    std::cout << "================================================================" << std::endl;

    std::vector<PointXYZ> raw_pts;
    loadPCD(pcd_path, raw_pts);

    std::vector<float> rx(raw_pts.size()), ry(raw_pts.size()), rz(raw_pts.size());
    for (size_t i = 0; i < raw_pts.size(); ++i) {
        rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z;
    }
    PointCloudSoA raw_cloud{rx.data(), ry.data(), rz.data(), raw_pts.size()};

    std::vector<PointXYZ> down_pts(raw_pts.size());
    size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, down_pts.data(), 0.10f);
    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z;
    }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};
    std::cout << "Downsampled cloud: " << n_down << " points.\n" << std::endl;

    // --- TEST 1: Grid Search & SOR with Contiguous CSR Grid ---
    std::cout << "--- [TEST 1] CONTIGUOUS CSR GRID FOR FUSED SOR ---" << std::endl;
    auto t0 = Clock::now();
    Contiguous3DSpatialGrid csr_grid(0.25f);
    csr_grid.build(dx.data(), dy.data(), dz.data(), n_down);
    double csr_build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    std::cout << "  Contiguous CSR Grid Build: " << std::fixed << std::setprecision(2) << csr_build_ms << " ms" << std::endl;

    t0 = Clock::now();
    std::vector<float> mean_dists(n_down, 0.25f);
    std::vector<float> all_nx(n_down, 0.0f), all_ny(n_down, 0.0f), all_nz(n_down, 1.0f);
    std::vector<int> valid_points;
    valid_points.reserve(n_down);
    std::vector<int> nbrs;
    std::vector<float> d2;
    nbrs.reserve(256); d2.reserve(256);
    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = 0.25f * 0.25f;

    for (size_t i = 0; i < n_down; ++i) {
        csr_grid.radiusSearch(down_cloud.x[i], down_cloud.y[i], down_cloud.z[i], sor_r2, nbrs, d2);
        int found = static_cast<int>(nbrs.size());
        if (found < 2) continue;

        int k_use = std::min(found - 1, 20);
        float sum_dist = 0.0f;
#if defined(__riscv) || defined(__riscv_vector)
        int rem = k_use; int offset = 1;
        while (rem > 0) {
            size_t vl = __riscv_vsetvl_e32m8(rem);
            vfloat32m8_t vd2 = __riscv_vle32_v_f32m8(d2.data() + offset, vl);
            vfloat32m8_t vd  = __riscv_vfsqrt_v_f32m8(vd2, vl);
            vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
            vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m8_f32m1(vd, zero, vl);
            sum_dist += __riscv_vfmv_f_s_f32m1_f32(v_sum);
            offset += vl; rem -= vl;
        }
#else
        for (int j = 1; j <= k_use; ++j) sum_dist += std::sqrt(d2[j]);
#endif
        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m;
        total_sum += m;
        total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));

        if (found >= 3) {
            float cx = 0, cy = 0, cz = 0;
            int n_norm = std::min(found, 16);
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                cx += down_cloud.x[idx]; cy += down_cloud.y[idx]; cz += down_cloud.z[idx];
            }
            float inv_n = 1.0f / static_cast<float>(n_norm);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                float ddx = down_cloud.x[idx] - cx, ddy = down_cloud.y[idx] - cy, ddz = down_cloud.z[idx] - cz;
                c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
            }

            float vx = c01 * c12 - c02 * c11;
            float vy = c01 * c02 - c00 * c12;
            float vz = c00 * c11 - c01 * c01;
            float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
            if (norm > 1e-6f) {
                float inv_norm = 1.0f / norm;
                all_nx[i] = vx * inv_norm; all_ny[i] = vy * inv_norm; all_nz[i] = vz * inv_norm;
            }
        }
    }
    double csr_sor_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    std::cout << "  Contiguous CSR Fused SOR Time: " << csr_sor_ms << " ms (Valid: " << valid_points.size() << " pts)\n" << std::endl;

    // Filter SOR points
    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float thresh = static_cast<float>(global_mean + 1.0f * stddev);

    std::vector<float> sx, sy, sz;
    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) {
            sx.push_back(down_cloud.x[idx]);
            sy.push_back(down_cloud.y[idx]);
            sz.push_back(down_cloud.z[idx]);
        }
    }
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), sx.size()};

    // Stage 8 RANSAC
    float model[4] = {0};
    int inliers = ransac_plane_rvv(sor_cloud, 0.20f, 1000, model);
    std::vector<PointXYZ> in_pts(sor_cloud.n), out_pts(sor_cloud.n);
    size_t n_in = 0, n_out = 0;
    extract_plane_inliers_outliers_rvv(sor_cloud, model, 0.20f, in_pts.data(), out_pts.data(), n_in, n_out);
    std::vector<float> ox(n_out), oy(n_out), oz(n_out);
    for (size_t i = 0; i < n_out; ++i) {
        ox[i] = out_pts[i].x; oy[i] = out_pts[i].y; oz[i] = out_pts[i].z;
    }
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_out};

    // --- TEST 2: Symmetric Half-Space Clustering (14 Cells vs 27 Cells) ---
    std::cout << "--- [TEST 2] SYMMETRIC HALF-SPACE CLUSTERING (14 Cells) ---" << std::endl;
    t0 = Clock::now();
    auto clusters_half = execute_halfspace_clustering(non_ground_cloud, 0.15f, 50, 100000);
    double half_cluster_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
    std::cout << "  Half-Space (14 Cells) Clustering: " << half_cluster_ms << " ms (" << clusters_half.size() << " clusters)" << std::endl;

    std::cout << "================================================================" << std::endl;
    return 0;
}
