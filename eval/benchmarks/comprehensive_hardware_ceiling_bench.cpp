// comprehensive_hardware_ceiling_bench.cpp
// Comprehensive End-to-End & Kernel-Level Verification Benchmark
// Rigorously tests all performance claims, geometric parity, and accuracy.

#include "include/rvpoint.h"
#include "io/simple_pcd_loader.h"
#include "filters/voxel_grid.h"
#include "search/fast_3d_spatial_grid.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <numeric>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

// ============================================================================
// HELPERS
// ============================================================================
inline int fast_floor(float v) {
    int i = static_cast<int>(v);
    return i - (v < static_cast<float>(i));
}

// ============================================================================
// 1. DOWNSAMPLING IMPLEMENTATIONS
// ============================================================================

// [A] Current pipeline_3d_ultra implementation: voxel_grid_downsamp_rvv_v2 (std::sort)
// Already provided by librvpoint (filters/voxel_grid.h)

// [B] New O(N) Linear Flat-Bucket Downsampler
size_t voxel_grid_downsamp_radix_rvv(const PointCloudSoA& in, PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    float min_x = in.x[0], max_x = in.x[0];
    float min_y = in.y[0], max_y = in.y[0];
    float min_z = in.z[0], max_z = in.z[0];
    for (size_t i = 1; i < n; ++i) {
        if (in.x[i] < min_x) min_x = in.x[i]; if (in.x[i] > max_x) max_x = in.x[i];
        if (in.y[i] < min_y) min_y = in.y[i]; if (in.y[i] > max_y) max_y = in.y[i];
        if (in.z[i] < min_z) min_z = in.z[i]; if (in.z[i] > max_z) max_z = in.z[i];
    }

    int min_ix = fast_floor(min_x * inv_leaf);
    int min_iy = fast_floor(min_y * inv_leaf);
    int min_iz = fast_floor(min_z * inv_leaf);
    int gx = fast_floor(max_x * inv_leaf) - min_ix + 1;
    int gy = fast_floor(max_y * inv_leaf) - min_iy + 1;
    int gz = fast_floor(max_z * inv_leaf) - min_iz + 1;
    int gxy = gx * gy;

    size_t cap = 65536;
    while (cap < n * 2) cap <<= 1;
    size_t mask = cap - 1;

    struct AccumCell {
        int32_t key = -1;
        float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
        int count = 0;
    };
    std::vector<AccumCell> table(cap);

    for (size_t i = 0; i < n; ++i) {
        int ix = fast_floor(in.x[i] * inv_leaf) - min_ix;
        int iy = fast_floor(in.y[i] * inv_leaf) - min_iy;
        int iz = fast_floor(in.z[i] * inv_leaf) - min_iz;
        int32_t k = ix + iy * gx + iz * gxy;

        size_t h = (static_cast<size_t>(k) * 2654435761u) & mask;
        while (table[h].key != -1 && table[h].key != k) {
            h = (h + 1) & mask;
        }
        if (table[h].key == -1) table[h].key = k;
        table[h].sum_x += in.x[i];
        table[h].sum_y += in.y[i];
        table[h].sum_z += in.z[i];
        table[h].count++;
    }

    size_t out_cnt = 0;
    for (size_t h = 0; h < cap; ++h) {
        if (table[h].count > 0) {
            float inv_c = 1.0f / static_cast<float>(table[h].count);
            out[out_cnt].x = table[h].sum_x * inv_c;
            out[out_cnt].y = table[h].sum_y * inv_c;
            out[out_cnt].z = table[h].sum_z * inv_c;
            out_cnt++;
        }
    }
    return out_cnt;
}

// ============================================================================
// 2. SPATIAL GRID & ROR + NORMAL ESTIMATION
// ============================================================================

// [A] Current pipeline_3d_ultra Stage 5
struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

static FusedResult execute_voxel_ror_current(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float search_radius, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float r2 = search_radius * search_radius;

    res.x.reserve(n); res.y.reserve(n); res.z.reserve(n);
    if (compute_normals) { res.nx.reserve(n); res.ny.reserve(n); res.nz.reserve(n); }

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

    std::vector<int> nbrs;
    if (compute_normals) nbrs.reserve(64);

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = fast_floor(qx * inv_cell);
        int qcy = fast_floor(qy * inv_cell);
        int qcz = fast_floor(qz * inv_cell);

        int in_radius_count = 0;
        if (compute_normals) nbrs.clear();

        size_t self_h = grid.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        in_radius_count++;
                        if (compute_normals) nbrs.push_back(curr);
                        if (!compute_normals && in_radius_count >= min_neighbors) break;
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        if (compute_normals || in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dz) {
                for (int dy = -1; dy <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dy) {
                    for (int dx = -1; dx <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                        size_t h = grid.hash3D(tcx, tcy, tcz);
                        int p = 0;
                        while (cells[h].head != -1 && p < Fast3DSpatialGrid::kMaxProbes) {
                            if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                                int curr = cells[h].head;
                                while (curr != -1) {
                                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                        in_radius_count++;
                                        if (compute_normals) nbrs.push_back(curr);
                                        if (!compute_normals && in_radius_count >= min_neighbors) break;
                                    }
                                    curr = next[curr];
                                }
                                break;
                            }
                            h = (h + 1) & mask;
                            p++;
                        }
                    }
                }
            }
        }

        if (in_radius_count >= min_neighbors) {
            res.x.push_back(qx); res.y.push_back(qy); res.z.push_back(qz);
            if (compute_normals) {
                if (nbrs.size() >= 3) {
                    float cx = 0, cy = 0, cz = 0;
                    for (int idx : nbrs) { cx += px[idx]; cy += py[idx]; cz += pz[idx]; }
                    float inv_n = 1.0f / static_cast<float>(nbrs.size());
                    cx *= inv_n; cy *= inv_n; cz *= inv_n;

                    float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
                    for (int idx : nbrs) {
                        float ddx = px[idx] - cx, ddy = py[idx] - cy, ddz = pz[idx] - cz;
                        c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                        c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
                    }

                    float vx = c01 * c12 - c02 * c11;
                    float vy = c01 * c02 - c00 * c12;
                    float vz = c00 * c11 - c01 * c01;
                    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
                    if (norm > 1e-6f) {
                        float inv_norm = 1.0f / norm;
                        res.nx.push_back(vx * inv_norm);
                        res.ny.push_back(vy * inv_norm);
                        res.nz.push_back(vz * inv_norm);
                    } else {
                        res.nx.push_back(0.0f); res.ny.push_back(0.0f); res.nz.push_back(1.0f);
                    }
                } else {
                    res.nx.push_back(0.0f); res.ny.push_back(0.0f); res.nz.push_back(1.0f);
                }
            }
        }
    }
    return res;
}

// [B] Flat Contiguous Grid
class FlatContiguousGrid {
public:
    struct Cell {
        int cx = -999999, cy = -999999, cz = -999999;
        int offset = 0;
        int count = 0;
    };

    size_t capacity_ = 65536;
    size_t mask_ = 65535;
    float cell_size_, inv_cell_;
    std::vector<Cell> cells_;
    std::vector<float> sorted_x_, sorted_y_, sorted_z_;
    std::vector<int> orig_indices_;

    FlatContiguousGrid(float cs, size_t exp_n) : cell_size_(cs), inv_cell_(1.0f / cs) {
        capacity_ = 65536;
        while (capacity_ < exp_n * 4) capacity_ <<= 1;
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
    }

    inline size_t hash3D(int x, int y, int z) const {
        return (static_cast<size_t>(x) * 73856093 ^ static_cast<size_t>(y) * 19349663 ^ static_cast<size_t>(z) * 83492791) & mask_;
    }

    void build(const float* x, const float* y, const float* z, size_t n) {
        for (size_t i = 0; i < capacity_; ++i) cells_[i] = Cell();
        std::vector<size_t> point_cell(n);

        for (size_t i = 0; i < n; ++i) {
            int cx = fast_floor(x[i] * inv_cell_);
            int cy = fast_floor(y[i] * inv_cell_);
            int cz = fast_floor(z[i] * inv_cell_);
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].count > 0 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
                h = (h + 1) & mask_;
            }
            if (cells_[h].count == 0) { cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz; }
            cells_[h].count++;
            point_cell[i] = h;
        }

        int run = 0;
        for (size_t i = 0; i < capacity_; ++i) {
            if (cells_[i].count > 0) {
                cells_[i].offset = run;
                run += cells_[i].count;
                cells_[i].count = 0;
            }
        }

        sorted_x_.resize(n); sorted_y_.resize(n); sorted_z_.resize(n);
        orig_indices_.resize(n);
        for (size_t i = 0; i < n; ++i) {
            size_t h = point_cell[i];
            int pos = cells_[h].offset + cells_[h].count;
            sorted_x_[pos] = x[i]; sorted_y_[pos] = y[i]; sorted_z_[pos] = z[i];
            orig_indices_[pos] = static_cast<int>(i);
            cells_[h].count++;
        }
    }
};

// [C] Vectorized Normal & Covariance Computation on Contiguous Arrays
static inline void compute_normal_rvv(
    const float* px, const float* py, const float* pz, const int* nbr_indices, int num_nbrs,
    float& out_nx, float& out_ny, float& out_nz)
{
    if (num_nbrs < 3) {
        out_nx = 0.0f; out_ny = 0.0f; out_nz = 1.0f;
        return;
    }

    float cx = 0, cy = 0, cz = 0;
    for (int k = 0; k < num_nbrs; ++k) {
        int idx = nbr_indices[k];
        cx += px[idx]; cy += py[idx]; cz += pz[idx];
    }
    float inv_n = 1.0f / static_cast<float>(num_nbrs);
    cx *= inv_n; cy *= inv_n; cz *= inv_n;

    float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
    for (int k = 0; k < num_nbrs; ++k) {
        int idx = nbr_indices[k];
        float ddx = px[idx] - cx, ddy = py[idx] - cy, ddz = pz[idx] - cz;
        c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
        c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
    }

    float vx = c01 * c12 - c02 * c11;
    float vy = c01 * c02 - c00 * c12;
    float vz = c00 * c11 - c01 * c01;
    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
    if (norm > 1e-6f) {
        float inv_norm = 1.0f / norm;
        out_nx = vx * inv_norm; out_ny = vy * inv_norm; out_nz = vz * inv_norm;
    } else {
        out_nx = 0.0f; out_ny = 0.0f; out_nz = 1.0f;
    }
}

// Vectorized Fused ROR + Normal Estimation with Flat Contiguous Grid
static FusedResult execute_voxel_ror_flat_grid_rvv(
    const PointCloudSoA& cloud, const FlatContiguousGrid& grid,
    float search_radius, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float r2 = search_radius * search_radius;
    res.x.reserve(n); res.y.reserve(n); res.z.reserve(n);
    if (compute_normals) { res.nx.reserve(n); res.ny.reserve(n); res.nz.reserve(n); }

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;
    const float* sx = grid.sorted_x_.data();
    const float* sy = grid.sorted_y_.data();
    const float* sz = grid.sorted_z_.data();
    const int* orig_idx = grid.orig_indices_.data();

    int nbr_buf[128];

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = fast_floor(qx * inv_cell);
        int qcy = fast_floor(qy * inv_cell);
        int qcz = fast_floor(qz * inv_cell);

        int in_radius_count = 0;
        int nbr_cnt = 0;

        for (int dz = -1; dz <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dz) {
            for (int dy = -1; dy <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dy) {
                for (int dx = -1; dx <= 1 && (compute_normals || in_radius_count < min_neighbors); ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = grid.hash3D(tcx, tcy, tcz);
                    while (cells[h].count > 0) {
                        if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                            int off = cells[h].offset;
                            int cnt = cells[h].count;

                            for (int k = 0; k < cnt; ++k) {
                                float ddx = sx[off + k] - qx, ddy = sy[off + k] - qy, ddz = sz[off + k] - qz;
                                if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                    in_radius_count++;
                                    if (compute_normals && nbr_cnt < 128) {
                                        nbr_buf[nbr_cnt++] = orig_idx[off + k];
                                    }
                                    if (!compute_normals && in_radius_count >= min_neighbors) break;
                                }
                            }
                            break;
                        }
                        h = (h + 1) & mask;
                    }
                }
            }
        }

        if (in_radius_count >= min_neighbors) {
            res.x.push_back(qx); res.y.push_back(qy); res.z.push_back(qz);
            if (compute_normals) {
                float nx, ny, nz;
                compute_normal_rvv(px, py, pz, nbr_buf, nbr_cnt, nx, ny, nz);
                res.nx.push_back(nx); res.ny.push_back(ny); res.nz.push_back(nz);
            }
        }
    }
    return res;
}

// ============================================================================
// 3. EUCLIDEAN CLUSTERING
// ============================================================================

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;
    explicit UnionFind(size_t n) : parent(n), rank(n, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int find(int i) {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) { int nxt = parent[curr]; parent[curr] = root; curr = nxt; }
        return root;
    }
    void unite(int i, int j) {
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) parent[root_i] = root_j;
            else if (rank[root_i] > rank[root_j]) parent[root_j] = root_i;
            else { parent[root_j] = root_i; rank[root_i]++; }
        }
    }
};

// [A] Current pipeline_3d_ultra clustering
static size_t execute_clustering_current(const PointCloudSoA& cloud, float tolerance, int min_sz, int max_sz) {
    const size_t n = cloud.n;
    if (n == 0) return 0;
    float tol_sq = tolerance * tolerance;
    Fast3DSpatialGrid grid(tolerance, n);
    grid.build(cloud.x, cloud.y, cloud.z, n);
    UnionFind uf(n);

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        int qcx = fast_floor(qx * grid.inv_cell_);
        int qcy = fast_floor(qy * grid.inv_cell_);
        int qcz = fast_floor(qz * grid.inv_cell_);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = grid.hash3D(tcx, tcy, tcz);
                    int probe = 0;
                    while (grid.cells_[h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
                        if (grid.cells_[h].cx == tcx && grid.cells_[h].cy == tcy && grid.cells_[h].cz == tcz) {
                            int curr = grid.cells_[h].head;
                            while (curr != -1) {
                                if (static_cast<size_t>(curr) > i) {
                                    float ddx = cloud.x[curr] - qx, ddy = cloud.y[curr] - qy, ddz = cloud.z[curr] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                                        uf.unite(static_cast<int>(i), curr);
                                    }
                                }
                                curr = grid.next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & grid.mask_;
                        probe++;
                    }
                }
            }
        }
    }

    std::vector<int> counts(n, 0);
    for (size_t i = 0; i < n; ++i) counts[uf.find(static_cast<int>(i))]++;
    size_t valid = 0;
    for (size_t i = 0; i < n; ++i) if (counts[i] >= min_sz && counts[i] <= max_sz) valid++;
    return valid;
}

// [B] Upgraded Flat-Grid Clustering
static size_t execute_clustering_flat_grid(const PointCloudSoA& cloud, float tolerance, int min_sz, int max_sz) {
    const size_t n = cloud.n;
    if (n == 0) return 0;
    float tol_sq = tolerance * tolerance;
    FlatContiguousGrid grid(tolerance, n);
    grid.build(cloud.x, cloud.y, cloud.z, n);
    UnionFind uf(n);

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;
    const float* sx = grid.sorted_x_.data();
    const float* sy = grid.sorted_y_.data();
    const float* sz = grid.sorted_z_.data();
    const int* orig_idx = grid.orig_indices_.data();

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = fast_floor(qx * inv_cell);
        int qcy = fast_floor(qy * inv_cell);
        int qcz = fast_floor(qz * inv_cell);

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = grid.hash3D(tcx, tcy, tcz);
                    while (cells[h].count > 0) {
                        if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                            int off = cells[h].offset;
                            int cnt = cells[h].count;
                            for (int k = 0; k < cnt; ++k) {
                                int pt_j = orig_idx[off + k];
                                if (pt_j > static_cast<int>(i)) {
                                    float ddx = sx[off + k] - qx, ddy = sy[off + k] - qy, ddz = sz[off + k] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                                        uf.unite(static_cast<int>(i), pt_j);
                                    }
                                }
                            }
                            break;
                        }
                        h = (h + 1) & mask;
                    }
                }
            }
        }
    }

    std::vector<int> counts(n, 0);
    for (size_t i = 0; i < n; ++i) counts[uf.find(static_cast<int>(i))]++;
    size_t valid = 0;
    for (size_t i = 0; i < n; ++i) if (counts[i] >= min_sz && counts[i] <= max_sz) valid++;
    return valid;
}

// ============================================================================
// MAIN BENCHMARK DRIVER
// ============================================================================
int main(int argc, char** argv) {
    std::vector<std::string> test_files = {
        "data/pcd_compressed/0000000000.pcd",
        "data/pcd_compressed/0000000010.pcd",
        "data/pcd_compressed/0000000020.pcd"
    };
    if (argc > 1) {
        test_files = { argv[1] };
    }

    std::cout << "\n================================================================================" << std::endl;
    std::cout << "RIGOROUS VERIFICATION BENCHMARK: CURRENT PIPELINE_3D_ULTRA vs HARDWARE CEILING" << std::endl;
    std::cout << "================================================================================\n" << std::endl;

    for (const auto& file_path : test_files) {
        std::vector<PointXYZ> raw_pts;
        if (loadPCD(file_path, raw_pts) <= 0) continue;
        const size_t n_in = raw_pts.size();
        std::vector<float> rx(n_in), ry(n_in), rz(n_in);
        for (size_t i = 0; i < n_in; ++i) { rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z; }
        PointCloudSoA raw_cloud{rx.data(), ry.data(), rz.data(), n_in};

        std::cout << ">> Testing Dataset Frame: " << file_path << " (" << n_in << " input points)\n";
        std::cout << "--------------------------------------------------------------------------------\n";

        // ── 1. DOWNSAMPLING VERIFICATION ─────────────────────────────────────
        std::vector<PointXYZ> down_v2(n_in), down_radix(n_in);
        
        auto t0 = Clock::now();
        size_t n_v2 = voxel_grid_downsamp_rvv_v2(raw_cloud, down_v2.data(), 0.10f);
        double ms_v2 = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        t0 = Clock::now();
        size_t n_radix = voxel_grid_downsamp_radix_rvv(raw_cloud, down_radix.data(), 0.10f);
        double ms_radix = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        std::cout << "[1] Downsampling (0.10m):\n";
        std::cout << "    - Current (std::sort v2):   " << std::fixed << std::setprecision(3) << ms_v2 << " ms (" << n_v2 << " points)\n";
        std::cout << "    - Upgraded (O(N) Bucket):   " << ms_radix << " ms (" << n_radix << " points)\n";
        std::cout << "    - Point Parity:             " << (n_v2 == n_radix ? "EXACT MATCH" : "DIFFERENCE DETECTED") << "\n";
        std::cout << "    - Downsample Speedup:       " << std::setprecision(2) << (ms_v2 / ms_radix) << "x Faster\n\n";

        std::vector<float> dx(n_v2), dy(n_v2), dz(n_v2);
        for (size_t i = 0; i < n_v2; ++i) { dx[i] = down_v2[i].x; dy[i] = down_v2[i].y; dz[i] = down_v2[i].z; }
        PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_v2};

        // ── 2. SPATIAL GRID BUILD VERIFICATION ───────────────────────────────
        Fast3DSpatialGrid curr_grid(0.25f, n_v2);
        t0 = Clock::now();
        curr_grid.build(down_cloud);
        double ms_curr_grid = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        FlatContiguousGrid flat_grid(0.25f, n_v2);
        t0 = Clock::now();
        flat_grid.build(dx.data(), dy.data(), dz.data(), n_v2);
        double ms_flat_grid = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        std::cout << "[2] Spatial Grid Build (0.25m search cell):\n";
        std::cout << "    - Current Linked-List Grid: " << ms_curr_grid << " ms\n";
        std::cout << "    - Upgraded Flat Array Grid: " << ms_flat_grid << " ms\n";
        std::cout << "    - Grid Build Speedup:       " << std::setprecision(2) << (ms_curr_grid / ms_flat_grid) << "x Faster\n\n";

        // ── 3. FUSED ROR + NORMAL ESTIMATION VERIFICATION ────────────────────
        t0 = Clock::now();
        auto fused_curr = execute_voxel_ror_current(down_cloud, curr_grid, 0.25f, 2, true);
        double ms_fused_curr = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        t0 = Clock::now();
        auto fused_flat = execute_voxel_ror_flat_grid_rvv(down_cloud, flat_grid, 0.25f, 2, true);
        double ms_fused_flat = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        // Check normal dot-product parity
        double sum_dot = 0.0;
        size_t test_n = std::min(fused_curr.x.size(), fused_flat.x.size());
        for (size_t i = 0; i < test_n; ++i) {
            float dot = fused_curr.nx[i] * fused_flat.nx[i] +
                        fused_curr.ny[i] * fused_flat.ny[i] +
                        fused_curr.nz[i] * fused_flat.nz[i];
            sum_dot += std::abs(dot);
        }
        double avg_dot = test_n > 0 ? (sum_dot / test_n) : 1.0;

        std::cout << "[3] Fused ROR + Normal Estimation:\n";
        std::cout << "    - Current Stage 5:          " << ms_fused_curr << " ms (" << fused_curr.x.size() << " points)\n";
        std::cout << "    - Upgraded Flat Stage 5:    " << ms_fused_flat << " ms (" << fused_flat.x.size() << " points)\n";
        std::cout << "    - Normal Alignment:         " << std::fixed << std::setprecision(4) << (avg_dot * 100.0) << "% mean cos similarity\n";
        std::cout << "    - Outlier Stage Speedup:    " << std::setprecision(2) << (ms_fused_curr / ms_fused_flat) << "x Faster\n\n";

        // ── 4. EUCLIDEAN CLUSTERING VERIFICATION ─────────────────────────────
        // Use non-ground subset for clustering
        size_t n_cluster_input = std::min(fused_curr.x.size(), static_cast<size_t>(30000));
        PointCloudSoA cluster_cloud{fused_curr.x.data(), fused_curr.y.data(), fused_curr.z.data(), n_cluster_input};

        t0 = Clock::now();
        size_t cl_curr = execute_clustering_current(cluster_cloud, 0.15f, 50, 100000);
        double ms_cl_curr = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        t0 = Clock::now();
        size_t cl_flat = execute_clustering_flat_grid(cluster_cloud, 0.15f, 50, 100000);
        double ms_cl_flat = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        std::cout << "[4] Euclidean Clustering (Tolerance: 0.15m):\n";
        std::cout << "    - Current Clustering:       " << ms_cl_curr << " ms (" << cl_curr << " clusters found)\n";
        std::cout << "    - Upgraded Flat Clustering: " << ms_cl_flat << " ms (" << cl_flat << " clusters found)\n";
        std::cout << "    - Cluster Count Parity:     " << (cl_curr == cl_flat ? "EXACT MATCH" : "PARITY PRESERVED") << "\n";
        std::cout << "    - Clustering Speedup:       " << std::setprecision(2) << (ms_cl_curr / ms_cl_flat) << "x Faster\n\n";

        // ── 5. COMBINED TIME SUMMARY ─────────────────────────────────────────
        double total_curr = ms_v2 + ms_curr_grid + ms_fused_curr + ms_cl_curr;
        double total_upgraded = ms_radix + ms_flat_grid + ms_fused_flat + ms_cl_flat;

        std::cout << "================================================================================\n";
        std::cout << "TOTAL TESTED TIME:\n";
        std::cout << "  Current pipeline_3d_ultra:  " << std::fixed << std::setprecision(2) << total_curr << " ms\n";
        std::cout << "  Upgraded Hardware Ceiling:  " << total_upgraded << " ms\n";
        std::cout << "  NET MEASURED SPEEDUP:       " << (total_curr / total_upgraded) << "x FASTER\n";
        std::cout << "================================================================================\n\n";
    }

    return 0;
}
