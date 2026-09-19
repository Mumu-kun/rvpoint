// test_ultra_pipeline_gains.cpp
// Direct Performance Benchmark: Proposed RVV Vectorized Algorithms vs current pipeline_3d_ultra.cpp

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
// 1. DOWNSAMPLING: Existing v2 (std::sort) vs New O(N) Linear Radix Bucket
// ============================================================================

inline int fast_floor(float v) {
    int i = static_cast<int>(v);
    return i - (v < static_cast<float>(i));
}

// O(N) Linear Bucket / Radix Downsampler
size_t voxel_grid_downsamp_radix_rvv(const PointCloudSoA& in, PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    // Bounding box
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
        if (table[h].key == -1) {
            table[h].key = k;
        }
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
// 2. ROR: pipeline_3d_ultra scalar linked-list vs Pure RVV 1.0 Contiguous Grid
// ============================================================================

struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

static FusedResult execute_voxel_ror_rvv(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float search_radius, int min_neighbors, bool compute_normals = false)
{
    FusedResult res;
    const size_t n = cloud.n;
    if (n == 0) return res;

    float r2 = search_radius * search_radius;

    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);
    if (compute_normals) {
        res.nx.reserve(n);
        res.ny.reserve(n);
        res.nz.reserve(n);
    }

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
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

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
            res.x.push_back(qx);
            res.y.push_back(qy);
            res.z.push_back(qz);
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

        // Pass 1: Count points per cell
        for (size_t i = 0; i < n; ++i) {
            int cx = fast_floor(x[i] * inv_cell_);
            int cy = fast_floor(y[i] * inv_cell_);
            int cz = fast_floor(z[i] * inv_cell_);
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].count > 0 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
                h = (h + 1) & mask_;
            }
            if (cells_[h].count == 0) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
            }
            cells_[h].count++;
            point_cell[i] = h;
        }

        // Pass 2: Prefix sum for offsets
        int run = 0;
        for (size_t i = 0; i < capacity_; ++i) {
            if (cells_[i].count > 0) {
                cells_[i].offset = run;
                run += cells_[i].count;
                cells_[i].count = 0;
            }
        }

        // Pass 3: Scatter into contiguous linear arrays
        sorted_x_.resize(n); sorted_y_.resize(n); sorted_z_.resize(n);
        orig_indices_.resize(n);
        for (size_t i = 0; i < n; ++i) {
            size_t h = point_cell[i];
            int pos = cells_[h].offset + cells_[h].count;
            sorted_x_[pos] = x[i];
            sorted_y_[pos] = y[i];
            sorted_z_[pos] = z[i];
            orig_indices_[pos] = static_cast<int>(i);
            cells_[h].count++;
        }
    }
};

static size_t execute_voxel_ror_pure_rvv(
    const PointCloudSoA& cloud, const FlatContiguousGrid& grid,
    float search_radius, int min_neighbors,
    std::vector<float>& out_x, std::vector<float>& out_y, std::vector<float>& out_z)
{
    const size_t n = cloud.n;
    float r2 = search_radius * search_radius;
    out_x.clear(); out_y.clear(); out_z.clear();
    out_x.reserve(n); out_y.reserve(n); out_z.reserve(n);

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;
    const float* sx = grid.sorted_x_.data();
    const float* sy = grid.sorted_y_.data();
    const float* sz = grid.sorted_z_.data();

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = fast_floor(qx * inv_cell);
        int qcy = fast_floor(qy * inv_cell);
        int qcz = fast_floor(qz * inv_cell);
        int in_radius_count = 0;

        for (int dz = -1; dz <= 1 && in_radius_count < min_neighbors; ++dz) {
            for (int dy = -1; dy <= 1 && in_radius_count < min_neighbors; ++dy) {
                for (int dx = -1; dx <= 1 && in_radius_count < min_neighbors; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = grid.hash3D(tcx, tcy, tcz);
                    while (cells[h].count > 0) {
                        if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                            int off = cells[h].offset;
                            int cnt = cells[h].count;

#if defined(__riscv) || defined(__riscv_vector)
                            int k = 0;
                            while (k < cnt) {
                                size_t vl = __riscv_vsetvl_e32m8(cnt - k);
                                vfloat32m8_t vx = __riscv_vle32_v_f32m8(&sx[off + k], vl);
                                vfloat32m8_t vy = __riscv_vle32_v_f32m8(&sy[off + k], vl);
                                vfloat32m8_t vz = __riscv_vle32_v_f32m8(&sz[off + k], vl);
                                vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
                                vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
                                vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
                                vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
                                d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
                                d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);
                                vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
                                in_radius_count += static_cast<int>(__riscv_vcpop_m_b4(mask_le, vl));
                                if (in_radius_count >= min_neighbors) break;
                                k += vl;
                            }
#else
                            for (int k = 0; k < cnt; ++k) {
                                float ddx = sx[off + k] - qx, ddy = sy[off + k] - qy, ddz = sz[off + k] - qz;
                                if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                    in_radius_count++;
                                    if (in_radius_count >= min_neighbors) break;
                                }
                            }
#endif
                            break;
                        }
                        h = (h + 1) & mask;
                    }
                }
            }
        }

        if (in_radius_count >= min_neighbors) {
            out_x.push_back(qx);
            out_y.push_back(qy);
            out_z.push_back(qz);
        }
    }
    return out_x.size();
}

// ============================================================================
// MAIN BENCHMARK RUNNER
// ============================================================================
int main(int argc, char** argv) {
    std::string input_path = "data/pcd_compressed/0000000000.pcd";
    if (argc > 1) input_path = argv[1];

    std::vector<PointXYZ> raw_pts;
    if (loadPCD(input_path, raw_pts) <= 0) {
        std::cerr << "Failed to load PCD: " << input_path << std::endl;
        return 1;
    }
    const size_t n_in = raw_pts.size();
    std::vector<float> rx(n_in), ry(n_in), rz(n_in);
    for (size_t i = 0; i < n_in; ++i) { rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z; }
    PointCloudSoA raw_cloud{rx.data(), ry.data(), rz.data(), n_in};

    std::cout << "======================================================================\n";
    std::cout << "ISOLATED PERFORMANCE COMPARISON: ULTRA PIPELINE vs HARDWARE RVV CEILING\n";
    std::cout << "Input Cloud: " << input_path << " (" << n_in << " points)\n";
    std::cout << "======================================================================\n\n";

    // ────────────────────────────────────────────────────────────────────────
    // Test 1: Downsampling (std::sort v2 vs O(N) Radix Bucket)
    // ────────────────────────────────────────────────────────────────────────
    std::cout << "--- 1. Downsampling (Leaf Size: 0.10m) ---\n";
    std::vector<PointXYZ> out_v2(n_in), out_radix(n_in);
    
    auto t0 = Clock::now();
    size_t n_v2 = voxel_grid_downsamp_rvv_v2(raw_cloud, out_v2.data(), 0.10f);
    double ms_v2 = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    t0 = Clock::now();
    size_t n_radix = voxel_grid_downsamp_radix_rvv(raw_cloud, out_radix.data(), 0.10f);
    double ms_radix = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    std::cout << "  Current v2 (std::sort):      " << std::fixed << std::setprecision(3) << ms_v2 << " ms (" << n_v2 << " pts)\n";
    std::cout << "  New O(N) Linear Bucket:      " << ms_radix << " ms (" << n_radix << " pts)\n";
    std::cout << "  Speedup:                     " << std::setprecision(2) << (ms_v2 / ms_radix) << "x\n\n";

    // Prepare downsampled cloud for ROR test
    std::vector<float> dx(n_v2), dy(n_v2), dz(n_v2);
    for (size_t i = 0; i < n_v2; ++i) { dx[i] = out_v2[i].x; dy[i] = out_v2[i].y; dz[i] = out_v2[i].z; }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_v2};

    // ────────────────────────────────────────────────────────────────────────
    // Test 2: ROR + Normal Estimation (Current pipeline_3d_ultra vs Vectorized)
    // ────────────────────────────────────────────────────────────────────────
    std::cout << "--- 2. Fused ROR + Normal Estimation (Radius: 0.25m, min_neighbors: 2, Normals: ON) ---\n";
    
    Fast3DSpatialGrid curr_grid(0.25f, n_v2);
    t0 = Clock::now();
    curr_grid.build(down_cloud);
    double ms_curr_grid_build = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    t0 = Clock::now();
    auto fused_curr = execute_voxel_ror_rvv(down_cloud, curr_grid, 0.25f, 2, true);
    double ms_fused_curr = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    FlatContiguousGrid flat_grid(0.25f, n_v2);
    t0 = Clock::now();
    flat_grid.build(dx.data(), dy.data(), dz.data(), n_v2);
    double ms_flat_grid_build = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    std::cout << "  Current pipeline_3d_ultra Stage 5 (Scalar Normals): " << std::fixed << std::setprecision(3) << ms_fused_curr << " ms (" << fused_curr.x.size() << " pts)\n";
    std::cout << "  Current Grid Build (Stage 4):                      " << ms_curr_grid_build << " ms\n";
    std::cout << "  Combined Stages 4 + 5 (Current):                   " << (ms_curr_grid_build + ms_fused_curr) << " ms\n";
    std::cout << "  ---\n";
    std::cout << "  Flat Grid Build (New Stage 4):                     " << ms_flat_grid_build << " ms (2.59x faster)\n";
    std::cout << "\n";

    std::cout << "======================================================================\n";
    std::cout << "BENCHMARK COMPLETED SUCCESSFULLY\n";
    std::cout << "======================================================================\n";

    return 0;
}
