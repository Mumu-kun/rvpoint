// eval/pipelines/pipeline_3d_stream_rvv_clust.cpp
// Multi-Threaded Continuous Stream True-3D Perception Pipeline
// Hardware RVV 1.0 Vectorized Euclidean Clustering Standalone Test Target

#include "include/rvpoint.h"
#include "search/fast_3d_spatial_grid.h"
#include "io/simple_pcd_loader.h"
#include "filters/voxel_grid.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#if defined(_OPENMP)
#include <omp.h>
#endif

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvpoint;

namespace {

struct StreamConfig {
    std::string input_path = "data/pcd_compressed";
    int max_frames = 20;
    int num_threads = 8;
    bool write_clusters = false;
    bool progress = true;
    std::string mode = "inter";
    float voxel_leaf_size = 0.10f;
    float ror_radius = 0.25f;
    int ror_min_neighbors = 2;
    float ransac_distance_threshold = 0.20f;
    int ransac_max_iterations = 250;
    float cluster_tolerance = 0.15f;
    int min_cluster_size = 50;
    int max_cluster_size = 100000;
};

struct FrameMetrics {
    int frame_id = 0;
    std::string filename;
    double io_read_ms = 0.0;
    double compute_ms = 0.0;
    double io_write_ms = 0.0;

    double voxel_ms = 0.0;
    double grid_build_ms = 0.0;
    double ror_ms = 0.0;
    double ransac_ms = 0.0;
    double cluster_ms = 0.0;

    size_t raw_points = 0;
    size_t downsampled_points = 0;
    size_t ror_points = 0;
    size_t ground_inliers = 0;
    size_t obstacle_points = 0;
    size_t cluster_count = 0;
};

struct FastPRNG {
    uint64_t state;
    explicit FastPRNG(uint64_t seed = 0x853c49e6748fea9bULL) : state(seed != 0 ? seed : 0x853c49e6748fea9bULL) {}
    inline uint32_t next() {
        uint64_t x = state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state = x;
        return static_cast<uint32_t>((x * 0x2545F4914F6CDD1DULL) >> 32);
    }
    inline uint32_t next_bounded(uint32_t bound) {
        return bound > 0 ? (next() % bound) : 0;
    }
};

class FastStreamGrid {
public:
    struct Cell {
        int cx = -999999, cy = -999999, cz = -999999;
        int head = -1;
    };
    static constexpr int kMaxProbes = 32;
    size_t capacity_ = 65536;
    size_t mask_ = 65535;
    float cell_size_;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    std::vector<uint32_t> touched_slots_;

    FastStreamGrid(float cell_size = 0.25f, size_t expected_points = 65536)
        : cell_size_(cell_size), inv_cell_(1.0f / cell_size)
    {
        capacity_ = next_power_of_2(std::max<size_t>(65536, expected_points * 2));
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
        touched_slots_.reserve(expected_points);
        next_.reserve(expected_points);
    }

    inline size_t hash3D(int x, int y, int z) const {
        size_t h = (static_cast<size_t>(x) * 73856093) ^
                   (static_cast<size_t>(y) * 19349663) ^
                   (static_cast<size_t>(z) * 83492791);
        return h & mask_;
    }

    bool build(const float* x, const float* y, const float* z, size_t n) {
        for (uint32_t slot : touched_slots_) {
            cells_[slot].head = -1;
            cells_[slot].cx = -999999;
        }
        touched_slots_.clear();

        if (next_.size() < n) next_.resize(n);

        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

            size_t h = hash3D(cx, cy, cz);
            int probe = 0;
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz) && probe < kMaxProbes) {
                h = (h + 1) & mask_;
                probe++;
            }
            if (cells_[h].head == -1) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
                touched_slots_.push_back(static_cast<uint32_t>(h));
            }
            next_[i] = cells_[h].head;
            cells_[h].head = static_cast<int>(i);
        }
        return true;
    }
};

struct UnionFind {
    std::vector<int> parent;
    std::vector<int> rank;

    void reset(size_t n) {
        if (parent.size() < n) {
            parent.resize(n);
            rank.resize(n);
        }
        std::iota(parent.begin(), parent.begin() + n, 0);
        std::fill(rank.begin(), rank.begin() + n, 0);
    }

    int find(int i) {
        int root = i;
        while (root != parent[root]) root = parent[root];
        int curr = i;
        while (curr != root) {
            int nxt = parent[curr];
            parent[curr] = root;
            curr = nxt;
        }
        return root;
    }

    void unite(int i, int j) {
        int root_i = find(i);
        int root_j = find(j);
        if (root_i != root_j) {
            if (rank[root_i] < rank[root_j]) {
                parent[root_i] = root_j;
            } else if (rank[root_i] > rank[root_j]) {
                parent[root_j] = root_i;
            } else {
                parent[root_j] = root_i;
                rank[root_i]++;
            }
        }
    }
};

struct ClusterResult {
    std::vector<int> indices;
};

struct alignas(64) ThreadFrameContext {
    std::vector<PointXYZ> raw_pts;
    std::vector<float> rx, ry, rz;

    std::vector<PointXYZ> downsampled_pts;
    std::vector<float> dx, dy, dz;

    FastStreamGrid ror_grid;
    std::vector<uint8_t> ror_keep;
    std::vector<float> fx, fy, fz;

    std::vector<float> sample_sx, sample_sy, sample_sz;
    std::vector<float> ox, oy, oz;

    FastStreamGrid cluster_grid;
    UnionFind uf;
    std::vector<std::vector<int>> root_to_pts;
    std::vector<ClusterResult> clusters;

    // RVV clustering candidate & CSR grouping buffers
    std::vector<int> cand_idx;
    std::vector<float> cand_x, cand_y, cand_z;
    std::vector<int> root_counts;
    std::vector<int> root_to_cid;

    ThreadFrameContext(float leaf_size = 0.10f, float ror_rad = 0.25f, float clust_tol = 0.15f)
        : ror_grid(ror_rad, 65536), cluster_grid(clust_tol, 65536)
    {
        raw_pts.reserve(130000);
        rx.reserve(130000); ry.reserve(130000); rz.reserve(130000);
        downsampled_pts.resize(130000);
        dx.reserve(65536); dy.reserve(65536); dz.reserve(65536);
        ror_keep.reserve(65536);
        fx.reserve(65536); fy.reserve(65536); fz.reserve(65536);
        sample_sx.reserve(2048); sample_sy.reserve(2048); sample_sz.reserve(2048);
        ox.reserve(65536); oy.reserve(65536); oz.reserve(65536);
        root_to_pts.resize(65536);
        clusters.reserve(256);

        cand_idx.reserve(256);
        cand_x.reserve(256);
        cand_y.reserve(256);
        cand_z.reserve(256);
        root_counts.reserve(65536);
        root_to_cid.reserve(65536);
    }
};

static size_t execute_fast_radix_voxel_downsample_ctx(
    ThreadFrameContext& ctx, size_t n, float leaf_size)
{
    if (n == 0) return 0;
    if (ctx.downsampled_pts.size() < n) ctx.downsampled_pts.resize(n);
    PointCloudSoA in{ctx.rx.data(), ctx.ry.data(), ctx.rz.data(), n};
    size_t n_down = voxel_grid_downsamp_rvv_v2(in, ctx.downsampled_pts.data(), leaf_size);
    ctx.dx.resize(n_down); ctx.dy.resize(n_down); ctx.dz.resize(n_down);

    for (size_t i = 0; i < n_down; ++i) {
        ctx.dx[i] = ctx.downsampled_pts[i].x;
        ctx.dy[i] = ctx.downsampled_pts[i].y;
        ctx.dz[i] = ctx.downsampled_pts[i].z;
    }
    return n_down;
}

static size_t execute_ror_rvv_ctx(
    ThreadFrameContext& ctx, size_t n, float search_radius, int min_neighbors)
{
    if (n == 0) return 0;
    float r2 = search_radius * search_radius;
    const float* px = ctx.dx.data();
    const float* py = ctx.dy.data();
    const float* pz = ctx.dz.data();
    const auto& cells = ctx.ror_grid.cells_;
    const auto& next = ctx.ror_grid.next_;
    const size_t mask = ctx.ror_grid.mask_;
    const float inv_cell = ctx.ror_grid.inv_cell_;

    if (ctx.ror_keep.size() < n) ctx.ror_keep.resize(n);
    uint8_t* keep = ctx.ror_keep.data();

    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        int in_radius_count = 0;

        size_t self_h = ctx.ror_grid.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < FastStreamGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        in_radius_count++;
                        if (in_radius_count >= min_neighbors) break;
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        if (in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && in_radius_count < min_neighbors; ++dz) {
                for (int dy = -1; dy <= 1 && in_radius_count < min_neighbors; ++dy) {
                    for (int dx = -1; dx <= 1 && in_radius_count < min_neighbors; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
                        int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                        size_t h = ctx.ror_grid.hash3D(tcx, tcy, tcz);
                        int p = 0;
                        while (cells[h].head != -1 && p < FastStreamGrid::kMaxProbes) {
                            if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                                int curr = cells[h].head;
                                while (curr != -1) {
                                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                        in_radius_count++;
                                        if (in_radius_count >= min_neighbors) break;
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
        keep[i] = (in_radius_count >= min_neighbors) ? 1 : 0;
    }

    ctx.fx.clear(); ctx.fy.clear(); ctx.fz.clear();
    for (size_t i = 0; i < n; ++i) {
        if (keep[i]) {
            ctx.fx.push_back(px[i]);
            ctx.fy.push_back(py[i]);
            ctx.fz.push_back(pz[i]);
        }
    }
    return ctx.fx.size();
}

static bool compute_plane_coeffs(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 float x3, float y3, float z3,
                                 float* model) 
{
    float v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    float v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < 1e-4f) return false;
    a /= norm; b /= norm; c /= norm;
    model[0] = a; model[1] = b; model[2] = c;
    model[3] = -(a*x1 + b*y1 + c*z1);
    return true;
}

static int ransac_plane_sprt_rvv_ctx(
    ThreadFrameContext& ctx, size_t n, float dist_thresh, int max_iters, float* model,
    const float* ground_normal_prior, float min_ground_dot, uint64_t seed = 42)
{
    if (n < 3) return 0;
    FastPRNG rng(seed);
    float best_model[4] = {0,0,0,0};
    int best_sample_inliers = 0;
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);

    const size_t sample_sz = std::min(n, static_cast<size_t>(2048));
    ctx.sample_sx.resize(sample_sz);
    ctx.sample_sy.resize(sample_sz);
    ctx.sample_sz.resize(sample_sz);
    float* sx = ctx.sample_sx.data();
    float* sy = ctx.sample_sy.data();
    float* sz = ctx.sample_sz.data();
    const float* fx = ctx.fx.data();
    const float* fy = ctx.fy.data();
    const float* fz = ctx.fz.data();

    size_t stride = std::max<size_t>(1, n / sample_sz);
    for (size_t k = 0; k < sample_sz; ++k) {
        size_t idx = std::min(k * stride, n - 1);
        sx[k] = fx[idx]; sy[k] = fy[idx]; sz[k] = fz[idx];
    }

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        int i1 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(n)));
        int i2 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(n)));
        int i3 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(n)));
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if (!compute_plane_coeffs(fx[i1], fy[i1], fz[i1],
                                 fx[i2], fy[i2], fz[i2],
                                 fx[i3], fy[i3], fz[i3], cand_model)) continue;

        if (ground_normal_prior != nullptr) {
            float dot = std::abs(cand_model[0] * ground_normal_prior[0] +
                                 cand_model[1] * ground_normal_prior[1] +
                                 cand_model[2] * ground_normal_prior[2]);
            if (dot < min_ground_dot) continue;
        }

        float a = cand_model[0], b = cand_model[1], c = cand_model[2], d = cand_model[3];
        int sample_inliers = 0;

#if defined(__riscv) || defined(__riscv_vector)
        size_t si = 0;
        bool reject = false;
        while (si < sample_sz) {
            size_t chunk_limit = std::min(sample_sz - si, static_cast<size_t>(256));
            size_t chunk_end = si + chunk_limit;
            while (si < chunk_end) {
                size_t vl = __riscv_vsetvl_e32m8(chunk_end - si);
                vfloat32m8_t vx = __riscv_vle32_v_f32m8(&sx[si], vl);
                vfloat32m8_t vy = __riscv_vle32_v_f32m8(&sy[si], vl);
                vfloat32m8_t vz = __riscv_vle32_v_f32m8(&sz[si], vl);
                vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
                dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
                dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
                dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
                vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
                vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
                vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
                sample_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
                si += vl;
            }
            if (si == 256 && best_sample_inliers > 50) {
                int min_expected = (best_sample_inliers * 256) / static_cast<int>(sample_sz) / 3;
                if (sample_inliers < min_expected) {
                    reject = true;
                    break;
                }
            }
        }
        if (reject) continue;
#else
        for (size_t pt = 0; pt < sample_sz; ++pt) {
            float dist = std::abs(a * sx[pt] + b * sy[pt] + c * sz[pt] + d);
            if (dist <= dist_thresh) sample_inliers++;
            if (pt == 256 && best_sample_inliers > 50) {
                int min_expected = (best_sample_inliers * 256) / static_cast<int>(sample_sz) / 3;
                if (sample_inliers < min_expected) break;
            }
        }
#endif

        if (sample_inliers > best_sample_inliers) {
            best_sample_inliers = sample_inliers;
            for (int k = 0; k < 4; k++) best_model[k] = cand_model[k];
            double w = static_cast<double>(best_sample_inliers) / static_cast<double>(sample_sz);
            double p_no_outliers = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) k_iters = dynamic_k;
            }
        }
    }

    if (best_sample_inliers >= 10) {
        float a0 = best_model[0], b0 = best_model[1], c0 = best_model[2], d0 = best_model[3];
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int inlier_cnt = 0;
        for (size_t k = 0; k < sample_sz; ++k) {
            float dist = std::abs(a0 * sx[k] + b0 * sy[k] + c0 * sz[k] + d0);
            if (dist <= dist_thresh) {
                sum_x += sx[k]; sum_y += sy[k]; sum_z += sz[k];
                inlier_cnt++;
            }
        }
        if (inlier_cnt >= 10) {
            double cx = sum_x / inlier_cnt, cy = sum_y / inlier_cnt, cz = sum_z / inlier_cnt;
            double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (size_t k = 0; k < sample_sz; ++k) {
                float dist = std::abs(a0 * sx[k] + b0 * sy[k] + c0 * sz[k] + d0);
                if (dist <= dist_thresh) {
                    double ddx = sx[k] - cx, ddy = sy[k] - cy, ddz = sz[k] - cz;
                    c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                    c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
                }
            }

            double vx = a0, vy = b0, vz = c0;
            for (int iter = 0; iter < 10; ++iter) {
                double rx = c00*vx + c01*vy + c02*vz;
                double ry = c01*vx + c11*vy + c12*vz;
                double rz = c02*vx + c12*vy + c22*vz;
                double trace = c00 + c11 + c22;
                double shift_x = trace*vx - rx;
                double shift_y = trace*vy - ry;
                double shift_z = trace*vz - rz;
                double s_norm = std::sqrt(shift_x*shift_x + shift_y*shift_y + shift_z*shift_z);
                if (s_norm > 1e-6) {
                    vx = shift_x / s_norm; vy = shift_y / s_norm; vz = shift_z / s_norm;
                }
            }
            if (ground_normal_prior != nullptr) {
                double dot = vx*ground_normal_prior[0] + vy*ground_normal_prior[1] + vz*ground_normal_prior[2];
                if (dot < 0) { vx = -vx; vy = -vy; vz = -vz; }
            }
            best_model[0] = static_cast<float>(vx);
            best_model[1] = static_cast<float>(vy);
            best_model[2] = static_cast<float>(vz);
            best_model[3] = static_cast<float>(-(vx*cx + vy*cy + vz*cz));
        }
    }

    for (int k = 0; k < 4; ++k) model[k] = best_model[k];

    ctx.ox.clear(); ctx.oy.clear(); ctx.oz.clear();
    ctx.ox.reserve(n); ctx.oy.reserve(n); ctx.oz.reserve(n);
    float a = model[0], b = model[1], c = model[2], d = model[3];
    int total_inliers = 0;

#if defined(__riscv) || defined(__riscv_vector)
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(fx + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(fy + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(fz + i, vl);

        vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
        dist = __riscv_vfadd_vf_f32m8(dist, d, vl);

        vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
        vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
        vbool4_t inlier_mask = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
        uint8_t mask_bytes[64];
        __riscv_vsm_v_b4(mask_bytes, inlier_mask, vl);

        for (size_t lane = 0; lane < vl; ++lane) {
            bool is_in = (mask_bytes[lane >> 3] >> (lane & 7u)) & 1u;
            if (is_in) {
                total_inliers++;
            } else {
                ctx.ox.push_back(fx[i + lane]);
                ctx.oy.push_back(fy[i + lane]);
                ctx.oz.push_back(fz[i + lane]);
            }
        }
        i += vl;
    }
#else
    for (size_t i = 0; i < n; ++i) {
        float dist = std::abs(a * fx[i] + b * fy[i] + c * fz[i] + d);
        if (dist <= dist_thresh) {
            total_inliers++;
        } else {
            ctx.ox.push_back(fx[i]);
            ctx.oy.push_back(fy[i]);
            ctx.oz.push_back(fz[i]);
        }
    }
#endif
    return total_inliers;
}

// Hardware RVV 1.0 Vectorized Euclidean Clustering for Continuous Stream
static size_t execute_clustering_ctx(
    ThreadFrameContext& ctx, size_t n, float tolerance, int min_cluster_size, int max_cluster_size)
{
    ctx.clusters.clear();
    if (n == 0) return 0;

    float tol_sq = tolerance * tolerance;
    ctx.cluster_grid.build(ctx.ox.data(), ctx.oy.data(), ctx.oz.data(), n);

    const size_t mask = ctx.cluster_grid.mask_;
    const auto& cells = ctx.cluster_grid.cells_;
    const auto& next = ctx.cluster_grid.next_;
    const float* ox = ctx.ox.data();
    const float* oy = ctx.oy.data();
    const float* oz = ctx.oz.data();
    const auto& touched = ctx.cluster_grid.touched_slots_;

    ctx.uf.reset(n);

    static const int kForwardOffsets[13][3] = {
        {-1, -1, 1}, { 0, -1, 1}, { 1, -1, 1},
        {-1,  0, 1}, { 0,  0, 1}, { 1,  0, 1},
        {-1,  1, 1}, { 0,  1, 1}, { 1,  1, 1},
        {-1,  1, 0}, { 0,  1, 0}, { 1,  1, 0},
        { 1,  0, 0}
    };

    std::vector<int> self_pts;
    self_pts.reserve(64);

    for (uint32_t slot : touched) {
        const auto& cell = cells[slot];
        if (cell.head == -1) continue;

        self_pts.clear();
        int curr = cell.head;
        while (curr != -1) {
            self_pts.push_back(curr);
            curr = next[curr];
        }

        size_t n_self = self_pts.size();

        // 1. Intra-cell pairwise checks
        for (size_t u = 0; u < n_self; ++u) {
            int p_u = self_pts[u];
            float ux = ox[p_u], uy = oy[p_u], uz = oz[p_u];
            for (size_t v = u + 1; v < n_self; ++v) {
                int p_v = self_pts[v];
                float ddx = ox[p_v] - ux, ddy = oy[p_v] - uy, ddz = oz[p_v] - uz;
                if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                    ctx.uf.unite(p_u, p_v);
                }
            }
        }

        // 2. Gather candidates from 13 forward neighbors ONCE for this cell
        ctx.cand_idx.clear();
        ctx.cand_x.clear();
        ctx.cand_y.clear();
        ctx.cand_z.clear();

        for (int k = 0; k < 13; ++k) {
            int tcx = cell.cx + kForwardOffsets[k][0];
            int tcy = cell.cy + kForwardOffsets[k][1];
            int tcz = cell.cz + kForwardOffsets[k][2];

            size_t h = ctx.cluster_grid.hash3D(tcx, tcy, tcz);
            int probe = 0;
            while (cells[h].head != -1 && probe < FastStreamGrid::kMaxProbes) {
                if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                    int c_nbr = cells[h].head;
                    while (c_nbr != -1) {
                        ctx.cand_idx.push_back(c_nbr);
                        ctx.cand_x.push_back(ox[c_nbr]);
                        ctx.cand_y.push_back(oy[c_nbr]);
                        ctx.cand_z.push_back(oz[c_nbr]);
                        c_nbr = next[c_nbr];
                    }
                    break;
                }
                h = (h + 1) & mask;
                probe++;
            }
        }

        // 3. Vectorized distance check
        size_t M = ctx.cand_idx.size();
        if (M > 0) {
            for (size_t u = 0; u < n_self; ++u) {
                int p_u = self_pts[u];
                float qx = ox[p_u], qy = oy[p_u], qz = oz[p_u];

#if defined(__riscv) || defined(__riscv_vector)
                size_t k = 0;
                while (k < M) {
                    size_t vl = __riscv_vsetvl_e32m8(M - k);
                    vfloat32m8_t vx = __riscv_vle32_v_f32m8(&ctx.cand_x[k], vl);
                    vfloat32m8_t vy = __riscv_vle32_v_f32m8(&ctx.cand_y[k], vl);
                    vfloat32m8_t vz = __riscv_vle32_v_f32m8(&ctx.cand_z[k], vl);

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
                                ctx.uf.unite(p_u, ctx.cand_idx[k + lane]);
                            }
                        }
                    }
                    k += vl;
                }
#else
                for (size_t k = 0; k < M; ++k) {
                    float ddx = ctx.cand_x[k] - qx;
                    float ddy = ctx.cand_y[k] - qy;
                    float ddz = ctx.cand_z[k] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= tol_sq) {
                        ctx.uf.unite(p_u, ctx.cand_idx[k]);
                    }
                }
#endif
            }
        }
    }

    if (ctx.root_counts.size() < n) ctx.root_counts.resize(n);
    if (ctx.root_to_cid.size() < n) ctx.root_to_cid.resize(n);
    std::fill(ctx.root_counts.begin(), ctx.root_counts.begin() + n, 0);
    std::fill(ctx.root_to_cid.begin(), ctx.root_to_cid.begin() + n, -1);

    for (size_t i = 0; i < n; ++i) {
        ctx.root_counts[ctx.uf.find(static_cast<int>(i))]++;
    }

    int num_valid = 0;
    for (size_t r = 0; r < n; ++r) {
        int sz = ctx.root_counts[r];
        if (sz >= min_cluster_size && sz <= max_cluster_size) {
            ctx.root_to_cid[r] = num_valid++;
        }
    }

    ctx.clusters.resize(num_valid);
    for (size_t r = 0; r < n; ++r) {
        int cid = ctx.root_to_cid[r];
        if (cid != -1) {
            ctx.clusters[cid].indices.clear();
            ctx.clusters[cid].indices.reserve(ctx.root_counts[r]);
        }
    }

    for (size_t i = 0; i < n; ++i) {
        int r = ctx.uf.find(static_cast<int>(i));
        int cid = ctx.root_to_cid[r];
        if (cid != -1) {
            ctx.clusters[cid].indices.push_back(static_cast<int>(i));
        }
    }

    return ctx.clusters.size();
}

struct RGBColor { uint8_t r, g, b; };
static std::vector<RGBColor> generateClusterColors(size_t count) {
    std::vector<RGBColor> colors(count);
    for (size_t i = 0; i < count; ++i) {
        float hue = std::fmod(i * 0.618033988749895f, 1.0f);
        float c = 0.95f * 0.85f;
        float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
        float m = 0.95f - c;
        float r = 0, g = 0, b = 0;
        if (hue < 1.0f/6.0f) { r = c; g = x; b = 0; }
        else if (hue < 2.0f/6.0f) { r = x; g = c; b = 0; }
        else if (hue < 3.0f/6.0f) { r = 0; g = c; b = x; }
        else if (hue < 4.0f/6.0f) { r = 0; g = x; b = c; }
        else if (hue < 5.0f/6.0f) { r = x; g = 0; b = c; }
        else { r = c; g = 0; b = x; }
        colors[i] = {
            static_cast<uint8_t>((r + m) * 255.0f),
            static_cast<uint8_t>((g + m) * 255.0f),
            static_cast<uint8_t>((b + m) * 255.0f)
        };
    }
    return colors;
}

static bool export_clusters_pcd(
    const std::string& out_filepath, const ThreadFrameContext& ctx)
{
    if (ctx.clusters.empty()) return true;
    std::filesystem::create_directories(std::filesystem::path(out_filepath).parent_path());
    auto colors = generateClusterColors(ctx.clusters.size());
    std::vector<PointXYZRGB> colored_pts;
    size_t total_pts = 0;
    for (const auto& cl : ctx.clusters) total_pts += cl.indices.size();
    colored_pts.reserve(total_pts);

    for (size_t c_idx = 0; c_idx < ctx.clusters.size(); ++c_idx) {
        const auto& col = colors[c_idx];
        for (int idx : ctx.clusters[c_idx].indices) {
            if (idx >= 0 && static_cast<size_t>(idx) < ctx.ox.size()) {
                colored_pts.push_back({ctx.ox[idx], ctx.oy[idx], ctx.oz[idx], col.r, col.g, col.b});
            }
        }
    }
    return savePCDRGB(out_filepath, colored_pts, true);
}

} // anonymous namespace

int main(int argc, char** argv) {
    StreamConfig cfg;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--max-frames" && i + 1 < argc) {
            cfg.max_frames = std::atoi(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            cfg.num_threads = std::atoi(argv[++i]);
        } else if (arg == "--leaf-size" && i + 1 < argc) {
            cfg.voxel_leaf_size = std::atof(argv[++i]);
        } else if (arg == "--cluster-tolerance" && i + 1 < argc) {
            cfg.cluster_tolerance = std::atof(argv[++i]);
        } else if (arg == "--min-cluster" && i + 1 < argc) {
            cfg.min_cluster_size = std::atoi(argv[++i]);
        } else if (arg == "--max-cluster" && i + 1 < argc) {
            cfg.max_cluster_size = std::atoi(argv[++i]);
        } else if (arg == "--write-clusters") {
            cfg.write_clusters = true;
        } else if (arg == "--no-write") {
            cfg.write_clusters = false;
        } else if ((arg == "--mode" || arg == "--stream-mode") && i + 1 < argc) {
            std::string mode_str = argv[++i];
            if (mode_str == "intra" || mode_str == "single") {
                cfg.mode = "intra";
            } else {
                cfg.mode = "inter";
            }
        } else if (arg == "--inter" || arg == "--parallel-frames") {
            cfg.mode = "inter";
        } else if (arg == "--intra") {
            cfg.mode = "intra";
        } else if (arg == "--progress") {
            cfg.progress = true;
        } else if (arg == "--no-progress") {
            cfg.progress = false;
        } else if (arg[0] != '-') {
            cfg.input_path = arg;
        }
    }

#if defined(_OPENMP)
    omp_set_num_threads(cfg.num_threads);
#endif

    std::vector<std::string> pcd_files;
    if (std::filesystem::is_directory(cfg.input_path)) {
        for (const auto& entry : std::filesystem::directory_iterator(cfg.input_path)) {
            if (entry.is_regular_file() && entry.path().extension() == ".pcd") {
                pcd_files.push_back(entry.path().string());
            }
        }
        std::sort(pcd_files.begin(), pcd_files.end());
    } else if (std::filesystem::is_regular_file(cfg.input_path)) {
        pcd_files.push_back(cfg.input_path);
    } else {
        std::cerr << "Error: Invalid input path: " << cfg.input_path << std::endl;
        return 1;
    }

    if (pcd_files.empty()) {
        std::cerr << "Error: No .pcd files found in: " << cfg.input_path << std::endl;
        return 1;
    }

    if (cfg.max_frames > 0 && pcd_files.size() > static_cast<size_t>(cfg.max_frames)) {
        pcd_files.resize(cfg.max_frames);
    }

    if (cfg.write_clusters) {
        std::filesystem::create_directories("output/stream_clusters");
    }

    std::cout << "========================================================================\n"
              << "  RVPoint Multi-Core Continuous Stream Pipeline (RVV 1.0 Clustering Test)\n"
              << "  Target Architecture: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)\n"
              << "========================================================================\n"
              << "  Input Directory : " << cfg.input_path << " (" << pcd_files.size() << " frames)\n"
              << "  Streaming Mode  : " << (cfg.mode == "inter" ? "INTER-FRAME (Asynchronous Multi-Core Stream Pool)" : "INTRA-FRAME (Low Latency)") << "\n"
              << "  Worker Threads  : " << cfg.num_threads << "\n"
              << "  Voxel Leaf Size : " << cfg.voxel_leaf_size << " m\n"
              << "  Cluster Tol     : " << cfg.cluster_tolerance << " m (min: " << cfg.min_cluster_size << ")\n"
              << "  Cluster Export  : " << (cfg.write_clusters ? "ENABLED (output/stream_clusters/)" : "DISABLED (Zero Disk I/O)") << "\n"
              << "========================================================================\n\n";

    const int pool_threads = std::max(1, cfg.num_threads);
    std::vector<ThreadFrameContext> thread_contexts;
    thread_contexts.reserve(pool_threads);
    for (int t = 0; t < pool_threads; ++t) {
        thread_contexts.emplace_back(cfg.voxel_leaf_size, cfg.ror_radius, cfg.cluster_tolerance);
    }

    std::vector<FrameMetrics> all_metrics(pcd_files.size());
    const std::vector<float> ground_normal_prior = {0.0f, 0.0f, 1.0f};
    const float min_ground_dot = 0.707f;

    auto stream_start_wall = Clock::now();

    auto process_single_frame = [&](size_t f_idx, int thread_id) {
        ThreadFrameContext& ctx = thread_contexts[thread_id];
        FrameMetrics& m = all_metrics[f_idx];
        m.frame_id = static_cast<int>(f_idx);
        m.filename = std::filesystem::path(pcd_files[f_idx]).filename().string();

        auto t_read_start = Clock::now();
        int64_t load_res = loadPCD(pcd_files[f_idx], ctx.raw_pts);
        auto t_read_end = Clock::now();
        m.io_read_ms = std::chrono::duration<double, std::milli>(t_read_end - t_read_start).count();

        if (load_res <= 0 || ctx.raw_pts.empty()) {
            std::cerr << "[Stream Warning] Frame " << f_idx << " failed to load: " << pcd_files[f_idx] << std::endl;
            return;
        }
        size_t n_raw = ctx.raw_pts.size();
        m.raw_points = n_raw;

        if (ctx.rx.size() < n_raw) {
            ctx.rx.resize(n_raw); ctx.ry.resize(n_raw); ctx.rz.resize(n_raw);
        }
        for (size_t i = 0; i < n_raw; ++i) {
            ctx.rx[i] = ctx.raw_pts[i].x;
            ctx.ry[i] = ctx.raw_pts[i].y;
            ctx.rz[i] = ctx.raw_pts[i].z;
        }

        auto t_comp_start = Clock::now();

        auto t1 = Clock::now();
        size_t n_down = execute_fast_radix_voxel_downsample_ctx(ctx, n_raw, cfg.voxel_leaf_size);
        auto t2 = Clock::now();
        m.voxel_ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
        m.downsampled_points = n_down;

        auto t3 = Clock::now();
        ctx.ror_grid.build(ctx.dx.data(), ctx.dy.data(), ctx.dz.data(), n_down);
        auto t4 = Clock::now();
        m.grid_build_ms = std::chrono::duration<double, std::milli>(t4 - t3).count();

        auto t5 = Clock::now();
        size_t n_filtered = execute_ror_rvv_ctx(ctx, n_down, cfg.ror_radius, cfg.ror_min_neighbors);
        auto t6 = Clock::now();
        m.ror_ms = std::chrono::duration<double, std::milli>(t6 - t5).count();
        m.ror_points = n_filtered;

        auto t7 = Clock::now();
        float plane_model[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        int inliers = ransac_plane_sprt_rvv_ctx(ctx, n_filtered, cfg.ransac_distance_threshold,
                                               cfg.ransac_max_iterations, plane_model,
                                               ground_normal_prior.data(), min_ground_dot, 42);
        auto t8 = Clock::now();
        m.ransac_ms = std::chrono::duration<double, std::milli>(t8 - t7).count();
        m.ground_inliers = inliers;
        m.obstacle_points = ctx.ox.size();

        auto t9 = Clock::now();
        size_t n_clusters = execute_clustering_ctx(ctx, ctx.ox.size(), cfg.cluster_tolerance, cfg.min_cluster_size, cfg.max_cluster_size);
        auto t10 = Clock::now();
        m.cluster_ms = std::chrono::duration<double, std::milli>(t10 - t9).count();
        m.cluster_count = n_clusters;

        auto t_comp_end = Clock::now();
        m.compute_ms = std::chrono::duration<double, std::milli>(t_comp_end - t_comp_start).count();

        if (cfg.write_clusters) {
            auto t_write_start = Clock::now();
            std::stringstream ss;
            ss << "output/stream_clusters/frame_" << std::setfill('0') << std::setw(6) << f_idx << "_clusters.pcd";
            export_clusters_pcd(ss.str(), ctx);
            auto t_write_end = Clock::now();
            m.io_write_ms = std::chrono::duration<double, std::milli>(t_write_end - t_write_start).count();
        }

        if (cfg.progress) {
            double fps = (m.compute_ms > 0.0) ? (1000.0 / m.compute_ms) : 0.0;
            #pragma omp critical
            {
                std::cout << "[stream-rvv] [Core " << thread_id << "] Frame " << std::setw(3) << f_idx << " (" << m.filename << ") | "
                          << "Compute: " << std::fixed << std::setprecision(2) << std::setw(6) << m.compute_ms << " ms ("
                          << std::setprecision(1) << std::setw(4) << fps << " FPS) | "
                          << "Clust: " << std::setw(5) << m.cluster_ms << " ms | "
                          << "Pts: " << std::setw(6) << m.raw_points << " -> " << std::setw(5) << m.downsampled_points << " | "
                          << "Obstacles: " << std::setw(5) << m.obstacle_points << " | "
                          << "Clusters: " << std::setw(2) << m.cluster_count << std::endl;
            }
        }
    };

    if (cfg.mode == "inter") {
#if defined(_OPENMP)
        #pragma omp parallel for schedule(dynamic, 1)
#endif
        for (size_t f_idx = 0; f_idx < pcd_files.size(); ++f_idx) {
            int tid = 0;
#if defined(_OPENMP)
            tid = omp_get_thread_num();
#endif
            process_single_frame(f_idx, tid);
        }
    } else {
        for (size_t f_idx = 0; f_idx < pcd_files.size(); ++f_idx) {
            process_single_frame(f_idx, 0);
        }
    }

    auto stream_end_wall = Clock::now();
    double total_wall_ms = std::chrono::duration<double, std::milli>(stream_end_wall - stream_start_wall).count();

    if (all_metrics.empty()) {
        std::cerr << "Error: No frames processed successfully." << std::endl;
        return 1;
    }

    double sum_comp = 0, min_comp = 1e9, max_comp = 0;
    double sum_read = 0, sum_write = 0;
    double sum_vox = 0, sum_grid = 0, sum_ror = 0, sum_ransac = 0, sum_clust = 0;

    for (const auto& m : all_metrics) {
        sum_comp += m.compute_ms;
        min_comp = std::min(min_comp, m.compute_ms);
        max_comp = std::max(max_comp, m.compute_ms);
        sum_read += m.io_read_ms;
        sum_write += m.io_write_ms;
        sum_vox += m.voxel_ms;
        sum_grid += m.grid_build_ms;
        sum_ror += m.ror_ms;
        sum_ransac += m.ransac_ms;
        sum_clust += m.cluster_ms;
    }

    size_t count = all_metrics.size();
    double avg_comp = sum_comp / count;
    double avg_read = sum_read / count;
    double avg_write = sum_write / count;
    double effective_fps = (avg_comp > 0.0) ? (1000.0 / avg_comp) : 0.0;
    double stream_wall_fps = (total_wall_ms > 0.0) ? (count * 1000.0 / total_wall_ms) : 0.0;

    std::cout << "\n========================================================================\n"
              << "  CONTINUOUS STREAM PERCEPTION BENCHMARK SUMMARY (RVV CLUSTERING) (" << count << " frames)\n"
              << "========================================================================\n"
              << "  [Overall Stream Throughput & Wall-Clock Runtime]:\n"
              << "    • Total Wall-Clock Time   : " << std::fixed << std::setprecision(2) << total_wall_ms << " ms\n"
              << "    • Sustained Stream Rate   : " << std::setprecision(2) << stream_wall_fps << " FPS (" << (cfg.mode == "inter" ? "Asynchronous Worker Pool" : "Single Stream") << ")\n"
              << "------------------------------------------------------------------------\n"
              << "  [Pure Compute Pipeline Latency] (Excluding all Disk I/O):\n"
              << "    • Average Compute Latency : " << std::fixed << std::setprecision(2) << avg_comp << " ms\n"
              << "    • Min Compute Latency     : " << min_comp << " ms\n"
              << "    • Max Compute Latency     : " << max_comp << " ms\n"
              << "    • Per-Frame Compute Rate  : " << std::setprecision(2) << effective_fps << " FPS\n"
              << "------------------------------------------------------------------------\n"
              << "  [Compute Stage Breakdown (Average per Frame)]:\n"
              << "    1. Voxel Downsample       : " << std::setw(6) << (sum_vox / count) << " ms ("
              << std::setprecision(1) << ((sum_vox / sum_comp) * 100.0) << "%)\n"
              << "    2. Fast Spatial Grid Build: " << std::setw(6) << (sum_grid / count) << " ms ("
              << std::setprecision(1) << ((sum_grid / sum_comp) * 100.0) << "%)\n"
              << "    3. RVV Radius Outlier Rem : " << std::setw(6) << (sum_ror / count) << " ms ("
              << std::setprecision(1) << ((sum_ror / sum_comp) * 100.0) << "%)\n"
              << "    4. RVV RANSAC Ground Fit  : " << std::setw(6) << (sum_ransac / count) << " ms ("
              << std::setprecision(1) << ((sum_ransac / sum_comp) * 100.0) << "%)\n"
              << "    5. Spatial Clustering     : " << std::setw(6) << (sum_clust / count) << " ms ("
              << std::setprecision(1) << ((sum_clust / sum_comp) * 100.0) << "%)\n"
              << "------------------------------------------------------------------------\n"
              << "  [Disk I/O Telemetry (Isolated from Compute)]:\n"
              << "    • Average PCD Load Time   : " << avg_read << " ms\n"
              << "    • Average Cluster Save    : " << avg_write << " ms\n"
              << "========================================================================\n";

    return 0;
}
