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
#include <random>
#include <unordered_map>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ============================================================================
// Fast 3D Spatial Grid
// ============================================================================
class Fast3DSpatialGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct Cell {
        int cx = -999999, cy = -999999, cz = -999999;
        int head = -1;
    };

    float cell_size_;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    Fast3DSpatialGrid(float cell_size = 0.25f) : cell_size_(cell_size), inv_cell_(1.0f / cell_size) {
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
        next_.assign(n, -1);

        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) {
                h = (h + 1) & kMask;
            }
            if (cells_[h].head == -1) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
            }
            next_[i] = cells_[h].head;
            cells_[h].head = static_cast<int>(i);
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

                    while (cells_[h].head != -1) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            int curr = cells_[h].head;
                            while (curr != -1) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    neighbors.push_back(curr);
                                    dists2.push_back(d2);
                                }
                                curr = next_[curr];
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
// 1. SOR + Normals: Current Turbo (Scalar) vs RVV Vectorized
// ============================================================================
struct FusedResult {
    std::vector<float> x, y, z;
    std::vector<float> nx, ny, nz;
};

// RVV Vectorized SOR + Normals
static FusedResult sor_normals_rvv_opt(
    const PointCloudSoA& cloud, const Fast3DSpatialGrid& grid,
    float sor_radius, int sor_k, float sor_alpha)
{
    const size_t n = cloud.n;
    std::vector<float> mean_dists(n, sor_radius);
    std::vector<float> all_nx(n, 0.0f), all_ny(n, 0.0f), all_nz(n, 1.0f);
    std::vector<int> valid_points;
    valid_points.reserve(n);

    std::vector<int> nbrs;
    std::vector<float> d2;
    nbrs.reserve(256); d2.reserve(256);

    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = sor_radius * sor_radius;

    for (size_t i = 0; i < n; ++i) {
        float qx = cloud.x[i], qy = cloud.y[i], qz = cloud.z[i];
        grid.radiusSearch(qx, qy, qz, sor_r2, nbrs, d2);

        int found = static_cast<int>(nbrs.size());
        if (found < 2) continue;

        int k_use = std::min(found - 1, sor_k);
        float sum_dist = 0.0f;

#if defined(__riscv) || defined(__riscv_vector)
        size_t vl = __riscv_vsetvl_e32m4(k_use);
        vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(d2.data() + 1, vl);
        vfloat32m4_t vd  = __riscv_vfsqrt_v_f32m4(vd2, vl);
        vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
        vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
        sum_dist = __riscv_vfmv_f_s_f32m1_f32(v_sum);
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
                cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
            }
            float inv_n = 1.0f / static_cast<float>(n_norm);
            cx *= inv_n; cy *= inv_n; cz *= inv_n;

            float c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (int k = 0; k < n_norm; ++k) {
                int idx = nbrs[k];
                float dx = cloud.x[idx] - cx, dy = cloud.y[idx] - cy, dz = cloud.z[idx] - cz;
                c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
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

    FusedResult res;
    if (valid_points.empty()) return res;

    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double variance = (total_sq_sum / d_count) - (global_mean * global_mean);
    double stddev = std::sqrt(std::max(0.0, variance));
    float thresh = static_cast<float>(global_mean + sor_alpha * stddev);

    res.x.reserve(valid_points.size());
    res.y.reserve(valid_points.size());
    res.z.reserve(valid_points.size());
    res.nx.reserve(valid_points.size());
    res.ny.reserve(valid_points.size());
    res.nz.reserve(valid_points.size());

    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) {
            res.x.push_back(cloud.x[idx]);
            res.y.push_back(cloud.y[idx]);
            res.z.push_back(cloud.z[idx]);
            res.nx.push_back(all_nx[idx]);
            res.ny.push_back(all_ny[idx]);
            res.nz.push_back(all_nz[idx]);
        }
    }
    return res;
}

// ============================================================================
// 2. RANSAC: SPRT Early-Rejection RVV
// ============================================================================
static bool compute_plane_coeffs(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 float x3, float y3, float z3,
                                 float* model, float collinear_thresh = 1e-4f) 
{
    float v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    float v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < collinear_thresh) return false;
    a /= norm; b /= norm; c /= norm;
    model[0] = a; model[1] = b; model[2] = c;
    model[3] = -(a*x1 + b*y1 + c*z1);
    return true;
}

int ransac_sprt_rvv(const PointCloudSoA& cloud, float dist_thresh, int max_iters, float* model) {
    if (cloud.n < 3) return 0;
    std::srand(0);
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);

    const size_t subsample_sz = std::min(cloud.n, static_cast<size_t>(512));

    for(int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        int i1 = std::rand() % cloud.n;
        int i2 = std::rand() % cloud.n;
        int i3 = std::rand() % cloud.n;
        if(i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if(!compute_plane_coeffs(cloud.x[i1], cloud.y[i1], cloud.z[i1],
                                 cloud.x[i2], cloud.y[i2], cloud.z[i2],
                                 cloud.x[i3], cloud.y[i3], cloud.z[i3], cand_model)) continue;

        if (std::abs(cand_model[2]) < 0.70f) continue;

        float a = cand_model[0], b = cand_model[1], c = cand_model[2], d = cand_model[3];

#if defined(__riscv) || defined(__riscv_vector)
        int pre_inliers = 0;
        size_t si = 0;
        while (si < subsample_sz) {
            size_t vl = __riscv_vsetvl_e32m8(subsample_sz - si);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[si], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[si], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[si], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            pre_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            si += vl;
        }

        if (best_inliers > 0) {
            float min_expected_ratio = static_cast<float>(best_inliers) / static_cast<float>(cloud.n) * 0.50f;
            if (static_cast<float>(pre_inliers) / static_cast<float>(subsample_sz) < min_expected_ratio) {
                continue;
            }
        }

        int current_inliers = pre_inliers;
        size_t n = cloud.n, i = subsample_sz;
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - i);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            current_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            i += vl;
        }
#endif

        if(current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for(int k=0; k<4; k++) best_model[k] = cand_model[k];
            double w = static_cast<double>(best_inliers) / static_cast<double>(cloud.n);
            double p_no_outliers = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) k_iters = dynamic_k;
            }
        }
    }
    for(int k=0; k<4; k++) model[k] = best_model[k];
    return best_inliers;
}

// Extract outliers directly to SoA (Zero-Copy)
void extract_outliers_soa_rvv(const PointCloudSoA& in, const float* model, float thresh,
                              std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    ox.clear(); oy.clear(); oz.clear();
    ox.reserve(in.n); oy.reserve(in.n); oz.reserve(in.n);

    for (size_t i = 0; i < in.n; ++i) {
        float dist = std::abs(a * in.x[i] + b * in.y[i] + c * in.z[i] + d);
        if (dist > thresh) {
            ox.push_back(in.x[i]);
            oy.push_back(in.y[i]);
            oz.push_back(in.z[i]);
        }
    }
}

// ============================================================================
// 3. CLUSTERING: Disjoint Set Union Find
// ============================================================================
struct DisjointSet {
    std::vector<int> parent;
    DisjointSet(int n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int i) { return (parent[i] == i) ? i : (parent[i] = find(parent[i])); }
    void unite(int i, int j) {
        int root_i = find(i), root_j = find(j);
        if (root_i != root_j) parent[root_i] = root_j;
    }
};

static std::vector<ClusterIndices> execute_union_find_clustering(
    const PointCloudSoA& cloud, float cluster_tol, int min_sz, int max_sz)
{
    const size_t n = cloud.n;
    if (n == 0) return {};

    Fast3DSpatialGrid grid(cluster_tol);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    DisjointSet ds(static_cast<int>(n));
    float tol_sq = cluster_tol * cluster_tol;

    std::vector<int> nbrs;
    std::vector<float> d2;
    nbrs.reserve(64); d2.reserve(64);

    for (size_t i = 0; i < n; ++i) {
        grid.radiusSearch(cloud.x[i], cloud.y[i], cloud.z[i], tol_sq, nbrs, d2);
        for (int nb : nbrs) ds.unite(static_cast<int>(i), nb);
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
    std::cout << " COMPLETE END-TO-END PIPELINE OPTIMIZATION BENCHMARK" << std::endl;
    std::cout << "================================================================" << std::endl;

    std::vector<PointXYZ> raw_pts;
    auto t0 = Clock::now();
    loadPCD(pcd_path, raw_pts);
    double load_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // Pure SoA Downsampling
    std::vector<float> rx(raw_pts.size()), ry(raw_pts.size()), rz(raw_pts.size());
    for (size_t i = 0; i < raw_pts.size(); ++i) {
        rx[i] = raw_pts[i].x; ry[i] = raw_pts[i].y; rz[i] = raw_pts[i].z;
    }
    PointCloudSoA raw_cloud{rx.data(), ry.data(), rz.data(), raw_pts.size()};

    std::vector<PointXYZ> down_pts(raw_pts.size());
    t0 = Clock::now();
    size_t n_down = voxel_grid_downsamp_rvv_v2(raw_cloud, down_pts.data(), 0.10f);
    double down_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) {
        dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z;
    }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // Stage 4: Grid Build
    t0 = Clock::now();
    Fast3DSpatialGrid grid(0.25f);
    grid.build(dx.data(), dy.data(), dz.data(), n_down);
    double grid4_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // Stage 5,6,7: Fused SOR + Normals (RVV)
    t0 = Clock::now();
    auto fused = sor_normals_rvv_opt(down_cloud, grid, 0.25f, 20, 1.0f);
    double sor_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // Stage 8: SPRT RVV RANSAC + Pure SoA Outliers
    t0 = Clock::now();
    PointCloudSoA sor_cloud{fused.x.data(), fused.y.data(), fused.z.data(), fused.x.size()};
    float model[4] = {0};
    int inliers = ransac_sprt_rvv(sor_cloud, 0.20f, 1000, model);
    std::vector<float> ox, oy, oz;
    extract_outliers_soa_rvv(sor_cloud, model, 0.20f, ox, oy, oz);
    double ransac_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    // Stage 9: Union-Find Clustering
    t0 = Clock::now();
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), ox.size()};
    auto clusters = execute_union_find_clustering(non_ground_cloud, 0.15f, 50, 100000);
    double cluster_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

    double total_compute_ms = down_ms + grid4_ms + sor_ms + ransac_ms + cluster_ms;

    std::cout << "\n--- END-TO-END TIMING BREAKDOWN ---" << std::endl;
    std::cout << "  Load PCD (Disk I/O)         : " << std::fixed << std::setprecision(2) << load_ms << " ms" << std::endl;
    std::cout << "  [Stage 3] RVV Downsampling  : " << down_ms << " ms (" << n_down << " pts)" << std::endl;
    std::cout << "  [Stage 4] Grid Build        : " << grid4_ms << " ms" << std::endl;
    std::cout << "  [Stage 5-7] Fused SOR+Normal: " << sor_ms << " ms (" << fused.x.size() << " pts)" << std::endl;
    std::cout << "  [Stage 8] SPRT RVV RANSAC   : " << ransac_ms << " ms (Inliers: " << inliers << ", Outliers: " << ox.size() << ")" << std::endl;
    std::cout << "  [Stage 9] Union-Find Cluster: " << cluster_ms << " ms (" << clusters.size() << " clusters)" << std::endl;
    std::cout << "----------------------------------------------------------------" << std::endl;
    std::cout << "  Total Compute Time (Stages 3-9): " << total_compute_ms << " ms" << std::endl;
    std::cout << "================================================================" << std::endl;

    return 0;
}
