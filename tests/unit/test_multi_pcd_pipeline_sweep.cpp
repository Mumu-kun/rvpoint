// test_multi_pcd_pipeline_sweep.cpp
// Multi-PCD Frame Algorithmic Benchmark Sweep (Pure Compute, Zero Disk I/O)
// Compares:
// 1. Official PCL Reference (Scalar Octree + Scalar SOR + Scalar Normals + Scalar RANSAC + Scalar Clustering)
// 2. pipeline_export (PointerOctree + Vectorized SOR + Cardano Normals + RVV RANSAC + Octree Clustering)
// 3. pipeline_3d_turbo (Fast3DSpatialGrid + Fused SOR/Normals + RVV RANSAC + Union-Find Clustering)
// 4. pipeline_3d_ultra (Fast3DSpatialGrid + RVV vfsqrt + Strided RVV RANSAC + 14-Cell Half-Space Union-Find)

#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include "pointer_octree/pointer_octree.h"
#include "euclidean_clustering.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>
#include <unordered_map>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ============================================================================
// PIPELINE CONFIGURATION
// ============================================================================
struct Config {
    float leaf_size = 0.10f;
    float sor_radius = 0.25f;
    int sor_k = 20;
    float sor_std = 1.0f;
    float normal_radius = 0.03f;
    int normal_k = 10;
    float ransac_thresh = 0.20f;
    int ransac_max_iters = 1000;
    float cluster_tol = 0.15f;
    int min_cluster = 50;
    int max_cluster = 100000;
};
constexpr Config kCfg;

// ============================================================================
// 1. OFFICIAL PCL REFERENCE (PURE SCALAR OCTREE PIPELINE)
// ============================================================================
namespace pcl_ref {

size_t sor_octree_scalar(const PointCloudSoA &in, const Octree &tree, PointXYZ *out) {
    if (in.n == 0) return 0;
    std::vector<float> mean_dists(in.n);
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;

    for (size_t i = 0; i < in.n; ++i) {
        PointXYZ query = {in.x[i], in.y[i], in.z[i]};
        tree.radiusSearch(query, kCfg.sor_radius, nbr_indices, nbr_dists);
        if (nbr_dists.size() > 1) {
            std::sort(nbr_dists.begin(), nbr_dists.end());
            float sum = 0.0f;
            int valid_k = std::min(kCfg.sor_k, static_cast<int>(nbr_dists.size()) - 1);
            for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(nbr_dists[j]);
            mean_dists[i] = sum / valid_k;
        } else {
            mean_dists[i] = kCfg.sor_radius;
        }
    }

    float global_sum = 0.0f;
    for (float d : mean_dists) global_sum += d;
    float global_mean = global_sum / in.n;

    float variance_sum = 0.0f;
    for (float d : mean_dists) variance_sum += (d - global_mean) * (d - global_mean);
    float global_std = std::sqrt(variance_sum / in.n);
    float thresh = global_mean + kCfg.sor_std * global_std;

    size_t count = 0;
    for (size_t i = 0; i < in.n; ++i) {
        if (mean_dists[i] <= thresh) out[count++] = {in.x[i], in.y[i], in.z[i]};
    }
    return count;
}

double run(const std::vector<PointXYZ>& input_pts, size_t& out_down, size_t& out_sor, size_t& out_inliers, size_t& out_clusters) {
    auto t_start = Clock::now();

    // 1. Downsampling (Scalar)
    std::vector<PointXYZ> down_pts(input_pts.size());
    size_t n_down = voxel_grid_downsamp_sc(input_pts.data(), input_pts.size(), down_pts.data(), kCfg.leaf_size);
    out_down = n_down;

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) { dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z; }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // 2. Octree Search 1
    Octree tree1;
    tree1.setInputCloud(down_cloud);
    tree1.build();

    // 3. SOR
    std::vector<PointXYZ> sor_pts(n_down);
    size_t n_sor = sor_octree_scalar(down_cloud, tree1, sor_pts.data());
    out_sor = n_sor;

    std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
    for (size_t i = 0; i < n_sor; ++i) { sx[i] = sor_pts[i].x; sy[i] = sor_pts[i].y; sz[i] = sor_pts[i].z; }
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), n_sor};

    // 4. Octree Search 2
    Octree tree2;
    tree2.setInputCloud(sor_cloud);
    tree2.build();

    // 5. Normal Estimation (Cardano)
    std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
    std::vector<int> nbrs; std::vector<float> dists;
    for (size_t i = 0; i < n_sor; ++i) {
        tree2.radiusSearch({sx[i], sy[i], sz[i]}, kCfg.normal_radius, nbrs, dists);
        if (nbrs.size() >= 3) {
            float cx=0, cy=0, cz=0;
            for (int id : nbrs) { cx += sx[id]; cy += sy[id]; cz += sz[id]; }
            float inv = 1.0f / nbrs.size(); cx*=inv; cy*=inv; cz*=inv;
            float c00=0, c01=0, c02=0, c11=0, c12=0, c22=0;
            for (int id : nbrs) {
                float ddx = sx[id]-cx, ddy = sy[id]-cy, ddz = sz[id]-cz;
                c00 += ddx*ddx; c01 += ddx*ddy; c02 += ddx*ddz;
                c11 += ddy*ddy; c12 += ddy*ddz; c22 += ddz*ddz;
            }
            float vx = c01*c12 - c02*c11, vy = c01*c02 - c00*c12, vz = c00*c11 - c01*c01;
            float norm = std::sqrt(vx*vx + vy*vy + vz*vz);
            if (norm > 1e-6f) { nx[i] = vx/norm; ny[i] = vy/norm; nz[i] = vz/norm; }
            else { nx[i]=0; ny[i]=0; nz[i]=1; }
        } else { nx[i]=0; ny[i]=0; nz[i]=1; }
    }

    // 6. Scalar RANSAC
    float model[4] = {0};
    int r_inliers = ransac_plane_sc(sor_pts.data(), n_sor, kCfg.ransac_thresh, kCfg.ransac_max_iters, model);
    out_inliers = r_inliers > 0 ? r_inliers : 0;

    std::vector<PointXYZ> non_ground_pts;
    for (size_t i = 0; i < n_sor; ++i) {
        float dist = std::abs(model[0]*sx[i] + model[1]*sy[i] + model[2]*sz[i] + model[3]);
        if (dist > kCfg.ransac_thresh) non_ground_pts.push_back(sor_pts[i]);
    }
    size_t n_ng = non_ground_pts.size();

    // 7. Octree Search 3 & Euclidean Clustering
    std::vector<float> ox(n_ng), oy(n_ng), oz(n_ng);
    for (size_t i = 0; i < n_ng; ++i) { ox[i] = non_ground_pts[i].x; oy[i] = non_ground_pts[i].y; oz[i] = non_ground_pts[i].z; }
    PointCloudSoA ng_cloud{ox.data(), oy.data(), oz.data(), n_ng};

    Octree tree3;
    tree3.setInputCloud(ng_cloud);
    tree3.build();

    EuclideanClustering ec;
    ec.setInputCloud(ng_cloud);
    ec.setNeighborSearch(&tree3);
    ec.setClusterTolerance(kCfg.cluster_tol);
    ec.setMinClusterSize(kCfg.min_cluster);
    ec.setMaxClusterSize(kCfg.max_cluster);
    auto clusters = ec.extract();
    out_clusters = clusters.size();

    return std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
}

} // namespace pcl_ref

// ============================================================================
// 2. PIPELINE_EXPORT (POINTER_OCTREE BASELINE)
// ============================================================================
namespace pipe_export {

double run(const std::vector<PointXYZ>& input_pts, size_t& out_down, size_t& out_sor, size_t& out_inliers, size_t& out_clusters) {
    auto t_start = Clock::now();
    size_t n_in = input_pts.size();

    std::vector<float> ix(n_in), iy(n_in), iz(n_in);
    for (size_t i = 0; i < n_in; ++i) { ix[i] = input_pts[i].x; iy[i] = input_pts[i].y; iz[i] = input_pts[i].z; }
    PointCloudSoA in_cloud{ix.data(), iy.data(), iz.data(), n_in};

    std::vector<PointXYZ> down_pts(n_in);
    size_t n_down = voxel_grid_downsamp_rvv_v2(in_cloud, down_pts.data(), kCfg.leaf_size);
    out_down = n_down;

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) { dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z; }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};

    // PointerOctree 1
    PointerOctree search1;
    search1.setInputCloud(down_cloud);
    search1.build();

    // SOR (PointerOctree)
    std::vector<PointXYZ> sor_pts(n_down);
    std::vector<float> mean_dists(n_down);
    std::vector<int> nbr_indices; std::vector<float> nbr_dists;
    double total_sum = 0.0, total_sq_sum = 0.0;
    size_t valid_count = 0;

    for (size_t i = 0; i < n_down; ++i) {
        search1.radiusSearch({dx[i], dy[i], dz[i]}, kCfg.sor_radius, nbr_indices, nbr_dists);
        if (nbr_dists.size() > 1) {
            int valid_k = std::min(kCfg.sor_k, static_cast<int>(nbr_dists.size()) - 1);
            std::nth_element(nbr_dists.begin(), nbr_dists.begin() + valid_k, nbr_dists.end());
            float sum = 0.0f;
#if defined(__riscv) || defined(__riscv_vector)
            size_t vl = __riscv_vsetvl_e32m4(valid_k);
            vfloat32m4_t vd2 = __riscv_vle32_v_f32m4(nbr_dists.data() + 1, vl);
            vfloat32m4_t vd = __riscv_vfsqrt_v_f32m4(vd2, vl);
            vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
            vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m4_f32m1(vd, zero, vl);
            sum = __riscv_vfmv_f_s_f32m1_f32(v_sum);
#else
            for (int j = 1; j <= valid_k; ++j) sum += std::sqrt(nbr_dists[j]);
#endif
            float m = sum / static_cast<float>(valid_k);
            mean_dists[i] = m; total_sum += m; total_sq_sum += (m * m);
            valid_count++;
        } else { mean_dists[i] = kCfg.sor_radius; }
    }

    double d_cnt = static_cast<double>(valid_count);
    double global_mean = total_sum / d_cnt;
    double stddev = std::sqrt(std::max(0.0, (total_sq_sum / d_cnt) - (global_mean * global_mean)));
    float thresh = static_cast<float>(global_mean + kCfg.sor_std * stddev);

    size_t n_sor = 0;
    for (size_t i = 0; i < n_down; ++i) {
        if (mean_dists[i] <= thresh) sor_pts[n_sor++] = {dx[i], dy[i], dz[i]};
    }
    out_sor = n_sor;

    std::vector<float> sx(n_sor), sy(n_sor), sz(n_sor);
    for (size_t i = 0; i < n_sor; ++i) { sx[i] = sor_pts[i].x; sy[i] = sor_pts[i].y; sz[i] = sor_pts[i].z; }
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), n_sor};

    // PointerOctree 2
    PointerOctree search2;
    search2.setInputCloud(sor_cloud);
    search2.build();

    // Normal estimation
    std::vector<float> nx(n_sor), ny(n_sor), nz(n_sor);
    for (size_t i = 0; i < n_sor; ++i) {
        search2.radiusSearch({sx[i], sy[i], sz[i]}, kCfg.normal_radius, nbr_indices, nbr_dists);
        if (nbr_indices.size() >= 3) {
            float cx=0, cy=0, cz=0;
            for (int id : nbr_indices) { cx += sx[id]; cy += sy[id]; cz += sz[id]; }
            float inv = 1.0f / nbr_indices.size(); cx*=inv; cy*=inv; cz*=inv;
            float c00=0, c01=0, c02=0, c11=0, c12=0, c22=0;
            for (int id : nbr_indices) {
                float ddx = sx[id]-cx, ddy = sy[id]-cy, ddz = sz[id]-cz;
                c00 += ddx*ddx; c01 += ddx*ddy; c02 += ddx*ddz;
                c11 += ddy*ddy; c12 += ddy*ddz; c22 += ddz*ddz;
            }
            float vx = c01*c12 - c02*c11, vy = c01*c02 - c00*c12, vz = c00*c11 - c01*c01;
            float norm = std::sqrt(vx*vx + vy*vy + vz*vz);
            if (norm > 1e-6f) { nx[i] = vx/norm; ny[i] = vy/norm; nz[i] = vz/norm; }
            else { nx[i]=0; ny[i]=0; nz[i]=1; }
        } else { nx[i]=0; ny[i]=0; nz[i]=1; }
    }

    // RVV RANSAC
    float model[4] = {0};
    std::vector<PointXYZ> in_pts(n_sor), out_pts(n_sor);
    size_t n_inliers = 0, n_outliers = 0;
    int r_cnt = ransac_plane_rvv(sor_cloud, kCfg.ransac_thresh, kCfg.ransac_max_iters, model);
    if (r_cnt > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud, model, kCfg.ransac_thresh, in_pts.data(), out_pts.data(), n_inliers, n_outliers);
    }
    out_inliers = n_inliers;

    // PointerOctree 3 & Clustering
    std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
    for (size_t i = 0; i < n_outliers; ++i) { ox[i] = out_pts[i].x; oy[i] = out_pts[i].y; oz[i] = out_pts[i].z; }
    PointCloudSoA ng_cloud{ox.data(), oy.data(), oz.data(), n_outliers};

    PointerOctree search3;
    search3.setInputCloud(ng_cloud);
    search3.build();

    EuclideanClustering ec;
    ec.setInputCloud(ng_cloud);
    ec.setNeighborSearch(&search3);
    ec.setClusterTolerance(kCfg.cluster_tol);
    ec.setMinClusterSize(kCfg.min_cluster);
    ec.setMaxClusterSize(kCfg.max_cluster);
    auto clusters = ec.extract();
    out_clusters = clusters.size();

    return std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
}

} // namespace pipe_export

// ============================================================================
// 3. PIPELINE_3D_TURBO
// ============================================================================
namespace pipe_turbo {

class Fast3DSpatialGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;
    struct Cell { int cx = -999999, cy = -999999, cz = -999999; int head = -1; };
    float cell_size_, inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    Fast3DSpatialGrid(float cell_size = 0.25f) : cell_size_(cell_size), inv_cell_(1.0f / cell_size), cells_(kCapacity) {}
    static inline size_t hash3D(int x, int y, int z) {
        return ((static_cast<size_t>(x) * 73856093) ^ (static_cast<size_t>(y) * 19349663) ^ (static_cast<size_t>(z) * 83492791)) & kMask;
    }
    void build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i] = Cell();
        next_.assign(n, -1);
        for (size_t i = 0; i < n; ++i) {
            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz)) h = (h + 1) & kMask;
            if (cells_[h].head == -1) { cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz; }
            next_[i] = cells_[h].head; cells_[h].head = static_cast<int>(i);
        }
    }
    inline void radiusSearch(float qx, float qy, float qz, float r2, std::vector<int>& nbrs, std::vector<float>& d2) const {
        nbrs.clear(); d2.clear();
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
                                float dist2 = ddx*ddx + ddy*ddy + ddz*ddz;
                                if (dist2 <= r2) { nbrs.push_back(curr); d2.push_back(dist2); }
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

struct DisjointSet {
    std::vector<int> parent;
    DisjointSet(int n) : parent(n) { std::iota(parent.begin(), parent.end(), 0); }
    int find(int i) { return (parent[i] == i) ? i : (parent[i] = find(parent[i])); }
    void unite(int i, int j) { int ri = find(i), rj = find(j); if (ri != rj) parent[ri] = rj; }
};

double run(const std::vector<PointXYZ>& input_pts, size_t& out_down, size_t& out_sor, size_t& out_inliers, size_t& out_clusters) {
    auto t_start = Clock::now();
    size_t n_in = input_pts.size();

    std::vector<float> ix(n_in), iy(n_in), iz(n_in);
    for (size_t i = 0; i < n_in; ++i) { ix[i] = input_pts[i].x; iy[i] = input_pts[i].y; iz[i] = input_pts[i].z; }
    PointCloudSoA in_cloud{ix.data(), iy.data(), iz.data(), n_in};

    std::vector<PointXYZ> down_pts(n_in);
    size_t n_down = voxel_grid_downsamp_rvv_v2(in_cloud, down_pts.data(), kCfg.leaf_size);
    out_down = n_down;

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) { dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z; }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};

    Fast3DSpatialGrid grid(0.25f);
    grid.build(dx.data(), dy.data(), dz.data(), n_down);

    // Fused SOR + Normals
    std::vector<float> mean_dists(n_down, 0.25f);
    std::vector<int> valid_points; valid_points.reserve(n_down);
    std::vector<int> nbrs; std::vector<float> d2;
    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = kCfg.sor_radius * kCfg.sor_radius;

    for (size_t i = 0; i < n_down; ++i) {
        grid.radiusSearch(dx[i], dy[i], dz[i], sor_r2, nbrs, d2);
        int found = static_cast<int>(nbrs.size());
        if (found < 2) continue;
        int k_use = std::min(found - 1, kCfg.sor_k);
        float sum_dist = 0.0f;
        for (int j = 1; j <= k_use; ++j) sum_dist += std::sqrt(d2[j]);
        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m; total_sum += m; total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));
    }

    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double stddev = std::sqrt(std::max(0.0, (total_sq_sum / d_count) - (global_mean * global_mean)));
    float thresh = static_cast<float>(global_mean + kCfg.sor_std * stddev);

    std::vector<float> sx, sy, sz;
    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) { sx.push_back(dx[idx]); sy.push_back(dy[idx]); sz.push_back(dz[idx]); }
    }
    size_t n_sor = sx.size();
    out_sor = n_sor;
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), n_sor};

    // RANSAC
    float model[4] = {0};
    std::vector<PointXYZ> in_pts(n_sor), out_pts(n_sor);
    size_t n_inliers = 0, n_outliers = 0;
    int r_cnt = ransac_plane_rvv(sor_cloud, kCfg.ransac_thresh, kCfg.ransac_max_iters, model);
    if (r_cnt > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud, model, kCfg.ransac_thresh, in_pts.data(), out_pts.data(), n_inliers, n_outliers);
    }
    out_inliers = n_inliers;

    // Union Find Clustering
    std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
    for (size_t i = 0; i < n_outliers; ++i) { ox[i] = out_pts[i].x; oy[i] = out_pts[i].y; oz[i] = out_pts[i].z; }
    PointCloudSoA non_ground_cloud{ox.data(), oy.data(), oz.data(), n_outliers};

    Fast3DSpatialGrid cl_grid(kCfg.cluster_tol);
    cl_grid.build(ox.data(), oy.data(), oz.data(), n_outliers);

    DisjointSet ds(static_cast<int>(n_outliers));
    float tol_sq = kCfg.cluster_tol * kCfg.cluster_tol;
    for (size_t i = 0; i < n_outliers; ++i) {
        cl_grid.radiusSearch(ox[i], oy[i], oz[i], tol_sq, nbrs, d2);
        for (int nb : nbrs) ds.unite(static_cast<int>(i), nb);
    }
    std::unordered_map<int, std::vector<int>> cmap;
    for (size_t i = 0; i < n_outliers; ++i) cmap[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));

    size_t n_cl = 0;
    for (auto& kv : cmap) {
        if (kv.second.size() >= static_cast<size_t>(kCfg.min_cluster) && kv.second.size() <= static_cast<size_t>(kCfg.max_cluster)) n_cl++;
    }
    out_clusters = n_cl;

    return std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
}

} // namespace pipe_turbo

// ============================================================================
// 4. PIPELINE_3D_ULTRA (MOST OPTIMIZED)
// ============================================================================
namespace pipe_ultra {

static bool compute_plane_coeffs(float x1, float y1, float z1, float x2, float y2, float z2, float x3, float y3, float z3, float* model) {
    float v1x = x2-x1, v1y = y2-y1, v1z = z2-z1;
    float v2x = x3-x1, v2y = y3-y1, v2z = z3-z1;
    float a = v1y*v2z - v1z*v2y, b = v1z*v2x - v1x*v2z, c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < 1e-4f) return false;
    a /= norm; b /= norm; c /= norm;
    model[0] = a; model[1] = b; model[2] = c; model[3] = -(a*x1 + b*y1 + c*z1);
    return true;
}

int ransac_ground_plane_fast_rvv(const PointCloudSoA& cloud, float dist_thresh, int max_iters, float* model) {
    if (cloud.n < 3) return 0;
    std::srand(0);
    int best_inliers = 0;
    float best_model[4] = {0};
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        int i1 = std::rand() % cloud.n, i2 = std::rand() % cloud.n, i3 = std::rand() % cloud.n;
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;
        float cand[4];
        if (!compute_plane_coeffs(cloud.x[i1], cloud.y[i1], cloud.z[i1], cloud.x[i2], cloud.y[i2], cloud.z[i2], cloud.x[i3], cloud.y[i3], cloud.z[i3], cand)) continue;
        if (std::abs(cand[2]) < 0.70f) continue;

        float a = cand[0], b = cand[1], c = cand[2], d = cand[3];
        size_t n = cloud.n;

#if defined(__riscv) || defined(__riscv_vector)
        if (best_inliers > 5000) {
            int coarse = 0; size_t si = 0;
            const ptrdiff_t bstride = 4 * sizeof(float);
            while (si < n) {
                size_t count = (n - si + 3) / 4;
                size_t vl = __riscv_vsetvl_e32m8(count);
                vfloat32m8_t vx = __riscv_vlse32_v_f32m8(&cloud.x[si], bstride, vl);
                vfloat32m8_t vy = __riscv_vlse32_v_f32m8(&cloud.y[si], bstride, vl);
                vfloat32m8_t vz = __riscv_vlse32_v_f32m8(&cloud.z[si], bstride, vl);
                vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
                dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
                dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
                dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
                vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
                vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
                vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
                coarse += __riscv_vcpop_m_b4(mask_in, vl);
                si += vl * 4;
            }
            if (coarse * 4 < static_cast<int>(best_inliers * 0.80f)) continue;
        }

        int current_inliers = 0; size_t i = 0;
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
#else
        int current_inliers = 0;
        for (size_t pt = 0; pt < n; ++pt) {
            float dist = std::abs(a*cloud.x[pt] + b*cloud.y[pt] + c*cloud.z[pt] + d);
            if (dist <= dist_thresh) current_inliers++;
        }
#endif
        if (current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for (int k = 0; k < 4; ++k) best_model[k] = cand[k];
            double w = static_cast<double>(best_inliers) / static_cast<double>(cloud.n);
            double p_no = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no = std::log(p_no);
            if (std::abs(log_no) > 1e-7) {
                int dyn_k = static_cast<int>(std::ceil(log_p / log_no));
                if (dyn_k > 0 && dyn_k < k_iters) k_iters = dyn_k;
            }
        }
    }
    for (int k = 0; k < 4; ++k) model[k] = best_model[k];
    return best_inliers;
}

struct DisjointSetRank {
    std::vector<int> parent, rank;
    DisjointSetRank(int n) : parent(n), rank(n, 0) { std::iota(parent.begin(), parent.end(), 0); }
    inline int find(int i) {
        int root = i; while (root != parent[root]) root = parent[root];
        int curr = i; while (curr != root) { int nxt = parent[curr]; parent[curr] = root; curr = nxt; }
        return root;
    }
    inline void unite(int i, int j) {
        int ri = find(i), rj = find(j);
        if (ri != rj) {
            if (rank[ri] < rank[rj]) parent[ri] = rj;
            else if (rank[ri] > rank[rj]) parent[rj] = ri;
            else { parent[rj] = ri; rank[ri]++; }
        }
    }
};

double run(const std::vector<PointXYZ>& input_pts, size_t& out_down, size_t& out_sor, size_t& out_inliers, size_t& out_clusters) {
    auto t_start = Clock::now();
    size_t n_in = input_pts.size();

    std::vector<float> ix(n_in), iy(n_in), iz(n_in);
    for (size_t i = 0; i < n_in; ++i) { ix[i] = input_pts[i].x; iy[i] = input_pts[i].y; iz[i] = input_pts[i].z; }
    PointCloudSoA in_cloud{ix.data(), iy.data(), iz.data(), n_in};

    std::vector<PointXYZ> down_pts(n_in);
    size_t n_down = voxel_grid_downsamp_rvv_v2(in_cloud, down_pts.data(), kCfg.leaf_size);
    out_down = n_down;

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) { dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z; }
    PointCloudSoA down_cloud{dx.data(), dy.data(), dz.data(), n_down};

    pipe_turbo::Fast3DSpatialGrid grid(0.25f);
    grid.build(dx.data(), dy.data(), dz.data(), n_down);

    // Vectorized SOR
    std::vector<float> mean_dists(n_down, 0.25f);
    std::vector<int> valid_points; valid_points.reserve(n_down);
    std::vector<int> nbrs; std::vector<float> d2;
    double total_sum = 0.0, total_sq_sum = 0.0;
    float sor_r2 = kCfg.sor_radius * kCfg.sor_radius;

    for (size_t i = 0; i < n_down; ++i) {
        grid.radiusSearch(dx[i], dy[i], dz[i], sor_r2, nbrs, d2);
        int found = static_cast<int>(nbrs.size());
        if (found < 2) continue;
        int k_use = std::min(found - 1, kCfg.sor_k);
        float sum_dist = 0.0f;
#if defined(__riscv) || defined(__riscv_vector)
        int rem = k_use; int offset = 1;
        while (rem > 0) {
            size_t vl = __riscv_vsetvl_e32m8(rem);
            vfloat32m8_t vd2 = __riscv_vle32_v_f32m8(d2.data() + offset, vl);
            vfloat32m8_t vd = __riscv_vfsqrt_v_f32m8(vd2, vl);
            vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
            vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m8_f32m1(vd, zero, vl);
            sum_dist += __riscv_vfmv_f_s_f32m1_f32(v_sum);
            offset += vl; rem -= vl;
        }
#else
        for (int j = 1; j <= k_use; ++j) sum_dist += std::sqrt(d2[j]);
#endif
        float m = sum_dist / static_cast<float>(k_use);
        mean_dists[i] = m; total_sum += m; total_sq_sum += (m * m);
        valid_points.push_back(static_cast<int>(i));
    }

    double d_count = static_cast<double>(valid_points.size());
    double global_mean = total_sum / d_count;
    double stddev = std::sqrt(std::max(0.0, (total_sq_sum / d_count) - (global_mean * global_mean)));
    float thresh = static_cast<float>(global_mean + kCfg.sor_std * stddev);

    std::vector<float> sx, sy, sz;
    for (int idx : valid_points) {
        if (mean_dists[idx] <= thresh) { sx.push_back(dx[idx]); sy.push_back(dy[idx]); sz.push_back(dz[idx]); }
    }
    size_t n_sor = sx.size();
    out_sor = n_sor;
    PointCloudSoA sor_cloud{sx.data(), sy.data(), sz.data(), n_sor};

    // Fast Strided RVV RANSAC
    float model[4] = {0};
    std::vector<PointXYZ> in_pts(n_sor), out_pts(n_sor);
    size_t n_inliers = 0, n_outliers = 0;
    int r_cnt = ransac_ground_plane_fast_rvv(sor_cloud, kCfg.ransac_thresh, kCfg.ransac_max_iters, model);
    if (r_cnt > 0) {
        extract_plane_inliers_outliers_rvv(sor_cloud, model, kCfg.ransac_thresh, in_pts.data(), out_pts.data(), n_inliers, n_outliers);
    }
    out_inliers = n_inliers;

    // 14-Cell Half-Space Clustering
    std::vector<float> ox(n_outliers), oy(n_outliers), oz(n_outliers);
    for (size_t i = 0; i < n_outliers; ++i) { ox[i] = out_pts[i].x; oy[i] = out_pts[i].y; oz[i] = out_pts[i].z; }

    pipe_turbo::Fast3DSpatialGrid cl_grid(kCfg.cluster_tol);
    cl_grid.build(ox.data(), oy.data(), oz.data(), n_outliers);

    DisjointSetRank ds(static_cast<int>(n_outliers));
    float tol_sq = kCfg.cluster_tol * kCfg.cluster_tol;

    static constexpr int kHalfOffsets[14][3] = {
        {0, 0, 0}, {1, 0, 0}, {-1, 1, 0}, {0, 1, 0}, {1, 1, 0},
        {-1, -1, 1}, {0, -1, 1}, {1, -1, 1}, {-1, 0, 1},  {0, 0, 1},
        {1, 0, 1},  {-1, 1, 1},  {0, 1, 1},  {1, 1, 1}
    };

    for (size_t i = 0; i < n_outliers; ++i) {
        float qx = ox[i], qy = oy[i], qz = oz[i];
        int qcx = static_cast<int>(std::floor(qx * cl_grid.inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * cl_grid.inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * cl_grid.inv_cell_));

        for (int o = 0; o < 14; ++o) {
            int tcx = qcx + kHalfOffsets[o][0];
            int tcy = qcy + kHalfOffsets[o][1];
            int tcz = qcz + kHalfOffsets[o][2];
            size_t h = pipe_turbo::Fast3DSpatialGrid::hash3D(tcx, tcy, tcz);

            while (cl_grid.cells_[h].head != -1) {
                if (cl_grid.cells_[h].cx == tcx && cl_grid.cells_[h].cy == tcy && cl_grid.cells_[h].cz == tcz) {
                    int curr = cl_grid.cells_[h].head;
                    while (curr != -1) {
                        if (curr > static_cast<int>(i)) {
                            float ddx = cl_grid.px_[curr] - qx, ddy = cl_grid.py_[curr] - qy, ddz = cl_grid.pz_[curr] - qz;
                            float dist2 = ddx*ddx + ddy*ddy + ddz*ddz;
                            if (dist2 <= tol_sq) ds.unite(static_cast<int>(i), curr);
                        }
                        curr = cl_grid.next_[curr];
                    }
                    break;
                }
                h = (h + 1) & pipe_turbo::Fast3DSpatialGrid::kMask;
            }
        }
    }

    std::unordered_map<int, std::vector<int>> cmap;
    for (size_t i = 0; i < n_outliers; ++i) cmap[ds.find(static_cast<int>(i))].push_back(static_cast<int>(i));

    size_t n_cl = 0;
    for (auto& kv : cmap) {
        if (kv.second.size() >= static_cast<size_t>(kCfg.min_cluster) && kv.second.size() <= static_cast<size_t>(kCfg.max_cluster)) n_cl++;
    }
    out_clusters = n_cl;

    return std::chrono::duration<double, std::milli>(Clock::now() - t_start).count();
}

} // namespace pipe_ultra

// ============================================================================
// MAIN BENCHMARK SWEEP HARNESS
// ============================================================================
int main(int argc, char** argv) {
    std::vector<std::string> frames = {
        "0000000000.pcd",
        "0000000020.pcd",
        "0000000040.pcd",
        "0000000060.pcd",
        "0000000090.pcd"
    };

    std::cout << "\n==========================================================================================" << std::endl;
    std::cout << " MULTI-PCD BENCHMARK SWEEP (PURE ALGORITHMIC COMPUTE, ZERO DISK I/O)" << std::endl;
    std::cout << "==========================================================================================" << std::endl;
    std::cout << std::left << std::setw(18) << "Frame"
              << std::setw(12) << "Points"
              << std::setw(16) << "PCL Official"
              << std::setw(18) << "pipeline_export"
              << std::setw(18) << "pipeline_3d_turbo"
              << std::setw(18) << "pipeline_3d_ultra" << std::endl;
    std::cout << "------------------------------------------------------------------------------------------" << std::endl;

    double sum_pcl = 0, sum_export = 0, sum_turbo = 0, sum_ultra = 0;
    size_t evaluated_frames = 0;

    for (const auto& fname : frames) {
        std::string path = "data/pcd_compressed/" + fname;
        if (!std::filesystem::exists(path)) path = "/workspace/" + path;
        if (!std::filesystem::exists(path)) continue;

        std::vector<PointXYZ> raw_pts;
        if (loadPCD(path, raw_pts) < 0) continue;

        size_t d1=0, s1=0, in1=0, c1=0;
        size_t d2=0, s2=0, in2=0, c2=0;
        size_t d3=0, s3=0, in3=0, c3=0;
        size_t d4=0, s4=0, in4=0, c4=0;

        double t_pcl    = pcl_ref::run(raw_pts, d1, s1, in1, c1);
        double t_export = pipe_export::run(raw_pts, d2, s2, in2, c2);
        double t_turbo  = pipe_turbo::run(raw_pts, d3, s3, in3, c3);
        double t_ultra  = pipe_ultra::run(raw_pts, d4, s4, in4, c4);

        sum_pcl += t_pcl; sum_export += t_export; sum_turbo += t_turbo; sum_ultra += t_ultra;
        evaluated_frames++;

        std::cout << std::left << std::setw(18) << fname
                  << std::setw(12) << raw_pts.size()
                  << std::right << std::fixed << std::setprecision(1)
                  << std::setw(10) << t_pcl << " ms  "
                  << std::setw(12) << t_export << " ms  "
                  << std::setw(12) << t_turbo << " ms  "
                  << std::setw(12) << t_ultra << " ms" << std::endl;
    }

    if (evaluated_frames > 0) {
        double avg_pcl = sum_pcl / evaluated_frames;
        double avg_export = sum_export / evaluated_frames;
        double avg_turbo = sum_turbo / evaluated_frames;
        double avg_ultra = sum_ultra / evaluated_frames;

        std::cout << "==========================================================================================" << std::endl;
        std::cout << std::left << std::setw(18) << "AVERAGE"
                  << std::setw(12) << "—"
                  << std::right << std::fixed << std::setprecision(1)
                  << std::setw(10) << avg_pcl << " ms  "
                  << std::setw(12) << avg_export << " ms  "
                  << std::setw(12) << avg_turbo << " ms  "
                  << std::setw(12) << avg_ultra << " ms" << std::endl;
        std::cout << "------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "SPEEDUP vs Official PCL:  [1.0x]             [" << std::setprecision(2) << (avg_pcl / avg_export)
                  << "x]               [" << (avg_pcl / avg_turbo) << "x]               [" << (avg_pcl / avg_ultra) << "x]" << std::endl;
        std::cout << "SPEEDUP vs Base Export:   [—]                [1.0x]               [" << std::setprecision(2)
                  << (avg_export / avg_turbo) << "x]               [" << (avg_export / avg_ultra) << "x]" << std::endl;
        std::cout << "==========================================================================================\n" << std::endl;
    }

    return 0;
}
