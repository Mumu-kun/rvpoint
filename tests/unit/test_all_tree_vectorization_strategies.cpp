#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <numeric>
#include <iomanip>
#include <algorithm>
#include "simple_pcd_loader.h"
#include "pointer_octree/pointer_octree.h"

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using namespace rvv_pcl;

// ============================================================================
// STRATEGY 5: Contiguous Z-Order Morton Octree (CSR Contiguous Leaf Array)
// ============================================================================
class ContiguousMortonOctree {
public:
    float voxel_size_ = 0.25f;
    float inv_voxel_ = 4.0f;
    float min_x_ = 0, min_y_ = 0, min_z_ = 0;

    static const size_t HASH_SIZE = 65536;
    static const size_t HASH_MASK = HASH_SIZE - 1;

    // Compact Cell Hash Table pointing directly to contiguous slice [start, count]
    struct CellSlice {
        uint64_t key = 0xFFFFFFFFFFFFFFFFULL;
        uint32_t start = 0;
        uint32_t count = 0;
    };

    std::vector<CellSlice> table_;
    std::vector<float> sorted_x_, sorted_y_, sorted_z_;
    std::vector<int> sorted_indices_;

    static inline uint64_t morton3D(uint32_t x, uint32_t y, uint32_t z) {
        auto split3 = [](uint32_t v) -> uint64_t {
            uint64_t x = v & 0x1fffff;
            x = (x | (x << 32)) & 0x1f00000000ffff;
            x = (x | (x << 16)) & 0x1f0000ff0000ff;
            x = (x | (x << 8))  & 0x100f00f00f00f00f;
            x = (x | (x << 4))  & 0x10c30c30c30c30c3;
            x = (x | (x << 2))  & 0x1249249249249249;
            return x;
        };
        return split3(x) | (split3(y) << 1) | (split3(z) << 2);
    }

    void build(const PointCloudSoA& cloud, float voxel_size = 0.25f) {
        voxel_size_ = voxel_size;
        inv_voxel_ = 1.0f / voxel_size_;

        table_.assign(HASH_SIZE, CellSlice{});
        sorted_x_.resize(cloud.n);
        sorted_y_.resize(cloud.n);
        sorted_z_.resize(cloud.n);
        sorted_indices_.resize(cloud.n);

        min_x_ = 1e9f; min_y_ = 1e9f; min_z_ = 1e9f;
        for (size_t i = 0; i < cloud.n; ++i) {
            min_x_ = std::min(min_x_, cloud.x[i]);
            min_y_ = std::min(min_y_, cloud.y[i]);
            min_z_ = std::min(min_z_, cloud.z[i]);
        }

        // Pair: (MortonCode, PointIndex)
        std::vector<std::pair<uint64_t, int>> pt_codes(cloud.n);
        for (size_t i = 0; i < cloud.n; ++i) {
            uint32_t gx = static_cast<uint32_t>((cloud.x[i] - min_x_) * inv_voxel_);
            uint32_t gy = static_cast<uint32_t>((cloud.y[i] - min_y_) * inv_voxel_);
            uint32_t gz = static_cast<uint32_t>((cloud.z[i] - min_z_) * inv_voxel_);
            pt_codes[i] = {morton3D(gx, gy, gz), static_cast<int>(i)};
        }

        // Sort by Z-order curve (Morton code)
        std::sort(pt_codes.begin(), pt_codes.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });

        // Populate contiguous arrays and build CellSlice lookup table
        size_t n = cloud.n;
        size_t idx = 0;
        while (idx < n) {
            uint64_t key = pt_codes[idx].first;
            size_t start = idx;
            while (idx < n && pt_codes[idx].first == key) {
                int orig_idx = pt_codes[idx].second;
                sorted_x_[idx] = cloud.x[orig_idx];
                sorted_y_[idx] = cloud.y[orig_idx];
                sorted_z_[idx] = cloud.z[orig_idx];
                sorted_indices_[idx] = orig_idx;
                idx++;
            }
            uint32_t count = static_cast<uint32_t>(idx - start);

            size_t slot = (key ^ (key >> 16) ^ (key >> 32)) & HASH_MASK;
            while (table_[slot].key != 0xFFFFFFFFFFFFFFFFULL && table_[slot].key != key) {
                slot = (slot + 1) & HASH_MASK;
            }
            table_[slot].key = key;
            table_[slot].start = static_cast<uint32_t>(start);
            table_[slot].count = count;
        }
    }

    size_t radiusSearchRVV(const PointXYZ& query, float radius, std::vector<int>& out_indices, std::vector<float>& out_dists) const {
        out_indices.clear();
        out_dists.clear();

        float r2 = radius * radius;
        float qx = query.x, qy = query.y, qz = query.z;

        int gx = static_cast<int>((qx - min_x_) * inv_voxel_);
        int gy = static_cast<int>((qy - min_y_) * inv_voxel_);
        int gz = static_cast<int>((qz - min_z_) * inv_voxel_);

        for (int dz = -1; dz <= 1; ++dz) {
            int nz = gz + dz;
            if (nz < 0) continue;
            for (int dy = -1; dy <= 1; ++dy) {
                int ny = gy + dy;
                if (ny < 0) continue;
                for (int dx = -1; dx <= 1; ++dx) {
                    int nx = gx + dx;
                    if (nx < 0) continue;

                    uint64_t key = morton3D(nx, ny, nz);
                    size_t slot = (key ^ (key >> 16) ^ (key >> 32)) & HASH_MASK;
                    while (table_[slot].key != 0xFFFFFFFFFFFFFFFFULL && table_[slot].key != key) {
                        slot = (slot + 1) & HASH_MASK;
                    }

                    if (table_[slot].key == key) {
                        uint32_t start = table_[slot].start;
                        uint32_t count = table_[slot].count;

#if defined(__riscv) || defined(__riscv_vector)
                        size_t i = 0;
                        while (i < count) {
                            size_t vl = __riscv_vsetvl_e32m4(count - i);
                            vfloat32m4_t vx = __riscv_vle32_v_f32m4(sorted_x_.data() + start + i, vl);
                            vfloat32m4_t vy = __riscv_vle32_v_f32m4(sorted_y_.data() + start + i, vl);
                            vfloat32m4_t vz = __riscv_vle32_v_f32m4(sorted_z_.data() + start + i, vl);

                            vfloat32m4_t vdx = __riscv_vfsub_vf_f32m4(vx, qx, vl);
                            vfloat32m4_t vdy = __riscv_vfsub_vf_f32m4(vy, qy, vl);
                            vfloat32m4_t vdz = __riscv_vfsub_vf_f32m4(vz, qz, vl);

                            vfloat32m4_t vd2 = __riscv_vfmul_vv_f32m4(vdx, vdx, vl);
                            vd2 = __riscv_vfmacc_vv_f32m4(vd2, vdy, vdy, vl);
                            vd2 = __riscv_vfmacc_vv_f32m4(vd2, vdz, vdz, vl);

                            vbool8_t mask = __riscv_vmfle_vf_f32m4_b8(vd2, r2, vl);
                            size_t hit_count = __riscv_vcpop_m_b8(mask, vl);

                            if (hit_count > 0) {
                                size_t old_size = out_indices.size();
                                out_indices.resize(old_size + hit_count);
                                out_dists.resize(old_size + hit_count);

                                vint32m4_t v_idx = __riscv_vle32_v_i32m4(sorted_indices_.data() + start + i, vl);
                                vint32m4_t v_match_idx = __riscv_vcompress_vm_i32m4(v_idx, mask, vl);
                                vfloat32m4_t v_match_dst = __riscv_vcompress_vm_f32m4(vd2, mask, vl);

                                __riscv_vse32_v_i32m4(out_indices.data() + old_size, v_match_idx, hit_count);
                                __riscv_vse32_v_f32m4(out_dists.data() + old_size, v_match_dst, hit_count);
                            }
                            i += vl;
                        }
#else
                        for (size_t i = 0; i < count; ++i) {
                            float vdx = sorted_x_[start + i] - qx;
                            float vdy = sorted_y_[start + i] - qy;
                            float vdz = sorted_z_[start + i] - qz;
                            float vd2 = vdx*vdx + vdy*vdy + vdz*vdz;
                            if (vd2 <= r2) {
                                out_indices.push_back(sorted_indices_[start + i]);
                                out_dists.push_back(vd2);
                            }
                        }
#endif
                    }
                }
            }
        }
        return out_indices.size();
    }
};

int main() {
    std::cout << "====================================================================================\n";
    std::cout << "        EMPIRICAL EVALUATION: VECTORIZING TREE TRAVERSAL & POINTER CHASING          \n";
    std::cout << "        Dataset: Frame 80 (data/pcd_compressed/0000000080.pcd)                      \n";
    std::cout << "====================================================================================\n\n";

    std::vector<PointXYZ> points;
    std::string path = "data/pcd_compressed/0000000080.pcd";
    if (loadPCD(path, points) <= 0) {
        std::cerr << "Failed to load " << path << "\n";
        return 1;
    }

    std::vector<float> vx(points.size()), vy(points.size()), vz(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        vx[i] = points[i].x; vy[i] = points[i].y; vz[i] = points[i].z;
    }
    PointCloudSoA cloud{vx.data(), vy.data(), vz.data(), points.size()};

    const size_t NUM_QUERIES = 1000;
    const float radius = 0.25f;

    std::vector<int> out_idx;
    std::vector<float> out_dst;

    // 0. Baseline: Traditional Heap PointerOctree
    PointerOctree baseline_tree;
    baseline_tree.setMaxPointsPerLeaf(64);
    baseline_tree.setInputCloud(cloud);
    auto t_b0 = std::chrono::high_resolution_clock::now();
    baseline_tree.build();
    auto t_b1 = std::chrono::high_resolution_clock::now();
    double dt_build_baseline = std::chrono::duration<double, std::milli>(t_b1 - t_b0).count();

    auto t0 = std::chrono::high_resolution_clock::now();
    size_t nbrs_baseline = 0;
    for (size_t i = 0; i < NUM_QUERIES; ++i) {
        size_t idx = i * (points.size() / NUM_QUERIES);
        nbrs_baseline += baseline_tree.radiusSearch(points[idx], radius, out_idx, out_dst);
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    double dt_search_baseline = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // 5. Contiguous Z-Order Morton Octree (CSR Contiguous)
    ContiguousMortonOctree contig_morton;
    t_b0 = std::chrono::high_resolution_clock::now();
    contig_morton.build(cloud, 0.25f);
    t_b1 = std::chrono::high_resolution_clock::now();
    double dt_build_contig = std::chrono::duration<double, std::milli>(t_b1 - t_b0).count();

    t0 = std::chrono::high_resolution_clock::now();
    size_t nbrs_contig = 0;
    for (size_t i = 0; i < NUM_QUERIES; ++i) {
        size_t idx = i * (points.size() / NUM_QUERIES);
        nbrs_contig += contig_morton.radiusSearchRVV(points[idx], radius, out_idx, out_dst);
    }
    t1 = std::chrono::high_resolution_clock::now();
    double dt_search_contig = std::chrono::duration<double, std::milli>(t1 - t0).count();

    // Print summary results
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "--- 1,000 Spatial Radius Searches (r = " << radius << "m) on 114,719 Points ---\n\n";

    std::cout << "| Strategy / Architecture | Build Time | 1,000 Queries Time | Inliers Found | Speedup vs Baseline |\n";
    std::cout << "| :--- | :---: | :---: | :---: | :---: |\n";
    std::cout << "| **0. Baseline PointerOctree** (Heap DFS)       | " << std::setw(6) << dt_build_baseline << " ms | " << std::setw(8) << dt_search_baseline << " ms | " << nbrs_baseline << " | 1.00x (Baseline) |\n";
    std::cout << "| **1. Contiguous Z-Order Morton Octree (RVV)**  | " << std::setw(6) << dt_build_contig << " ms | " << std::setw(8) << dt_search_contig << " ms | " << nbrs_contig << " | **" << (dt_search_baseline / dt_search_contig) << "x Faster** 🚀 |\n\n";

    return 0;
}
