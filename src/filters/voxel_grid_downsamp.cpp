#include "filters/voxel_grid.h"
#include "filters/radix_sort.h"
#include <cmath>
#include <map>
#include <tuple>
#include <algorithm>
#include <vector>
#include <numeric>
#include <limits>
#if defined(_OPENMP)
#include <omp.h>
#endif

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

// ============================================================================
// Internal Helpers for Fully Vectorized Voxel Grid (v2)
// ============================================================================
namespace {

#if defined(__riscv_vector)
// Helper: Compute bounding box using RVV reductions (LMUL=m4)
void compute_bbox_rvv(const PointCloudSoA& in,
                      float& out_min_x, float& out_min_y, float& out_min_z,
                      float& out_max_x, float& out_max_y, float& out_max_z) {
    size_t n = in.n;
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();

    size_t vl_init = __riscv_vsetvl_e32m4(n);
    vfloat32m4_t vmin_x = __riscv_vfmv_v_f_f32m4(min_x, vl_init);
    vfloat32m4_t vmin_y = __riscv_vfmv_v_f_f32m4(min_y, vl_init);
    vfloat32m4_t vmin_z = __riscv_vfmv_v_f_f32m4(min_z, vl_init);
    vfloat32m4_t vmax_x = __riscv_vfmv_v_f_f32m4(max_x, vl_init);
    vfloat32m4_t vmax_y = __riscv_vfmv_v_f_f32m4(max_y, vl_init);
    vfloat32m4_t vmax_z = __riscv_vfmv_v_f_f32m4(max_z, vl_init);

    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m4(n - i);
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(&in.x[i], vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(&in.y[i], vl);
        vfloat32m4_t vz = __riscv_vle32_v_f32m4(&in.z[i], vl);

        vmin_x = __riscv_vfmin_vv_f32m4(vmin_x, vx, vl);
        vmin_y = __riscv_vfmin_vv_f32m4(vmin_y, vy, vl);
        vmin_z = __riscv_vfmin_vv_f32m4(vmin_z, vz, vl);
        vmax_x = __riscv_vfmax_vv_f32m4(vmax_x, vx, vl);
        vmax_y = __riscv_vfmax_vv_f32m4(vmax_y, vy, vl);
        vmax_z = __riscv_vfmax_vv_f32m4(vmax_z, vz, vl);

        i += vl;
    }

    // Horizontal reductions
    size_t vl_red = __riscv_vsetvl_e32m4(n);
    vfloat32m1_t seed_min = __riscv_vfmv_v_f_f32m1(std::numeric_limits<float>::max(), 1);
    vfloat32m1_t seed_max = __riscv_vfmv_v_f_f32m1(std::numeric_limits<float>::lowest(), 1);

    vfloat32m1_t r_min_x = __riscv_vfredmin_vs_f32m4_f32m1(vmin_x, seed_min, vl_red);
    vfloat32m1_t r_min_y = __riscv_vfredmin_vs_f32m4_f32m1(vmin_y, seed_min, vl_red);
    vfloat32m1_t r_min_z = __riscv_vfredmin_vs_f32m4_f32m1(vmin_z, seed_min, vl_red);
    vfloat32m1_t r_max_x = __riscv_vfredmax_vs_f32m4_f32m1(vmax_x, seed_max, vl_red);
    vfloat32m1_t r_max_y = __riscv_vfredmax_vs_f32m4_f32m1(vmax_y, seed_max, vl_red);
    vfloat32m1_t r_max_z = __riscv_vfredmax_vs_f32m4_f32m1(vmax_z, seed_max, vl_red);

    __riscv_vse32_v_f32m1(&out_min_x, r_min_x, 1);
    __riscv_vse32_v_f32m1(&out_min_y, r_min_y, 1);
    __riscv_vse32_v_f32m1(&out_min_z, r_min_z, 1);
    __riscv_vse32_v_f32m1(&out_max_x, r_max_x, 1);
    __riscv_vse32_v_f32m1(&out_max_y, r_max_y, 1);
    __riscv_vse32_v_f32m1(&out_max_z, r_max_z, 1);
}

// Helper: Vectorized floor(float)->int using RVV
static inline vint32m4_t vfloor_i32m4(vfloat32m4_t v, size_t vl) {
    vint32m4_t trunc_i = __riscv_vfcvt_rtz_x_f_v_i32m4(v, vl);
    vfloat32m4_t trunc_f = __riscv_vfcvt_f_x_v_f32m4(trunc_i, vl);
    // mask: true where v < trunc_f (i.e. truncation rounded up, need -1)
    vbool8_t needs_fix = __riscv_vmflt_vv_f32m4_b8(v, trunc_f, vl);
    // Subtract 1 where mask is set
    vint32m4_t ones = __riscv_vmv_v_x_i32m4(1, vl);
    return __riscv_vsub_vv_i32m4_mu(needs_fix, trunc_i, trunc_i, ones, vl);
}

// Helper: Compute linear voxel keys using RVV (LMUL=m4)
void compute_voxel_keys_rvv(const PointCloudSoA& in,
                             float inv_leaf, int min_ix, int min_iy, int min_iz,
                             int grid_x, int grid_xy,
                             int32_t* keys, size_t n) {
#if defined(_OPENMP)
    #pragma omp parallel
    {
        int tid = omp_get_thread_num();
        int num_threads = omp_get_num_threads();
        size_t start = (n * tid) / num_threads;
        size_t end   = (n * (tid + 1)) / num_threads;
        size_t i = start;
        while (i < end) {
            size_t vl = __riscv_vsetvl_e32m4(end - i);

            // Load coordinates
            vfloat32m4_t vx = __riscv_vle32_v_f32m4(&in.x[i], vl);
            vfloat32m4_t vy = __riscv_vle32_v_f32m4(&in.y[i], vl);
            vfloat32m4_t vz = __riscv_vle32_v_f32m4(&in.z[i], vl);

            // Scale: coord * inv_leaf (same as scalar)
            vx = __riscv_vfmul_vf_f32m4(vx, inv_leaf, vl);
            vy = __riscv_vfmul_vf_f32m4(vy, inv_leaf, vl);
            vz = __riscv_vfmul_vf_f32m4(vz, inv_leaf, vl);

            // Vectorized floor -> int (handles negative values correctly)
            vint32m4_t ix = vfloor_i32m4(vx, vl);
            vint32m4_t iy = vfloor_i32m4(vy, vl);
            vint32m4_t iz = vfloor_i32m4(vz, vl);

            // Shift to 0-based indices using the global minimum voxel index
            ix = __riscv_vsub_vx_i32m4(ix, min_ix, vl);
            iy = __riscv_vsub_vx_i32m4(iy, min_iy, vl);
            iz = __riscv_vsub_vx_i32m4(iz, min_iz, vl);

            // Linear key = ix + iy * grid_x + iz * grid_x * grid_y
            vint32m4_t iy_scaled = __riscv_vmul_vx_i32m4(iy, grid_x, vl);
            vint32m4_t iz_scaled = __riscv_vmul_vx_i32m4(iz, grid_xy, vl);
            vint32m4_t key = __riscv_vadd_vv_i32m4(ix, iy_scaled, vl);
            key = __riscv_vadd_vv_i32m4(key, iz_scaled, vl);

            __riscv_vse32_v_i32m4(keys + i, key, vl);
            i += vl;
        }
    }
#else
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m4(n - i);

        // Load coordinates
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(&in.x[i], vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(&in.y[i], vl);
        vfloat32m4_t vz = __riscv_vle32_v_f32m4(&in.z[i], vl);

        // Scale: coord * inv_leaf (same as scalar)
        vx = __riscv_vfmul_vf_f32m4(vx, inv_leaf, vl);
        vy = __riscv_vfmul_vf_f32m4(vy, inv_leaf, vl);
        vz = __riscv_vfmul_vf_f32m4(vz, inv_leaf, vl);

        // Vectorized floor -> int (handles negative values correctly)
        vint32m4_t ix = vfloor_i32m4(vx, vl);
        vint32m4_t iy = vfloor_i32m4(vy, vl);
        vint32m4_t iz = vfloor_i32m4(vz, vl);

        // Shift to 0-based indices using the global minimum voxel index
        ix = __riscv_vsub_vx_i32m4(ix, min_ix, vl);
        iy = __riscv_vsub_vx_i32m4(iy, min_iy, vl);
        iz = __riscv_vsub_vx_i32m4(iz, min_iz, vl);

        // Linear key = ix + iy * grid_x + iz * grid_x * grid_y
        vint32m4_t iy_scaled = __riscv_vmul_vx_i32m4(iy, grid_x, vl);
        vint32m4_t iz_scaled = __riscv_vmul_vx_i32m4(iz, grid_xy, vl);
        vint32m4_t key = __riscv_vadd_vv_i32m4(ix, iy_scaled, vl);
        key = __riscv_vadd_vv_i32m4(key, iz_scaled, vl);

        __riscv_vse32_v_i32m4(keys + i, key, vl);
        i += vl;
    }
#endif
}

// Helper: Vectorized centroid reduction for a group of points (LMUL=m2)
void centroid_reduce_rvv(const PointCloudSoA& in,
                         const uint32_t* order, size_t group_start,
                         size_t group_size, PointXYZ& out) {
    if (__builtin_expect(group_size == 1, 1)) {
        uint32_t idx = order[group_start];
        out.x = in.x[idx];
        out.y = in.y[idx];
        out.z = in.z[idx];
        return;
    }
    if (group_size == 2) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        out.x = 0.5f * (in.x[i0] + in.x[i1]);
        out.y = 0.5f * (in.y[i0] + in.y[i1]);
        out.z = 0.5f * (in.z[i0] + in.z[i1]);
        return;
    }
    if (group_size == 3) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        constexpr float f = 1.0f / 3.0f;
        out.x = (in.x[i0] + in.x[i1] + in.x[i2]) * f;
        out.y = (in.y[i0] + in.y[i1] + in.y[i2]) * f;
        out.z = (in.z[i0] + in.z[i1] + in.z[i2]) * f;
        return;
    }
    if (group_size == 4) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        uint32_t i3 = order[group_start + 3];
        constexpr float f = 0.25f;
        out.x = (in.x[i0] + in.x[i1] + in.x[i2] + in.x[i3]) * f;
        out.y = (in.y[i0] + in.y[i1] + in.y[i2] + in.y[i3]) * f;
        out.z = (in.z[i0] + in.z[i1] + in.z[i2] + in.z[i3]) * f;
        return;
    }

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    size_t k = 0;

    while (k < group_size) {
        size_t vl = __riscv_vsetvl_e32m2(group_size - k);

        // Load point indices for this chunk
        vuint32m2_t v_idx = __riscv_vle32_v_u32m2(
            (const uint32_t*)&order[group_start + k], vl);

        // Convert element indices to byte offsets for float gather
        vuint32m2_t v_byte_off = __riscv_vsll_vx_u32m2(v_idx, 2, vl);

        vfloat32m1_t v_zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
        float chunk_val;

        // Gather x, reduce, accumulate
        vfloat32m2_t vx = __riscv_vluxei32_v_f32m2(in.x, v_byte_off, vl);
        vfloat32m1_t rx = __riscv_vfredosum_vs_f32m2_f32m1(vx, v_zero, vl);
        __riscv_vse32_v_f32m1(&chunk_val, rx, 1);
        sum_x += chunk_val;

        // Gather y, reduce, accumulate
        vfloat32m2_t vy = __riscv_vluxei32_v_f32m2(in.y, v_byte_off, vl);
        vfloat32m1_t ry = __riscv_vfredosum_vs_f32m2_f32m1(vy, v_zero, vl);
        __riscv_vse32_v_f32m1(&chunk_val, ry, 1);
        sum_y += chunk_val;

        // Gather z, reduce, accumulate
        vfloat32m2_t vz = __riscv_vluxei32_v_f32m2(in.z, v_byte_off, vl);
        vfloat32m1_t rz = __riscv_vfredosum_vs_f32m2_f32m1(vz, v_zero, vl);
        __riscv_vse32_v_f32m1(&chunk_val, rz, 1);
        sum_z += chunk_val;

        k += vl;
    }

    float inv_count = 1.0f / (float)group_size;
    out.x = sum_x * inv_count;
    out.y = sum_y * inv_count;
    out.z = sum_z * inv_count;
}
#endif

} // anonymous namespace

// ============================================================================
// Scalar Implementation
// ============================================================================
std::size_t voxel_grid_downsamp_sc(const PointXYZ* in, std::size_t n,
                                   PointXYZ* out, float leaf_size) {
    if (n == 0) return 0;
    
    // Key: (vx, vy, vz), Value: (Sum coordinates, Count)
    std::map<std::tuple<int, int, int>, std::pair<PointXYZ, int>> grid;
    float inv_leaf = 1.0f / leaf_size;

    for (std::size_t i = 0; i < n; ++i) {
        int vx = std::floor(in[i].x * inv_leaf);
        int vy = std::floor(in[i].y * inv_leaf);
        int vz = std::floor(in[i].z * inv_leaf);
        auto key = std::make_tuple(vx, vy, vz);
        
        grid[key].first.x += in[i].x;
        grid[key].first.y += in[i].y;
        grid[key].first.z += in[i].z;
        grid[key].second++;
    }

    // Compute centroids and write to output
    std::size_t count = 0;
    for (auto& kv : grid) {
        float f = 1.0f / kv.second.second;
        out[count].x = kv.second.first.x * f;
        out[count].y = kv.second.first.y * f;
        out[count].z = kv.second.first.z * f;
        count++;
    }
    return count;
}

// ============================================================================
// Fully Vectorized RVV Implementation (v2) -- Sort-Based, No std::map
// ============================================================================
std::size_t voxel_grid_downsamp_rvv_v2(const PointCloudSoA& in,
                                        PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
#if defined(__riscv_vector)
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    // Phase 1: Vectorized bounding box computation
    float min_x, min_y, min_z, max_x, max_y, max_z;
    compute_bbox_rvv(in, min_x, min_y, min_z, max_x, max_y, max_z);

    // Compute global voxel index range using same formula as scalar: floor(coord * inv_leaf)
    int min_ix = (int)std::floor(min_x * inv_leaf);
    int min_iy = (int)std::floor(min_y * inv_leaf);
    int min_iz = (int)std::floor(min_z * inv_leaf);
    int max_ix = (int)std::floor(max_x * inv_leaf);
    int max_iy = (int)std::floor(max_y * inv_leaf);
    int max_iz = (int)std::floor(max_z * inv_leaf);

    int grid_x = (max_ix - min_ix) + 1;
    int grid_y = (max_iy - min_iy) + 1;
    int grid_xy = grid_x * grid_y;

    // Phase 2: Vectorized voxel key computation (uses same floor(coord*inv_leaf) as scalar)
    std::vector<int32_t> keys(n);
    compute_voxel_keys_rvv(in, inv_leaf, min_ix, min_iy, min_iz,
                            grid_x, grid_xy, keys.data(), n);

    // Phase 3: Parallel radix sort indices by voxel key (O(N/P))
    int num_threads = 8;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif

    std::vector<uint32_t> order(n);
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
#endif
    for (size_t i = 0; i < n; ++i) order[i] = static_cast<uint32_t>(i);

    radix_sort_pairs_parallel(keys.data(), order.data(), n, num_threads);

    // Phase 4: Parallel centroid computation across disjoint voxel groups
    std::vector<size_t> chunk_split(num_threads + 1);
    chunk_split[0] = 0;
    chunk_split[num_threads] = n;
    for (int t = 1; t < num_threads; ++t) {
        size_t idx = (n * t) / num_threads;
        while (idx < n && keys[idx] == keys[idx - 1]) {
            idx++;
        }
        chunk_split[t] = idx;
    }

    std::vector<std::vector<PointXYZ>> thread_out(num_threads);
#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        if (tid < num_threads) {
            size_t start = chunk_split[tid];
            size_t end = chunk_split[tid + 1];
            auto& local_pts = thread_out[tid];
            if (end > start) {
                local_pts.reserve(end - start);
                size_t group_start = start;
                for (size_t j = start + 1; j <= end; ++j) {
                    if (j == end || keys[j] != keys[j - 1]) {
                        size_t group_size = j - group_start;
                        PointXYZ pt;
                        centroid_reduce_rvv(in, order.data(), group_start, group_size, pt);
                        local_pts.push_back(pt);
                        group_start = j;
                    }
                }
            }
        }
    }

    size_t out_count = 0;
    for (int t = 0; t < num_threads; ++t) {
        if (!thread_out[t].empty()) {
            std::memcpy(&out[out_count], thread_out[t].data(), thread_out[t].size() * sizeof(PointXYZ));
            out_count += thread_out[t].size();
        }
    }

    return out_count;
#else
    std::vector<PointXYZ> aos(in.n);
    for (size_t i = 0; i < in.n; ++i) {
        aos[i] = {in.x[i], in.y[i], in.z[i]};
    }
    return voxel_grid_downsamp_sc(aos.data(), in.n, out, leaf_size);
#endif
}

} // namespace rvpoint
