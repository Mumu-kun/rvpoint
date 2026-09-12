#include "filters/voxel_grid.h"
#include "filters/radix_sort.h"
#include <cmath>
#include <map>
#include <tuple>
#include <algorithm>
#include <vector>
#include <numeric>
#include <limits>
#include <cstring>

#if defined(_OPENMP)
#include <omp.h>
#endif

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

// ============================================================================
// Internal Helpers for Vector and Scalar Voxel Grid Downsampling
// ============================================================================
namespace {

#if defined(__riscv_vector)
// Helper: Compute bounding box using RVV reductions (LMUL=m4)
void compute_bbox_rvv(const PointCloudView& in,
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
void compute_voxel_keys_rvv(const PointCloudView& in,
                            float inv_leaf, int min_ix, int min_iy, int min_iz,
                            int grid_x, int grid_xy,
                            int32_t* keys, size_t n, int num_threads) {
#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
    {
        int tid = omp_get_thread_num();
        size_t start = (n * tid) / num_threads;
        size_t end   = (n * (tid + 1)) / num_threads;
        size_t i = start;
        while (i < end) {
            size_t vl = __riscv_vsetvl_e32m4(end - i);

            // Load coordinates
            vfloat32m4_t vx = __riscv_vle32_v_f32m4(&in.x[i], vl);
            vfloat32m4_t vy = __riscv_vle32_v_f32m4(&in.y[i], vl);
            vfloat32m4_t vz = __riscv_vle32_v_f32m4(&in.z[i], vl);

            // Scale: coord * inv_leaf
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
    (void)num_threads;
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m4(n - i);

        // Load coordinates
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(&in.x[i], vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(&in.y[i], vl);
        vfloat32m4_t vz = __riscv_vle32_v_f32m4(&in.z[i], vl);

        // Scale: coord * inv_leaf
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
inline void centroid_reduce_rvv(const PointCloudView& in,
                                const uint32_t* order, size_t group_start,
                                size_t group_size, float& ox, float& oy, float& oz) {
    if (__builtin_expect(group_size == 1, 1)) {
        uint32_t idx = order[group_start];
        ox = in.x[idx];
        oy = in.y[idx];
        oz = in.z[idx];
        return;
    }
    if (group_size == 2) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        ox = 0.5f * (in.x[i0] + in.x[i1]);
        oy = 0.5f * (in.y[i0] + in.y[i1]);
        oz = 0.5f * (in.z[i0] + in.z[i1]);
        return;
    }
    if (group_size == 3) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        constexpr float f = 1.0f / 3.0f;
        ox = (in.x[i0] + in.x[i1] + in.x[i2]) * f;
        oy = (in.y[i0] + in.y[i1] + in.y[i2]) * f;
        oz = (in.z[i0] + in.z[i1] + in.z[i2]) * f;
        return;
    }
    if (group_size == 4) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        uint32_t i3 = order[group_start + 3];
        constexpr float f = 0.25f;
        ox = (in.x[i0] + in.x[i1] + in.x[i2] + in.x[i3]) * f;
        oy = (in.y[i0] + in.y[i1] + in.y[i2] + in.y[i3]) * f;
        oz = (in.z[i0] + in.z[i1] + in.z[i2] + in.z[i3]) * f;
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
    ox = sum_x * inv_count;
    oy = sum_y * inv_count;
    oz = sum_z * inv_count;
}
#endif

// Helper: Scalar bounding box
void compute_bbox_scalar(const PointCloudView& in,
                         float& out_min_x, float& out_min_y, float& out_min_z,
                         float& out_max_x, float& out_max_y, float& out_max_z) {
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float max_y = std::numeric_limits<float>::lowest();
    float max_z = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < in.n; ++i) {
        if (in.x[i] < min_x) min_x = in.x[i];
        if (in.y[i] < min_y) min_y = in.y[i];
        if (in.z[i] < min_z) min_z = in.z[i];
        if (in.x[i] > max_x) max_x = in.x[i];
        if (in.y[i] > max_y) max_y = in.y[i];
        if (in.z[i] > max_z) max_z = in.z[i];
    }

    out_min_x = min_x;
    out_min_y = min_y;
    out_min_z = min_z;
    out_max_x = max_x;
    out_max_y = max_y;
    out_max_z = max_z;
}

// Helper: Scalar voxel keys
void compute_voxel_keys_scalar(const PointCloudView& in,
                               float inv_leaf, int min_ix, int min_iy, int min_iz,
                               int grid_x, int grid_xy,
                               int32_t* keys, size_t n, int num_threads) {
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
    for (size_t i = 0; i < n; ++i) {
        int ix = (int)std::floor(in.x[i] * inv_leaf) - min_ix;
        int iy = (int)std::floor(in.y[i] * inv_leaf) - min_iy;
        int iz = (int)std::floor(in.z[i] * inv_leaf) - min_iz;
        keys[i] = ix + iy * grid_x + iz * grid_xy;
    }
#else
    (void)num_threads;
    for (size_t i = 0; i < n; ++i) {
        int ix = (int)std::floor(in.x[i] * inv_leaf) - min_ix;
        int iy = (int)std::floor(in.y[i] * inv_leaf) - min_iy;
        int iz = (int)std::floor(in.z[i] * inv_leaf) - min_iz;
        keys[i] = ix + iy * grid_x + iz * grid_xy;
    }
#endif
}

// Helper: Scalar centroid reduction
inline void centroid_reduce_scalar(const PointCloudView& in,
                                   const uint32_t* order, size_t group_start,
                                   size_t group_size, float& ox, float& oy, float& oz) {
    if (group_size == 1) {
        uint32_t idx = order[group_start];
        ox = in.x[idx];
        oy = in.y[idx];
        oz = in.z[idx];
        return;
    }
    if (group_size == 2) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        ox = 0.5f * (in.x[i0] + in.x[i1]);
        oy = 0.5f * (in.y[i0] + in.y[i1]);
        oz = 0.5f * (in.z[i0] + in.z[i1]);
        return;
    }
    if (group_size == 3) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        constexpr float f = 1.0f / 3.0f;
        ox = (in.x[i0] + in.x[i1] + in.x[i2]) * f;
        oy = (in.y[i0] + in.y[i1] + in.y[i2]) * f;
        oz = (in.z[i0] + in.z[i1] + in.z[i2]) * f;
        return;
    }
    if (group_size == 4) {
        uint32_t i0 = order[group_start];
        uint32_t i1 = order[group_start + 1];
        uint32_t i2 = order[group_start + 2];
        uint32_t i3 = order[group_start + 3];
        constexpr float f = 0.25f;
        ox = (in.x[i0] + in.x[i1] + in.x[i2] + in.x[i3]) * f;
        oy = (in.y[i0] + in.y[i1] + in.y[i2] + in.y[i3]) * f;
        oz = (in.z[i0] + in.z[i1] + in.z[i2] + in.z[i3]) * f;
        return;
    }

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    for (size_t k = 0; k < group_size; ++k) {
        uint32_t idx = order[group_start + k];
        sum_x += in.x[idx];
        sum_y += in.y[idx];
        sum_z += in.z[idx];
    }
    float inv_count = 1.0f / (float)group_size;
    ox = sum_x * inv_count;
    oy = sum_y * inv_count;
    oz = sum_z * inv_count;
}

} // anonymous namespace

// ============================================================================
// class VoxelGrid Implementation
// ============================================================================

VoxelGrid::VoxelGrid(float leaf_size, Backend backend)
    : leaf_size_(leaf_size), backend_(backend) {}

void VoxelGrid::reserve(std::size_t max_points) {
    int max_threads = 1;
#if defined(_OPENMP)
    max_threads = omp_get_max_threads();
#endif
    if (max_threads < 1) max_threads = 1;

    keys_.reserve(max_points);
    order_.reserve(max_points);
    tmp_k_.reserve(max_points);
    tmp_v_.reserve(max_points);
    chunk_split_.resize(max_threads + 1);

    thread_scratch_.resize(max_threads);
    size_t per_thread_cap = (max_points / max_threads) + 256;
    for (int t = 0; t < max_threads; ++t) {
        thread_scratch_[t].reserve(per_thread_cap);
    }
}

std::size_t VoxelGrid::operator()(const PointCloudView& in, PointCloud& out, float leaf_size) {
    bool use_rvv = false;
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        use_rvv = true;
    }
#else
    if (backend_ == Backend::RVV) {
        use_rvv = false;
    }
#endif

    if (use_rvv) {
        return filter_rvv(in, out, leaf_size);
    } else {
        return filter_scalar(in, out, leaf_size);
    }
}

std::size_t VoxelGrid::filter(const PointCloudView& in, PointXYZ* out, float leaf_size) {
    bool use_rvv = false;
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        use_rvv = true;
    }
#else
    if (backend_ == Backend::RVV) {
        use_rvv = false;
    }
#endif

    if (use_rvv) {
        return filter_rvv_aos(in, out, leaf_size);
    } else {
        return filter_scalar_aos(in, out, leaf_size);
    }
}

std::size_t VoxelGrid::filter_rvv(const PointCloudView& in, PointCloud& out, float leaf_size) {
    if (in.n == 0) {
        out.clear();
        return 0;
    }
#if defined(__riscv_vector)
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    if (n > keys_.capacity()) {
        reserve(n);
    }

    keys_.resize(n);
    order_.resize(n);
    tmp_k_.resize(n);
    tmp_v_.resize(n);

    // Phase 1: Vectorized bounding box
    float min_x, min_y, min_z, max_x, max_y, max_z;
    compute_bbox_rvv(in, min_x, min_y, min_z, max_x, max_y, max_z);

    int min_ix = (int)std::floor(min_x * inv_leaf);
    int min_iy = (int)std::floor(min_y * inv_leaf);
    int min_iz = (int)std::floor(min_z * inv_leaf);
    int max_ix = (int)std::floor(max_x * inv_leaf);
    int max_iy = (int)std::floor(max_y * inv_leaf);
    int max_iz = (int)std::floor(max_z * inv_leaf);

    int grid_x = (max_ix - min_ix) + 1;
    int grid_y = (max_iy - min_iy) + 1;
    int grid_xy = grid_x * grid_y;

    int num_threads = 1;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif
    if (num_threads > (int)thread_scratch_.size()) {
        thread_scratch_.resize(num_threads);
        chunk_split_.resize(num_threads + 1);
    }

    // Phase 2: Vectorized voxel keys
    compute_voxel_keys_rvv(in, inv_leaf, min_ix, min_iy, min_iz,
                           grid_x, grid_xy, keys_.data(), n, num_threads);

    // Initialize order buffer
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
#endif
    for (size_t i = 0; i < n; ++i) order_[i] = static_cast<uint32_t>(i);

    // Phase 3: Parallel radix sort
    radix_sort_pairs_parallel(keys_.data(), order_.data(), n, num_threads, tmp_k_.data(), tmp_v_.data());

    // Phase 4: Chunk split across disjoint voxel groups
    chunk_split_[0] = 0;
    chunk_split_[num_threads] = n;
    for (int t = 1; t < num_threads; ++t) {
        size_t idx = (n * t) / num_threads;
        while (idx < n && keys_[idx] == keys_[idx - 1]) {
            idx++;
        }
        chunk_split_[t] = idx;
    }

    // Phase 5: Centroid computation
#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        if (tid < num_threads) {
            size_t start = chunk_split_[tid];
            size_t end   = chunk_split_[tid + 1];
            auto& scratch = thread_scratch_[tid];
            scratch.clear();

            if (end > start) {
                size_t group_start = start;
                for (size_t j = start + 1; j <= end; ++j) {
                    if (j == end || keys_[j] != keys_[j - 1]) {
                        size_t group_size = j - group_start;
                        float ox, oy, oz;
                        centroid_reduce_rvv(in, order_.data(), group_start, group_size, ox, oy, oz);
                        scratch.push_back(ox, oy, oz);
                        group_start = j;
                    }
                }
            }
        }
    }

    // Gather into out (PointCloud SoA)
    size_t total_count = 0;
    for (int t = 0; t < num_threads; ++t) {
        total_count += thread_scratch_[t].size();
    }

    out.resize(total_count);
    size_t offset = 0;
    for (int t = 0; t < num_threads; ++t) {
        size_t cnt = thread_scratch_[t].size();
        if (cnt > 0) {
            std::memcpy(out.x.data() + offset, thread_scratch_[t].x.data(), cnt * sizeof(float));
            std::memcpy(out.y.data() + offset, thread_scratch_[t].y.data(), cnt * sizeof(float));
            std::memcpy(out.z.data() + offset, thread_scratch_[t].z.data(), cnt * sizeof(float));
            offset += cnt;
        }
    }

    return total_count;
#else
    return filter_scalar(in, out, leaf_size);
#endif
}

std::size_t VoxelGrid::filter_scalar(const PointCloudView& in, PointCloud& out, float leaf_size) {
    if (in.n == 0) {
        out.clear();
        return 0;
    }
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    if (n > keys_.capacity()) {
        reserve(n);
    }

    keys_.resize(n);
    order_.resize(n);
    tmp_k_.resize(n);
    tmp_v_.resize(n);

    // Phase 1: Scalar bounding box
    float min_x, min_y, min_z, max_x, max_y, max_z;
    compute_bbox_scalar(in, min_x, min_y, min_z, max_x, max_y, max_z);

    int min_ix = (int)std::floor(min_x * inv_leaf);
    int min_iy = (int)std::floor(min_y * inv_leaf);
    int min_iz = (int)std::floor(min_z * inv_leaf);
    int max_ix = (int)std::floor(max_x * inv_leaf);
    int max_iy = (int)std::floor(max_y * inv_leaf);
    int max_iz = (int)std::floor(max_z * inv_leaf);

    int grid_x = (max_ix - min_ix) + 1;
    int grid_y = (max_iy - min_iy) + 1;
    int grid_xy = grid_x * grid_y;

    int num_threads = 1;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif
    if (num_threads > (int)thread_scratch_.size()) {
        thread_scratch_.resize(num_threads);
        chunk_split_.resize(num_threads + 1);
    }

    // Phase 2: Compute voxel keys
    compute_voxel_keys_scalar(in, inv_leaf, min_ix, min_iy, min_iz,
                              grid_x, grid_xy, keys_.data(), n, num_threads);

    // Initialize order buffer
#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
#endif
    for (size_t i = 0; i < n; ++i) order_[i] = static_cast<uint32_t>(i);

    // Phase 3: Parallel radix sort
    radix_sort_pairs_parallel(keys_.data(), order_.data(), n, num_threads, tmp_k_.data(), tmp_v_.data());

    // Phase 4: Chunk split
    chunk_split_[0] = 0;
    chunk_split_[num_threads] = n;
    for (int t = 1; t < num_threads; ++t) {
        size_t idx = (n * t) / num_threads;
        while (idx < n && keys_[idx] == keys_[idx - 1]) {
            idx++;
        }
        chunk_split_[t] = idx;
    }

    // Phase 5: Centroid computation
#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        if (tid < num_threads) {
            size_t start = chunk_split_[tid];
            size_t end   = chunk_split_[tid + 1];
            auto& scratch = thread_scratch_[tid];
            scratch.clear();

            if (end > start) {
                size_t group_start = start;
                for (size_t j = start + 1; j <= end; ++j) {
                    if (j == end || keys_[j] != keys_[j - 1]) {
                        size_t group_size = j - group_start;
                        float ox, oy, oz;
                        centroid_reduce_scalar(in, order_.data(), group_start, group_size, ox, oy, oz);
                        scratch.push_back(ox, oy, oz);
                        group_start = j;
                    }
                }
            }
        }
    }

    // Gather into out (PointCloud SoA)
    size_t total_count = 0;
    for (int t = 0; t < num_threads; ++t) {
        total_count += thread_scratch_[t].size();
    }

    out.resize(total_count);
    size_t offset = 0;
    for (int t = 0; t < num_threads; ++t) {
        size_t cnt = thread_scratch_[t].size();
        if (cnt > 0) {
            std::memcpy(out.x.data() + offset, thread_scratch_[t].x.data(), cnt * sizeof(float));
            std::memcpy(out.y.data() + offset, thread_scratch_[t].y.data(), cnt * sizeof(float));
            std::memcpy(out.z.data() + offset, thread_scratch_[t].z.data(), cnt * sizeof(float));
            offset += cnt;
        }
    }

    return total_count;
}

std::size_t VoxelGrid::filter_rvv_aos(const PointCloudView& in, PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
#if defined(__riscv_vector)
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    if (n > keys_.capacity()) {
        reserve(n);
    }

    keys_.resize(n);
    order_.resize(n);
    tmp_k_.resize(n);
    tmp_v_.resize(n);

    float min_x, min_y, min_z, max_x, max_y, max_z;
    compute_bbox_rvv(in, min_x, min_y, min_z, max_x, max_y, max_z);

    int min_ix = (int)std::floor(min_x * inv_leaf);
    int min_iy = (int)std::floor(min_y * inv_leaf);
    int min_iz = (int)std::floor(min_z * inv_leaf);
    int max_ix = (int)std::floor(max_x * inv_leaf);
    int max_iy = (int)std::floor(max_y * inv_leaf);
    int max_iz = (int)std::floor(max_z * inv_leaf);

    int grid_x = (max_ix - min_ix) + 1;
    int grid_y = (max_iy - min_iy) + 1;
    int grid_xy = grid_x * grid_y;

    int num_threads = 1;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif
    if (num_threads > (int)thread_scratch_.size()) {
        thread_scratch_.resize(num_threads);
        chunk_split_.resize(num_threads + 1);
    }

    compute_voxel_keys_rvv(in, inv_leaf, min_ix, min_iy, min_iz,
                           grid_x, grid_xy, keys_.data(), n, num_threads);

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
#endif
    for (size_t i = 0; i < n; ++i) order_[i] = static_cast<uint32_t>(i);

    radix_sort_pairs_parallel(keys_.data(), order_.data(), n, num_threads, tmp_k_.data(), tmp_v_.data());

    chunk_split_[0] = 0;
    chunk_split_[num_threads] = n;
    for (int t = 1; t < num_threads; ++t) {
        size_t idx = (n * t) / num_threads;
        while (idx < n && keys_[idx] == keys_[idx - 1]) {
            idx++;
        }
        chunk_split_[t] = idx;
    }

#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        if (tid < num_threads) {
            size_t start = chunk_split_[tid];
            size_t end   = chunk_split_[tid + 1];
            auto& scratch = thread_scratch_[tid];
            scratch.clear();

            if (end > start) {
                size_t group_start = start;
                for (size_t j = start + 1; j <= end; ++j) {
                    if (j == end || keys_[j] != keys_[j - 1]) {
                        size_t group_size = j - group_start;
                        float ox, oy, oz;
                        centroid_reduce_rvv(in, order_.data(), group_start, group_size, ox, oy, oz);
                        scratch.push_back(ox, oy, oz);
                        group_start = j;
                    }
                }
            }
        }
    }

    size_t offset = 0;
    for (int t = 0; t < num_threads; ++t) {
        size_t cnt = thread_scratch_[t].size();
        for (size_t j = 0; j < cnt; ++j) {
            out[offset + j] = { thread_scratch_[t].x[j], thread_scratch_[t].y[j], thread_scratch_[t].z[j] };
        }
        offset += cnt;
    }

    return offset;
#else
    return filter_scalar_aos(in, out, leaf_size);
#endif
}

std::size_t VoxelGrid::filter_scalar_aos(const PointCloudView& in, PointXYZ* out, float leaf_size) {
    if (in.n == 0) return 0;
    const size_t n = in.n;
    const float inv_leaf = 1.0f / leaf_size;

    if (n > keys_.capacity()) {
        reserve(n);
    }

    keys_.resize(n);
    order_.resize(n);
    tmp_k_.resize(n);
    tmp_v_.resize(n);

    float min_x, min_y, min_z, max_x, max_y, max_z;
    compute_bbox_scalar(in, min_x, min_y, min_z, max_x, max_y, max_z);

    int min_ix = (int)std::floor(min_x * inv_leaf);
    int min_iy = (int)std::floor(min_y * inv_leaf);
    int min_iz = (int)std::floor(min_z * inv_leaf);
    int max_ix = (int)std::floor(max_x * inv_leaf);
    int max_iy = (int)std::floor(max_y * inv_leaf);
    int max_iz = (int)std::floor(max_z * inv_leaf);

    int grid_x = (max_ix - min_ix) + 1;
    int grid_y = (max_iy - min_iy) + 1;
    int grid_xy = grid_x * grid_y;

    int num_threads = 1;
#if defined(_OPENMP)
    num_threads = omp_get_max_threads();
#endif
    if (num_threads > (int)thread_scratch_.size()) {
        thread_scratch_.resize(num_threads);
        chunk_split_.resize(num_threads + 1);
    }

    compute_voxel_keys_scalar(in, inv_leaf, min_ix, min_iy, min_iz,
                              grid_x, grid_xy, keys_.data(), n, num_threads);

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(num_threads)
#endif
    for (size_t i = 0; i < n; ++i) order_[i] = static_cast<uint32_t>(i);

    radix_sort_pairs_parallel(keys_.data(), order_.data(), n, num_threads, tmp_k_.data(), tmp_v_.data());

    chunk_split_[0] = 0;
    chunk_split_[num_threads] = n;
    for (int t = 1; t < num_threads; ++t) {
        size_t idx = (n * t) / num_threads;
        while (idx < n && keys_[idx] == keys_[idx - 1]) {
            idx++;
        }
        chunk_split_[t] = idx;
    }

#if defined(_OPENMP)
    #pragma omp parallel num_threads(num_threads)
#endif
    {
        int tid = 0;
#if defined(_OPENMP)
        tid = omp_get_thread_num();
#endif
        if (tid < num_threads) {
            size_t start = chunk_split_[tid];
            size_t end   = chunk_split_[tid + 1];
            auto& scratch = thread_scratch_[tid];
            scratch.clear();

            if (end > start) {
                size_t group_start = start;
                for (size_t j = start + 1; j <= end; ++j) {
                    if (j == end || keys_[j] != keys_[j - 1]) {
                        size_t group_size = j - group_start;
                        float ox, oy, oz;
                        centroid_reduce_scalar(in, order_.data(), group_start, group_size, ox, oy, oz);
                        scratch.push_back(ox, oy, oz);
                        group_start = j;
                    }
                }
            }
        }
    }

    size_t offset = 0;
    for (int t = 0; t < num_threads; ++t) {
        size_t cnt = thread_scratch_[t].size();
        for (size_t j = 0; j < cnt; ++j) {
            out[offset + j] = { thread_scratch_[t].x[j], thread_scratch_[t].y[j], thread_scratch_[t].z[j] };
        }
        offset += cnt;
    }

    return offset;
}

} // namespace rvpoint

