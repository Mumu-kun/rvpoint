#include "types.h"

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#if defined(__has_include)
#  if __has_include(<riscv_vector.h>)
#    include <riscv_vector.h>
#    define RVV_HAS_INTRINSICS 1
#  endif
#endif

#ifndef RVV_HAS_INTRINSICS
#define RVV_HAS_INTRINSICS 0
#endif

extern void morton_range(float qx, float qy, float qz,
                         float r, float voxel_size,
                         uint64_t* out_min, uint64_t* out_max);
extern uint64_t morton_encode(uint32_t ix, uint32_t iy, uint32_t iz);

int g_leaf_threshold = 64;
int g_T = 1024;
int g_C = 32;
int g_vl = 8;

long long g_last_candidate_checks_rvv = 0;
long long g_last_rvv_vector_ops = 0;
long long g_last_rvv_vector_iters = 0;

struct QueryOrder {
    int idx;
    uint64_t morton;
};

static int compare_query_order(const void* lhs, const void* rhs)
{
    const struct QueryOrder* a = (const struct QueryOrder*)lhs;
    const struct QueryOrder* b = (const struct QueryOrder*)rhs;
    if (a->morton < b->morton) {
        return -1;
    }
    if (a->morton > b->morton) {
        return 1;
    }
    return a->idx - b->idx;
}

static int binary_search_lower(uint64_t* morton, int N, uint64_t val)
{
    int lo = 0;
    int hi = N;
    while (lo < hi) {
        int mid = lo + ((hi - lo) >> 1);
        if (morton[mid] < val) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

static int binary_search_upper(uint64_t* morton, int N, uint64_t val)
{
    int lo = 0;
    int hi = N;
    while (lo < hi) {
        int mid = lo + ((hi - lo) >> 1);
        if (morton[mid] <= val) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

void rvv_init(void)
{
#if RVV_HAS_INTRINSICS
    g_vl = (int)__riscv_vsetvl_e32m1(1024);
    if (g_vl < 1) {
        g_vl = 1;
    }
#else
    g_vl = 1;
#endif
    g_leaf_threshold = g_vl * 4;
    g_T = g_vl * 16;
    g_C = 32;
}

static uint64_t query_morton_key(float qx, float qy, float qz, float voxel_size)
{
    uint32_t ix = qx < 0.0f ? 0U : (uint32_t)floorf(qx / voxel_size);
    uint32_t iy = qy < 0.0f ? 0U : (uint32_t)floorf(qy / voxel_size);
    uint32_t iz = qz < 0.0f ? 0U : (uint32_t)floorf(qz / voxel_size);
    return morton_encode(ix, iy, iz);
}

static void option_b_kernel(struct Cloud* cloud,
                            int c_start, int c_end,
                            float r2,
                            float* tile_qx,
                            float* tile_qy,
                            float* tile_qz,
                            int* tile_qi,
                            int tile_size,
                            struct Results* out,
                            int store_results)
{
    if (c_start > c_end || tile_size <= 0) {
        return;
    }

    for (int c = c_start; c <= c_end; ++c) {
        float px = cloud->x[(size_t)(c)];
        float py = cloud->y[(size_t)(c)];
        float pz = cloud->z[(size_t)(c)];
        int p_original_idx = cloud->sorted_idx[(size_t)(c)];

        int q_remaining = tile_size;
        int q_offset = 0;

        while (q_remaining > 0) {
            int vl = q_remaining < g_vl ? q_remaining : g_vl;

#if RVV_HAS_INTRINSICS
            size_t rvvl = __riscv_vsetvl_e32m1((size_t)vl);
            vfloat32m1_t vqx = __riscv_vle32_v_f32m1(tile_qx + q_offset, rvvl);
            vfloat32m1_t vqy = __riscv_vle32_v_f32m1(tile_qy + q_offset, rvvl);
            vfloat32m1_t vqz = __riscv_vle32_v_f32m1(tile_qz + q_offset, rvvl);

            vfloat32m1_t vpx = __riscv_vfmv_v_f_f32m1(px, rvvl);
            vfloat32m1_t vpy = __riscv_vfmv_v_f_f32m1(py, rvvl);
            vfloat32m1_t vpz = __riscv_vfmv_v_f_f32m1(pz, rvvl);

            vfloat32m1_t dx = __riscv_vfsub_vv_f32m1(vpx, vqx, rvvl);
            vfloat32m1_t dy = __riscv_vfsub_vv_f32m1(vpy, vqy, rvvl);
            vfloat32m1_t dz = __riscv_vfsub_vv_f32m1(vpz, vqz, rvvl);

            vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, rvvl);
            d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, rvvl);
            d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, rvvl);
  
vbool32_t mask =
    __riscv_vmfle_vf_f32m1_b32(d2, r2, rvvl);
            uint8_t mask_bytes[(vl + 7) / 8];
            __riscv_vsm_v_b32(mask_bytes, mask, rvvl);
            for (int i = 0; i < vl; ++i) {
                if (mask_bytes[i / 8] & (1u << (i % 8))) {
                    int qi = tile_qi[q_offset + i];
                    if (store_results) {
                        int write_pos = out->result_offsets[qi]
                                    + out->result_counts[qi];
                        out->result_flat[write_pos] = p_original_idx;
                    }
                    ++out->result_counts[qi];
                }
            }

#else
            for (int lane = 0; lane < vl; ++lane) {
                float dx = px - tile_qx[q_offset + lane];
                float dy = py - tile_qy[q_offset + lane];
                float dz = pz - tile_qz[q_offset + lane];
                float d2 = dx * dx + dy * dy + dz * dz;
                if (d2 <= r2) {
                    int qi = tile_qi[q_offset + lane];
                    if (store_results) {
                        int write_pos = out->result_offsets[qi] + out->result_counts[qi];
                        out->result_flat[write_pos] = p_original_idx;
                    }
                    ++out->result_counts[qi];
                }
            }
#endif

            q_remaining -= vl;
            q_offset += vl;
        }

        g_last_candidate_checks_rvv += tile_size;
        g_last_rvv_vector_ops += tile_size;
        g_last_rvv_vector_iters += (tile_size + g_vl - 1) / g_vl;
    }
}

static int radius_search_single(struct Cloud* cloud,
                                float qx, float qy, float qz, float r,
                                int* results, int store_results)
{
    uint64_t range_min = 0;
    uint64_t range_max = 0;
    morton_range(qx, qy, qz, r, cloud->voxel_size, &range_min, &range_max);

    int start = binary_search_lower(cloud->morton, cloud->N, range_min);
    int end = binary_search_upper(cloud->morton, cloud->N, range_max) - 1;
    if (start < 0) {
        start = 0;
    }
    if (end >= cloud->N) {
        end = cloud->N - 1;
    }
    if (start > end) {
        return 0;
    }

    int count = 0;
    float r2 = r * r;

    for (int p = start; p <= end; ) {
        int remaining = end - p + 1;
        int vl = remaining < g_vl ? remaining : g_vl;

#if RVV_HAS_INTRINSICS
        size_t rvvl = __riscv_vsetvl_e32m1((size_t)vl);
        vfloat32m1_t vx = __riscv_vle32_v_f32m1(&cloud->x[p], rvvl);
        vfloat32m1_t vy = __riscv_vle32_v_f32m1(&cloud->y[p], rvvl);
        vfloat32m1_t vz = __riscv_vle32_v_f32m1(&cloud->z[p], rvvl);
        vfloat32m1_t dx = __riscv_vfsub_vf_f32m1(vx, qx, rvvl);
        vfloat32m1_t dy = __riscv_vfsub_vf_f32m1(vy, qy, rvvl);
        vfloat32m1_t dz = __riscv_vfsub_vf_f32m1(vz, qz, rvvl);
        vfloat32m1_t d2 = __riscv_vfmul_vv_f32m1(dx, dx, rvvl);
        d2 = __riscv_vfmacc_vv_f32m1(d2, dy, dy, rvvl);
        d2 = __riscv_vfmacc_vv_f32m1(d2, dz, dz, rvvl);
        float scratch[vl];
        __riscv_vse32_v_f32m1(scratch, d2, rvvl);
        for (int lane = 0; lane < vl; ++lane) {
            if (scratch[lane] <= r2) {
                if (store_results) {
                    results[count] = cloud->sorted_idx[p + lane];
                }
                ++count;
            }
        }
#else
        for (int lane = 0; lane < vl; ++lane) {
            float dx = cloud->x[p + lane] - qx;
            float dy = cloud->y[p + lane] - qy;
            float dz = cloud->z[p + lane] - qz;
            float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 <= r2) {
                if (store_results) {
                    results[count] = cloud->sorted_idx[p + lane];
                }
                ++count;
            }
        }
#endif
        g_last_candidate_checks_rvv += vl;
        g_last_rvv_vector_ops += vl;
        ++g_last_rvv_vector_iters;
        p += vl;
    }

    return count;
}

void radiusSearchBatch_rvv(struct Cloud* cloud,
                           struct Node* nodes,
                           int node_count, struct QueryBatch* batch,
                           struct Results* out)
{
    (void)nodes;
    (void)node_count;

    rvv_init();

    const int Q = batch->Q;
    const int N = cloud->N;
    const float r = batch->r;
    const float r2 = r * r;

    struct QueryOrder* order = (struct QueryOrder*)malloc(sizeof(struct QueryOrder) * (size_t)Q);
    for (int q = 0; q < Q; ++q) {
        order[q].idx = q;
        order[q].morton = query_morton_key(batch->qx[q], batch->qy[q], batch->qz[q], cloud->voxel_size);
    }
    qsort(order, (size_t)Q, sizeof(struct QueryOrder), compare_query_order);

    out->result_counts = (int*)calloc((size_t)Q, sizeof(int));
    out->result_offsets = (int*)malloc(sizeof(int) * (size_t)(Q + 1));
    out->result_flat = NULL;
    out->total_results = 0;

    float* tile_qx = (float*)malloc(sizeof(float) * (size_t)g_T);
    float* tile_qy = (float*)malloc(sizeof(float) * (size_t)g_T);
    float* tile_qz = (float*)malloc(sizeof(float) * (size_t)g_T);
    int* tile_qi = (int*)malloc(sizeof(int) * (size_t)g_T);

    g_last_candidate_checks_rvv = 0;
    g_last_rvv_vector_ops = 0;
    g_last_rvv_vector_iters = 0;

    for (int tile_start = 0; tile_start < Q; tile_start += g_T) {
        int tile_end = tile_start + g_T;
        if (tile_end > Q) {
            tile_end = Q;
        }
        int tile_size = tile_end - tile_start;

        for (int i = 0; i < tile_size; ++i) {
            int qi = order[tile_start + i].idx;
            tile_qx[i] = batch->qx[(size_t)(qi)];
            tile_qy[i] = batch->qy[(size_t)(qi)];
            tile_qz[i] = batch->qz[(size_t)(qi)];
            tile_qi[i] = qi;
        }

        uint64_t sup_min = UINT64_MAX;
        uint64_t sup_max = 0;
        for (int i = 0; i < tile_size; ++i) {
            uint64_t mn = 0;
            uint64_t mx = 0;
            morton_range(tile_qx[i], tile_qy[i], tile_qz[i], r, cloud->voxel_size, &mn, &mx);
            if (mn < sup_min) {
                sup_min = mn;
            }
            if (mx > sup_max) {
                sup_max = mx;
            }
        }

        int c_start = binary_search_lower(cloud->morton, cloud->N, sup_min);
        int c_end = binary_search_upper(cloud->morton, cloud->N, sup_max) - 1;
        if (c_start > c_end) {
            continue;
        }

        option_b_kernel(cloud, c_start, c_end, r2,
                        tile_qx, tile_qy, tile_qz, tile_qi,
                        tile_size, out, 0);
    }

    out->result_offsets[0] = 0;
    for (int q = 0; q < Q; ++q) {
        out->result_offsets[q + 1] = out->result_offsets[q] + out->result_counts[q];
    }
    int total = out->result_offsets[Q];
    out->result_flat = total > 0 ? (int*)malloc(sizeof(int) * (size_t)total) : NULL;
    out->total_results = total;

    memset(out->result_counts, 0, sizeof(int) * (size_t)Q);

    for (int tile_start = 0; tile_start < Q; tile_start += g_T) {
        int tile_end = tile_start + g_T;
        if (tile_end > Q) {
            tile_end = Q;
        }
        int tile_size = tile_end - tile_start;

        for (int i = 0; i < tile_size; ++i) {
            int qi = order[tile_start + i].idx;
            tile_qx[i] = batch->qx[(size_t)(qi)];
            tile_qy[i] = batch->qy[(size_t)(qi)];
            tile_qz[i] = batch->qz[(size_t)(qi)];
            tile_qi[i] = qi;
        }

        uint64_t sup_min = UINT64_MAX;
        uint64_t sup_max = 0;
        for (int i = 0; i < tile_size; ++i) {
            uint64_t mn = 0;
            uint64_t mx = 0;
            morton_range(tile_qx[i], tile_qy[i], tile_qz[i], r, cloud->voxel_size, &mn, &mx);
            if (mn < sup_min) {
                sup_min = mn;
            }
            if (mx > sup_max) {
                sup_max = mx;
            }
        }

        int c_start = binary_search_lower(cloud->morton, cloud->N, sup_min);
        int c_end = binary_search_upper(cloud->morton, cloud->N, sup_max) - 1;
        if (c_start > c_end) {
            continue;
        }

        option_b_kernel(cloud, c_start, c_end, r2,
                        tile_qx, tile_qy, tile_qz, tile_qi,
                        tile_size, out, 1);
    }

    if (total > 0) {
        out->result_flat = (int*)realloc(out->result_flat,
                                         sizeof(int) * (size_t)total);
    } else {
        free(out->result_flat);
        out->result_flat = NULL;
    }

    free(tile_qx);
    free(tile_qy);
    free(tile_qz);
    free(tile_qi);
    free(order);
}