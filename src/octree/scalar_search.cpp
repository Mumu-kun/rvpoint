#include "types.h"

#include <cmath>
#include <cstdlib>

extern "C" void morton_range(float qx, float qy, float qz,
                               float r, float voxel_size,
                               uint64_t* out_min, uint64_t* out_max);

long long g_last_candidate_checks_scalar = 0;

int binary_search_lower(uint64_t* morton, int N, uint64_t val)
{
    int lo = 0;
    int hi = N;
    while (lo < hi) {
        int mid = lo + ((hi - lo) >> 1);
        if (morton[static_cast<size_t>(mid)] < val) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

int binary_search_upper(uint64_t* morton, int N, uint64_t val)
{
    int lo = 0;
    int hi = N;
    while (lo < hi) {
        int mid = lo + ((hi - lo) >> 1);
        if (morton[static_cast<size_t>(mid)] <= val) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    return lo;
}

void radiusSearch(struct Cloud* cloud,
                  float qx, float qy, float qz, float r,
                  int* results, int* result_count)
{
    g_last_candidate_checks_scalar = 0;
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
    *result_count = 0;
    if (start > end) {
        return;
    }

    float r2 = r * r;
    for (int i = start; i <= end; ++i) {
        ++g_last_candidate_checks_scalar;
        float dx = cloud->x[static_cast<size_t>(i)] - qx;
        float dy = cloud->y[static_cast<size_t>(i)] - qy;
        float dz = cloud->z[static_cast<size_t>(i)] - qz;
        float d2 = dx * dx + dy * dy + dz * dz;
        if (d2 <= r2) {
            results[*result_count] = cloud->sorted_idx[static_cast<size_t>(i)];
            ++(*result_count);
        }
    }
}

void radiusSearchBatch_scalar(struct Cloud* cloud,
                              struct QueryBatch* batch, struct Results* out)
{
    const int Q = batch->Q;
    const int N = cloud->N;
    out->result_counts = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(Q)));
    out->result_offsets = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(Q + 1)));
    out->result_flat = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(N) * static_cast<size_t>(Q)));
    out->total_results = 0;

    int cursor = 0;
    for (int q = 0; q < Q; ++q) {
        out->result_offsets[q] = cursor;
        int count = 0;
        radiusSearch(cloud,
                     batch->qx[q], batch->qy[q], batch->qz[q],
                     batch->r,
                     out->result_flat + cursor,
                     &count);
        out->result_counts[q] = count;
        cursor += count;
    }
    out->result_offsets[Q] = cursor;
    out->total_results = cursor;

    if (cursor > 0) {
        out->result_flat = static_cast<int*>(std::realloc(out->result_flat,
                                                          sizeof(int) * static_cast<size_t>(cursor)));
    } else {
        std::free(out->result_flat);
        out->result_flat = nullptr;
    }
}