#include "types.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

extern void cloud_alloc(struct Cloud* cloud, int N, float voxel_size);
extern void cloud_free(struct Cloud* cloud);
extern void build(struct Cloud* cloud, struct Node** nodes, int* node_count, int leaf_threshold);
extern void radiusSearchBatch_scalar(struct Cloud* cloud, struct QueryBatch* batch, struct Results* out);
extern "C" void radiusSearchBatch_rvv(struct Cloud* cloud, struct Node* nodes, int node_count, struct QueryBatch* batch, struct Results* out);

static void free_results(struct Results* results)
{
    std::free(results->result_flat);
    std::free(results->result_offsets);
    std::free(results->result_counts);
    results->result_flat = nullptr;
    results->result_offsets = nullptr;
    results->result_counts = nullptr;
    results->total_results = 0;
}

static void sort_slice(std::vector<int>& values)
{
    std::sort(values.begin(), values.end());
}

static std::vector<int> slice_results(const struct Results& results, int query_index)
{
    int begin = results.result_offsets[query_index];
    int end = results.result_offsets[query_index + 1];
    std::vector<int> values;
    values.reserve(static_cast<size_t>(end - begin));
    for (int i = begin; i < end; ++i) {
        values.push_back(results.result_flat[i]);
    }
    return values;
}

static bool compare_query_results(int query_index,
                                  float qx, float qy, float qz, float r,
                                  const struct Results& scalar,
                                  const struct Results& other,
                                  int* mismatch_count)
{
    std::vector<int> scalar_values = slice_results(scalar, query_index);
    std::vector<int> other_values = slice_results(other, query_index);
    sort_slice(scalar_values);
    sort_slice(other_values);

    if (scalar_values == other_values) {
        return true;
    }

    ++(*mismatch_count);
    std::printf("FAIL query %d: point=(%.3f,%.3f,%.3f) r=%.3f\n", query_index, qx, qy, qz, r);
    std::printf("  scalar found: [");
    for (size_t i = 0; i < scalar_values.size(); ++i) {
        std::printf("%d%s", scalar_values[i], i + 1 == scalar_values.size() ? "" : ", ");
    }
    std::printf("]\n");
    std::printf("  rvv found:    [");
    for (size_t i = 0; i < other_values.size(); ++i) {
        std::printf("%d%s", other_values[i], i + 1 == other_values.size() ? "" : ", ");
    }
    std::printf("]\n");

    std::printf("  missing: [");
    bool first = true;
    for (int value : scalar_values) {
        if (!std::binary_search(other_values.begin(), other_values.end(), value)) {
            std::printf(first ? "%d" : ", %d", value);
            first = false;
        }
    }
    std::printf("]\n");

    std::printf("  extra:   [");
    first = true;
    for (int value : other_values) {
        if (!std::binary_search(scalar_values.begin(), scalar_values.end(), value)) {
            std::printf(first ? "%d" : ", %d", value);
            first = false;
        }
    }
    std::printf("]\n");
    return false;
}

int main()
{
    const int N = 100000;
    const int Q = 1000;
    const float voxel_size = 0.01f;
    const float radius = voxel_size * 2.0f;

    std::mt19937 rng_cloud(42);
    std::uniform_real_distribution<float> uniform01(0.0f, 1.0f);
    std::mt19937 rng_query(99);

    struct Cloud cloud = {};
    cloud_alloc(&cloud, N, voxel_size);

    for (int i = 0; i < N; ++i) {
        cloud.x[static_cast<size_t>(i)] = uniform01(rng_cloud);
        cloud.y[static_cast<size_t>(i)] = uniform01(rng_cloud);
        cloud.z[static_cast<size_t>(i)] = uniform01(rng_cloud);
    }

    struct Node* nodes = nullptr;
    int node_count = 0;
    build(&cloud, &nodes, &node_count, 64);

    std::vector<float> qx(static_cast<size_t>(Q));
    std::vector<float> qy(static_cast<size_t>(Q));
    std::vector<float> qz(static_cast<size_t>(Q));
    for (int i = 0; i < Q; ++i) {
        qx[static_cast<size_t>(i)] = uniform01(rng_query);
        qy[static_cast<size_t>(i)] = uniform01(rng_query);
        qz[static_cast<size_t>(i)] = uniform01(rng_query);
    }

    struct QueryBatch batch = {};
    batch.qx = qx.data();
    batch.qy = qy.data();
    batch.qz = qz.data();
    batch.Q = Q;
    batch.r = radius;

    struct Results scalar_a = {};
    struct Results scalar_b = {};
    struct Results rvv = {};

    radiusSearchBatch_scalar(&cloud, &batch, &scalar_a);
    radiusSearchBatch_scalar(&cloud, &batch, &scalar_b);

    int mismatch_count = 0;
    for (int q = 0; q < Q; ++q) {
        compare_query_results(q,
                              qx[static_cast<size_t>(q)],
                              qy[static_cast<size_t>(q)],
                              qz[static_cast<size_t>(q)],
                              radius,
                              scalar_a,
                              scalar_b,
                              &mismatch_count);
    }

    if (mismatch_count != 0) {
        std::printf("FAIL: %d scalar sanity mismatches\n", mismatch_count);
        free_results(&scalar_a);
        free_results(&scalar_b);
        cloud_free(&cloud);
        std::free(nodes);
        return 1;
    }

    radiusSearchBatch_rvv(&cloud, nodes, node_count, &batch, &rvv);

    mismatch_count = 0;
    for (int q = 0; q < Q; ++q) {
        compare_query_results(q,
                              qx[static_cast<size_t>(q)],
                              qy[static_cast<size_t>(q)],
                              qz[static_cast<size_t>(q)],
                              radius,
                              scalar_a,
                              rvv,
                              &mismatch_count);
    }

    if (mismatch_count == 0) {
        std::printf("PASS: %d queries verified\n", Q);
        free_results(&scalar_a);
        free_results(&scalar_b);
        free_results(&rvv);
        cloud_free(&cloud);
        std::free(nodes);
        return 0;
    }

    std::printf("FAIL: %d mismatches\n", mismatch_count);
    free_results(&scalar_a);
    free_results(&scalar_b);
    free_results(&rvv);
    cloud_free(&cloud);
    std::free(nodes);
    return 1;
}