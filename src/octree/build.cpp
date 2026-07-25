#include "types.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

extern "C" uint64_t morton_encode(uint32_t ix, uint32_t iy, uint32_t iz);

struct RangeItem {
    int start;
    int end;
};

void cloud_alloc(struct Cloud* cloud, int N, float voxel_size)
{
    cloud->x = static_cast<float*>(std::malloc(sizeof(float) * static_cast<size_t>(N)));
    cloud->y = static_cast<float*>(std::malloc(sizeof(float) * static_cast<size_t>(N)));
    cloud->z = static_cast<float*>(std::malloc(sizeof(float) * static_cast<size_t>(N)));
    cloud->morton = static_cast<uint64_t*>(std::malloc(sizeof(uint64_t) * static_cast<size_t>(N)));
    cloud->sorted_idx = static_cast<int*>(std::malloc(sizeof(int) * static_cast<size_t>(N)));
    cloud->N = N;
    cloud->voxel_size = voxel_size;
}

void cloud_free(struct Cloud* cloud)
{
    std::free(cloud->x);
    std::free(cloud->y);
    std::free(cloud->z);
    std::free(cloud->morton);
    std::free(cloud->sorted_idx);
    cloud->x = nullptr;
    cloud->y = nullptr;
    cloud->z = nullptr;
    cloud->morton = nullptr;
    cloud->sorted_idx = nullptr;
    cloud->N = 0;
    cloud->voxel_size = 0.0f;
}

void build(struct Cloud* cloud, struct Node** nodes, int* node_count, int leaf_threshold)
{
    if (cloud == nullptr || cloud->N <= 0) {
        *nodes = nullptr;
        *node_count = 0;
        return;
    }

    if (leaf_threshold < 1) {
        leaf_threshold = 1;
    }

    const int N = cloud->N;
    std::vector<int> order(static_cast<size_t>(N));
    for (int i = 0; i < N; ++i) {
        float fx = cloud->x[static_cast<size_t>(i)] / cloud->voxel_size;
        float fy = cloud->y[static_cast<size_t>(i)] / cloud->voxel_size;
        float fz = cloud->z[static_cast<size_t>(i)] / cloud->voxel_size;
        uint32_t ix = fx < 0.0f ? 0U : static_cast<uint32_t>(std::floor(fx));
        uint32_t iy = fy < 0.0f ? 0U : static_cast<uint32_t>(std::floor(fy));
        uint32_t iz = fz < 0.0f ? 0U : static_cast<uint32_t>(std::floor(fz));
        cloud->morton[static_cast<size_t>(i)] = morton_encode(ix, iy, iz);
        cloud->sorted_idx[static_cast<size_t>(i)] = i;
        order[static_cast<size_t>(i)] = i;
    }

    std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
        uint64_t lm = cloud->morton[static_cast<size_t>(lhs)];
        uint64_t rm = cloud->morton[static_cast<size_t>(rhs)];
        if (lm != rm) {
            return lm < rm;
        }
        return lhs < rhs;
    });

    std::vector<float> x_sorted(static_cast<size_t>(N));
    std::vector<float> y_sorted(static_cast<size_t>(N));
    std::vector<float> z_sorted(static_cast<size_t>(N));
    std::vector<uint64_t> morton_sorted(static_cast<size_t>(N));
    std::vector<int> idx_sorted(static_cast<size_t>(N));

    for (int i = 0; i < N; ++i) {
        int src = order[static_cast<size_t>(i)];
        x_sorted[static_cast<size_t>(i)] = cloud->x[static_cast<size_t>(src)];
        y_sorted[static_cast<size_t>(i)] = cloud->y[static_cast<size_t>(src)];
        z_sorted[static_cast<size_t>(i)] = cloud->z[static_cast<size_t>(src)];
        morton_sorted[static_cast<size_t>(i)] = cloud->morton[static_cast<size_t>(src)];
        idx_sorted[static_cast<size_t>(i)] = cloud->sorted_idx[static_cast<size_t>(src)];
    }

    std::memcpy(cloud->x, x_sorted.data(), sizeof(float) * static_cast<size_t>(N));
    std::memcpy(cloud->y, y_sorted.data(), sizeof(float) * static_cast<size_t>(N));
    std::memcpy(cloud->z, z_sorted.data(), sizeof(float) * static_cast<size_t>(N));
    std::memcpy(cloud->morton, morton_sorted.data(), sizeof(uint64_t) * static_cast<size_t>(N));
    std::memcpy(cloud->sorted_idx, idx_sorted.data(), sizeof(int) * static_cast<size_t>(N));

    std::vector<struct Node> flat_nodes;
    flat_nodes.reserve(static_cast<size_t>(N) * 2U + 1U);

    std::vector<RangeItem> stack;
    stack.push_back({0, N - 1});

    while (!stack.empty()) {
        RangeItem range = stack.back();
        stack.pop_back();

        struct Node node;
        node.point_start = range.start;
        node.point_count = range.end - range.start + 1;
        node.morton_min = cloud->morton[static_cast<size_t>(range.start)];
        node.morton_max = cloud->morton[static_cast<size_t>(range.end)];
        node.is_leaf = node.point_count <= leaf_threshold || node.morton_min == node.morton_max;
        flat_nodes.push_back(node);

        if (node.is_leaf) {
            continue;
        }

        uint64_t diff = node.morton_min ^ node.morton_max;
        if (diff == 0U) {
            continue;
        }

        int highest_bit = 63 - __builtin_clzll(diff);
        int split_group = highest_bit / 3;
        uint64_t shift = static_cast<uint64_t>(split_group) * 3U;

        std::vector<RangeItem> children;
        int child_start = range.start;
        while (child_start <= range.end) {
            uint64_t child_key = (cloud->morton[static_cast<size_t>(child_start)] >> shift) & 7ULL;
            int child_end = child_start;
            while (child_end + 1 <= range.end) {
                uint64_t next_key = (cloud->morton[static_cast<size_t>(child_end + 1)] >> shift) & 7ULL;
                if (next_key != child_key) {
                    break;
                }
                ++child_end;
            }
            children.push_back({child_start, child_end});
            child_start = child_end + 1;
        }

        if (children.size() <= 1U) {
            flat_nodes.back().is_leaf = true;
            continue;
        }

        for (int i = static_cast<int>(children.size()) - 1; i >= 0; --i) {
            stack.push_back(children[static_cast<size_t>(i)]);
        }
    }

    *node_count = static_cast<int>(flat_nodes.size());
    *nodes = static_cast<struct Node*>(std::malloc(sizeof(struct Node) * flat_nodes.size()));
    std::memcpy(*nodes, flat_nodes.data(), sizeof(struct Node) * flat_nodes.size());
}