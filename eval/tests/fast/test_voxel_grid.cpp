#include "core/point_types.h"
#include "filters/voxel_grid.h"
#include <vector>
#include <random>
#include <iostream>
#include <cmath>
#include <algorithm>

using namespace rvpoint;

static bool are_points_close(float ax, float ay, float az, float bx, float by, float bz, float eps = 1e-4f) {
    return std::abs(ax - bx) < eps &&
           std::abs(ay - by) < eps &&
           std::abs(az - bz) < eps;
}

int main() {
    std::cout << "=== Running test_voxel_grid ===" << std::endl;
    const size_t N = 1000;
    const float LEAF = 0.5f;

    PointCloud cloud;
    cloud.reserve(N);

    std::mt19937 gen(42);
    std::uniform_real_distribution<float> dist(0.0f, 10.0f);

    for (size_t i = 0; i < N; ++i) {
        cloud.push_back(dist(gen), dist(gen), dist(gen));
    }

    // 1. Test class VoxelGrid Functor with RVV and Scalar backends
    VoxelGrid vg_rvv(LEAF, Backend::RVV);
    VoxelGrid vg_scalar(LEAF, Backend::Scalar);

    vg_rvv.reserve(N);
    vg_scalar.reserve(N);

    PointCloud out_rvv;
    PointCloud out_scalar;
    out_rvv.reserve(N);
    out_scalar.reserve(N);

    size_t count_rvv = vg_rvv(cloud.view(), out_rvv, LEAF);
    size_t count_scalar = vg_scalar(cloud.view(), out_scalar, LEAF);

    std::cout << "VoxelGrid RVV Count:    " << count_rvv << " (out.size()=" << out_rvv.size() << ")" << std::endl;
    std::cout << "VoxelGrid Scalar Count: " << count_scalar << " (out.size()=" << out_scalar.size() << ")" << std::endl;

    if (count_rvv != count_scalar || out_rvv.size() != count_rvv || out_scalar.size() != count_scalar) {
        std::cerr << "[FAIL] VoxelGrid RVV vs Scalar count mismatch!" << std::endl;
        return 1;
    }

    // Sort both sets of centroids and verify parity
    std::vector<PointXYZ> pts_rvv(count_rvv);
    std::vector<PointXYZ> pts_scalar(count_scalar);
    for (size_t i = 0; i < count_rvv; ++i) {
        pts_rvv[i] = {out_rvv.x[i], out_rvv.y[i], out_rvv.z[i]};
        pts_scalar[i] = {out_scalar.x[i], out_scalar.y[i], out_scalar.z[i]};
    }

    auto sort_fn = [](const PointXYZ& a, const PointXYZ& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    };
    std::sort(pts_rvv.begin(), pts_rvv.end(), sort_fn);
    std::sort(pts_scalar.begin(), pts_scalar.end(), sort_fn);

    for (size_t i = 0; i < count_rvv; ++i) {
        if (!are_points_close(pts_rvv[i].x, pts_rvv[i].y, pts_rvv[i].z,
                              pts_scalar[i].x, pts_scalar[i].y, pts_scalar[i].z)) {
            std::cerr << "[FAIL] Centroid parity mismatch at index " << i << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] VoxelGrid class RVV vs Scalar parity verified." << std::endl;

    // 2. Test multi-frame warmup and zero-reallocation stability
    for (int frame = 0; frame < 5; ++frame) {
        size_t c = vg_rvv(cloud.view(), out_rvv);
        if (c != count_rvv) {
            std::cerr << "[FAIL] Multi-frame downsampling inconsistency on frame " << frame << std::endl;
            return 1;
        }
    }
    std::cout << "[PASS] Multi-frame execution stability verified." << std::endl;

    // 3. Test AoS filter export methods
    std::vector<PointXYZ> legacy_sc_out(N);
    std::vector<PointXYZ> legacy_rvv_out(N);
    size_t leg_sc_cnt = vg_scalar.filter(cloud.view(), legacy_sc_out.data(), LEAF);
    size_t leg_rvv_cnt = vg_rvv.filter(cloud.view(), legacy_rvv_out.data(), LEAF);

    if (leg_sc_cnt != count_rvv || leg_rvv_cnt != count_rvv) {
        std::cerr << "[FAIL] AoS filter export count mismatch: leg_sc=" << leg_sc_cnt << ", leg_rvv=" << leg_rvv_cnt << std::endl;
        return 1;
    }
    std::cout << "[PASS] AoS export filter methods verified." << std::endl;

    // 4. Test edge cases: empty cloud
    PointCloud empty_in;
    PointCloud empty_out;
    size_t empty_cnt = vg_rvv(empty_in.view(), empty_out);
    if (empty_cnt != 0 || !empty_out.empty()) {
        std::cerr << "[FAIL] Empty cloud downsampling failed!" << std::endl;
        return 1;
    }
    std::cout << "[PASS] Empty cloud handled correctly." << std::endl;

    std::cout << "All VoxelGrid tests PASS." << std::endl;
    return 0;
}
