#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

using namespace rvv_pcl;

struct FastPRNG {
    uint64_t state;
    explicit FastPRNG(uint64_t seed = 0x853c49e6748fea9bULL) : state(seed != 0 ? seed : 0x853c49e6748fea9bULL) {}
    inline uint32_t next() {
        uint64_t x = state;
        x ^= x >> 12;
        x ^= x << 25;
        x ^= x >> 27;
        state = x;
        return static_cast<uint32_t>((x * 0x2545F4914F6CDD1DULL) >> 32);
    }
    inline uint32_t next_bounded(uint32_t bound) {
        return bound > 0 ? (next() % bound) : 0;
    }
};

static bool compute_plane_coeffs(float x1, float y1, float z1,
                                 float x2, float y2, float z2,
                                 float x3, float y3, float z3,
                                 float* model) 
{
    float v1x = x2 - x1, v1y = y2 - y1, v1z = z2 - z1;
    float v2x = x3 - x1, v2y = y3 - y1, v2z = z3 - z1;
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < 1e-4f) return false;
    a /= norm; b /= norm; c /= norm;
    model[0] = a; model[1] = b; model[2] = c;
    model[3] = -(a*x1 + b*y1 + c*z1);
    return true;
}

// Ultra-Accurate Strided Subsample Screening + Refinement
static int ransac_fast_uniform_screened(const PointCloudSoA& cloud, float dist_thresh, int max_iters, float* model) {
    if (cloud.n < 3) return 0;
    FastPRNG rng(42);
    float best_model[4] = {0,0,0,0};
    int best_sample_inliers = 0;
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);

    // Uniform strided sample covering the ENTIRE cloud evenly
    const size_t sample_sz = std::min(cloud.n, static_cast<size_t>(2048));
    std::vector<float> sx(sample_sz), sy(sample_sz), sz(sample_sz);
    size_t stride = std::max<size_t>(1, cloud.n / sample_sz);
    for (size_t k = 0; k < sample_sz; ++k) {
        size_t idx = std::min(k * stride, cloud.n - 1);
        sx[k] = cloud.x[idx]; sy[k] = cloud.y[idx]; sz[k] = cloud.z[idx];
    }

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        int i1 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        int i2 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        int i3 = static_cast<int>(rng.next_bounded(static_cast<uint32_t>(cloud.n)));
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if (!compute_plane_coeffs(cloud.x[i1], cloud.y[i1], cloud.z[i1],
                                 cloud.x[i2], cloud.y[i2], cloud.z[i2],
                                 cloud.x[i3], cloud.y[i3], cloud.z[i3], cand_model)) continue;

        if (std::abs(cand_model[2]) < 0.707f) continue;
        float a = cand_model[0], b = cand_model[1], c = cand_model[2], d = cand_model[3];

        // Evaluate candidate on uniform 2048-point strided sample in RVV
        int sample_inliers = 0;
#if defined(__riscv_vector)
        size_t si = 0;
        while (si < sample_sz) {
            size_t vl = __riscv_vsetvl_e32m8(sample_sz - si);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&sx[si], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&sy[si], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&sz[si], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            sample_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
            si += vl;
        }
#else
        for (size_t pt = 0; pt < sample_sz; ++pt) {
            float dist = std::abs(a * sx[pt] + b * sy[pt] + c * sz[pt] + d);
            if (dist <= dist_thresh) sample_inliers++;
        }
#endif

        if (sample_inliers > best_sample_inliers) {
            best_sample_inliers = sample_inliers;
            for (int k = 0; k < 4; k++) best_model[k] = cand_model[k];
            double w = static_cast<double>(best_sample_inliers) / static_cast<double>(sample_sz);
            double p_no_outliers = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) k_iters = dynamic_k;
            }
        }
    }

    // Final single full evaluation on the winning best model
    float a = best_model[0], b = best_model[1], c = best_model[2], d = best_model[3];
    int total_inliers = 0;
#if defined(__riscv_vector)
    size_t n = cloud.n, i = 0;
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
        total_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
        i += vl;
    }
#else
    for (size_t pt = 0; pt < cloud.n; ++pt) {
        float dist = std::abs(a * cloud.x[pt] + b * cloud.y[pt] + c * cloud.z[pt] + d);
        if (dist <= dist_thresh) total_inliers++;
    }
#endif

    for (int k = 0; k < 4; k++) model[k] = best_model[k];
    return total_inliers;
}

int main() {
    std::vector<PointXYZ> pts;
    if (loadPCD("data/pcd_compressed/0000000045.pcd", pts) < 0) return 1;

    std::vector<float> x(pts.size()), y(pts.size()), z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { x[i] = pts[i].x; y[i] = pts[i].y; z[i] = pts[i].z; }
    PointCloudSoA cloud{x.data(), y.data(), z.data(), pts.size()};

    // 1. Leaf 0.10
    std::vector<PointXYZ> ds10(pts.size());
    size_t nd10 = voxel_grid_downsamp_rvv_v2(cloud, ds10.data(), 0.10f);
    ds10.resize(nd10);
    std::vector<float> dx10(nd10), dy10(nd10), dz10(nd10);
    for (size_t i = 0; i < nd10; ++i) { dx10[i] = ds10[i].x; dy10[i] = ds10[i].y; dz10[i] = ds10[i].z; }
    PointCloudSoA dcloud10{dx10.data(), dy10.data(), dz10.data(), nd10};

    float model10[4];
    auto t1 = std::chrono::high_resolution_clock::now();
    int inliers10 = ransac_fast_uniform_screened(dcloud10, 0.20f, 250, model10);
    auto t2 = std::chrono::high_resolution_clock::now();
    double ms10 = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "[Leaf 0.10] Inliers=" << inliers10 << " (" << (inliers10 * 100.0f / nd10) << "%), time=" << ms10 << " ms" << std::endl;

    // 2. Leaf 0.02
    std::vector<PointXYZ> ds02(pts.size());
    size_t nd02 = voxel_grid_downsamp_rvv_v2(cloud, ds02.data(), 0.02f);
    ds02.resize(nd02);
    std::vector<float> dx02(nd02), dy02(nd02), dz02(nd02);
    for (size_t i = 0; i < nd02; ++i) { dx02[i] = ds02[i].x; dy02[i] = ds02[i].y; dz02[i] = ds02[i].z; }
    PointCloudSoA dcloud02{dx02.data(), dy02.data(), dz02.data(), nd02};

    float model02[4];
    t1 = std::chrono::high_resolution_clock::now();
    int inliers02 = ransac_fast_uniform_screened(dcloud02, 0.20f, 250, model02);
    t2 = std::chrono::high_resolution_clock::now();
    double ms02 = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "[Leaf 0.02] Inliers=" << inliers02 << " (" << (inliers02 * 100.0f / nd02) << "%), time=" << ms02 << " ms" << std::endl;

    return 0;
}
