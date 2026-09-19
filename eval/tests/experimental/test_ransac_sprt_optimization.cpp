#include "io/simple_pcd_loader.h"
#include "include/rvpoint.h"
#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>

using namespace rvpoint;

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

// 3. Multi-Stage Cascaded SPRT
static int ransac_cascaded_sprt(const PointCloudSoA& cloud, float dist_thresh, int max_iters, float* model, int& full_evals) {
    if (cloud.n < 3) return 0;
    FastPRNG rng(42);
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - 0.99);
    full_evals = 0;

    const size_t check1_sz = std::min(cloud.n, static_cast<size_t>(512));
    const size_t check2_sz = std::min(cloud.n, static_cast<size_t>(2048));

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

#if defined(__riscv_vector)
        // Stage 1: Fast Check (512 points)
        int inliers1 = 0;
        size_t si = 0;
        while (si < check1_sz) {
            size_t vl = __riscv_vsetvl_e32m8(check1_sz - si);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[si], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[si], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[si], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            inliers1 += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
            si += vl;
        }

        if (best_inliers > 0) {
            // Require at least 80% of current best inlier density
            float min_ratio = (static_cast<float>(best_inliers) / static_cast<float>(cloud.n)) * 0.80f;
            if (static_cast<float>(inliers1) / static_cast<float>(check1_sz) < min_ratio) {
                continue;
            }
        }

        // Stage 2: Medium Check (2048 points)
        int inliers2 = inliers1;
        while (si < check2_sz) {
            size_t vl = __riscv_vsetvl_e32m8(check2_sz - si);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[si], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[si], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[si], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            inliers2 += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
            si += vl;
        }

        if (best_inliers > 0) {
            float min_ratio = (static_cast<float>(best_inliers) / static_cast<float>(cloud.n)) * 0.90f;
            if (static_cast<float>(inliers2) / static_cast<float>(check2_sz) < min_ratio) {
                continue;
            }
        }

        // Stage 3: Full Evaluation (only for true candidates!)
        full_evals++;
        int current_inliers = inliers2;
        size_t n = cloud.n, i = check2_sz;
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
            current_inliers += static_cast<int>(__riscv_vcpop_m_b4(mask_in, vl));
            i += vl;
        }
#else
        full_evals++;
        int current_inliers = 0;
        for (size_t pt = 0; pt < cloud.n; ++pt) {
            float dist = std::abs(a * cloud.x[pt] + b * cloud.y[pt] + c * cloud.z[pt] + d);
            if (dist <= dist_thresh) current_inliers++;
        }
#endif

        if (current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for (int k = 0; k < 4; k++) best_model[k] = cand_model[k];
            double w = static_cast<double>(best_inliers) / static_cast<double>(cloud.n);
            double p_no_outliers = std::clamp(1.0 - std::pow(w, 3.0), 1e-7, 1.0 - 1e-7);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) k_iters = dynamic_k;
            }
        }
    }
    for (int k = 0; k < 4; k++) model[k] = best_model[k];
    return best_inliers;
}

int main() {
    std::vector<PointXYZ> pts;
    if (loadPCD("data/pcd_compressed/0000000045.pcd", pts) < 0) return 1;

    std::vector<float> x(pts.size()), y(pts.size()), z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { x[i] = pts[i].x; y[i] = pts[i].y; z[i] = pts[i].z; }
    PointCloudSoA cloud{x.data(), y.data(), z.data(), pts.size()};

    std::vector<PointXYZ> ds(pts.size());
    size_t nd = voxel_grid_downsamp_rvv_v2(cloud, ds.data(), 0.10f);
    ds.resize(nd);
    std::vector<float> dx(nd), dy(nd), dz(nd);
    for (size_t i = 0; i < nd; ++i) { dx[i] = ds[i].x; dy[i] = ds[i].y; dz[i] = ds[i].z; }
    PointCloudSoA dcloud{dx.data(), dy.data(), dz.data(), nd};

    float model[4];
    int full_evals = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    int inliers = ransac_cascaded_sprt(dcloud, 0.20f, 250, model, full_evals);
    auto t2 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();

    std::cout << "Cascaded SPRT: inliers=" << inliers << ", full_evals=" << full_evals << ", time=" << ms << " ms" << std::endl;
    return 0;
}
