#include "segmentation/ransac_plane.h"
#include "core/rvv_common.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <limits>
#include <cstring>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

// ============================================================================
// Internal Helpers
// ============================================================================
namespace {

// Compute plane coefficients from 3 points: ax + by + cz + d = 0
static bool compute_plane_coefficients(float x1, float y1, float z1,
                                       float x2, float y2, float z2,
                                       float x3, float y3, float z3,
                                       float* model, float collinear_thresh)
{
    float v1x = x2 - x1;
    float v1y = y2 - y1;
    float v1z = z2 - z1;

    float v2x = x3 - x1;
    float v2y = y3 - y1;
    float v2z = z3 - z1;

    // Cross product
    float a = v1y*v2z - v1z*v2y;
    float b = v1z*v2x - v1x*v2z;
    float c = v1x*v2y - v1y*v2x;

    // Normalize
    float norm = std::sqrt(a*a + b*b + c*c);
    if (norm < collinear_thresh) return false; // Collinear

    float inv = 1.0f / norm;
    a *= inv;
    b *= inv;
    c *= inv;
    float d = -(a*x1 + b*y1 + c*z1);

    model[0] = a;
    model[1] = b;
    model[2] = c;
    model[3] = d;
    return true;
}

#if defined(__riscv_vector)
static inline vfloat32m8_t plane_dist_rvv(float a, float b, float c, float d,
                                          const float *px, const float *py,
                                          const float *pz, size_t vl) {
    vfloat32m8_t vx = __riscv_vle32_v_f32m8(px, vl);
    vfloat32m8_t vy = __riscv_vle32_v_f32m8(py, vl);
    vfloat32m8_t vz = __riscv_vle32_v_f32m8(pz, vl);
    vfloat32m8_t dist = __riscv_vfmv_v_f_f32m8(d, vl);
    dist = __riscv_vfmacc_vf_f32m8(dist, a, vx, vl);
    dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
    dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
    return dist;
}

static inline void compress_to_aos(vfloat32m8_t vx, vfloat32m8_t vy, vfloat32m8_t vz,
                                   vbool4_t mask, size_t vl,
                                   PointXYZ *out, std::size_t off) {
    long cnt = __riscv_vcpop_m_b4(mask, vl);
    if (cnt <= 0) return;
    float *base = reinterpret_cast<float*>(out + off);
    const ptrdiff_t stride = (ptrdiff_t)sizeof(PointXYZ);
    __riscv_vsse32_v_f32m8(base + 0, stride, __riscv_vcompress_vm_f32m8(vx, mask, vl), cnt);
    __riscv_vsse32_v_f32m8(base + 1, stride, __riscv_vcompress_vm_f32m8(vy, mask, vl), cnt);
    __riscv_vsse32_v_f32m8(base + 2, stride, __riscv_vcompress_vm_f32m8(vz, mask, vl), cnt);
}
#endif

} // anonymous namespace

// ============================================================================
// class RansacPlane Implementation
// ============================================================================

RansacPlane::RansacPlane(float dist_thresh, int max_iters, Backend backend)
    : dist_thresh_(dist_thresh), max_iters_(max_iters), backend_(backend) {}

void RansacPlane::reserve(std::size_t max_points) {
    inlier_indices_scratch_.reserve(max_points);
    inlier_mask_scratch_.reserve(max_points);
    sample_x_.reserve(2048);
    sample_y_.reserve(2048);
    sample_z_.reserve(2048);
}

int RansacPlane::operator()(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters) {
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
        return fit_rvv(in, model, dist_thresh, max_iters);
    } else {
        return fit_scalar(in, model, dist_thresh, max_iters);
    }
}

int RansacPlane::fit_rvv(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters) {
    if (in.n < 3) {
        model = PlaneModel();
        return 0;
    }
#if defined(__riscv_vector)
    rng_state_ = (seed_ != 0) ? seed_ : 0x12345678u;

    int best_inliers = 0;
    float best_model[4] = {0, 0, 0, 0};

    int k_iters = max_iters;
    const double log_p = std::log(1.0 - std::clamp(static_cast<double>(probability_), 0.5, 0.9999));

    size_t sample_sz = in.n;
    const float* eval_x = in.x;
    const float* eval_y = in.y;
    const float* eval_z = in.z;

    if (use_sample_screening_ && in.n > 2048) {
        sample_sz = 2048;
        if (sample_x_.size() < sample_sz) {
            sample_x_.resize(sample_sz);
            sample_y_.resize(sample_sz);
            sample_z_.resize(sample_sz);
        }
        size_t stride = in.n / sample_sz;
        for (size_t k = 0; k < sample_sz; ++k) {
            size_t idx = std::min(k * stride, in.n - 1);
            sample_x_[k] = in.x[idx];
            sample_y_[k] = in.y[idx];
            sample_z_[k] = in.z[idx];
        }
        eval_x = sample_x_.data();
        eval_y = sample_y_.data();
        eval_z = sample_z_.data();
    }

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        size_t i1 = next_rand() % in.n;
        size_t i2 = next_rand() % in.n;
        size_t i3 = next_rand() % in.n;
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if (!compute_plane_coefficients(in.x[i1], in.y[i1], in.z[i1],
                                        in.x[i2], in.y[i2], in.z[i2],
                                        in.x[i3], in.y[i3], in.z[i3],
                                        cand_model, collinear_thresh_)) continue;

        if (has_prior_) {
            float dot = std::abs(cand_model[0] * prior_nx_ +
                                 cand_model[1] * prior_ny_ +
                                 cand_model[2] * prior_nz_);
            if (dot < min_ground_dot_) continue;
        }

        float a = cand_model[0];
        float b = cand_model[1];
        float c = cand_model[2];
        float d = cand_model[3];

        int current_inliers = 0;
        size_t i = 0;

        while (i < sample_sz) {
            size_t vl = __riscv_vsetvl_e32m8(sample_sz - i);

            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&eval_x[i], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&eval_y[i], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&eval_z[i], vl);

            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);

            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);

            current_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            i += vl;
        }

        if (current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for (int k = 0; k < 4; ++k) best_model[k] = cand_model[k];

            double w = static_cast<double>(best_inliers) / static_cast<double>(sample_sz);
            double p_no_outliers = 1.0 - std::pow(w, 3.0);
            p_no_outliers = std::max(std::numeric_limits<double>::epsilon(), p_no_outliers);
            p_no_outliers = std::min(1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) {
                    k_iters = dynamic_k;
                }
            }
        }
    }

    // Optional analytical covariance refinement (sub-millimeter precision)
    if (use_cov_refinement_ && best_inliers >= 10) {
        float a0 = best_model[0], b0 = best_model[1], c0 = best_model[2], d0 = best_model[3];
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int inlier_cnt = 0;
        for (size_t k = 0; k < sample_sz; ++k) {
            float dist = std::abs(a0 * eval_x[k] + b0 * eval_y[k] + c0 * eval_z[k] + d0);
            if (dist <= dist_thresh) {
                sum_x += eval_x[k]; sum_y += eval_y[k]; sum_z += eval_z[k];
                inlier_cnt++;
            }
        }
        if (inlier_cnt >= 10) {
            double cx = sum_x / inlier_cnt, cy = sum_y / inlier_cnt, cz = sum_z / inlier_cnt;
            double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (size_t k = 0; k < sample_sz; ++k) {
                float dist = std::abs(a0 * eval_x[k] + b0 * eval_y[k] + c0 * eval_z[k] + d0);
                if (dist <= dist_thresh) {
                    double dx = eval_x[k] - cx, dy = eval_y[k] - cy, dz = eval_z[k] - cz;
                    c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                    c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
                }
            }
            double rnx = c01 * c12 - c02 * c11;
            double rny = c01 * c02 - c00 * c12;
            double rnz = c00 * c11 - c01 * c01;
            double rlen = std::sqrt(rnx * rnx + rny * rny + rnz * rnz);
            if (rlen > 1e-6) {
                rnx /= rlen; rny /= rlen; rnz /= rlen;
                if (rnz < 0) { rnx = -rnx; rny = -rny; rnz = -rnz; }
                best_model[0] = static_cast<float>(rnx);
                best_model[1] = static_cast<float>(rny);
                best_model[2] = static_cast<float>(rnz);
                best_model[3] = static_cast<float>(-(rnx * cx + rny * cy + rnz * cz));
            }
        }
    }

    // If screening was used, re-evaluate winning model over full cloud to get exact full inlier count
    if (use_sample_screening_ && in.n > 2048) {
        float a = best_model[0], b = best_model[1], c = best_model[2], d = best_model[3];
        best_inliers = 0;
        size_t n = in.n, i = 0;
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - i);
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl);
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            best_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            i += vl;
        }
    }

    model = PlaneModel(best_model[0], best_model[1], best_model[2], best_model[3], best_inliers);
    return best_inliers;
#else
    return fit_scalar(in, model, dist_thresh, max_iters);
#endif
}

int RansacPlane::fit_scalar(const PointCloudView& in, PlaneModel& model, float dist_thresh, int max_iters) {
    if (in.n < 3) {
        model = PlaneModel();
        return 0;
    }
    rng_state_ = (seed_ != 0) ? seed_ : 0x12345678u;

    int best_inliers = 0;
    float best_model[4] = {0, 0, 0, 0};

    int k_iters = max_iters;
    const double log_p = std::log(1.0 - std::clamp(static_cast<double>(probability_), 0.5, 0.9999));

    size_t sample_sz = in.n;
    const float* eval_x = in.x;
    const float* eval_y = in.y;
    const float* eval_z = in.z;

    if (use_sample_screening_ && in.n > 2048) {
        sample_sz = 2048;
        if (sample_x_.size() < sample_sz) {
            sample_x_.resize(sample_sz);
            sample_y_.resize(sample_sz);
            sample_z_.resize(sample_sz);
        }
        size_t stride = in.n / sample_sz;
        for (size_t k = 0; k < sample_sz; ++k) {
            size_t idx = std::min(k * stride, in.n - 1);
            sample_x_[k] = in.x[idx];
            sample_y_[k] = in.y[idx];
            sample_z_[k] = in.z[idx];
        }
        eval_x = sample_x_.data();
        eval_y = sample_y_.data();
        eval_z = sample_z_.data();
    }

    for (int iter = 0; iter < k_iters && iter < max_iters; ++iter) {
        size_t i1 = next_rand() % in.n;
        size_t i2 = next_rand() % in.n;
        size_t i3 = next_rand() % in.n;
        if (i1 == i2 || i1 == i3 || i2 == i3) continue;

        float cand_model[4];
        if (!compute_plane_coefficients(in.x[i1], in.y[i1], in.z[i1],
                                        in.x[i2], in.y[i2], in.z[i2],
                                        in.x[i3], in.y[i3], in.z[i3],
                                        cand_model, collinear_thresh_)) continue;

        if (has_prior_) {
            float dot = std::abs(cand_model[0] * prior_nx_ +
                                 cand_model[1] * prior_ny_ +
                                 cand_model[2] * prior_nz_);
            if (dot < min_ground_dot_) continue;
        }

        int current_inliers = 0;
        for (size_t i = 0; i < sample_sz; ++i) {
            float dist = std::abs(cand_model[0] * eval_x[i] +
                                  cand_model[1] * eval_y[i] +
                                  cand_model[2] * eval_z[i] +
                                  cand_model[3]);
            if (dist <= dist_thresh) {
                current_inliers++;
            }
        }

        if (current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for (int k = 0; k < 4; ++k) best_model[k] = cand_model[k];

            double w = static_cast<double>(best_inliers) / static_cast<double>(sample_sz);
            double p_no_outliers = 1.0 - std::pow(w, 3.0);
            p_no_outliers = std::max(std::numeric_limits<double>::epsilon(), p_no_outliers);
            p_no_outliers = std::min(1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);
            double log_no_outliers = std::log(p_no_outliers);
            if (std::abs(log_no_outliers) > 1e-7) {
                int dynamic_k = static_cast<int>(std::ceil(log_p / log_no_outliers));
                if (dynamic_k > 0 && dynamic_k < k_iters) {
                    k_iters = dynamic_k;
                }
            }
        }
    }

    // Optional analytical covariance refinement (sub-millimeter precision)
    if (use_cov_refinement_ && best_inliers >= 10) {
        float a0 = best_model[0], b0 = best_model[1], c0 = best_model[2], d0 = best_model[3];
        double sum_x = 0, sum_y = 0, sum_z = 0;
        int inlier_cnt = 0;
        for (size_t k = 0; k < sample_sz; ++k) {
            float dist = std::abs(a0 * eval_x[k] + b0 * eval_y[k] + c0 * eval_z[k] + d0);
            if (dist <= dist_thresh) {
                sum_x += eval_x[k]; sum_y += eval_y[k]; sum_z += eval_z[k];
                inlier_cnt++;
            }
        }
        if (inlier_cnt >= 10) {
            double cx = sum_x / inlier_cnt, cy = sum_y / inlier_cnt, cz = sum_z / inlier_cnt;
            double c00 = 0, c01 = 0, c02 = 0, c11 = 0, c12 = 0, c22 = 0;
            for (size_t k = 0; k < sample_sz; ++k) {
                float dist = std::abs(a0 * eval_x[k] + b0 * eval_y[k] + c0 * eval_z[k] + d0);
                if (dist <= dist_thresh) {
                    double dx = eval_x[k] - cx, dy = eval_y[k] - cy, dz = eval_z[k] - cz;
                    c00 += dx * dx; c01 += dx * dy; c02 += dx * dz;
                    c11 += dy * dy; c12 += dy * dz; c22 += dz * dz;
                }
            }
            double rnx = c01 * c12 - c02 * c11;
            double rny = c01 * c02 - c00 * c12;
            double rnz = c00 * c11 - c01 * c01;
            double rlen = std::sqrt(rnx * rnx + rny * rny + rnz * rnz);
            if (rlen > 1e-6) {
                rnx /= rlen; rny /= rlen; rnz /= rlen;
                if (rnz < 0) { rnx = -rnx; rny = -rny; rnz = -rnz; }
                best_model[0] = static_cast<float>(rnx);
                best_model[1] = static_cast<float>(rny);
                best_model[2] = static_cast<float>(rnz);
                best_model[3] = static_cast<float>(-(rnx * cx + rny * cy + rnz * cz));
            }
        }
    }

    // If screening was used, re-evaluate winning model over full cloud to get exact full inlier count
    if (use_sample_screening_ && in.n > 2048) {
        best_inliers = 0;
        for (size_t i = 0; i < in.n; ++i) {
            float dist = std::abs(best_model[0] * in.x[i] +
                                  best_model[1] * in.y[i] +
                                  best_model[2] * in.z[i] +
                                  best_model[3]);
            if (dist <= dist_thresh) {
                best_inliers++;
            }
        }
    }

    model = PlaneModel(best_model[0], best_model[1], best_model[2], best_model[3], best_inliers);
    return best_inliers;
}

void RansacPlane::extract(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
                         PointCloud* inliers, PointCloud* outliers) {
    if (!inliers && !outliers) return;
    if (inliers) inliers->clear();
    if (outliers) outliers->clear();
    if (in.n == 0) return;

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
        extract_rvv(in, model, dist_thresh, inliers, outliers);
    } else {
        extract_scalar(in, model, dist_thresh, inliers, outliers);
    }
}

void RansacPlane::extract_rvv(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
                             PointCloud* inliers, PointCloud* outliers) {
    if (!inliers && !outliers) return;
    if (in.n == 0) return;

#if defined(__riscv_vector)
    const size_t n = in.n;
    if (inliers) inliers->reserve(model.inliers > 0 ? (size_t)model.inliers : n);
    if (outliers) outliers->reserve(n);

    float a = model.a, b = model.b, c = model.c, d = model.d;
    size_t i = 0;

    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[i], vl);
        vfloat32m8_t dist = plane_dist_rvv(a, b, c, d, &in.x[i], &in.y[i], &in.z[i], vl);

        vbool4_t in_mask = __riscv_vmand_mm_b4(
            __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl),
            __riscv_vmfle_vf_f32m8_b4(dist,  dist_thresh, vl), vl);

        if (inliers) {
            long ic = __riscv_vcpop_m_b4(in_mask, vl);
            if (ic > 0) {
                size_t cur = inliers->n;
                inliers->resize(cur + ic);
                vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, in_mask, vl);
                vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, in_mask, vl);
                vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, in_mask, vl);
                __riscv_vse32_v_f32m8(inliers->x.data() + cur, cx, ic);
                __riscv_vse32_v_f32m8(inliers->y.data() + cur, cy, ic);
                __riscv_vse32_v_f32m8(inliers->z.data() + cur, cz, ic);
            }
        }

        if (outliers) {
            vbool4_t out_mask = __riscv_vmnot_m_b4(in_mask, vl);
            long oc = __riscv_vcpop_m_b4(out_mask, vl);
            if (oc > 0) {
                size_t cur = outliers->n;
                outliers->resize(cur + oc);
                vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, out_mask, vl);
                vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, out_mask, vl);
                vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, out_mask, vl);
                __riscv_vse32_v_f32m8(outliers->x.data() + cur, cx, oc);
                __riscv_vse32_v_f32m8(outliers->y.data() + cur, cy, oc);
                __riscv_vse32_v_f32m8(outliers->z.data() + cur, cz, oc);
            }
        }

        i += vl;
    }
#else
    extract_scalar(in, model, dist_thresh, inliers, outliers);
#endif
}

void RansacPlane::extract_scalar(const PointCloudView& in, const PlaneModel& model, float dist_thresh,
                                PointCloud* inliers, PointCloud* outliers) {
    if (!inliers && !outliers) return;
    if (in.n == 0) return;

    if (inliers) inliers->reserve(model.inliers > 0 ? (size_t)model.inliers : in.n);
    if (outliers) outliers->reserve(in.n);

    float a = model.a, b = model.b, c = model.c, d = model.d;
    for (size_t j = 0; j < in.n; ++j) {
        float dist = a * in.x[j] + b * in.y[j] + c * in.z[j] + d;
        if (dist >= -dist_thresh && dist <= dist_thresh) {
            if (inliers) inliers->push_back(in.x[j], in.y[j], in.z[j]);
        } else {
            if (outliers) outliers->push_back(in.x[j], in.y[j], in.z[j]);
        }
    }
}

std::size_t RansacPlane::extract_inliers(const PointCloudView& in, const PlaneModel& model,
                                        float dist_thresh, PointCloud& inliers) {
    extract(in, model, dist_thresh, &inliers, nullptr);
    return inliers.size();
}

std::size_t RansacPlane::extract_outliers(const PointCloudView& in, const PlaneModel& model,
                                         float dist_thresh, PointCloud& outliers) {
    extract(in, model, dist_thresh, nullptr, &outliers);
    return outliers.size();
}

std::size_t RansacPlane::extract_inliers(const PointCloudView& in, const float* model,
                                        float dist_thresh, PointXYZ* inliers) {
    PlaneModel pm(model[0], model[1], model[2], model[3]);
    PointCloud pc;
    extract(in, pm, dist_thresh, &pc, nullptr);
    for (size_t i = 0; i < pc.size(); ++i) {
        inliers[i] = {pc.x[i], pc.y[i], pc.z[i]};
    }
    return pc.size();
}

std::size_t RansacPlane::extract_outliers(const PointCloudView& in, const float* model,
                                         float dist_thresh, PointXYZ* outliers) {
    PlaneModel pm(model[0], model[1], model[2], model[3]);
    PointCloud pc;
    extract(in, pm, dist_thresh, nullptr, &pc);
    for (size_t i = 0; i < pc.size(); ++i) {
        outliers[i] = {pc.x[i], pc.y[i], pc.z[i]};
    }
    return pc.size();
}

void RansacPlane::extract_inliers_outliers(const PointCloudView& in, const float* model,
                                          float dist_thresh, PointXYZ* inliers, PointXYZ* outliers,
                                          std::size_t& n_inliers, std::size_t& n_outliers) {
    PlaneModel pm(model[0], model[1], model[2], model[3]);
    PointCloud in_pc, out_pc;
    extract(in, pm, dist_thresh, &in_pc, &out_pc);
    n_inliers = in_pc.size();
    n_outliers = out_pc.size();
    for (size_t i = 0; i < n_inliers; ++i) {
        inliers[i] = {in_pc.x[i], in_pc.y[i], in_pc.z[i]};
    }
    for (size_t i = 0; i < n_outliers; ++i) {
        outliers[i] = {out_pc.x[i], out_pc.y[i], out_pc.z[i]};
    }
}

} // namespace rvpoint
