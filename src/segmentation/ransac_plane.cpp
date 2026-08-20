#include "segmentation/ransac_plane.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <limits>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

// Helper: Compute plane coefficients from 3 points
// ax + by + cz + d = 0
// Returns false if collinear
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
    
    a /= norm;
    b /= norm;
    c /= norm;
    float d = -(a*x1 + b*y1 + c*z1);
    
    model[0] = a;
    model[1] = b;
    model[2] = c;
    model[3] = d;
    return true;
}

// ============================================================================
// Scalar Implementation
// ============================================================================
int ransac_plane_sc(const PointXYZ* cloud, std::size_t n, 
                    float dist_thresh, int max_iters, float* model,
                    float collinear_thresh, float probability) 
{
    if (n < 3) return 0;
    std::srand(0); // Fixed seed for reproducibility
    
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - std::clamp(static_cast<double>(probability), 0.5, 0.9999));
    
    for(int iter=0; iter<k_iters && iter<max_iters; ++iter) {
        int i1 = std::rand() % n;
        int i2 = std::rand() % n;
        int i3 = std::rand() % n;
        if(i1 == i2 || i1 == i3 || i2 == i3) continue;
        
        float cand_model[4];
        if(!compute_plane_coefficients(cloud[i1].x, cloud[i1].y, cloud[i1].z,
                                       cloud[i2].x, cloud[i2].y, cloud[i2].z,
                                       cloud[i3].x, cloud[i3].y, cloud[i3].z,
                                       cand_model, collinear_thresh)) continue;
        if(std::abs(cand_model[2]) < 0.70f) continue;
                                       
        int current_inliers = 0;
        for(size_t i=0; i<n; ++i) {
            float dist = std::abs(cand_model[0]*cloud[i].x + 
                                  cand_model[1]*cloud[i].y + 
                                  cand_model[2]*cloud[i].z + 
                                  cand_model[3]);
            if(dist <= dist_thresh) {
                current_inliers++;
            }
        }
        
        if(current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for(int k=0; k<4; k++) best_model[k] = cand_model[k];
            
            double w = static_cast<double>(best_inliers) / static_cast<double>(n);
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
    
    for(int k=0; k<4; k++) model[k] = best_model[k];
    return best_inliers;
}

// ============================================================================
// RVV Implementation
// ============================================================================
int ransac_plane_rvv(const PointCloudSoA& cloud, 
                     float dist_thresh, int max_iters, float* model,
                     float collinear_thresh, float probability) 
{
    if (cloud.n < 3) return 0;
#if defined(__riscv_vector)
    std::srand(0);
    
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    
    int k_iters = max_iters;
    const double log_p = std::log(1.0 - std::clamp(static_cast<double>(probability), 0.5, 0.9999));
    
    for(int iter=0; iter<k_iters && iter<max_iters; ++iter) {
        int i1 = std::rand() % cloud.n;
        int i2 = std::rand() % cloud.n;
        int i3 = std::rand() % cloud.n;
        if(i1 == i2 || i1 == i3 || i2 == i3) continue;
        
        float cand_model[4];
        if(!compute_plane_coefficients(cloud.x[i1], cloud.y[i1], cloud.z[i1],
                                       cloud.x[i2], cloud.y[i2], cloud.z[i2],
                                       cloud.x[i3], cloud.y[i3], cloud.z[i3],
                                       cand_model, collinear_thresh)) continue;
        if(std::abs(cand_model[2]) < 0.70f) continue;
        
        float a = cand_model[0];
        float b = cand_model[1];
        float c = cand_model[2];
        float d = cand_model[3];
        
        int current_inliers = 0;
        size_t n = cloud.n;
        size_t i = 0;
        
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
            
            current_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            
            i += vl;
        }

        if(current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for(int k=0; k<4; k++) best_model[k] = cand_model[k];
            
            double w = static_cast<double>(best_inliers) / static_cast<double>(cloud.n);
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
    
    for(int k=0; k<4; k++) model[k] = best_model[k];
    return best_inliers;
#else
    std::vector<PointXYZ> aos(cloud.n);
    for (size_t i = 0; i < cloud.n; ++i) {
        aos[i] = {cloud.x[i], cloud.y[i], cloud.z[i]};
    }
    return ransac_plane_sc(aos.data(), cloud.n, dist_thresh, max_iters, model, collinear_thresh, probability);
#endif
}

// ============================================================================
// Extract plane inliers / outliers (RVV)
// ============================================================================

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

#ifndef GEM5_BUILD
static inline void compress_to_aos(vfloat32m8_t vx, vfloat32m8_t vy, vfloat32m8_t vz,
                                    vbool4_t mask, size_t vl,
                                    PointXYZ *out, std::size_t off) {
    long cnt = __riscv_vcpop_m_b4(mask, vl);
    if (cnt <= 0) return;
    float *base = reinterpret_cast<float*>(out + off);
    const ptrdiff_t stride = (ptrdiff_t)sizeof(PointXYZ);  // 12 bytes
    __riscv_vsse32_v_f32m8(base + 0, stride, __riscv_vcompress_vm_f32m8(vx, mask, vl), cnt);
    __riscv_vsse32_v_f32m8(base + 1, stride, __riscv_vcompress_vm_f32m8(vy, mask, vl), cnt);
    __riscv_vsse32_v_f32m8(base + 2, stride, __riscv_vcompress_vm_f32m8(vz, mask, vl), cnt);
}
#endif
#endif

std::size_t extract_plane_inliers_rvv(const PointCloudSoA &cloud,
                                       const float *model, float dist_thresh,
                                       PointXYZ *inliers) {
    float a = model[0], b = model[1], c = model[2], d = model[3];
    std::size_t n = cloud.n, count = 0, i = 0;
#if defined(__riscv_vector)
#ifdef GEM5_BUILD
    std::vector<float> dists(n);
    {   size_t j = 0;
        while (j < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - j);
            vfloat32m8_t dist = plane_dist_rvv(a, b, c, d,
                                                &cloud.x[j], &cloud.y[j], &cloud.z[j], vl);
            __riscv_vse32_v_f32m8(&dists[j], dist, vl);
            j += vl;
        }
    }
    for (size_t j = 0; j < n; ++j)
        if (dists[j] >= -dist_thresh && dists[j] <= dist_thresh)
            inliers[count++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
#else
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
        vfloat32m8_t dist = plane_dist_rvv(a, b, c, d, &cloud.x[i], &cloud.y[i], &cloud.z[i], vl);
        vbool4_t mask = __riscv_vmand_mm_b4(
                            __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl),
                            __riscv_vmfle_vf_f32m8_b4(dist,  dist_thresh, vl), vl);
        long cnt = __riscv_vcpop_m_b4(mask, vl);
        compress_to_aos(vx, vy, vz, mask, vl, inliers, count);
        count += (size_t)cnt;
        i += vl;
    }
#endif
#else
    for (size_t j = 0; j < n; ++j) {
        float dist = a * cloud.x[j] + b * cloud.y[j] + c * cloud.z[j] + d;
        if (dist >= -dist_thresh && dist <= dist_thresh)
            inliers[count++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
    }
#endif
    return count;
}

std::size_t extract_plane_outliers_rvv(const PointCloudSoA &cloud,
                                        const float *model, float dist_thresh,
                                        PointXYZ *outliers) {
    float a = model[0], b = model[1], c = model[2], d = model[3];
    std::size_t n = cloud.n, count = 0, i = 0;
#if defined(__riscv_vector)
#ifdef GEM5_BUILD
    std::vector<float> dists(n);
    {   size_t j = 0;
        while (j < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - j);
            vfloat32m8_t dist = plane_dist_rvv(a, b, c, d,
                                                &cloud.x[j], &cloud.y[j], &cloud.z[j], vl);
            __riscv_vse32_v_f32m8(&dists[j], dist, vl);
            j += vl;
        }
    }
    for (size_t j = 0; j < n; ++j)
        if (dists[j] < -dist_thresh || dists[j] > dist_thresh)
            outliers[count++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
#else
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
        vfloat32m8_t dist = plane_dist_rvv(a, b, c, d, &cloud.x[i], &cloud.y[i], &cloud.z[i], vl);
        vbool4_t in_mask = __riscv_vmand_mm_b4(
                               __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl),
                               __riscv_vmfle_vf_f32m8_b4(dist,  dist_thresh, vl), vl);
        vbool4_t out_mask = __riscv_vmnot_m_b4(in_mask, vl);
        long cnt = __riscv_vcpop_m_b4(out_mask, vl);
        compress_to_aos(vx, vy, vz, out_mask, vl, outliers, count);
        count += (size_t)cnt;
        i += vl;
    }
#endif
#else
    for (size_t j = 0; j < n; ++j) {
        float dist = a * cloud.x[j] + b * cloud.y[j] + c * cloud.z[j] + d;
        if (dist < -dist_thresh || dist > dist_thresh)
            outliers[count++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
    }
#endif
    return count;
}

void extract_plane_inliers_outliers_rvv(const PointCloudSoA &cloud,
                                         const float *model, float dist_thresh,
                                         PointXYZ *inliers, PointXYZ *outliers,
                                         std::size_t &n_inliers,
                                         std::size_t &n_outliers) {
    float a = model[0], b = model[1], c = model[2], d = model[3];
    std::size_t n = cloud.n;
    n_inliers = 0; n_outliers = 0;
    size_t i = 0;
#if defined(__riscv_vector)
#ifdef GEM5_BUILD
    std::vector<float> dists(n);
    {   size_t j = 0;
        while (j < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - j);
            vfloat32m8_t dist = plane_dist_rvv(a, b, c, d,
                                                &cloud.x[j], &cloud.y[j], &cloud.z[j], vl);
            __riscv_vse32_v_f32m8(&dists[j], dist, vl);
            j += vl;
        }
    }
    for (size_t j = 0; j < n; ++j) {
        if (dists[j] >= -dist_thresh && dists[j] <= dist_thresh)
            inliers[n_inliers++]  = {cloud.x[j], cloud.y[j], cloud.z[j]};
        else
            outliers[n_outliers++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
    }
#else
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
        vfloat32m8_t dist = plane_dist_rvv(a, b, c, d, &cloud.x[i], &cloud.y[i], &cloud.z[i], vl);
        vbool4_t in_mask  = __riscv_vmand_mm_b4(
                                __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl),
                                __riscv_vmfle_vf_f32m8_b4(dist,  dist_thresh, vl), vl);
        vbool4_t out_mask = __riscv_vmnot_m_b4(in_mask, vl);
        long ic = __riscv_vcpop_m_b4(in_mask,  vl);
        long oc = __riscv_vcpop_m_b4(out_mask, vl);
        compress_to_aos(vx, vy, vz, in_mask,  vl, inliers,  n_inliers);
        compress_to_aos(vx, vy, vz, out_mask, vl, outliers, n_outliers);
        n_inliers  += (size_t)ic;
        n_outliers += (size_t)oc;
        i += vl;
    }
#endif
#else
    for (size_t j = 0; j < n; ++j) {
        float dist = a * cloud.x[j] + b * cloud.y[j] + c * cloud.z[j] + d;
        if (dist >= -dist_thresh && dist <= dist_thresh)
            inliers[n_inliers++]  = {cloud.x[j], cloud.y[j], cloud.z[j]};
        else
            outliers[n_outliers++] = {cloud.x[j], cloud.y[j], cloud.z[j]};
    }
#endif
}

} // namespace rvpoint
