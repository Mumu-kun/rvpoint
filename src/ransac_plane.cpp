#include "include/rvv_pcl.h"
#include <vector>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <algorithm>

namespace rvv_pcl {

// Helper: Compute plane coefficients from 3 points
// ax + by + cz + d = 0
// Returns false if collinear
static bool compute_plane_coefficients(float x1, float y1, float z1,
                                       float x2, float y2, float z2,
                                       float x3, float y3, float z3,
                                       float* model) 
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
    if (norm < 1e-6) return false; // Collinear
    
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
                    float dist_thresh, int max_iters, float* model) 
{
    if (n < 3) return 0;
    std::srand(0); // Fixed seed for reproducibility
    
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    
    for(int iter=0; iter<max_iters; ++iter) {
        // 1. Pick 3 random points
        int i1 = std::rand() % n;
        int i2 = std::rand() % n;
        int i3 = std::rand() % n;
        if(i1 == i2 || i1 == i3 || i2 == i3) continue;
        
        float cand_model[4];
        if(!compute_plane_coefficients(cloud[i1].x, cloud[i1].y, cloud[i1].z,
                                       cloud[i2].x, cloud[i2].y, cloud[i2].z,
                                       cloud[i3].x, cloud[i3].y, cloud[i3].z,
                                       cand_model)) continue;
                                       
        // 2. Count Inliers
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
        }
    }
    
    for(int k=0; k<4; k++) model[k] = best_model[k];
    return best_inliers;
}

// ============================================================================
// RVV Implementation
// ============================================================================
int ransac_plane_rvv(const PointCloudSoA& cloud, 
                     float dist_thresh, int max_iters, float* model) 
{
    if (cloud.n < 3) return 0;
    std::srand(0);
    
    int best_inliers = 0;
    float best_model[4] = {0,0,0,0};
    
    for(int iter=0; iter<max_iters; ++iter) {
        // 1. Pick 3 random points (Scalar)
        int i1 = std::rand() % cloud.n;
        int i2 = std::rand() % cloud.n;
        int i3 = std::rand() % cloud.n;
        if(i1 == i2 || i1 == i3 || i2 == i3) continue;
        
        float cand_model[4];
        if(!compute_plane_coefficients(cloud.x[i1], cloud.y[i1], cloud.z[i1],
                                       cloud.x[i2], cloud.y[i2], cloud.z[i2],
                                       cloud.x[i3], cloud.y[i3], cloud.z[i3],
                                       cand_model)) continue;
        
        float a = cand_model[0];
        float b = cand_model[1];
        float c = cand_model[2];
        float d = cand_model[3];
        
        // 2. Count Inliers (RVV)
        int current_inliers = 0;
        size_t n = cloud.n;
        size_t i = 0;
        
#ifdef __riscv_vector
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - i);
            
            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&cloud.x[i], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&cloud.y[i], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&cloud.z[i], vl);
            
            // dist = a*x + b*y + c*z + d
            vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
            dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
            dist = __riscv_vfadd_vf_f32m8(dist, d, vl); // Add D
            
            // abs(dist)
            // No direct vfabs in standard arithmetic, but we can do bitwise clear sign?
            // Or max(x, -x). Let's use vfsgnjx for absolute value if available or just check bounds.
            // Actually, RISC-V V spec has vfabs.v as pseudo for fsgnjx.
            // But intrinsic is __riscv_vfabs_v_f32m8? Use pseudo if not sure.
            // Let's use mask: -thresh <= dist <= thresh.
            
            vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, dist_thresh, vl);
            vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -dist_thresh, vl);
            vbool4_t mask_in = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
            
            // Count set bits
            current_inliers += __riscv_vcpop_m_b4(mask_in, vl);
            
            i += vl;
        }
#else
        // Fallback if not compiled with vector
        for(; i < n; ++i) {
             float val = a*cloud.x[i] + b*cloud.y[i] + c*cloud.z[i] + d;
             if(std::abs(val) <= dist_thresh) current_inliers++;
        }
#endif

        if(current_inliers > best_inliers) {
            best_inliers = current_inliers;
            for(int k=0; k<4; k++) best_model[k] = cand_model[k];
        }
    }
    
    for(int k=0; k<4; k++) model[k] = best_model[k];
    return best_inliers;
}

} // namespace rvv_pcl
