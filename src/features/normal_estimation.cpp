#include "features/normal_estimation.h"
#include "search/octree.h"
#include "search/spatial_hashing.h"
#include "search/pointer_octree.h"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

namespace rvpoint {

// Helper: Diagonalize 3x3 symmetric matrix A
// Returns eigenvector corresponding to smallest eigenvalue
void simple_eigen3x3_smallest(float cov[3][3], float& nx, float& ny, float& nz, int eigen_iters) {
    float A[3][3];
    for(int i=0;i<3;i++) for(int j=0;j<3;j++) A[i][j] = cov[i][j];
    
    float V[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
    
    for(int iter=0; iter<eigen_iters; ++iter) { 
        int p=0, q=1;
        float max_off = std::abs(A[0][1]);
        if(std::abs(A[0][2]) > max_off) { p=0; q=2; max_off=std::abs(A[0][2]); }
        if(std::abs(A[1][2]) > max_off) { p=1; q=2; }
        
        float phi = 0.5f * std::atan2(2*A[p][q], A[p][p] - A[q][q]);
        float c = std::cos(phi);
        float s = std::sin(phi);
        
        float app = A[p][p], aqq = A[q][q], apq = A[p][q];
        A[p][p] = c*c*app - 2*s*c*apq + s*s*aqq;
        A[q][q] = s*s*app + 2*s*c*apq + c*c*aqq;
        A[p][q] = 0;
        
        for(int k=0; k<3; ++k) {
             float vip = V[k][p];
             float viq = V[k][q];
             V[k][p] = c*vip - s*viq;
             V[k][q] = s*vip + c*viq;
        }
    }
    
    int min_idx = 0;
    if(A[1][1] < A[min_idx][min_idx]) min_idx = 1;
    if(A[2][2] < A[min_idx][min_idx]) min_idx = 2;
    
    nx = V[0][min_idx];
    ny = V[1][min_idx];
    nz = V[2][min_idx];
}

// Helper: Flip normal if it points away from viewpoint (standard PCL behavior)
void flipNormalTowardsViewpoint(const PointXYZ& point, float vp_x, float vp_y, float vp_z,
                                float& nx, float& ny, float& nz) {
    float vx = vp_x - point.x;
    float vy = vp_y - point.y;
    float vz = vp_z - point.z;
    
    float dot = vx*nx + vy*ny + vz*nz;
    if (dot < 0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
    }
}

// ============================================================================
// Scalar Implementation
// ============================================================================
void normal_estimation_sc(const PointXYZ* in, std::size_t n,
                          float* nx, float* ny, float* nz, int k, float radius,
                          float vp_x, float vp_y, float vp_z, int eigen_iters) {
    if (n == 0) return;
    std::vector<float> dists(n);
    std::vector<int> indices(n);

    for(size_t i=0; i<n; ++i) {
        for(size_t j=0; j<n; ++j) {
            float dx = in[i].x - in[j].x;
            float dy = in[i].y - in[j].y;
            float dz = in[i].z - in[j].z;
            dists[j] = dx*dx + dy*dy + dz*dz;
            indices[j] = j;
        }
        std::partial_sort(indices.begin(), indices.begin()+k+1, indices.end(),
            [&](int a, int b){ return dists[a] < dists[b]; });

        float cx=0, cy=0, cz=0;
        for(int j=0; j<=k; ++j) {
            int idx = indices[j];
            cx += in[idx].x; cy += in[idx].y; cz += in[idx].z;
        }
        cx /= (k+1); cy /= (k+1); cz /= (k+1);

        float cov[3][3] = {0};
        for(int j=0; j<=k; ++j) {
            int idx = indices[j];
            float dx = in[idx].x - cx;
            float dy = in[idx].y - cy;
            float dz = in[idx].z - cz;
            cov[0][0] += dx*dx; cov[0][1] += dx*dy; cov[0][2] += dx*dz;
            cov[1][1] += dy*dy; cov[1][2] += dy*dz;
            cov[2][2] += dz*dz;
        }
        cov[1][0]=cov[0][1]; cov[2][0]=cov[0][2]; cov[2][1]=cov[1][2];

        simple_eigen3x3_smallest(cov, nx[i], ny[i], nz[i], eigen_iters);
        flipNormalTowardsViewpoint(in[i], vp_x, vp_y, vp_z, nx[i], ny[i], nz[i]);
    }
}

// ============================================================================
// Step-by-Step Implementations
// ============================================================================
void compute_covariance_rvv(const PointCloudSoA& cloud, const std::vector<int>& indices, float cov[3][3], float centroid[3]) {
    float cx=0, cy=0, cz=0;
    for(int idx : indices) {
        cx += cloud.x[idx]; cy += cloud.y[idx]; cz += cloud.z[idx];
    }
    float inv_n = 1.0f / indices.size();
    cx *= inv_n; cy *= inv_n; cz *= inv_n;
    centroid[0]=cx; centroid[1]=cy; centroid[2]=cz;

    float c00=0, c01=0, c02=0;
    float c11=0, c12=0;
    float c22=0;

    for(int idx : indices) {
        float dx = cloud.x[idx] - cx;
        float dy = cloud.y[idx] - cy;
        float dz = cloud.z[idx] - cz;
        c00 += dx*dx; c01 += dx*dy; c02 += dx*dz;
        c11 += dy*dy; c12 += dy*dz;
        c22 += dz*dz;
    }
    
    cov[0][0] = c00; cov[0][1] = c01; cov[0][2] = c02;
    cov[1][0] = c01; cov[1][1] = c11; cov[1][2] = c12;
    cov[2][0] = c02; cov[2][1] = c12; cov[2][2] = c22;
}

void eigen_decomposition_rvv(float cov[3][3], float& nx, float& ny, float& nz, int eigen_iters) {
    simple_eigen3x3_smallest(cov, nx, ny, nz, eigen_iters);
}

void flip_normal_rvv(const PointXYZ& point, float vp_x, float vp_y, float vp_z, float& nx, float& ny, float& nz) {
    flipNormalTowardsViewpoint(point, vp_x, vp_y, vp_z, nx, ny, nz);
}

// ============================================================================
// RVV Implementation (Octree passed externally)
// ============================================================================
void normal_estimation_rvv(const PointCloudSoA& in,
                           const Octree& octree,
                           float* nx, float* ny, float* nz, int k, float radius,
                           float vp_x, float vp_y, float vp_z, int eigen_iters) {
    if(in.n == 0) return;

    float search_radius = radius;
    std::vector<int> indices;
    std::vector<float> dists;
    indices.reserve(k * 2);

    for(size_t i=0; i<in.n; ++i) {
         PointXYZ query = {in.x[i], in.y[i], in.z[i]};
         octree.radiusSearch(query, search_radius, indices, dists);
         
         if (indices.size() < 3) {
             nx[i] = ny[i] = nz[i] = 0; 
             continue;
         }
         
         float cov[3][3];
         float centroid[3];
         compute_covariance_rvv(in, indices, cov, centroid);

         float n_x, n_y, n_z;
         eigen_decomposition_rvv(cov, n_x, n_y, n_z, eigen_iters);
         flip_normal_rvv(query, vp_x, vp_y, vp_z, n_x, n_y, n_z);
         
         nx[i] = n_x;
         ny[i] = n_y;
         nz[i] = n_z;
    }
}

void normal_estimation_rvv(const PointCloudSoA& in,
                           const SpatialHash& hash,
                           float* nx, float* ny, float* nz, int k, float radius,
                           float vp_x, float vp_y, float vp_z, int eigen_iters) {
    if (in.n == 0) return;

    std::vector<int>   indices;
    std::vector<float> dists;
    indices.reserve(k * 2);

    for (size_t i = 0; i < in.n; ++i) {
        PointXYZ query = {in.x[i], in.y[i], in.z[i]};
        hash.radiusSearch(query, radius, indices, dists);

        if (indices.size() < 3) { nx[i] = ny[i] = nz[i] = 0; continue; }

        float cov[3][3], centroid[3];
        compute_covariance_rvv(in, indices, cov, centroid);

        float n_x, n_y, n_z;
        eigen_decomposition_rvv(cov, n_x, n_y, n_z, eigen_iters);
        flip_normal_rvv(query, vp_x, vp_y, vp_z, n_x, n_y, n_z);

        nx[i] = n_x; ny[i] = n_y; nz[i] = n_z;
    }
}

void normal_estimation_rvv(const PointCloudSoA& in,
                           const PointerOctree& octree,
                           float* nx, float* ny, float* nz, int k, float radius,
                           float vp_x, float vp_y, float vp_z, int eigen_iters) {
    if (in.n == 0) return;

    std::vector<int>   indices;
    std::vector<float> dists;
    indices.reserve(k * 2);

    for (size_t i = 0; i < in.n; ++i) {
        PointXYZ query = {in.x[i], in.y[i], in.z[i]};
        indices.clear();
        dists.clear();

        octree.radiusSearch(query, radius, indices, dists);

        if (indices.size() < 3) { nx[i] = ny[i] = nz[i] = 0; continue; }

        float cov[3][3], centroid[3];
        compute_covariance_rvv(in, indices, cov, centroid);

        float n_x, n_y, n_z;
        eigen_decomposition_rvv(cov, n_x, n_y, n_z, eigen_iters);
        flip_normal_rvv(query, vp_x, vp_y, vp_z, n_x, n_y, n_z);

        nx[i] = n_x; ny[i] = n_y; nz[i] = n_z;
    }
}

void normal_estimation_rvv(const PointCloudSoA& in,
                           float* nx, float* ny, float* nz, int k, float radius,
                           float vp_x, float vp_y, float vp_z, int eigen_iters) {
    PointerOctree octree;
    octree.setInputCloud(in);
    octree.build();
    normal_estimation_rvv(in, octree, nx, ny, nz, k, radius, vp_x, vp_y, vp_z, eigen_iters);
}

} // namespace rvpoint
