#include "features/normal_estimation.h"
#include "search/octree.h"
#include "search/pointer_octree.h"
#include "search/spatial_hashing.h"

#include <algorithm>
#include <cmath>
#include <vector>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace rvpoint {

// Helper: Diagonalize 3x3 symmetric matrix A
// Returns eigenvector corresponding to smallest eigenvalue
void simple_eigen3x3_smallest(float cov[3][3], float &nx, float &ny, float &nz,
                              int eigen_iters) {
  float A[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      A[i][j] = cov[i][j];

  float V[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  for (int iter = 0; iter < eigen_iters; ++iter) {
    int p = 0, q = 1;
    float max_off = std::abs(A[0][1]);
    if (std::abs(A[0][2]) > max_off) {
      p = 0;
      q = 2;
      max_off = std::abs(A[0][2]);
    }
    if (std::abs(A[1][2]) > max_off) {
      p = 1;
      q = 2;
    }

    float phi = 0.5f * std::atan2(2 * A[p][q], A[p][p] - A[q][q]);
    float c = std::cos(phi);
    float s = std::sin(phi);

    float app = A[p][p], aqq = A[q][q], apq = A[p][q];
    A[p][p] = c * c * app - 2 * s * c * apq + s * s * aqq;
    A[q][q] = s * s * app + 2 * s * c * apq + c * c * aqq;
    A[p][q] = 0;

    for (int k = 0; k < 3; ++k) {
      float vip = V[k][p];
      float viq = V[k][q];
      V[k][p] = c * vip - s * viq;
      V[k][q] = s * vip + c * viq;
    }
  }

  int min_idx = 0;
  if (A[1][1] < A[min_idx][min_idx])
    min_idx = 1;
  if (A[2][2] < A[min_idx][min_idx])
    min_idx = 2;

  nx = V[0][min_idx];
  ny = V[1][min_idx];
  nz = V[2][min_idx];
}

// Helper: Flip normal if it points away from viewpoint (standard PCL behavior)
void flipNormalTowardsViewpoint(const PointXYZ &point, float vp_x, float vp_y,
                                float vp_z, float &nx, float &ny, float &nz) {
  float vx = vp_x - point.x;
  float vy = vp_y - point.y;
  float vz = vp_z - point.z;

  float dot = vx * nx + vy * ny + vz * nz;
  if (dot < 0) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
}

// ============================================================================
// class NormalEstimation Implementation
// ============================================================================

NormalEstimation::NormalEstimation(int k, float radius, float vp_x, float vp_y, float vp_z, Backend backend)
    : k_(k), radius_(radius), vp_x_(vp_x), vp_y_(vp_y), vp_z_(vp_z), backend_(backend) {}

void NormalEstimation::reserve(std::size_t max_points) {
    int max_th = num_threads_ > 0 ? num_threads_ : 8;
#if defined(_OPENMP)
    max_th = std::max(max_th, omp_get_max_threads());
#endif
    thread_neighbors_.resize(max_th);
    thread_dists2_.resize(max_th);
    for (int t = 0; t < max_th; ++t) {
        thread_neighbors_[t].reserve(std::min<size_t>(max_points, 512));
        thread_dists2_[t].reserve(std::min<size_t>(max_points, 512));
    }
    neighbors_.reserve(max_points);
    dists2_.reserve(max_points);
    grid_ = Fast3DSpatialGrid(radius_ > 0.0f ? radius_ : 0.2f, max_points);
}

void NormalEstimation::operator()(const PointCloudView& in, PointCloud& normals,
                                  int k, float radius, float vpx, float vpy, float vpz) {
    normals.resize(in.n);
    compute(in, normals.x.data(), normals.y.data(), normals.z.data(), k, radius, vpx, vpy, vpz);
}

void NormalEstimation::operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid, PointCloud& normals,
                                  int k, float radius, float vpx, float vpy, float vpz) {
    normals.resize(in.n);
    compute(in, grid, normals.x.data(), normals.y.data(), normals.z.data(), k, radius, vpx, vpy, vpz);
}

void NormalEstimation::compute(const PointCloudView& in, float* nx, float* ny, float* nz,
                               int k, float radius, float vpx, float vpy, float vpz) {
    if (in.n == 0) return;
    float search_r = radius > 0.0f ? radius : (radius_ > 0.0f ? radius_ : 0.2f);
    grid_.cell_size_ = search_r;
    grid_.inv_cell_ = 1.0f / search_r;
    grid_.build(in);
    compute(in, grid_, nx, ny, nz, k, radius, vpx, vpy, vpz);
}

void NormalEstimation::compute(const PointCloudView& in, const Fast3DSpatialGrid& grid, float* nx, float* ny, float* nz,
                               int k, float radius, float vpx, float vpy, float vpz) {
    if (in.n == 0) return;
    const size_t n = in.n;

    float search_r = radius > 0.0f ? radius : (radius_ > 0.0f ? radius_ : 0.2f);
    float r2 = search_r * search_r;

    int eff_threads = num_threads_;
#if defined(_OPENMP)
    if (eff_threads <= 0) eff_threads = omp_get_max_threads();
    if (omp_in_parallel()) eff_threads = 1; // anti-oversubscription
#else
    eff_threads = 1;
#endif

    if (static_cast<int>(thread_neighbors_.size()) < eff_threads) {
        thread_neighbors_.resize(eff_threads);
        thread_dists2_.resize(eff_threads);
        for (int t = 0; t < eff_threads; ++t) {
            thread_neighbors_[t].reserve(256);
            thread_dists2_[t].reserve(256);
        }
    }

    auto process_point = [&](size_t i, std::vector<int>& nbrs, std::vector<float>& dists) {
        const float qx = in.x[i];
        const float qy = in.y[i];
        const float qz = in.z[i];

        grid.radiusSearch(qx, qy, qz, r2, nbrs, dists);

        if (nbrs.size() < 3) {
            nx[i] = 0.0f;
            ny[i] = 0.0f;
            nz[i] = 1.0f;
            return;
        }

        size_t actual_k = nbrs.size();
        if (k > 0 && actual_k > static_cast<size_t>(k + 1)) {
            actual_k = static_cast<size_t>(k + 1);
            std::partial_sort(nbrs.begin(), nbrs.begin() + actual_k, nbrs.end(),
                              [&](int a, int b) {
                                  float da = (in.x[a] - qx) * (in.x[a] - qx) +
                                             (in.y[a] - qy) * (in.y[a] - qy) +
                                             (in.z[a] - qz) * (in.z[a] - qz);
                                  float db = (in.x[b] - qx) * (in.x[b] - qx) +
                                             (in.y[b] - qy) * (in.y[b] - qy) +
                                             (in.z[b] - qz) * (in.z[b] - qz);
                                  return da < db;
                              });
        }

        float cx = 0.0f, cy = 0.0f, cz = 0.0f;
        for (size_t j = 0; j < actual_k; ++j) {
            int idx = nbrs[j];
            cx += in.x[idx];
            cy += in.y[idx];
            cz += in.z[idx];
        }
        float inv_k = 1.0f / static_cast<float>(actual_k);
        cx *= inv_k;
        cy *= inv_k;
        cz *= inv_k;

        float cov[3][3] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
        for (size_t j = 0; j < actual_k; ++j) {
            int idx = nbrs[j];
            float dx = in.x[idx] - cx;
            float dy = in.y[idx] - cy;
            float dz = in.z[idx] - cz;
            cov[0][0] += dx * dx;
            cov[0][1] += dx * dy;
            cov[0][2] += dx * dz;
            cov[1][1] += dy * dy;
            cov[1][2] += dy * dz;
            cov[2][2] += dz * dz;
        }
        cov[1][0] = cov[0][1];
        cov[2][0] = cov[0][2];
        cov[2][1] = cov[1][2];

        simple_eigen3x3_smallest(cov, nx[i], ny[i], nz[i], eigen_iters_);
        flipNormalTowardsViewpoint(PointXYZ{qx, qy, qz}, vpx, vpy, vpz, nx[i], ny[i], nz[i]);
    };

#if defined(_OPENMP)
    #pragma omp parallel num_threads(eff_threads) if(eff_threads > 1)
    {
        int tid = omp_get_thread_num();
        auto& t_nbrs = thread_neighbors_[tid];
        auto& t_dists = thread_dists2_[tid];

        #pragma omp for schedule(dynamic, 128)
        for (size_t i = 0; i < n; ++i) {
            process_point(i, t_nbrs, t_dists);
        }
    }
#else
    for (size_t i = 0; i < n; ++i) {
        process_point(i, neighbors_, dists2_);
    }
#endif
}

} // namespace rvpoint

