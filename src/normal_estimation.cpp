#include "include/rvv_pcl.h"

#include <algorithm>
#include <cmath>

namespace rvv_pcl {

namespace {

void simple_eigen3x3_smallest(float cov[3][3], float &nx, float &ny, float &nz) {
  float A[3][3];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      A[i][j] = cov[i][j];
    }
  }

  float V[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  for (int iter = 0; iter < 12; ++iter) {
    int p = 0;
    int q = 1;
    float max_off = std::abs(A[0][1]);
    if (std::abs(A[0][2]) > max_off) {
      p = 0;
      q = 2;
      max_off = std::abs(A[0][2]);
    }
    if (std::abs(A[1][2]) > max_off) {
      p = 1;
      q = 2;
      max_off = std::abs(A[1][2]);
    }
    if (max_off < 1e-6f) {
      break;
    }

    const float phi = 0.5f * std::atan2(2.0f * A[p][q], A[q][q] - A[p][p]);
    const float c = std::cos(phi);
    const float s = std::sin(phi);
    const float app = A[p][p];
    const float aqq = A[q][q];
    const float apq = A[p][q];
    const int r = 3 - p - q;
    const float arp = A[r][p];
    const float arq = A[r][q];
    A[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq;
    A[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq;
    A[p][q] = 0.0f;
    A[q][p] = 0.0f;
    A[r][p] = c * arp - s * arq;
    A[p][r] = A[r][p];
    A[r][q] = s * arp + c * arq;
    A[q][r] = A[r][q];

    for (int k = 0; k < 3; ++k) {
      const float vip = V[k][p];
      const float viq = V[k][q];
      V[k][p] = c * vip - s * viq;
      V[k][q] = s * vip + c * viq;
    }
  }

  int min_idx = 0;
  if (A[1][1] < A[min_idx][min_idx]) {
    min_idx = 1;
  }
  if (A[2][2] < A[min_idx][min_idx]) {
    min_idx = 2;
  }

  nx = V[0][min_idx];
  ny = V[1][min_idx];
  nz = V[2][min_idx];
}

void flip_towards_origin(const PointXYZ &point, float &nx, float &ny, float &nz) {
  const float vx = -point.x;
  const float vy = -point.y;
  const float vz = -point.z;
  if (vx * nx + vy * ny + vz * nz < 0.0f) {
    nx = -nx;
    ny = -ny;
    nz = -nz;
  }
}

} // namespace

void NormalEstimation::computeFeature(int index, const std::vector<int> &neighborIndices,
                                      FeatureCloud &featureCloud) const {
  auto *normals = dynamic_cast<NormalCloud *>(&featureCloud);
  if (!normals || !input_) {
    return;
  }

  if (neighborIndices.size() < 3) {
    normals->setNormal(static_cast<std::size_t>(index), 0.0f, 0.0f, 0.0f);
    return;
  }

  float cx = 0.0f;
  float cy = 0.0f;
  float cz = 0.0f;
  for (const int neighbor : neighborIndices) {
    const PointXYZ point = input_->point(static_cast<std::size_t>(neighbor));
    cx += point.x;
    cy += point.y;
    cz += point.z;
  }
  const float inv_count = 1.0f / static_cast<float>(neighborIndices.size());
  cx *= inv_count;
  cy *= inv_count;
  cz *= inv_count;

  float cov[3][3] = {};
  for (const int neighbor : neighborIndices) {
    const PointXYZ point = input_->point(static_cast<std::size_t>(neighbor));
    const float dx = point.x - cx;
    const float dy = point.y - cy;
    const float dz = point.z - cz;
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

  float nx = 0.0f;
  float ny = 0.0f;
  float nz = 0.0f;
  simple_eigen3x3_smallest(cov, nx, ny, nz);
  flip_towards_origin(input_->point(static_cast<std::size_t>(index)), nx, ny, nz);
  normals->setNormal(static_cast<std::size_t>(index), nx, ny, nz);
}

} // namespace rvv_pcl
