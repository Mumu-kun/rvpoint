#include "features/fused_filter_normals.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <algorithm>

namespace rvpoint {

FusedFilterNormals::FusedFilterNormals(float search_radius, int min_neighbors,
                                       bool compute_normals, Backend backend)
    : search_radius_(search_radius), min_neighbors_(min_neighbors),
      compute_normals_(compute_normals), backend_(backend) {}

void FusedFilterNormals::reserve(std::size_t max_points) {
    grid_ = Fast3DSpatialGrid(search_radius_, max_points);
    neighbors_.reserve(256);
    keep_mask_.resize(max_points, 0);
}

std::size_t FusedFilterNormals::operator()(const PointCloudView& in, PointCloud& filtered, PointCloud* normals,
                                          float radius, int min_neighbors, bool compute_normals) {
    filtered.clear();
    if (normals) normals->clear();
    const std::size_t n = in.n;
    if (n == 0 || !in.x || !in.y || !in.z) return 0;

    grid_.cell_size_ = radius;
    grid_.inv_cell_ = 1.0f / radius;
    if (!grid_.build(in.x, in.y, in.z, n)) {
        return 0;
    }

    if (keep_mask_.size() < n) {
        keep_mask_.resize(n, 0);
    } else {
        std::memset(keep_mask_.data(), 0, n * sizeof(uint8_t));
    }

    const float r2 = radius * radius;
    const float inv_cell = grid_.inv_cell_;
    const size_t mask = grid_.mask_;
    const auto& cells = grid_.cells_;
    const auto& next = grid_.next_;

    filtered.reserve(n);
    if (normals && compute_normals) {
        normals->reserve(n);
    }

    std::size_t kept_count = 0;

    for (std::size_t i = 0; i < n; ++i) {
        float qx = in.x[i];
        float qy = in.y[i];
        float qz = in.z[i];

        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        neighbors_.clear();

        // 1. Fastpath: query self-cell first
        size_t self_h = grid_.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = in.x[curr] - qx;
                    float ddy = in.y[curr] - qy;
                    float ddz = in.z[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        neighbors_.push_back(curr);
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        // 2. Query 26 neighboring cells
        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    int tcx = qcx + dx;
                    int tcy = qcy + dy;
                    int tcz = qcz + dz;
                    size_t h = grid_.hash3D(tcx, tcy, tcz);
                    int p = 0;
                    while (cells[h].head != -1 && p < Fast3DSpatialGrid::kMaxProbes) {
                        if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                            int curr = cells[h].head;
                            while (curr != -1) {
                                float ddx = in.x[curr] - qx;
                                float ddy = in.y[curr] - qy;
                                float ddz = in.z[curr] - qz;
                                if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                    neighbors_.push_back(curr);
                                }
                                curr = next[curr];
                            }
                            break;
                        }
                        h = (h + 1) & mask;
                        p++;
                    }
                }
            }
        }

        if (static_cast<int>(neighbors_.size()) >= min_neighbors) {
            filtered.push_back(qx, qy, qz);
            kept_count++;

            if (normals && compute_normals) {
                if (neighbors_.size() >= 3) {
                    float cx = 0.0f, cy = 0.0f, cz = 0.0f;
                    for (int idx : neighbors_) {
                        cx += in.x[idx];
                        cy += in.y[idx];
                        cz += in.z[idx];
                    }
                    float inv_sz = 1.0f / static_cast<float>(neighbors_.size());
                    cx *= inv_sz; cy *= inv_sz; cz *= inv_sz;

                    float c00 = 0.0f, c01 = 0.0f, c02 = 0.0f;
                    float c11 = 0.0f, c12 = 0.0f, c22 = 0.0f;
                    for (int idx : neighbors_) {
                        float ddx = in.x[idx] - cx;
                        float ddy = in.y[idx] - cy;
                        float ddz = in.z[idx] - cz;
                        c00 += ddx * ddx; c01 += ddx * ddy; c02 += ddx * ddz;
                        c11 += ddy * ddy; c12 += ddy * ddz; c22 += ddz * ddz;
                    }

                    // Cardano / analytic normal solver from covariance matrix
                    float vx = c01 * c12 - c02 * c11;
                    float vy = c01 * c02 - c00 * c12;
                    float vz = c00 * c11 - c01 * c01;
                    float norm = std::sqrt(vx * vx + vy * vy + vz * vz);
                    if (norm > 1e-6f) {
                        float inv_norm = 1.0f / norm;
                        normals->push_back(vx * inv_norm, vy * inv_norm, vz * inv_norm);
                    } else {
                        normals->push_back(0.0f, 0.0f, 1.0f);
                    }
                } else {
                    normals->push_back(0.0f, 0.0f, 1.0f);
                }
            }
        }
    }

    return kept_count;
}

} // namespace rvpoint

