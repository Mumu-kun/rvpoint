#pragma once

#include "core/point_types.h"
#include <vector>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace rvpoint {

inline size_t next_power_of_2(size_t v) {
    if (v == 0) return 1;
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;
    return v + 1;
}

class Fast3DSpatialGrid {
public:
    struct Cell {
        int cx = 0, cy = 0, cz = 0;
        int head = -1;
    };

    static constexpr int kMaxProbes = 64;
    size_t capacity_ = 65536;
    size_t mask_ = 65535;
    float cell_size_;
    float inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    Fast3DSpatialGrid(float cell_size = 0.25f, size_t expected_points = 65536)
        : cell_size_(cell_size), inv_cell_(1.0f / cell_size)
    {
        capacity_ = next_power_of_2(std::max<size_t>(65536, expected_points * 4));
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
    }

    inline size_t hash3D(int x, int y, int z) const {
        size_t h = (static_cast<size_t>(x) * 73856093) ^
                   (static_cast<size_t>(y) * 19349663) ^
                   (static_cast<size_t>(z) * 83492791);
        return h & mask_;
    }

    bool build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z; n_pts_ = n;
        if (capacity_ < n * 4) {
            capacity_ = next_power_of_2(std::max<size_t>(65536, n * 4));
            mask_ = capacity_ - 1;
            cells_.resize(capacity_);
        }
        for (size_t i = 0; i < capacity_; ++i) cells_[i].head = -1;
        next_.assign(n, -1);

        for (size_t i = 0; i < n; ++i) {
            if (!std::isfinite(x[i]) || !std::isfinite(y[i]) || !std::isfinite(z[i])) {
                return false;
            }

            int cx = static_cast<int>(std::floor(x[i] * inv_cell_));
            int cy = static_cast<int>(std::floor(y[i] * inv_cell_));
            int cz = static_cast<int>(std::floor(z[i] * inv_cell_));

            size_t h = hash3D(cx, cy, cz);
            int probe = 0;
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz) && probe < kMaxProbes) {
                h = (h + 1) & mask_;
                probe++;
            }
            if (probe >= kMaxProbes) {
                return false; // Fail fast on probe exhaustion rather than silently dropping points
            }
            if (cells_[h].head == -1) {
                cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz;
            }
            next_[i] = cells_[h].head;
            cells_[h].head = static_cast<int>(i);
        }
        return true;
    }

    bool build(const PointCloudSoA &cloud) {
        return build(cloud.x, cloud.y, cloud.z, cloud.n);
    }

    inline void radiusSearch(float qx, float qy, float qz, float r2,
                             std::vector<int>& neighbors, std::vector<float>& dists2) const {
        neighbors.clear(); dists2.clear();
        int qcx = static_cast<int>(std::floor(qx * inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = hash3D(tcx, tcy, tcz);
                    int probe = 0;
                    while (cells_[h].head != -1 && probe < kMaxProbes) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            int curr = cells_[h].head;
                            while (curr != -1) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    neighbors.push_back(curr);
                                    dists2.push_back(d2);
                                }
                                curr = next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & mask_;
                        probe++;
                    }
                }
            }
        }
    }

    inline void radiusSearchDists(float qx, float qy, float qz, float r2,
                                  std::vector<float>& dists2) const {
        dists2.clear();
        int qcx = static_cast<int>(std::floor(qx * inv_cell_));
        int qcy = static_cast<int>(std::floor(qy * inv_cell_));
        int qcz = static_cast<int>(std::floor(qz * inv_cell_));

        for (int dz = -1; dz <= 1; ++dz) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                    size_t h = hash3D(tcx, tcy, tcz);
                    int probe = 0;
                    while (cells_[h].head != -1 && probe < kMaxProbes) {
                        if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                            int curr = cells_[h].head;
                            while (curr != -1) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    dists2.push_back(d2);
                                }
                                curr = next_[curr];
                            }
                            break;
                        }
                        h = (h + 1) & mask_;
                        probe++;
                    }
                }
            }
        }
    }
};

} // namespace rvpoint
