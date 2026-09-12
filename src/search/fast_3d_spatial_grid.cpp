#include "search/fast_3d_spatial_grid.h"
#include <algorithm>
#include <cmath>

namespace rvpoint {

Fast3DSpatialGrid::Fast3DSpatialGrid(float cell_size, size_t expected_points)
    : cell_size_(cell_size), inv_cell_(1.0f / cell_size)
{
    capacity_ = next_power_of_2(std::max<size_t>(1024, expected_points * 2));
    mask_ = capacity_ - 1;
    cells_.resize(capacity_);
    touched_slots_.reserve(std::min<size_t>(capacity_, 65536));
}

bool Fast3DSpatialGrid::build(const float* x, const float* y, const float* z, size_t n) {
    px_ = x; py_ = y; pz_ = z; n_pts_ = n;
    if (capacity_ < n * 2) {
        capacity_ = next_power_of_2(std::max<size_t>(1024, n * 2));
        mask_ = capacity_ - 1;
        cells_.resize(capacity_);
        for (size_t i = 0; i < capacity_; ++i) cells_[i].head = -1;
        touched_slots_.clear();
    } else {
        for (uint32_t slot : touched_slots_) {
            cells_[slot].head = -1;
        }
        touched_slots_.clear();
    }
    if (next_.size() < n) next_.resize(n);

    for (size_t i = 0; i < n; ++i) {
        int cx = rv_fast_floor(x[i] * inv_cell_);
        int cy = rv_fast_floor(y[i] * inv_cell_);
        int cz = rv_fast_floor(z[i] * inv_cell_);

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
            touched_slots_.push_back(static_cast<uint32_t>(h));
        }
        next_[i] = cells_[h].head;
        cells_[h].head = static_cast<int>(i);
    }
    return true;
}

bool Fast3DSpatialGrid::build(const PointCloudView &cloud) {
    return build(cloud.x, cloud.y, cloud.z, cloud.n);
}

void Fast3DSpatialGrid::radiusSearch(float qx, float qy, float qz, float r2,
                                     std::vector<int>& neighbors, std::vector<float>& dists2) const {
    neighbors.clear(); dists2.clear();
    int qcx = rv_fast_floor(qx * inv_cell_);
    int qcy = rv_fast_floor(qy * inv_cell_);
    int qcz = rv_fast_floor(qz * inv_cell_);

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

void Fast3DSpatialGrid::radiusSearchDists(float qx, float qy, float qz, float r2,
                                          std::vector<float>& dists2) const {
    dists2.clear();
    int qcx = rv_fast_floor(qx * inv_cell_);
    int qcy = rv_fast_floor(qy * inv_cell_);
    int qcz = rv_fast_floor(qz * inv_cell_);

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

int Fast3DSpatialGrid::countNeighbors(float qx, float qy, float qz, float r2, int max_needed) const {
    int count = 0;
    int qcx = rv_fast_floor(qx * inv_cell_);
    int qcy = rv_fast_floor(qy * inv_cell_);
    int qcz = rv_fast_floor(qz * inv_cell_);

    // 1. Check self cell first (fastpath)
    size_t self_h = hash3D(qcx, qcy, qcz);
    int probe = 0;
    while (cells_[self_h].head != -1 && probe < kMaxProbes) {
        if (cells_[self_h].cx == qcx && cells_[self_h].cy == qcy && cells_[self_h].cz == qcz) {
            int curr = cells_[self_h].head;
            while (curr != -1) {
                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                    if (++count >= max_needed) return count;
                }
                curr = next_[curr];
            }
            break;
        }
        self_h = (self_h + 1) & mask_;
        probe++;
    }

    // 2. Check 26 neighbor cells if needed
    for (int dz = -1; dz <= 1 && count < max_needed; ++dz) {
        for (int dy = -1; dy <= 1 && count < max_needed; ++dy) {
            for (int dx = -1; dx <= 1 && count < max_needed; ++dx) {
                if (dx == 0 && dy == 0 && dz == 0) continue;
                int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                size_t h = hash3D(tcx, tcy, tcz);
                int p = 0;
                while (cells_[h].head != -1 && p < kMaxProbes) {
                    if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                        int curr = cells_[h].head;
                        while (curr != -1) {
                            float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                            if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                if (++count >= max_needed) return count;
                            }
                            curr = next_[curr];
                        }
                        break;
                    }
                    h = (h + 1) & mask_;
                    p++;
                }
            }
        }
    }
    return count;
}

void Fast3DSpatialGrid::radius_search(float qx, float qy, float qz, float radius, NeighborQueryResult& result) const {
    result.clear();
    result.offsets.push_back(0);
    float r2 = radius * radius;
    int qcx = rv_fast_floor(qx * inv_cell_);
    int qcy = rv_fast_floor(qy * inv_cell_);
    int qcz = rv_fast_floor(qz * inv_cell_);

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
                                result.indices.push_back(static_cast<int32_t>(curr));
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
    result.offsets.push_back(static_cast<uint32_t>(result.indices.size()));
}

int Fast3DSpatialGrid::count_neighbors(float qx, float qy, float qz, float radius, int max_needed) const {
    return countNeighbors(qx, qy, qz, radius * radius, max_needed);
}

void Fast3DSpatialGrid::query_unvisited(float qx, float qy, float qz, float r2,
                                       const uint8_t* visited, std::vector<int>& out_neighbors) const {
    out_neighbors.clear();
    int qcx = rv_fast_floor(qx * inv_cell_);
    int qcy = rv_fast_floor(qy * inv_cell_);
    int qcz = rv_fast_floor(qz * inv_cell_);

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
                            if (!visited[curr]) {
                                float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                                float d2 = ddx * ddx + ddy * ddy + ddz * ddz;
                                if (d2 <= r2) {
                                    out_neighbors.push_back(curr);
                                }
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

} // namespace rvpoint

