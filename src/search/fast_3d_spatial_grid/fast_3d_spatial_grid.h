#pragma once

#include "core/point_types.h"
#include <vector>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <algorithm>

namespace rvpoint {

inline int rv_fast_floor(float f) {
    int i = static_cast<int>(f);
    return i - (f < static_cast<float>(i));
}

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

    std::vector<uint32_t> touched_slots_;

    Fast3DSpatialGrid(float cell_size = 0.25f, size_t expected_points = 65536);

    inline size_t hash3D(int x, int y, int z) const {
        size_t h = (static_cast<size_t>(x) * 73856093) ^
                   (static_cast<size_t>(y) * 19349663) ^
                   (static_cast<size_t>(z) * 83492791);
        return h & mask_;
    }

    bool build(const float* x, const float* y, const float* z, size_t n);
    bool build(const PointCloudView &cloud);

    // Legacy radius search interface (r2 is radius squared)
    void radiusSearch(float qx, float qy, float qz, float r2,
                      std::vector<int>& neighbors, std::vector<float>& dists2) const;

    void radiusSearchDists(float qx, float qy, float qz, float r2,
                           std::vector<float>& dists2) const;

    int countNeighbors(float qx, float qy, float qz, float r2, int max_needed) const;

    // Modern zero-heap RadiusSearchable interface
    void radius_search(float qx, float qy, float qz, float radius, NeighborQueryResult& result) const;
    int count_neighbors(float qx, float qy, float qz, float radius, int max_needed) const;

    // Zero-overhead unvisited neighbor search for connected-component clustering
    void query_unvisited(float qx, float qy, float qz, float r2,
                         const uint8_t* visited, std::vector<int>& out_neighbors) const;

    float cell_size() const noexcept { return cell_size_; }
    float inv_cell() const noexcept { return inv_cell_; }
    size_t capacity() const noexcept { return capacity_; }
    size_t size() const noexcept { return n_pts_; }
};

/**
 * @brief Zero-allocation Functor for building Fast3DSpatialGrid stages in DAG pipelines.
 * Adheres to ADR-0010 and ADR-0011.
 */
class SpatialGridBuilder {
public:
    void operator()(const PointCloudView& in, Fast3DSpatialGrid& grid, float cell_size) const {
        if (grid.cell_size_ != cell_size || grid.capacity_ < in.n) {
            grid = Fast3DSpatialGrid(cell_size, std::max<size_t>(65536, in.n));
        }
        grid.build(in);
    }
};

} // namespace rvpoint
