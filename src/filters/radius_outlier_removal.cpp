#include "filters/radius_outlier_removal.h"
#include <algorithm>

namespace rvpoint {

RadiusOutlierRemoval::RadiusOutlierRemoval(float search_radius, int min_neighbors, Backend backend)
    : search_radius_(search_radius), min_neighbors_(min_neighbors), backend_(backend), grid_(search_radius, 65536) {}

void RadiusOutlierRemoval::reserve(std::size_t max_points) {
    grid_ = Fast3DSpatialGrid(search_radius_, max_points);
}

std::size_t RadiusOutlierRemoval::operator()(const PointCloudView& in, PointCloud& out, float search_radius, int min_neighbors) {
std::size_t RadiusOutlierRemoval::operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid,
                                              PointCloud& out, float search_radius, int min_neighbors) {
    if (in.empty()) {
        out.clear();
        return 0;
    }

    if (grid_.cell_size_ != search_radius) {
        grid_ = Fast3DSpatialGrid(search_radius, in.n);
    }
    grid_.build(in);

    out.clear();
    out.reserve(in.n);

    float r2 = search_radius * search_radius;
    for (std::size_t i = 0; i < in.n; ++i) {
        int count = grid_.countNeighbors(in.x[i], in.y[i], in.z[i], r2, min_neighbors);
        int count = grid.countNeighbors(in.x[i], in.y[i], in.z[i], r2, min_neighbors);
        if (count >= min_neighbors) {
            out.push_back(in.x[i], in.y[i], in.z[i]);
        }
    }

    return out.size();
}

std::size_t RadiusOutlierRemoval::operator()(const PointCloudView& in, PointCloud& out, float search_radius, int min_neighbors) {
    if (in.empty()) {
        out.clear();
        return 0;
    }

    if (grid_.cell_size_ != search_radius) {
        grid_ = Fast3DSpatialGrid(search_radius, in.n);
    }
    grid_.build(in);

    return (*this)(in, grid_, out, search_radius, min_neighbors);
}

std::size_t RadiusOutlierRemoval::operator()(const PointCloudView& in, PointCloud& out) {
    return (*this)(in, out, search_radius_, min_neighbors_);
}

} // namespace rvpoint

