#include "filters/radius_outlier_removal.h"
#include <algorithm>

#if defined(_OPENMP)
#include <omp.h>
#endif

namespace rvpoint {

RadiusOutlierRemoval::RadiusOutlierRemoval(float search_radius, int min_neighbors, Backend backend)
    : search_radius_(search_radius), min_neighbors_(min_neighbors), backend_(backend), grid_(search_radius, 65536) {}

void RadiusOutlierRemoval::reserve(std::size_t max_points) {
    grid_ = Fast3DSpatialGrid(search_radius_, max_points);
    keep_.reserve(max_points);
}

std::size_t RadiusOutlierRemoval::operator()(const PointCloudView& in, const Fast3DSpatialGrid& grid,
                                              PointCloud& out, float search_radius, int min_neighbors) {
    if (in.empty()) {
        out.clear();
        return 0;
    }

    if (keep_.size() < in.n) {
        keep_.resize(in.n);
    }
    std::fill(keep_.begin(), keep_.begin() + in.n, uint8_t(0));

    int eff_threads = num_threads_;
#if defined(_OPENMP)
    if (eff_threads <= 0) eff_threads = omp_get_max_threads();
    if (omp_in_parallel()) eff_threads = 1; // anti-oversubscription inside active parallel regions
#else
    eff_threads = 1;
#endif

    float r2 = search_radius * search_radius;
    const float* px = in.x;
    const float* py = in.y;
    const float* pz = in.z;
    uint8_t* keep_ptr = keep_.data();

#if defined(_OPENMP)
    #pragma omp parallel for num_threads(eff_threads) schedule(dynamic, 128) if(eff_threads > 1)
#endif
    for (std::size_t i = 0; i < in.n; ++i) {
        int count = grid.countNeighbors(px[i], py[i], pz[i], r2, min_neighbors);
        if (count >= min_neighbors) {
            keep_ptr[i] = 1;
        }
    }

    out.clear();
    out.reserve(in.n);
    for (std::size_t i = 0; i < in.n; ++i) {
        if (keep_ptr[i]) {
            out.push_back(px[i], py[i], pz[i]);
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
