#include "include/rvv_pcl.h"
#include <cmath>
#include <algorithm>
#include <limits>

#ifdef __riscv_vector
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

// ============================================================================
// SpatialHash Implementation
// ============================================================================

SpatialHash::SpatialHash() 
    : cell_size_(0.0f), min_x_(0), min_y_(0), min_z_(0),
      max_x_(0), max_y_(0), max_z_(0),
      grid_size_x_(0), grid_size_y_(0), grid_size_z_(0) {}

SpatialHash::~SpatialHash() {}

void SpatialHash::setInputCloud(const PointCloudSoA& cloud, float cell_size) {
    cloud_ = cloud;
    cell_size_ = cell_size;
    
    if (cloud.n == 0) return;
    
    // Compute bounding box using RVV
    min_x_ = min_y_ = min_z_ = std::numeric_limits<float>::max();
    max_x_ = max_y_ = max_z_ = std::numeric_limits<float>::lowest();
    
#ifdef __riscv_vector
    // RVV-optimized bounding box computation
    size_t n = cloud.n;
    size_t i = 0;
    
    vfloat32m4_t vmin_x = __riscv_vfmv_v_f_f32m4(min_x_, __riscv_vsetvl_e32m4(1));
    vfloat32m4_t vmin_y = __riscv_vfmv_v_f_f32m4(min_y_, __riscv_vsetvl_e32m4(1));
    vfloat32m4_t vmin_z = __riscv_vfmv_v_f_f32m4(min_z_, __riscv_vsetvl_e32m4(1));
    vfloat32m4_t vmax_x = __riscv_vfmv_v_f_f32m4(max_x_, __riscv_vsetvl_e32m4(1));
    vfloat32m4_t vmax_y = __riscv_vfmv_v_f_f32m4(max_y_, __riscv_vsetvl_e32m4(1));
    vfloat32m4_t vmax_z = __riscv_vfmv_v_f_f32m4(max_z_, __riscv_vsetvl_e32m4(1));
    
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m4(n - i);
        
        vfloat32m4_t vx = __riscv_vle32_v_f32m4(&cloud.x[i], vl);
        vfloat32m4_t vy = __riscv_vle32_v_f32m4(&cloud.y[i], vl);
        vfloat32m4_t vz = __riscv_vle32_v_f32m4(&cloud.z[i], vl);
        
        vmin_x = __riscv_vfmin_vv_f32m4(vmin_x, vx, vl);
        vmin_y = __riscv_vfmin_vv_f32m4(vmin_y, vy, vl);
        vmin_z = __riscv_vfmin_vv_f32m4(vmin_z, vz, vl);
        
        vmax_x = __riscv_vfmax_vv_f32m4(vmax_x, vx, vl);
        vmax_y = __riscv_vfmax_vv_f32m4(vmax_y, vy, vl);
        vmax_z = __riscv_vfmax_vv_f32m4(vmax_z, vz, vl);
        
        i += vl;
    }
    
    // Reduction to get final min/max
    vfloat32m1_t v_scalar = __riscv_vfmv_v_f_f32m1(std::numeric_limits<float>::max(), 1);
    vfloat32m1_t min_x_red = __riscv_vfredmin_vs_f32m4_f32m1(vmin_x, v_scalar, __riscv_vsetvl_e32m4(n));
    vfloat32m1_t min_y_red = __riscv_vfredmin_vs_f32m4_f32m1(vmin_y, v_scalar, __riscv_vsetvl_e32m4(n));
    vfloat32m1_t min_z_red = __riscv_vfredmin_vs_f32m4_f32m1(vmin_z, v_scalar, __riscv_vsetvl_e32m4(n));
    
    v_scalar = __riscv_vfmv_v_f_f32m1(std::numeric_limits<float>::lowest(), 1);
    vfloat32m1_t max_x_red = __riscv_vfredmax_vs_f32m4_f32m1(vmax_x, v_scalar, __riscv_vsetvl_e32m4(n));
    vfloat32m1_t max_y_red = __riscv_vfredmax_vs_f32m4_f32m1(vmax_y, v_scalar, __riscv_vsetvl_e32m4(n));
    vfloat32m1_t max_z_red = __riscv_vfredmax_vs_f32m4_f32m1(vmax_z, v_scalar, __riscv_vsetvl_e32m4(n));
    
    __riscv_vse32_v_f32m1(&min_x_, min_x_red, 1);
    __riscv_vse32_v_f32m1(&min_y_, min_y_red, 1);
    __riscv_vse32_v_f32m1(&min_z_, min_z_red, 1);
    __riscv_vse32_v_f32m1(&max_x_, max_x_red, 1);
    __riscv_vse32_v_f32m1(&max_y_, max_y_red, 1);
    __riscv_vse32_v_f32m1(&max_z_, max_z_red, 1);
    
#else
    // Scalar fallback
    for (size_t i = 0; i < cloud.n; ++i) {
        min_x_ = std::min(min_x_, cloud.x[i]);
        min_y_ = std::min(min_y_, cloud.y[i]);
        min_z_ = std::min(min_z_, cloud.z[i]);
        max_x_ = std::max(max_x_, cloud.x[i]);
        max_y_ = std::max(max_y_, cloud.y[i]);
        max_z_ = std::max(max_z_, cloud.z[i]);
    }
#endif
    
    // Add small epsilon to avoid boundary issues
    min_x_ -= cell_size_ * 0.01f;
    min_y_ -= cell_size_ * 0.01f;
    min_z_ -= cell_size_ * 0.01f;
    max_x_ += cell_size_ * 0.01f;
    max_y_ += cell_size_ * 0.01f;
    max_z_ += cell_size_ * 0.01f;
    
    grid_size_x_ = (int)std::ceil((max_x_ - min_x_) / cell_size_) + 1;
    grid_size_y_ = (int)std::ceil((max_y_ - min_y_) / cell_size_) + 1;
    grid_size_z_ = (int)std::ceil((max_z_ - min_z_) / cell_size_) + 1;
}

void SpatialHash::build() {
    grid_.clear();
    grid_.reserve(cloud_.n / 4); // Heuristic: average 4 points per cell
    
    // Insert all points into grid using RVV-accelerated hashing
    for (size_t i = 0; i < cloud_.n; ++i) {
        int ix, iy, iz;
        getCellIndices(cloud_.x[i], cloud_.y[i], cloud_.z[i], ix, iy, iz);
        int64_t hash = hashCell(ix, iy, iz);
        grid_[hash].push_back(i);
    }
}

std::size_t SpatialHash::radiusSearch(const PointXYZ& query, float radius,
                                       std::vector<int>& indices,
                                       std::vector<float>& dists,
                                       int max_nn) const {
    indices.clear();
    dists.clear();
    
    float r2 = radius * radius;
    
    // Get query cell
    int qix, qiy, qiz;
    getCellIndices(query.x, query.y, query.z, qix, qiy, qiz);
    
    // Determine search range in cell coordinates
    int cell_range = (int)std::ceil(radius / cell_size_) + 1;
    
    // Search neighboring cells
    for (int dx = -cell_range; dx <= cell_range; ++dx) {
        for (int dy = -cell_range; dy <= cell_range; ++dy) {
            for (int dz = -cell_range; dz <= cell_range; ++dz) {
                int cx = qix + dx;
                int cy = qiy + dy;
                int cz = qiz + dz;
                
                int64_t hash = hashCell(cx, cy, cz);
                auto it = grid_.find(hash);
                if (it == grid_.end()) continue;
                
                const std::vector<int>& cell_indices = it->second;
                if (cell_indices.empty()) continue;
                
                // Use RVV fused gather-filter kernel for this cell
                get_inds_in_radius_rvv(
                    cloud_.x, cloud_.y, cloud_.z,
                    cell_indices.data(), cell_indices.size(),
                    query.x, query.y, query.z, r2,
                    indices, dists
                );
            }
        }
    }
    
    // Sort by distance (optional, for consistency with Octree)
    if (!indices.empty()) {
        std::vector<std::pair<float, int>> pairs;
        pairs.reserve(indices.size());
        for (size_t i = 0; i < indices.size(); ++i) {
            pairs.emplace_back(dists[i], indices[i]);
        }
        std::sort(pairs.begin(), pairs.end());
        
        for (size_t i = 0; i < pairs.size(); ++i) {
            dists[i] = pairs[i].first;
            indices[i] = pairs[i].second;
        }
        
        // Apply max_nn limit if specified
        if (max_nn > 0 && indices.size() > (size_t)max_nn) {
            indices.resize(max_nn);
            dists.resize(max_nn);
        }
    }
    
    return indices.size();
}

} // namespace rvv_pcl
