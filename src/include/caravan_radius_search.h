#pragma once

#include "rvv_pcl.h"
#include <cstdint>
#include <vector>

namespace rvv_pcl {

class CaravanRadiusSearch {
public:
  CaravanRadiusSearch() = default;
  ~CaravanRadiusSearch() = default;

  void setInputCloud(const PointCloudSoA &cloud) {
    cloud_ = cloud;
  }

  /**
   * @brief Caravan Query-Pack Vectorized Radius Search
   * @param queries Reference to SoA query cloud
   * @param radius Search radius R
   * @param results Output neighbor indices for each query point
   */
  void batchRadiusSearch(const PointCloudSoA &queries, float radius,
                         std::vector<std::vector<int32_t>> &results) const;

  /**
   * @brief Single PointXYZ query radius search
   */
  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int32_t> &indices) const;

private:
  PointCloudSoA cloud_;
};

/**
 * @brief Grid-Caravan Bounding Box Pruned SOR (O(N_local * Q / VL)).
 * Computes tile AABB + radius, queries SpatialHash cells, and streams candidate points into vector registers.
 */
std::size_t sor_grid_caravan(const PointCloudSoA &in, PointXYZ *out, int k, float alpha, float search_radius = 0.5f);

} // namespace rvv_pcl
