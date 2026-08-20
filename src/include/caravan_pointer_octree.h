#pragma once

#include "core/point_types.h"
#include "search/pointer_octree.h"
#include <cstdint>
#include <vector>

namespace rvv_pcl {

/**
 * @brief Caravan-PointerOctree Hybrid
 * Combines spatial octree tile pruning with RISC-V Vector scalar-broadcast leaf registers.
 */
class CaravanPointerOctree {
public:
  CaravanPointerOctree() = default;
  ~CaravanPointerOctree() = default;

  void setInputCloud(const PointCloudSoA &cloud);
  void build();

  /**
   * @brief Caravan Query-Pack Batch Radius Search integrated into PointerOctree.
   * Tiles queries into chunks of width VL, prunes non-overlapping sub-trees via TileAABB,
   * and runs Caravan vector registers on local leaf float arrays.
   */
  void batchRadiusSearch(const PointCloudSoA &queries, float radius,
                         std::vector<std::vector<int32_t>> &results) const;

  /**
   * @brief Statistical Outlier Removal using Caravan-PointerOctree tile search.
   */
  std::size_t sorFilter(PointXYZ *out, int k, float alpha, float search_radius = 0.5f) const;

private:
  PointCloudSoA cloud_;
  PointerOctree ptr_octree_;
};

} // namespace rvv_pcl
