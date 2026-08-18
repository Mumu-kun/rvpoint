#pragma once

#include "core/point_types.h"
#include "search/pointer_octree.h"
#include <cstdint>
#include <vector>
#include <cstddef>

namespace rvpoint {

class CaravanPointerOctree {
public:
  CaravanPointerOctree() = default;
  ~CaravanPointerOctree() = default;

  void setInputCloud(const PointCloudSoA &cloud);
  void build();

  void batchRadiusSearch(const PointCloudSoA &queries, float radius,
                         std::vector<std::vector<int32_t>> &results) const;

  std::size_t sorFilter(PointXYZ *out, int k, float alpha, float search_radius = 0.5f) const;

private:
  PointCloudSoA cloud_;
  PointerOctree ptr_octree_;
};

} // namespace rvpoint
