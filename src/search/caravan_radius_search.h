#pragma once

#include "core/point_types.h"
#include <cstdint>
#include <vector>
#include <cstddef>

namespace rvpoint {

class CaravanRadiusSearch {
public:
  CaravanRadiusSearch() = default;
  ~CaravanRadiusSearch() = default;

  void setInputCloud(const PointCloudSoA &cloud) {
    cloud_ = cloud;
  }

  void batchRadiusSearch(const PointCloudSoA &queries, float radius,
                         std::vector<std::vector<int32_t>> &results) const;

  void batchRadiusSearch(const PointCloudView &queries, float radius,
                         NeighborQueryResult &results) const;

  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int32_t> &indices) const;

private:
  PointCloudSoA cloud_;
};

} // namespace rvpoint

