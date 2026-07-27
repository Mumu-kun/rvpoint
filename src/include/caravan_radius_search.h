#pragma once

#include "rvv_pcl.h"
#include <cstdint>
#include <memory>
#include <vector>

namespace rvv_pcl {

class CaravanRadiusSearch : public NeighborSearch {
public:
  CaravanRadiusSearch() = default;
  ~CaravanRadiusSearch() override = default;

  void setInputCloud(const PointCloudSoA &cloud) override {
    NeighborSearch::setInputCloud(cloud);
    cloud_ = nullptr;
  }

  void setInputCloud(PointCloudSoAPtr cloud) {
    cloud_ = cloud;
    if (cloud) {
      NeighborSearch::setInputCloud(*cloud);
    } else {
      pointCloud_ = nullptr;
    }
  }

  void buildIndex() override {}

  /// Expose the stored search radius so batch callers can retrieve it.
  float searchRadius() const { return searchRadius_; }

  /**
   * @brief NeighborSearch base interface implementation for radius search by query index
   */
  std::size_t radiusSearch(int queryPointIndex, std::vector<int> &resultIndices,
                           std::vector<float> *resultDistances = nullptr,
                           int maxResults = 0) const override;

  /**
   * @brief Caravan Query-Pack Vectorized Radius Search for query cloud pointer
   * @param queries Shared pointer to SoA query cloud
   * @param radius Search radius R
   * @param results Output neighbor indices for each query point
   */
  void batchRadiusSearch(const PointCloudSoAPtr &queries, float radius,
                         std::vector<std::vector<int32_t>> &results) const;

  /**
   * @brief Caravan Query-Pack Vectorized Radius Search for query cloud reference
   * @param queries Reference to SoA query cloud
   * @param radius Search radius R
   * @param results Output neighbor indices for each query point
   */
  void batchRadiusSearch(const PointCloudSoA &queries, float radius,
                         std::vector<std::vector<int>> &results) const override;

  /**
   * @brief Single PointXYZ query radius search
   */
  void radiusSearch(const PointXYZ &query, float radius,
                    std::vector<int32_t> &indices) const;

private:
  PointCloudSoAPtr cloud_ = nullptr;
};

} // namespace rvv_pcl
