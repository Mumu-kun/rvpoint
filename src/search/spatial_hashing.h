#pragma once

#include "core/point_types.h"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace rvpoint {

class SpatialHash {
public:
  SpatialHash();
  ~SpatialHash();

  void setInputCloud(const PointCloudSoA &cloud, float cell_size);
  void build();

  std::size_t radiusSearch(const PointXYZ &query, float radius,
                           std::vector<int> &indices, std::vector<float> &dists,
                           int max_nn = 0) const;

  void setHashPrimes(int64_t p1, int64_t p2) {
    p1_ = p1;
    p2_ = p2;
  }

private:
  PointCloudSoA cloud_;
  float cell_size_;
  float eps_scale_ = 0.01f;
  float reserve_factor_ = 0.25f;
  int64_t p1_ = 73856093LL;
  int64_t p2_ = 19349663LL;

  std::unordered_map<int64_t, std::vector<int>> grid_;

  float min_x_, min_y_, min_z_;
  float max_x_, max_y_, max_z_;
  int grid_size_x_, grid_size_y_, grid_size_z_;

  inline int64_t hashCell(int ix, int iy, int iz) const {
    return (int64_t)ix + (int64_t)iy * p1_ + (int64_t)iz * p2_;
  }

  inline void getCellIndices(float x, float y, float z, int &ix, int &iy,
                             int &iz) const {
    ix = (int)std::floor((x - min_x_) / cell_size_);
    iy = (int)std::floor((y - min_y_) / cell_size_);
    iz = (int)std::floor((z - min_z_) / cell_size_);
  }
};

} // namespace rvpoint
