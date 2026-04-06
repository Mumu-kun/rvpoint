#include "include/rvv_pcl.h"

#include <cmath>
#include <map>
#include <tuple>

namespace rvv_pcl {

void VoxelGridFilter::setLeafSize(float size) { leafSize_ = size; }

std::int64_t VoxelGridFilter::hashPointToVoxel(float x, float y, float z) const {
  const int ix = static_cast<int>(std::floor(x / leafSize_));
  const int iy = static_cast<int>(std::floor(y / leafSize_));
  const int iz = static_cast<int>(std::floor(z / leafSize_));
  return static_cast<std::int64_t>(ix) * 73856093LL +
         static_cast<std::int64_t>(iy) * 19349663LL +
         static_cast<std::int64_t>(iz) * 83492791LL;
}

PointXYZ VoxelGridFilter::computeVoxelAverage(const std::vector<int> &indices) const {
  PointXYZ sum;
  for (const int index : indices) {
    const PointXYZ point = input_->point(static_cast<std::size_t>(index));
    sum.x += point.x;
    sum.y += point.y;
    sum.z += point.z;
  }

  const float inv = 1.0f / static_cast<float>(indices.size());
  return {sum.x * inv, sum.y * inv, sum.z * inv};
}

void VoxelGridFilter::filter(PointCloudSoA &output) const {
  output.clear();
  if (!input_ || input_->empty()) {
    return;
  }

  std::map<std::tuple<int, int, int>, std::vector<int>> voxel_map;
  const float inv_leaf = 1.0f / leafSize_;
  for (std::size_t i = 0; i < input_->size(); ++i) {
    const PointXYZ point = input_->point(i);
    const int ix = static_cast<int>(std::floor(point.x * inv_leaf));
    const int iy = static_cast<int>(std::floor(point.y * inv_leaf));
    const int iz = static_cast<int>(std::floor(point.z * inv_leaf));
    voxel_map[std::make_tuple(ix, iy, iz)].push_back(static_cast<int>(i));
  }

  output.reserve(voxel_map.size());
  for (const auto &entry : voxel_map) {
    output.push_back(computeVoxelAverage(entry.second));
  }
}

} // namespace rvv_pcl
