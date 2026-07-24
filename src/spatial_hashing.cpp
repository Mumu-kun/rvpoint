#include "include/rvv_pcl.h"

#include <algorithm>
#include <cmath>

namespace rvv_pcl {

void SpatialHashNeighborSearch::setCellSize(float size) { cellSize_ = size; }

void SpatialHashNeighborSearch::buildIndex() { buildHashTable(); }

void SpatialHashNeighborSearch::buildHashTable() {
  grid_.clear();
  if (!pointCloud_ || pointCloud_->empty()) {
    return;
  }

  min_x_ = RVVHelper::vmin(pointCloud_->xData(), pointCloud_->size());
  min_y_ = RVVHelper::vmin(pointCloud_->yData(), pointCloud_->size());
  min_z_ = RVVHelper::vmin(pointCloud_->zData(), pointCloud_->size());
  float max_x = RVVHelper::vmax(pointCloud_->xData(), pointCloud_->size());
  float max_y = RVVHelper::vmax(pointCloud_->yData(), pointCloud_->size());
  float max_z = RVVHelper::vmax(pointCloud_->zData(), pointCloud_->size());
  tableSize_ = static_cast<int>(pointCloud_->size());

  for (std::size_t i = 0; i < pointCloud_->size(); ++i) {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    getCellIndices(pointCloud_->xCoords()[i], pointCloud_->yCoords()[i],
                   pointCloud_->zCoords()[i], ix, iy, iz);
    grid_[computeHashKey(ix, iy, iz)].push_back(static_cast<int>(i));
  }
}

std::int64_t SpatialHashNeighborSearch::computeHashKey(int ix, int iy, int iz) const {
  return static_cast<std::int64_t>(ix) + static_cast<std::int64_t>(iy) * p1_ +
         static_cast<std::int64_t>(iz) * p2_;
}

void SpatialHashNeighborSearch::getCellIndices(float x, float y, float z, int &ix, int &iy,
                                               int &iz) const {
  ix = static_cast<int>(std::floor((x - min_x_) / cellSize_));
  iy = static_cast<int>(std::floor((y - min_y_) / cellSize_));
  iz = static_cast<int>(std::floor((z - min_z_) / cellSize_));
}

void SpatialHashNeighborSearch::getCellNeighborPoints(std::int64_t cellKey,
                                                      std::vector<int> &resultIndices) const {
  const auto it = grid_.find(cellKey);
  if (it == grid_.end()) {
    return;
  }
  resultIndices.insert(resultIndices.end(), it->second.begin(), it->second.end());
}

std::size_t SpatialHashNeighborSearch::radiusSearch(
    int queryPointIndex, std::vector<int> &resultIndices, std::vector<float> *resultDistances,
    int maxResults) const {
  resultIndices.clear();
  std::vector<float> local_dists;
  std::vector<float> &dists = resultDistances ? *resultDistances : local_dists;
  dists.clear();

  if (!pointCloud_ || queryPointIndex < 0 ||
      static_cast<std::size_t>(queryPointIndex) >= pointCloud_->size()) {
    return 0;
  }

  const PointXYZ query = queryPoint(queryPointIndex);
  int qix = 0;
  int qiy = 0;
  int qiz = 0;
  getCellIndices(query.x, query.y, query.z, qix, qiy, qiz);

  const int cell_range = static_cast<int>(std::ceil(searchRadius_ / cellSize_)) + 1;
  for (int dx = -cell_range; dx <= cell_range; ++dx) {
    for (int dy = -cell_range; dy <= cell_range; ++dy) {
      for (int dz = -cell_range; dz <= cell_range; ++dz) {
        const std::int64_t key = computeHashKey(qix + dx, qiy + dy, qiz + dz);
        auto it = grid_.find(key);
        if (it == grid_.end()) {
          continue;
        }
        const std::size_t num = it->second.size();
        if (num == 0) continue;
        std::vector<float> d2(num);
        const float r2 = searchRadius_ * searchRadius_;
        RVVHelper::gatherDistanceSquared(pointCloud_->xData(), pointCloud_->yData(), pointCloud_->zData(),
                                         it->second.data(), num, query.x, query.y, query.z, d2.data());
        for (std::size_t i = 0; i < num; ++i) {
          if (d2[i] <= r2) {
            resultIndices.push_back(it->second[i]);
            dists.push_back(d2[i]);
          }
        }
      }
    }
  }

  std::vector<std::pair<float, int>> zipped;
  zipped.reserve(resultIndices.size());
  for (std::size_t i = 0; i < resultIndices.size(); ++i) {
    zipped.emplace_back(dists[i], resultIndices[i]);
  }
  std::sort(zipped.begin(), zipped.end());
  for (std::size_t i = 0; i < zipped.size(); ++i) {
    dists[i] = zipped[i].first;
    resultIndices[i] = zipped[i].second;
  }

  if (maxResults > 0 && resultIndices.size() > static_cast<std::size_t>(maxResults)) {
    resultIndices.resize(static_cast<std::size_t>(maxResults));
    dists.resize(static_cast<std::size_t>(maxResults));
  }
  return resultIndices.size();
}

} // namespace rvv_pcl
