#include "include/caravan_radius_search.h"
#include <algorithm>
#include <cmath>

namespace rvv_pcl {

void CaravanRadiusSearch::batchRadiusSearch(
    const PointCloudSoA &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
  if (!pointCloud_ || pointCloud_->empty() || queries.empty()) {
    results.clear();
    return;
  }

  const std::size_t num_points = pointCloud_->size();
  const std::size_t num_queries = queries.size();
  results.resize(num_queries);

  const float r_sq = radius * radius;
  const float *const px = pointCloud_->xData();
  const float *const py = pointCloud_->yData();
  const float *const pz = pointCloud_->zData();

  const float *const qx = queries.xData();
  const float *const qy = queries.yData();
  const float *const qz = queries.zData();

  std::vector<float> d2(num_points);
  for (std::size_t q = 0; q < num_queries; ++q) {
    results[q].clear();
    RVVHelper::distanceSquared(px, py, pz, num_points, qx[q], qy[q], qz[q], d2.data());
    for (std::size_t i = 0; i < num_points; ++i) {
      if (d2[i] <= r_sq) {
        results[q].push_back(static_cast<int32_t>(i));
      }
    }
  }
}

void CaravanRadiusSearch::batchRadiusSearch(
    const PointCloudSoAPtr &queries,
    float radius,
    std::vector<std::vector<int32_t>> &results
) const {
  if (!queries) {
    results.clear();
    return;
  }
  batchRadiusSearch(*queries, radius, results);
}

void CaravanRadiusSearch::radiusSearch(
    const PointXYZ &query,
    float radius,
    std::vector<int32_t> &indices
) const {
  PointCloudSoA q_cloud;
  q_cloud.push_back(query);
  std::vector<std::vector<int32_t>> batch_res;
  batchRadiusSearch(q_cloud, radius, batch_res);
  if (!batch_res.empty()) {
    indices = std::move(batch_res[0]);
  } else {
    indices.clear();
  }
}

std::size_t CaravanRadiusSearch::radiusSearch(
    int queryPointIndex,
    std::vector<int> &resultIndices,
    std::vector<float> *resultDistances,
    int maxResults
) const {
  resultIndices.clear();
  if (resultDistances) {
    resultDistances->clear();
  }
  if (!pointCloud_ || queryPointIndex < 0 ||
      static_cast<std::size_t>(queryPointIndex) >= pointCloud_->size()) {
    return 0;
  }

  const PointXYZ q = queryPoint(queryPointIndex);
  const float r = (searchRadius_ > 0.0f) ? searchRadius_ : 0.0f;
  const float r2 = r * r;

  const std::size_t num_points = pointCloud_->size();
  std::vector<float> d2(num_points);
  RVVHelper::distanceSquared(pointCloud_->xData(), pointCloud_->yData(), pointCloud_->zData(),
                             num_points, q.x, q.y, q.z, d2.data());

  for (std::size_t i = 0; i < num_points; ++i) {
    if (d2[i] <= r2) {
      resultIndices.push_back(static_cast<int>(i));
      if (resultDistances) {
        resultDistances->push_back(d2[i]);
      }
      if (maxResults > 0 && resultIndices.size() >= static_cast<std::size_t>(maxResults)) {
        break;
      }
    }
  }

  return resultIndices.size();
}

} // namespace rvv_pcl
