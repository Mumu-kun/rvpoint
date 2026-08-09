// caravan_strategies.cpp
// Strategy 1: Grid-Caravan Bounding Box Pruned SOR Implementation

#include "caravan_strategies.h"
#include "rvv_pcl.h"
#include <algorithm>
#include <cmath>
#include <vector>

#if defined(RVV_PCL_USE_RVV) && defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvv_pcl {

// Helper: Filter cloud based on computed mean distances and alpha threshold
static std::size_t filter_by_mean_dists(const PointCloudSoA &in,
                                        const std::vector<float> &mean_dists,
                                        PointXYZ *out, float alpha) {
  if (in.n == 0) return 0;
  float global_sum = 0.0f;
  for (float d : mean_dists) global_sum += d;
  float global_mean = global_sum / in.n;

  float variance_sum = 0.0f;
  for (float d : mean_dists) {
    float diff = d - global_mean;
    variance_sum += diff * diff;
  }
  float global_std = std::sqrt(variance_sum / in.n);
  float thresh = global_mean + alpha * global_std;

  std::size_t count = 0;
  for (size_t i = 0; i < in.n; ++i) {
    if (mean_dists[i] <= thresh) {
      out[count].x = in.x[i];
      out[count].y = in.y[i];
      out[count].z = in.z[i];
      count++;
    }
  }
  return count;
}

// ----------------------------------------------------------------------------
// Strategy 1: Grid-Caravan Bounding Box Pruned SOR
// ----------------------------------------------------------------------------
std::size_t sor_grid_caravan(const PointCloudSoA &in, PointXYZ *out, int k,
                             float alpha, float search_radius) {
  if (in.n == 0) return 0;
  std::vector<float> mean_dists(in.n, 0.0f);

  SpatialHash hash;
  hash.setInputCloud(in, search_radius);
  hash.build();

  for (size_t i = 0; i < in.n; ++i) {
    PointXYZ query = {in.x[i], in.y[i], in.z[i]};
    std::vector<int> nbr_indices;
    std::vector<float> nbr_dists;
    hash.radiusSearch(query, search_radius, nbr_indices, nbr_dists, k + 1);

    if (nbr_dists.size() > 1) {
      std::sort(nbr_dists.begin(), nbr_dists.end());
      float sum = 0.0f;
      int valid_k = std::min(k, static_cast<int>(nbr_dists.size()) - 1);
      for (int j = 1; j <= valid_k; ++j) {
        sum += std::sqrt(nbr_dists[j]);
      }
      mean_dists[i] = sum / valid_k;
    } else {
      mean_dists[i] = search_radius;
    }
  }
  return filter_by_mean_dists(in, mean_dists, out, alpha);
}

} // namespace rvv_pcl
