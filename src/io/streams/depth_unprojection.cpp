#include "io/streams/depth_unprojection.h"

#include <cmath>

namespace rvpoint {

void unproject_depth_map(const float* depth,
                         const uint8_t* conf,
                         uint32_t w, uint32_t h,
                         float fx, float fy, float cx, float cy,
                         uint8_t min_conf,
                         float min_range,
                         float max_range,
                         PointCloud& cloud) {
    cloud.clear();
    if (cloud.capacity() < w * h) {
        cloud.reserve(w * h);
    }

    for (uint32_t v = 0; v < h; ++v) {
        float dy = (static_cast<float>(v) - cy) / fy;
        uint32_t row_offset = v * w;
        for (uint32_t u = 0; u < w; ++u) {
            uint32_t idx = row_offset + u;
            float z = depth[idx];
            uint8_t c = conf ? conf[idx] : 2;

            if (c >= min_conf && z >= min_range && z <= max_range && std::isfinite(z)) {
                float dx = (static_cast<float>(u) - cx) / fx;
                cloud.push_back(dx * z, dy * z, z);
            }
        }
    }
}

} // namespace rvpoint

