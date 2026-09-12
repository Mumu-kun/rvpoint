#include "filters/statistical_outlier_removal.h"
#include "core/rvv_common.h"
#include "search/octree.h"
#include "search/pointer_octree.h"
#include "search/spatial_hashing.h"

#include <algorithm>
#include <cmath>
#include <vector>
#include <queue>
#include <cstring>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

StatisticalOutlierRemoval::StatisticalOutlierRemoval(int k, float alpha, Backend backend)
    : k_(k), alpha_(alpha), backend_(backend) {}

void StatisticalOutlierRemoval::reserve(std::size_t max_points) {
    mean_dists_.reserve(max_points);
    dists_.reserve(max_points);
    dists_scratch_.reserve(64);
}

std::size_t StatisticalOutlierRemoval::operator()(const PointCloudView& in, PointCloud& out, int k, float alpha) {
    bool use_rvv = false;
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        use_rvv = true;
    }
#else
    if (backend_ == Backend::RVV) {
        use_rvv = false;
    }
#endif

    if (use_rvv) {
        return filter_rvv(in, out, k, alpha);
    } else {
        return filter_scalar(in, out, k, alpha);
    }
}

std::size_t StatisticalOutlierRemoval::filter(const PointCloudView& in, PointXYZ* out, int k, float alpha) {
    bool use_rvv = false;
#if defined(__riscv_vector)
    if (backend_ == Backend::Auto || backend_ == Backend::RVV) {
        use_rvv = true;
    }
#else
    if (backend_ == Backend::RVV) {
        use_rvv = false;
    }
#endif

    if (use_rvv) {
        return filter_rvv_aos(in, out, k, alpha);
    } else {
        return filter_scalar_aos(in, out, k, alpha);
    }
}

std::size_t StatisticalOutlierRemoval::filter_rvv(const PointCloudView& in, PointCloud& out, int k, float alpha) {
    out.clear();
    if (in.n == 0) return 0;
#if defined(__riscv_vector)
    const size_t n = in.n;
    if (mean_dists_.size() < n) mean_dists_.resize(n);
    if (dists_.size() < n) dists_.resize(n);

    // 1. Compute mean K-NN distance using RVV Kernel + Priority Queue
    for (size_t i = 0; i < n; ++i) {
        const float qx = in.x[i];
        const float qy = in.y[i];
        const float qz = in.z[i];

        size_t j = 0;
        while (j < n) {
            size_t vl = __riscv_vsetvl_e32m8(n - j);

            vfloat32m8_t vx = __riscv_vle32_v_f32m8(&in.x[j], vl);
            vfloat32m8_t vy = __riscv_vle32_v_f32m8(&in.y[j], vl);
            vfloat32m8_t vz = __riscv_vle32_v_f32m8(&in.z[j], vl);

            vfloat32m8_t dx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
            vfloat32m8_t dy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
            vfloat32m8_t dz = __riscv_vfsub_vf_f32m8(vz, qz, vl);

            vfloat32m8_t sum = __riscv_vfmul_vv_f32m8(dx, dx, vl);
            sum = __riscv_vfmacc_vv_f32m8(sum, dy, dy, vl);
            sum = __riscv_vfmacc_vv_f32m8(sum, dz, dz, vl);

            __riscv_vse32_v_f32m8(&dists_[j], sum, vl);
            j += vl;
        }

        dists_scratch_.clear();
        for (size_t l = 0; l < n; ++l) {
            if (l == i) continue;
            float d2 = dists_[l];
            if (static_cast<int>(dists_scratch_.size()) < k) {
                dists_scratch_.push_back(d2);
                std::push_heap(dists_scratch_.begin(), dists_scratch_.end());
            } else if (d2 < dists_scratch_.front()) {
                std::pop_heap(dists_scratch_.begin(), dists_scratch_.end());
                dists_scratch_.back() = d2;
                std::push_heap(dists_scratch_.begin(), dists_scratch_.end());
            }
        }

        float sum_dists = 0.0f;
        for (float d2 : dists_scratch_) {
            sum_dists += std::sqrt(d2);
        }
        mean_dists_[i] = (k > 0) ? (sum_dists / k) : 0.0f;
    }

    // 2. Global statistics
    float global_sum = 0.0f;
    for (size_t i = 0; i < n; ++i) global_sum += mean_dists_[i];
    float global_mean = global_sum / n;

    float variance_sum = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        float diff = mean_dists_[i] - global_mean;
        variance_sum += diff * diff;
    }
    float global_std = std::sqrt(variance_sum / n);

    // 3. Filter
    float thresh = global_mean + alpha * global_std;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (mean_dists_[i] <= thresh) {
            out.push_back(in.x[i], in.y[i], in.z[i]);
        }
    }
    return out.size();
#else
    return filter_scalar(in, out, k, alpha);
#endif
}

std::size_t StatisticalOutlierRemoval::filter_scalar(const PointCloudView& in, PointCloud& out, int k, float alpha) {
    out.clear();
    if (in.n == 0) return 0;
    const size_t n = in.n;
    if (mean_dists_.size() < n) mean_dists_.resize(n);
    if (dists_.size() < n) dists_.resize(n);

    // 1. Compute mean K-NN distance for each point
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            float dx = in.x[i] - in.x[j];
            float dy = in.y[i] - in.y[j];
            float dz = in.z[i] - in.z[j];
            dists_[j] = dx * dx + dy * dy + dz * dz;
        }

        std::partial_sort(dists_.begin(), dists_.begin() + k + 1, dists_.begin() + n);

        float sum = 0.0f;
        for (int j = 1; j <= k && j < (int)n; ++j) {
            sum += std::sqrt(dists_[j]);
        }
        mean_dists_[i] = (k > 0) ? (sum / k) : 0.0f;
    }

    // 2. Compute Global Statistics
    float global_sum = 0.0f;
    for (size_t i = 0; i < n; ++i) global_sum += mean_dists_[i];
    float global_mean = global_sum / n;

    float variance_sum = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        float diff = mean_dists_[i] - global_mean;
        variance_sum += diff * diff;
    }
    float global_std = std::sqrt(variance_sum / n);

    // 3. Filter
    float thresh = global_mean + alpha * global_std;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        if (mean_dists_[i] <= thresh) {
            out.push_back(in.x[i], in.y[i], in.z[i]);
        }
    }
    return out.size();
}

std::size_t StatisticalOutlierRemoval::filter_rvv_aos(const PointCloudView& in, PointXYZ* out, int k, float alpha) {
    if (in.n == 0) return 0;
    PointCloud pc;
    filter_rvv(in, pc, k, alpha);
    for (size_t i = 0; i < pc.size(); ++i) {
        out[i] = {pc.x[i], pc.y[i], pc.z[i]};
    }
    return pc.size();
}

std::size_t StatisticalOutlierRemoval::filter_scalar_aos(const PointCloudView& in, PointXYZ* out, int k, float alpha) {
    if (in.n == 0) return 0;
    PointCloud pc;
    filter_scalar(in, pc, k, alpha);
    for (size_t i = 0; i < pc.size(); ++i) {
        out[i] = {pc.x[i], pc.y[i], pc.z[i]};
    }
    return pc.size();
}

} // namespace rvpoint
