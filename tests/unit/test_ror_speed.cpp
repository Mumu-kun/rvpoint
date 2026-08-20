#include "simple_pcd_loader.h"
#include "fast_3d_spatial_grid.h"
#include "rvv_pcl.h"
#include <iostream>
#include <vector>
#include <chrono>

using namespace rvv_pcl;

static void benchmark_ror_voxel_accelerated(const PointCloudSoA& cloud, float search_radius, int min_neighbors) {
    const size_t n = cloud.n;
    float r2 = search_radius * search_radius;
    Fast3DSpatialGrid grid(search_radius, n);
    grid.build(cloud.x, cloud.y, cloud.z, n);

    auto t1 = std::chrono::high_resolution_clock::now();

    std::vector<float> rx, ry, rz;
    rx.reserve(n); ry.reserve(n); rz.reserve(n);

    const float* px = cloud.x;
    const float* py = cloud.y;
    const float* pz = cloud.z;
    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

    // Array to track if a voxel is already validated
    // We can evaluate points directly with cell-first fastpath
    for (size_t i = 0; i < n; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        int in_radius_count = 0;

        // 1. Fastpath: check self cell first
        size_t self_h = grid.hash3D(qcx, qcy, qcz);
        int probe = 0;
        while (cells[self_h].head != -1 && probe < Fast3DSpatialGrid::kMaxProbes) {
            if (cells[self_h].cx == qcx && cells[self_h].cy == qcy && cells[self_h].cz == qcz) {
                int curr = cells[self_h].head;
                while (curr != -1) {
                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                        if (++in_radius_count >= min_neighbors) break;
                    }
                    curr = next[curr];
                }
                break;
            }
            self_h = (self_h + 1) & mask;
            probe++;
        }

        // If self cell already satisfies min_neighbors, skip all 26 other cells!
        if (in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && in_radius_count < min_neighbors; ++dz) {
                for (int dy = -1; dy <= 1 && in_radius_count < min_neighbors; ++dy) {
                    for (int dx = -1; dx <= 1 && in_radius_count < min_neighbors; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue; // Already checked self cell
                        int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
                        size_t h = grid.hash3D(tcx, tcy, tcz);
                        int p = 0;
                        while (cells[h].head != -1 && p < Fast3DSpatialGrid::kMaxProbes) {
                            if (cells[h].cx == tcx && cells[h].cy == tcy && cells[h].cz == tcz) {
                                int curr = cells[h].head;
                                while (curr != -1) {
                                    float ddx = px[curr] - qx, ddy = py[curr] - qy, ddz = pz[curr] - qz;
                                    if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                                        if (++in_radius_count >= min_neighbors) break;
                                    }
                                    curr = next[curr];
                                }
                                break;
                            }
                            h = (h + 1) & mask;
                            p++;
                        }
                    }
                }
            }
        }

        if (in_radius_count >= min_neighbors) {
            rx.push_back(qx);
            ry.push_back(qy);
            rz.push_back(qz);
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t2 - t1).count();
    std::cout << "Self-Cell Fastpath True ROR on Frame 90: N=" << n << " -> " << rx.size() << " in " << ms << " ms" << std::endl;
}

int main() {
    std::vector<PointXYZ> pts;
    if (loadPCD("data/pcd_compressed/0000000090.pcd", pts) < 0) return 1;

    std::vector<float> x(pts.size()), y(pts.size()), z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { x[i] = pts[i].x; y[i] = pts[i].y; z[i] = pts[i].z; }
    PointCloudSoA raw_cloud{x.data(), y.data(), z.data(), pts.size()};

    std::vector<PointXYZ> ds(pts.size());
    size_t nd = voxel_grid_downsamp_rvv_v2(raw_cloud, ds.data(), 0.10f);
    ds.resize(nd);

    std::vector<float> dx(nd), dy(nd), dz(nd);
    for (size_t i = 0; i < nd; ++i) { dx[i] = ds[i].x; dy[i] = ds[i].y; dz[i] = ds[i].z; }
    PointCloudSoA dcloud{dx.data(), dy.data(), dz.data(), nd};

    benchmark_ror_voxel_accelerated(dcloud, 0.25f, 2);
    return 0;
}
