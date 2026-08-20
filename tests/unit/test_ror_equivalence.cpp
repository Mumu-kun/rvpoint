#include "simple_pcd_loader.h"
#include "fast_3d_spatial_grid.h"
#include "rvv_pcl.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

using namespace rvv_pcl;

int main() {
    std::vector<PointXYZ> pts;
    if (loadPCD("data/pcd_compressed/0000000090.pcd", pts) < 0) return 1;

    std::vector<float> x(pts.size()), y(pts.size()), z(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) { x[i] = pts[i].x; y[i] = pts[i].y; z[i] = pts[i].z; }
    PointCloudSoA raw_cloud{x.data(), y.data(), z.data(), pts.size()};

    std::vector<PointXYZ> ds(pts.size());
    size_t nd = voxel_grid_downsamp_rvv_v2(raw_cloud, ds.data(), 0.10f);
    ds.resize(nd);

    // Test on 3000 points
    nd = std::min(nd, static_cast<size_t>(3000));

    std::vector<float> dx(nd), dy(nd), dz(nd);
    for (size_t i = 0; i < nd; ++i) { dx[i] = ds[i].x; dy[i] = ds[i].y; dz[i] = ds[i].z; }
    PointCloudSoA dcloud{dx.data(), dy.data(), dz.data(), nd};

    Fast3DSpatialGrid grid(0.25f, nd);
    grid.build(dcloud.x, dcloud.y, dcloud.z, nd);

    const float r2 = 0.25f * 0.25f;
    const int min_neighbors = 2;

    // 1. Reference brute-force True ROR
    std::vector<int> ref_valid;
    for (size_t i = 0; i < nd; ++i) {
        float qx = dx[i], qy = dy[i], qz = dz[i];
        int count = 0;
        for (size_t j = 0; j < nd; ++j) {
            float ddx = dx[j] - qx, ddy = dy[j] - qy, ddz = dz[j] - qz;
            if (ddx * ddx + ddy * ddy + ddz * ddz <= r2) {
                if (++count >= min_neighbors) break;
            }
        }
        if (count >= min_neighbors) ref_valid.push_back(static_cast<int>(i));
    }

    // 2. Fastpath True ROR
    std::vector<int> fast_valid;
    const float* px = dcloud.x;
    const float* py = dcloud.y;
    const float* pz = dcloud.z;
    const auto& cells = grid.cells_;
    const auto& next = grid.next_;
    const size_t mask = grid.mask_;
    const float inv_cell = grid.inv_cell_;

    for (size_t i = 0; i < nd; ++i) {
        float qx = px[i], qy = py[i], qz = pz[i];
        int qcx = static_cast<int>(std::floor(qx * inv_cell));
        int qcy = static_cast<int>(std::floor(qy * inv_cell));
        int qcz = static_cast<int>(std::floor(qz * inv_cell));

        int in_radius_count = 0;
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

        if (in_radius_count < min_neighbors) {
            for (int dz = -1; dz <= 1 && in_radius_count < min_neighbors; ++dz) {
                for (int dy = -1; dy <= 1 && in_radius_count < min_neighbors; ++dy) {
                    for (int dx = -1; dx <= 1 && in_radius_count < min_neighbors; ++dx) {
                        if (dx == 0 && dy == 0 && dz == 0) continue;
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

        if (in_radius_count >= min_neighbors) fast_valid.push_back(static_cast<int>(i));
    }

    std::cout << "Reference True ROR Valid Count: " << ref_valid.size() << std::endl;
    std::cout << "Fastpath  True ROR Valid Count: " << fast_valid.size() << std::endl;

    if (ref_valid == fast_valid) {
        std::cout << "SUCCESS: Fastpath is 100% IDENTICAL to Ground Truth True Euclidean ROR!" << std::endl;
        return 0;
    } else {
        std::cerr << "MISMATCH DETECTED!" << std::endl;
        return 1;
    }
}
