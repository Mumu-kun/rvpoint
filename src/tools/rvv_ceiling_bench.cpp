// rvv_ceiling_bench.cpp
// Hypothesis-Testing Benchmark for RVV Single-Core Optimization Ceiling
//
// Tests 4 hypotheses:
//   H1: SOR bottleneck is linked-list grid traversal, not sqrt+sum
//   H2: Flat-array grid enables RVV-vectorized neighbor distance computation
//   H3: Voxel-based Radius Outlier Removal can replace SOR at 10-20x speedup
//   H4: Vectorized extract_inliers_outliers is faster than scalar loop

#include "simple_pcd_loader.h"
#include "rvv_pcl.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#if defined(__riscv) || defined(__riscv_vector)
#include <riscv_vector.h>
#endif

using Clock = std::chrono::high_resolution_clock;
using namespace rvv_pcl;

// ============================================================================
// EXISTING: Linked-List Spatial Grid (from pipeline_3d_ultimate)
// ============================================================================
class LinkedListGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct Cell { int cx = -999999, cy = -999999, cz = -999999; int head = -1; };
    float cell_size_, inv_cell_;
    std::vector<Cell> cells_;
    std::vector<int> next_;
    const float *px_ = nullptr, *py_ = nullptr, *pz_ = nullptr;
    size_t n_pts_ = 0;

    LinkedListGrid(float cs) : cell_size_(cs), inv_cell_(1.0f / cs) { cells_.resize(kCapacity); }
    static inline size_t hash3D(int x, int y, int z) {
        return ((size_t)x * 73856093 ^ (size_t)y * 19349663 ^ (size_t)z * 83492791) & kMask;
    }
    void build(const float* x, const float* y, const float* z, size_t n) {
        px_ = x; py_ = y; pz_ = z; n_pts_ = n;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i] = Cell();
        next_.assign(n, -1);
        for (size_t i = 0; i < n; ++i) {
            int cx = (int)std::floor(x[i] * inv_cell_);
            int cy = (int)std::floor(y[i] * inv_cell_);
            int cz = (int)std::floor(z[i] * inv_cell_);
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].head != -1 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz))
                h = (h + 1) & kMask;
            if (cells_[h].head == -1) { cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz; }
            next_[i] = cells_[h].head;
            cells_[h].head = (int)i;
        }
    }
    void radiusSearchDists(float qx, float qy, float qz, float r2, std::vector<float>& dists2) const {
        dists2.clear();
        int qcx = (int)std::floor(qx * inv_cell_), qcy = (int)std::floor(qy * inv_cell_), qcz = (int)std::floor(qz * inv_cell_);
        for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
            size_t h = hash3D(tcx, tcy, tcz);
            while (cells_[h].head != -1) {
                if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                    int curr = cells_[h].head;
                    while (curr != -1) {
                        float ddx = px_[curr] - qx, ddy = py_[curr] - qy, ddz = pz_[curr] - qz;
                        float d2 = ddx*ddx + ddy*ddy + ddz*ddz;
                        if (d2 <= r2) dists2.push_back(d2);
                        curr = next_[curr];
                    }
                    break;
                }
                h = (h + 1) & kMask;
            }
        }
    }
};

// ============================================================================
// NEW: Flat-Array Spatial Grid (contiguous per-cell storage)
// ============================================================================
class FlatArrayGrid {
public:
    static constexpr size_t kCapacity = 65536;
    static constexpr size_t kMask = kCapacity - 1;

    struct CellMeta { int cx = -999999, cy = -999999, cz = -999999; int offset = 0; int count = 0; };
    float cell_size_, inv_cell_;
    std::vector<CellMeta> cells_;
    // Contiguous point storage sorted by cell
    std::vector<float> sorted_x_, sorted_y_, sorted_z_;
    size_t n_pts_ = 0;

    FlatArrayGrid(float cs) : cell_size_(cs), inv_cell_(1.0f / cs) { cells_.resize(kCapacity); }

    static inline size_t hash3D(int x, int y, int z) {
        return ((size_t)x * 73856093 ^ (size_t)y * 19349663 ^ (size_t)z * 83492791) & kMask;
    }

    void build(const float* x, const float* y, const float* z, size_t n) {
        n_pts_ = n;
        for (size_t i = 0; i < kCapacity; ++i) cells_[i] = CellMeta();

        // Pass 1: Count points per cell
        std::vector<size_t> point_cell(n);
        for (size_t i = 0; i < n; ++i) {
            int cx = (int)std::floor(x[i] * inv_cell_);
            int cy = (int)std::floor(y[i] * inv_cell_);
            int cz = (int)std::floor(z[i] * inv_cell_);
            size_t h = hash3D(cx, cy, cz);
            while (cells_[h].count > 0 && (cells_[h].cx != cx || cells_[h].cy != cy || cells_[h].cz != cz))
                h = (h + 1) & kMask;
            if (cells_[h].count == 0) { cells_[h].cx = cx; cells_[h].cy = cy; cells_[h].cz = cz; }
            cells_[h].count++;
            point_cell[i] = h;
        }

        // Pass 2: Compute offsets (prefix sum)
        int running = 0;
        for (size_t i = 0; i < kCapacity; ++i) {
            if (cells_[i].count > 0) {
                cells_[i].offset = running;
                running += cells_[i].count;
                cells_[i].count = 0;  // Reset for Pass 3
            }
        }

        // Pass 3: Scatter points into contiguous arrays
        sorted_x_.resize(n);
        sorted_y_.resize(n);
        sorted_z_.resize(n);
        for (size_t i = 0; i < n; ++i) {
            size_t h = point_cell[i];
            int pos = cells_[h].offset + cells_[h].count;
            sorted_x_[pos] = x[i];
            sorted_y_[pos] = y[i];
            sorted_z_[pos] = z[i];
            cells_[h].count++;
        }
    }

    void radiusSearchDists(float qx, float qy, float qz, float r2, std::vector<float>& dists2) const {
        dists2.clear();
        int qcx = (int)std::floor(qx * inv_cell_), qcy = (int)std::floor(qy * inv_cell_), qcz = (int)std::floor(qz * inv_cell_);
        for (int dz = -1; dz <= 1; ++dz) for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            int tcx = qcx + dx, tcy = qcy + dy, tcz = qcz + dz;
            size_t h = hash3D(tcx, tcy, tcz);
            while (cells_[h].count > 0) {
                if (cells_[h].cx == tcx && cells_[h].cy == tcy && cells_[h].cz == tcz) {
                    int off = cells_[h].offset;
                    int cnt = cells_[h].count;
#if defined(__riscv) || defined(__riscv_vector)
                    // RVV vectorized distance computation on contiguous arrays!
                    int k = 0;
                    while (k < cnt) {
                        size_t vl = __riscv_vsetvl_e32m8(cnt - k);
                        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&sorted_x_[off + k], vl);
                        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&sorted_y_[off + k], vl);
                        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&sorted_z_[off + k], vl);
                        vfloat32m8_t ddx = __riscv_vfsub_vf_f32m8(vx, qx, vl);
                        vfloat32m8_t ddy = __riscv_vfsub_vf_f32m8(vy, qy, vl);
                        vfloat32m8_t ddz = __riscv_vfsub_vf_f32m8(vz, qz, vl);
                        vfloat32m8_t d2 = __riscv_vfmul_vv_f32m8(ddx, ddx, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddy, ddy, vl);
                        d2 = __riscv_vfmacc_vv_f32m8(d2, ddz, ddz, vl);
                        // Filter: d2 <= r2
                        vbool4_t mask = __riscv_vmfle_vf_f32m8_b4(d2, r2, vl);
                        // Store mask bits, extract matching distances
                        uint8_t mask_bytes[64];
                        __riscv_vsm_v_b4(mask_bytes, mask, vl);
                        // Store all distances temporarily
                        float tmp_d2[256];
                        __riscv_vse32_v_f32m8(tmp_d2, d2, vl);
                        for (size_t lane = 0; lane < vl; ++lane) {
                            if ((mask_bytes[lane >> 3] >> (lane & 7u)) & 1u) {
                                dists2.push_back(tmp_d2[lane]);
                            }
                        }
                        k += vl;
                    }
#else
                    for (int k = 0; k < cnt; ++k) {
                        float ddx = sorted_x_[off+k] - qx, ddy = sorted_y_[off+k] - qy, ddz = sorted_z_[off+k] - qz;
                        float d2 = ddx*ddx + ddy*ddy + ddz*ddz;
                        if (d2 <= r2) dists2.push_back(d2);
                    }
#endif
                    break;
                }
                h = (h + 1) & kMask;
            }
        }
    }
};

// ============================================================================
// H3: Voxel-Based Radius Outlier Removal (ROR)
// ============================================================================
struct RorResult {
    std::vector<float> x, y, z;
};

static RorResult execute_voxel_ror(
    const float* px, const float* py, const float* pz, size_t n,
    float cell_size, int min_neighbors)
{
    // Phase 1: Compute voxel keys (same as voxel grid downsamp)
    float inv_cell = 1.0f / cell_size;

    // Compute bounding box
    float min_x = px[0], min_y = py[0], min_z = pz[0];
    float max_x = px[0], max_y = py[0], max_z = pz[0];
#if defined(__riscv) || defined(__riscv_vector)
    {
        size_t vl_init = __riscv_vsetvl_e32m4(n);
        vfloat32m4_t vmin_x = __riscv_vfmv_v_f_f32m4(min_x, vl_init);
        vfloat32m4_t vmin_y = __riscv_vfmv_v_f_f32m4(min_y, vl_init);
        vfloat32m4_t vmin_z = __riscv_vfmv_v_f_f32m4(min_z, vl_init);
        vfloat32m4_t vmax_x = __riscv_vfmv_v_f_f32m4(max_x, vl_init);
        vfloat32m4_t vmax_y = __riscv_vfmv_v_f_f32m4(max_y, vl_init);
        vfloat32m4_t vmax_z = __riscv_vfmv_v_f_f32m4(max_z, vl_init);
        size_t i = 0;
        while (i < n) {
            size_t vl = __riscv_vsetvl_e32m4(n - i);
            vfloat32m4_t vx = __riscv_vle32_v_f32m4(&px[i], vl);
            vfloat32m4_t vy = __riscv_vle32_v_f32m4(&py[i], vl);
            vfloat32m4_t vz = __riscv_vle32_v_f32m4(&pz[i], vl);
            vmin_x = __riscv_vfmin_vv_f32m4(vmin_x, vx, vl);
            vmin_y = __riscv_vfmin_vv_f32m4(vmin_y, vy, vl);
            vmin_z = __riscv_vfmin_vv_f32m4(vmin_z, vz, vl);
            vmax_x = __riscv_vfmax_vv_f32m4(vmax_x, vx, vl);
            vmax_y = __riscv_vfmax_vv_f32m4(vmax_y, vy, vl);
            vmax_z = __riscv_vfmax_vv_f32m4(vmax_z, vz, vl);
            i += vl;
        }
        size_t vl_red = __riscv_vsetvl_e32m4(n);
        vfloat32m1_t seed_min = __riscv_vfmv_v_f_f32m1(1e30f, 1);
        vfloat32m1_t seed_max = __riscv_vfmv_v_f_f32m1(-1e30f, 1);
        __riscv_vse32_v_f32m1(&min_x, __riscv_vfredmin_vs_f32m4_f32m1(vmin_x, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&min_y, __riscv_vfredmin_vs_f32m4_f32m1(vmin_y, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&min_z, __riscv_vfredmin_vs_f32m4_f32m1(vmin_z, seed_min, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_x, __riscv_vfredmax_vs_f32m4_f32m1(vmax_x, seed_max, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_y, __riscv_vfredmax_vs_f32m4_f32m1(vmax_y, seed_max, vl_red), 1);
        __riscv_vse32_v_f32m1(&max_z, __riscv_vfredmax_vs_f32m4_f32m1(vmax_z, seed_max, vl_red), 1);
    }
#else
    for (size_t i = 1; i < n; ++i) {
        if (px[i] < min_x) min_x = px[i]; if (px[i] > max_x) max_x = px[i];
        if (py[i] < min_y) min_y = py[i]; if (py[i] > max_y) max_y = py[i];
        if (pz[i] < min_z) min_z = pz[i]; if (pz[i] > max_z) max_z = pz[i];
    }
#endif

    int min_ix = (int)std::floor(min_x * inv_cell);
    int min_iy = (int)std::floor(min_y * inv_cell);
    int min_iz = (int)std::floor(min_z * inv_cell);
    int max_ix = (int)std::floor(max_x * inv_cell);
    int max_iy = (int)std::floor(max_y * inv_cell);
    int max_iz = (int)std::floor(max_z * inv_cell);
    int gx = max_ix - min_ix + 1;
    int gy = max_iy - min_iy + 1;
    int gz = max_iz - min_iz + 1;
    int grid_xy = gx * gy;

    // Phase 2: Compute voxel keys + count per voxel using hash map
    static constexpr size_t kHCap = 131072;
    static constexpr size_t kHMask = kHCap - 1;
    struct VoxelEntry { int32_t key = -1; int count = 0; };
    std::vector<VoxelEntry> htable(kHCap);

    // Compute keys for each point
    std::vector<int32_t> point_keys(n);
    for (size_t i = 0; i < n; ++i) {
        int ix = (int)std::floor(px[i] * inv_cell) - min_ix;
        int iy = (int)std::floor(py[i] * inv_cell) - min_iy;
        int iz = (int)std::floor(pz[i] * inv_cell) - min_iz;
        point_keys[i] = ix + iy * gx + iz * grid_xy;
    }

    // Count per voxel
    auto vhash = [](int32_t key) -> size_t { return ((size_t)key * 2654435761u) & kHMask; };
    for (size_t i = 0; i < n; ++i) {
        int32_t k = point_keys[i];
        size_t h = vhash(k);
        while (htable[h].key != -1 && htable[h].key != k) h = (h + 1) & kHMask;
        htable[h].key = k;
        htable[h].count++;
    }

    // Phase 3: For each voxel, count neighbors in 26-neighborhood
    // Build a lookup: key -> total neighbor count (sum of counts in 3x3x3 neighborhood)
    // For efficiency, compute this for occupied voxels only
    auto get_voxel_count = [&](int32_t key) -> int {
        size_t h = vhash(key);
        while (htable[h].key != -1) {
            if (htable[h].key == key) return htable[h].count;
            h = (h + 1) & kHMask;
        }
        return 0;
    };

    // Phase 4: Filter points - keep if 3x3x3 neighborhood has >= min_neighbors
    RorResult res;
    res.x.reserve(n);
    res.y.reserve(n);
    res.z.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        int32_t k = point_keys[i];
        // Decompose key back to ix,iy,iz
        int iz = k / grid_xy;
        int rem = k % grid_xy;
        int iy = rem / gx;
        int ix = rem % gx;

        int total = 0;
        for (int dz2 = -1; dz2 <= 1; ++dz2) {
            int niz = iz + dz2;
            if (niz < 0 || niz >= gz) continue;
            for (int dy2 = -1; dy2 <= 1; ++dy2) {
                int niy = iy + dy2;
                if (niy < 0 || niy >= gy) continue;
                for (int dx2 = -1; dx2 <= 1; ++dx2) {
                    int nix = ix + dx2;
                    if (nix < 0 || nix >= gx) continue;
                    int32_t nk = nix + niy * gx + niz * grid_xy;
                    total += get_voxel_count(nk);
                }
            }
        }

        if (total >= min_neighbors) {
            res.x.push_back(px[i]);
            res.y.push_back(py[i]);
            res.z.push_back(pz[i]);
        }
    }
    return res;
}

// ============================================================================
// H4: Vectorized vs Scalar extract_inliers_outliers
// ============================================================================
static void extract_scalar(
    const float* x, const float* y, const float* z, size_t n,
    const float* model, float thresh,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    ox.clear(); oy.clear(); oz.clear();
    ox.reserve(n); oy.reserve(n); oz.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        float dist = std::abs(a * x[i] + b * y[i] + c * z[i] + d);
        if (dist > thresh) {
            ox.push_back(x[i]);
            oy.push_back(y[i]);
            oz.push_back(z[i]);
        }
    }
}

#if defined(__riscv) || defined(__riscv_vector)
static void extract_rvv(
    const float* x, const float* y, const float* z, size_t n,
    const float* model, float thresh,
    std::vector<float>& ox, std::vector<float>& oy, std::vector<float>& oz)
{
    float a = model[0], b = model[1], c = model[2], d = model[3];
    ox.clear(); oy.clear(); oz.clear();
    ox.resize(n); oy.resize(n); oz.resize(n);
    size_t count = 0;
    size_t i = 0;
    while (i < n) {
        size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(&x[i], vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(&y[i], vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(&z[i], vl);

        // dist = a*x + b*y + c*z + d
        vfloat32m8_t dist = __riscv_vfmul_vf_f32m8(vx, a, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, b, vy, vl);
        dist = __riscv_vfmacc_vf_f32m8(dist, c, vz, vl);
        dist = __riscv_vfadd_vf_f32m8(dist, d, vl);

        // outlier: dist > thresh OR dist < -thresh (i.e., NOT inlier)
        vbool4_t mask_le = __riscv_vmfle_vf_f32m8_b4(dist, thresh, vl);
        vbool4_t mask_ge = __riscv_vmfge_vf_f32m8_b4(dist, -thresh, vl);
        vbool4_t inlier_mask = __riscv_vmand_mm_b4(mask_le, mask_ge, vl);
        vbool4_t outlier_mask = __riscv_vmnot_m_b4(inlier_mask, vl);

        long cnt = __riscv_vcpop_m_b4(outlier_mask, vl);
        if (cnt > 0) {
            // Compress and store
            vfloat32m8_t cx = __riscv_vcompress_vm_f32m8(vx, outlier_mask, vl);
            vfloat32m8_t cy = __riscv_vcompress_vm_f32m8(vy, outlier_mask, vl);
            vfloat32m8_t cz = __riscv_vcompress_vm_f32m8(vz, outlier_mask, vl);
            __riscv_vse32_v_f32m8(&ox[count], cx, cnt);
            __riscv_vse32_v_f32m8(&oy[count], cy, cnt);
            __riscv_vse32_v_f32m8(&oz[count], cz, cnt);
            count += cnt;
        }
        i += vl;
    }
    ox.resize(count);
    oy.resize(count);
    oz.resize(count);
}
#endif

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input.pcd> [--leaf-size <val>]" << std::endl;
        return 1;
    }

    float leaf_size = 0.10f;
    std::string input_file = argv[1];
    for (int i = 2; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--leaf-size" && i + 1 < argc) leaf_size = std::stof(argv[++i]);
    }

    // Load and downsample
    std::vector<PointXYZ> loaded;
    int cnt = loadPCD(input_file, loaded);
    if (cnt < 0) { std::cerr << "Failed to load " << input_file << std::endl; return 1; }
    std::cout << "Loaded " << loaded.size() << " points" << std::endl;

    size_t n_in = loaded.size();
    std::vector<float> ix(n_in), iy(n_in), iz(n_in);
    for (size_t i = 0; i < n_in; ++i) { ix[i] = loaded[i].x; iy[i] = loaded[i].y; iz[i] = loaded[i].z; }
    PointCloudSoA in_cloud{ix.data(), iy.data(), iz.data(), n_in};

    std::vector<PointXYZ> down_pts(n_in);
    size_t n_down = voxel_grid_downsamp_rvv_v2(in_cloud, down_pts.data(), leaf_size);
    down_pts.resize(n_down);
    std::cout << "Downsampled to " << n_down << " points" << std::endl;

    std::vector<float> dx(n_down), dy(n_down), dz(n_down);
    for (size_t i = 0; i < n_down; ++i) { dx[i] = down_pts[i].x; dy[i] = down_pts[i].y; dz[i] = down_pts[i].z; }

    const float sor_radius = 0.25f;
    const float sor_r2 = sor_radius * sor_radius;
    const int sor_k = 20;

    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "HYPOTHESIS TESTING: RVV Single-Core Optimization Ceiling" << std::endl;
    std::cout << std::string(70, '=') << "\n" << std::endl;

    // ========================================================================
    // H1: SOR time breakdown — grid traversal vs sqrt+sum
    // ========================================================================
    std::cout << "--- H1: SOR Bottleneck Breakdown ---" << std::endl;
    {
        LinkedListGrid grid(sor_radius);
        auto t0 = Clock::now();
        grid.build(dx.data(), dy.data(), dz.data(), n_down);
        double build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        // Measure grid traversal only (no sqrt+sum)
        std::vector<float> d2;
        d2.reserve(256);
        t0 = Clock::now();
        for (size_t i = 0; i < n_down; ++i) {
            grid.radiusSearchDists(dx[i], dy[i], dz[i], sor_r2, d2);
        }
        double traverse_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        // Measure sqrt+sum only (reuse cached results)
        t0 = Clock::now();
        float dummy = 0;
        for (size_t i = 0; i < n_down; ++i) {
            grid.radiusSearchDists(dx[i], dy[i], dz[i], sor_r2, d2);
            int k_use = std::min((int)d2.size() - 1, sor_k);
            if (k_use < 1) continue;
            float sum = 0;
#if defined(__riscv) || defined(__riscv_vector)
            int rem = k_use, offset = 1;
            while (rem > 0) {
                size_t vl = __riscv_vsetvl_e32m8(rem);
                vfloat32m8_t vd2 = __riscv_vle32_v_f32m8(d2.data() + offset, vl);
                vfloat32m8_t vd = __riscv_vfsqrt_v_f32m8(vd2, vl);
                vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
                vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m8_f32m1(vd, zero, vl);
                sum += __riscv_vfmv_f_s_f32m1_f32(v_sum);
                offset += vl;
                rem -= vl;
            }
#else
            for (int j = 1; j <= k_use; ++j) sum += std::sqrt(d2[j]);
#endif
            dummy += sum / k_use;
        }
        double full_sor_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
        double sqrt_sum_only_ms = full_sor_ms - traverse_ms;

        std::cout << "  Grid build:       " << std::fixed << std::setprecision(3) << build_ms << " ms" << std::endl;
        std::cout << "  Grid traversal:   " << traverse_ms << " ms (" << std::setprecision(1) << (traverse_ms / full_sor_ms * 100) << "% of SOR)" << std::endl;
        std::cout << "  sqrt+sum (RVV):   " << std::setprecision(3) << sqrt_sum_only_ms << " ms (" << std::setprecision(1) << (sqrt_sum_only_ms / full_sor_ms * 100) << "% of SOR)" << std::endl;
        std::cout << "  Full SOR total:   " << std::setprecision(3) << full_sor_ms << " ms" << std::endl;
        std::cout << "  (dummy=" << dummy << ")" << std::endl;
        std::cout << std::endl;
    }

    // ========================================================================
    // H2: Flat-array grid vs linked-list grid
    // ========================================================================
    std::cout << "--- H2: Flat-Array Grid vs Linked-List Grid ---" << std::endl;
    {
        // Linked-list grid
        LinkedListGrid ll_grid(sor_radius);
        ll_grid.build(dx.data(), dy.data(), dz.data(), n_down);

        std::vector<float> d2;
        d2.reserve(256);
        auto t0 = Clock::now();
        int ll_total = 0;
        for (size_t i = 0; i < n_down; ++i) {
            ll_grid.radiusSearchDists(dx[i], dy[i], dz[i], sor_r2, d2);
            ll_total += d2.size();
        }
        double ll_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        // Flat-array grid
        FlatArrayGrid fa_grid(sor_radius);
        auto t_build = Clock::now();
        fa_grid.build(dx.data(), dy.data(), dz.data(), n_down);
        double fa_build_ms = std::chrono::duration<double, std::milli>(Clock::now() - t_build).count();

        t0 = Clock::now();
        int fa_total = 0;
        for (size_t i = 0; i < n_down; ++i) {
            fa_grid.radiusSearchDists(dx[i], dy[i], dz[i], sor_r2, d2);
            fa_total += d2.size();
        }
        double fa_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        std::cout << "  Linked-list grid search:  " << std::fixed << std::setprecision(3) << ll_ms << " ms (total neighbors: " << ll_total << ")" << std::endl;
        std::cout << "  Flat-array grid build:    " << fa_build_ms << " ms" << std::endl;
        std::cout << "  Flat-array grid search:   " << fa_ms << " ms (total neighbors: " << fa_total << ")" << std::endl;
        std::cout << "  Speedup:                  " << std::setprecision(2) << (ll_ms / fa_ms) << "x" << std::endl;
        std::cout << std::endl;
    }

    // ========================================================================
    // H3: Voxel-based ROR vs full SOR
    // ========================================================================
    std::cout << "--- H3: Voxel-Based ROR vs Full SOR ---" << std::endl;
    {
        // Full SOR (existing implementation from pipeline_3d_ultimate)
        LinkedListGrid grid(sor_radius);
        grid.build(dx.data(), dy.data(), dz.data(), n_down);

        std::vector<float> d2;
        d2.reserve(256);
        std::vector<float> mean_dists(n_down, sor_radius);
        std::vector<int> valid_points;
        valid_points.reserve(n_down);
        double total_sum = 0, total_sq_sum = 0;

        auto t0 = Clock::now();
        for (size_t i = 0; i < n_down; ++i) {
            grid.radiusSearchDists(dx[i], dy[i], dz[i], sor_r2, d2);
            int found = d2.size();
            if (found < 2) continue;
            int k_use = std::min(found - 1, sor_k);
            float sum = 0;
#if defined(__riscv) || defined(__riscv_vector)
            int rem = k_use, offset = 1;
            while (rem > 0) {
                size_t vl = __riscv_vsetvl_e32m8(rem);
                vfloat32m8_t vd2 = __riscv_vle32_v_f32m8(d2.data() + offset, vl);
                vfloat32m8_t vd = __riscv_vfsqrt_v_f32m8(vd2, vl);
                vfloat32m1_t zero = __riscv_vfmv_v_f_f32m1(0.0f, 1);
                vfloat32m1_t v_sum = __riscv_vfredusum_vs_f32m8_f32m1(vd, zero, vl);
                sum += __riscv_vfmv_f_s_f32m1_f32(v_sum);
                offset += vl;
                rem -= vl;
            }
#else
            for (int j = 1; j <= k_use; ++j) sum += std::sqrt(d2[j]);
#endif
            float m = sum / k_use;
            mean_dists[i] = m;
            total_sum += m;
            total_sq_sum += m * m;
            valid_points.push_back(i);
        }
        double dc = valid_points.size();
        double gm = total_sum / dc;
        double var = (total_sq_sum / dc) - (gm * gm);
        float thresh = (float)(gm + 1.0 * std::sqrt(std::max(0.0, var)));
        size_t sor_kept = 0;
        for (int idx : valid_points) if (mean_dists[idx] <= thresh) sor_kept++;
        double sor_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        // Voxel-based ROR
        t0 = Clock::now();
        auto ror_res = execute_voxel_ror(dx.data(), dy.data(), dz.data(), n_down, sor_radius, 3);
        double ror_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();

        std::cout << "  Full SOR:          " << std::fixed << std::setprecision(3) << sor_ms << " ms, kept " << sor_kept << "/" << n_down << " points" << std::endl;
        std::cout << "  Voxel ROR:         " << ror_ms << " ms, kept " << ror_res.x.size() << "/" << n_down << " points" << std::endl;
        std::cout << "  Speedup:           " << std::setprecision(2) << (sor_ms / ror_ms) << "x" << std::endl;
        std::cout << "  Point difference:  " << (int)ror_res.x.size() - (int)sor_kept << " (ROR keeps " 
                  << std::setprecision(1) << ((float)ror_res.x.size() / n_down * 100) << "% vs SOR " 
                  << ((float)sor_kept / n_down * 100) << "%)" << std::endl;
        std::cout << std::endl;
    }

    // ========================================================================
    // H4: Vectorized vs Scalar extract_inliers_outliers
    // ========================================================================
    std::cout << "--- H4: Vectorized vs Scalar Extract ---" << std::endl;
    {
        // First get a plane model via RANSAC
        PointCloudSoA dc{dx.data(), dy.data(), dz.data(), n_down};
        float model[4] = {0, 0, 1, 0};  // dummy Z-up plane
        ransac_plane_rvv(dc, 0.20f, 250, model);

        std::vector<float> ox, oy, oz;

        // Scalar extract
        auto t0 = Clock::now();
        for (int rep = 0; rep < 5; ++rep)
            extract_scalar(dx.data(), dy.data(), dz.data(), n_down, model, 0.20f, ox, oy, oz);
        double scalar_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / 5.0;
        size_t scalar_count = ox.size();

#if defined(__riscv) || defined(__riscv_vector)
        // RVV extract
        t0 = Clock::now();
        for (int rep = 0; rep < 5; ++rep)
            extract_rvv(dx.data(), dy.data(), dz.data(), n_down, model, 0.20f, ox, oy, oz);
        double rvv_ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count() / 5.0;
        size_t rvv_count = ox.size();

        std::cout << "  Scalar extract:  " << std::fixed << std::setprecision(3) << scalar_ms << " ms, " << scalar_count << " outliers" << std::endl;
        std::cout << "  RVV extract:     " << rvv_ms << " ms, " << rvv_count << " outliers" << std::endl;
        std::cout << "  Speedup:         " << std::setprecision(2) << (scalar_ms / rvv_ms) << "x" << std::endl;
#else
        std::cout << "  Scalar extract:  " << std::fixed << std::setprecision(3) << scalar_ms << " ms, " << scalar_count << " outliers" << std::endl;
        std::cout << "  (RVV not available for comparison)" << std::endl;
#endif
        std::cout << std::endl;
    }

    std::cout << std::string(70, '=') << std::endl;
    std::cout << "HYPOTHESIS TESTING COMPLETE" << std::endl;
    std::cout << std::string(70, '=') << std::endl;

    return 0;
}
