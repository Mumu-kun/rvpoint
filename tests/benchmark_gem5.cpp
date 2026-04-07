// benchmark_gem5.cpp
// Single-algorithm, single-pass runner designed for gem5 cycle-accurate simulation.
//
// Usage: benchmark_gem5 <algorithm> <N>
//
// Algorithms:
//   voxel_sc      voxel_rvv
//   ransac_sc     ransac_rvv
//   sor_sc        sor_rvv
//   normal_sc     normal_rvv    (normal_rvv includes octree build)
//   pipeline_sc   pipeline_rvv  (full pipeline, single pass)
//
// Design rules for gem5:
//  - Statically linked (elf toolchain) so gem5 SE mode doesn't need sysroot.
//  - Data generation happens before the algorithm call — gem5 measures
//    total cycles but the algo dominates for reasonable N.
//  - One pass only: gem5 is slow (~1-10 MIPS simulated vs 1 BIPS real hw).
//  - No rdinstret — gem5 extracts cycle counts from its own stats.txt.
//  - Prints a single result line to stdout so gem5_bench.sh can verify exit.

#include "../src/include/rvv_pcl.h"
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <random>
#include <string>
#include <vector>

using namespace rvv_pcl;

// ─── Synthetic data generation ────────────────────────────────────────────────
struct CloudData {
    std::vector<float>    x, y, z;
    std::vector<PointXYZ> aos;
    PointCloudSoA         soa;
};

static CloudData make_cloud(size_t N, bool with_ground = true) {
    CloudData d;
    d.x.resize(N); d.y.resize(N); d.z.resize(N); d.aos.resize(N);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> rxy(0.f, 100.f);
    std::uniform_real_distribution<float> rz (1.f, 100.f);

    size_t n_obj = with_ground ? N * 8 / 10 : N;
    for (size_t i = 0; i < n_obj; ++i) {
        d.x[i] = rxy(rng); d.y[i] = rxy(rng); d.z[i] = rz(rng);
    }
    for (size_t i = n_obj; i < N; ++i) {
        d.x[i] = rxy(rng); d.y[i] = rxy(rng); d.z[i] = 0.f;
    }
    for (size_t i = 0; i < N; ++i)
        d.aos[i] = {d.x[i], d.y[i], d.z[i]};

    d.soa = {d.x.data(), d.y.data(), d.z.data(), N};
    return d;
}

// ─── Algorithm runners ────────────────────────────────────────────────────────

static size_t run_voxel_sc(CloudData& c) {
    std::vector<PointXYZ> out(c.x.size());
    return voxel_grid_downsamp_sc(c.aos.data(), c.x.size(), out.data(), 1.f);
}
static size_t run_voxel_rvv(CloudData& c) {
    std::vector<PointXYZ> out(c.x.size());
    return voxel_grid_downsamp_rvv_v2(c.soa, out.data(), 1.f);
}

static size_t run_sor_sc(CloudData& c) {
    std::vector<PointXYZ> out(c.x.size());
    return sor_sc(c.aos.data(), c.x.size(), out.data(), 10, 1.f);
}
static size_t run_sor_rvv(CloudData& c) {
    std::vector<PointXYZ> out(c.x.size());
    return sor_rvv(c.soa, out.data(), 10, 1.f);
}

static int run_ransac_sc(CloudData& c) {
    float model[4];
    return ransac_plane_sc(c.aos.data(), c.x.size(), 0.5f, 200, model);
}
static int run_ransac_rvv(CloudData& c) {
    float model[4];
    return ransac_plane_rvv(c.soa, 0.5f, 200, model);
}

static void run_normal_sc(CloudData& c) {
    size_t N = c.x.size();
    std::vector<float> nx(N), ny(N), nz(N);
    normal_estimation_sc(c.aos.data(), N, nx.data(), ny.data(), nz.data(), 10, 2.f);
}
static void run_normal_rvv(CloudData& c) {
    // Self-contained: builds Octree internally (cost included)
    size_t N = c.x.size();
    std::vector<float> nx(N), ny(N), nz(N);
    normal_estimation_rvv(c.soa, nx.data(), ny.data(), nz.data(), 10, 2.f);
}

// Full scalar pipeline
static void run_pipeline_sc(CloudData& c) {
    size_t N = c.x.size();

    std::vector<PointXYZ> vox(N), obj(N), sor_buf(N);
    std::vector<float> nx(N), ny(N), nz(N);
    float model[4];

    size_t n_vox = voxel_grid_downsamp_sc(c.aos.data(), N, vox.data(), 1.f);
    ransac_plane_sc(vox.data(), n_vox, 0.5f, 200, model);

    size_t n_obj = 0;
    for (size_t j = 0; j < n_vox; ++j) {
        float d = model[0]*vox[j].x + model[1]*vox[j].y + model[2]*vox[j].z + model[3];
        if (d < -0.5f || d > 0.5f) obj[n_obj++] = vox[j];
    }

    size_t n_sor = sor_sc(obj.data(), n_obj, sor_buf.data(), 10, 1.f);
    normal_estimation_sc(sor_buf.data(), n_sor, nx.data(), ny.data(), nz.data(), 10, 2.f);
}

// Full RVV pipeline
static void run_pipeline_rvv(CloudData& c) {
    size_t N = c.x.size();

    std::vector<PointXYZ> vox(N), obj(N), sor_buf(N);
    std::vector<float> vx(N), vy(N), vz(N);
    std::vector<float> ox(N), oy(N), oz(N);
    std::vector<float> sx(N), sy(N), sz(N);
    std::vector<float> nx(N), ny(N), nz(N);
    float model[4];

    size_t n_vox = voxel_grid_downsamp_rvv_v2(c.soa, vox.data(), 1.f);
    for (size_t j = 0; j < n_vox; ++j) { vx[j]=vox[j].x; vy[j]=vox[j].y; vz[j]=vox[j].z; }
    PointCloudSoA vox_soa = {vx.data(), vy.data(), vz.data(), n_vox};

    ransac_plane_rvv(vox_soa, 0.5f, 200, model);

    size_t n_obj = extract_plane_outliers_rvv(vox_soa, model, 0.5f, obj.data());
    for (size_t j = 0; j < n_obj; ++j) { ox[j]=obj[j].x; oy[j]=obj[j].y; oz[j]=obj[j].z; }
    PointCloudSoA obj_soa = {ox.data(), oy.data(), oz.data(), n_obj};

    size_t n_sor = sor_rvv(obj_soa, sor_buf.data(), 10, 1.f);
    for (size_t j = 0; j < n_sor; ++j) { sx[j]=sor_buf[j].x; sy[j]=sor_buf[j].y; sz[j]=sor_buf[j].z; }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

    // Octree build + normal estimation (both counted — real pipeline cost)
    normal_estimation_rvv(sor_soa, nx.data(), ny.data(), nz.data(), 10, 2.f);
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: benchmark_gem5 <algorithm> <N>\n";
        std::cerr << "  algorithms: voxel_sc voxel_rvv sor_sc sor_rvv ransac_sc ransac_rvv\n";
        std::cerr << "              normal_sc normal_rvv pipeline_sc pipeline_rvv\n";
        return 1;
    }

    std::string algo(argv[1]);
    size_t N = (size_t)std::atoi(argv[2]);

    if (N == 0 || N > 1000000) {
        std::cerr << "N must be between 1 and 1000000\n";
        return 1;
    }

    // Generate data BEFORE gem5 starts measuring (startup is fast, algo dominates)
    CloudData cloud = make_cloud(N, /*with_ground=*/true);

    size_t result = 0;

    if      (algo == "voxel_sc")    result = run_voxel_sc(cloud);
    else if (algo == "voxel_rvv")   result = run_voxel_rvv(cloud);
    else if (algo == "sor_sc")      result = run_sor_sc(cloud);
    else if (algo == "sor_rvv")     result = run_sor_rvv(cloud);
    else if (algo == "ransac_sc")   result = (size_t)run_ransac_sc(cloud);
    else if (algo == "ransac_rvv")  result = (size_t)run_ransac_rvv(cloud);
    else if (algo == "normal_sc")   { run_normal_sc(cloud);  result = N; }
    else if (algo == "normal_rvv")  { run_normal_rvv(cloud); result = N; }
    else if (algo == "pipeline_sc") { run_pipeline_sc(cloud); result = N; }
    else if (algo == "pipeline_rvv"){ run_pipeline_rvv(cloud); result = N; }
    else {
        std::cerr << "Unknown algorithm: " << algo << "\n";
        return 1;
    }

    // Print result so gem5_bench.sh can verify the run completed
    std::cout << "gem5_bench: " << algo << " N=" << N << " result=" << result << "\n";
    return 0;
}
