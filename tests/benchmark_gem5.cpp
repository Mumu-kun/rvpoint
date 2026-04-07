// benchmark_gem5.cpp
// Single-algorithm, single-pass runner designed for gem5 cycle-accurate simulation.
//
// Usage: benchmark_gem5 <algorithm> <N> [pcd_path]
//
// Algorithms (synthetic):
//   voxel_sc      voxel_rvv
//   ransac_sc     ransac_rvv
//   sor_sc        sor_rvv
//   normal_sc     normal_rvv    (normal_rvv includes octree build)
//   pipeline_sc   pipeline_rvv  (full pipeline, single pass)
//
// Algorithms (real LiDAR — require pcd_path as argv[3]):
//   pipeline_pcd_sc   pipeline_pcd_rvv
//     N = max points to subsample from the PCD (keep small for gem5, e.g. 512)
//     params: leaf=0.05m  ransac_thresh=0.05m  norm_r=0.1m (metre-scale LiDAR)
//
// Design rules for gem5:
//  - Statically linked (elf toolchain) so gem5 SE mode doesn't need sysroot.
//  - Data generation happens before the algorithm call — gem5 measures
//    total cycles but the algo dominates for reasonable N.
//  - One pass only: gem5 is slow (~1-10 MIPS simulated vs 1 BIPS real hw).
//  - No rdinstret — gem5 extracts cycle counts from its own stats.txt.
//  - Prints a single result line to stdout so gem5_bench.sh can verify exit.

#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"
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

// ─── Real PCD loader (metre-scale LiDAR, subsampled for gem5) ────────────────
// Loads a PCD file and returns a CloudData with at most max_pts points
// selected by uniform stride (deterministic, no shuffle required).
static CloudData load_pcd_cloud(const std::string& path, size_t max_pts) {
    std::vector<PointXYZ> pts;
    int n = rvv_pcl::loadPCD(path, pts);
    if (n < 0) {
        std::cerr << "Failed to load PCD: " << path << "\n";
        std::exit(1);
    }

    if (max_pts > 0 && pts.size() > max_pts) {
        std::vector<PointXYZ> sub;
        sub.reserve(max_pts);
        size_t stride = pts.size() / max_pts;
        for (size_t i = 0; i < max_pts; ++i)
            sub.push_back(pts[i * stride]);
        pts = std::move(sub);
    }

    CloudData d;
    size_t N = pts.size();
    d.x.resize(N); d.y.resize(N); d.z.resize(N); d.aos.resize(N);
    for (size_t i = 0; i < N; ++i) {
        d.x[i] = pts[i].x; d.y[i] = pts[i].y; d.z[i] = pts[i].z;
        d.aos[i] = pts[i];
    }
    d.soa = {d.x.data(), d.y.data(), d.z.data(), N};
    return d;
}

// ─── PCD pipeline runners ─────────────────────────────────────────────────────
// Two parameter sets:
//   "lidar" (table_scene_lms400 etc): large outdoor scan, metre-scale
//      leaf=0.05m  ransac_thresh=0.05m  norm_r=0.1m
//   "bunny" (Stanford bunny, ~5cm object): centimetre-scale object scan
//      leaf=0.005m  ransac_thresh=0.003m  norm_r=0.015m

struct PipelineParams {
    float leaf, ransac_thresh, norm_r;
    int k, ransac_iters;
};
static constexpr PipelineParams P_LIDAR  = {0.05f,  0.05f,  0.10f,  10, 200};
static constexpr PipelineParams P_BUNNY  = {0.005f, 0.003f, 0.015f, 10, 200};

// ─── Stage helpers ────────────────────────────────────────────────────────────

// Read retired instruction counter (CSR 0xC02).
// gem5 SE mode maps rdinstret to its sim_insts counter.
static inline uint64_t read_instret() {
    uint64_t v = 0;
    __asm__ volatile("rdinstret %0" : "=r"(v));
    return v;
}

// Save stage PCD.  Directories are pre-created by the shell script before
// gem5 starts, so no mkdir() syscall is needed — just open() + write().
static void save_stage(const std::string& save_dir, const char* stage_name,
                       const PointXYZ* pts, size_t n) {
    if (save_dir.empty() || n == 0) return;
    std::string path = save_dir + "/" + stage_name + "/output.pcd";
    savePCD(path, std::vector<PointXYZ>(pts, pts + n));
}

// Print a machine-readable stage stats line to stdout.
// Parsed by gem5_bench_run.sh into stage_stats.csv.
//   GEM5_STAGE <algo> <mode> <stage> <n_in> <n_out> <delta_instret>
static void print_stage(const char* algo, const char* mode, const char* stage,
                        size_t n_in, size_t n_out, uint64_t delta) {
    std::cout << "GEM5_STAGE " << algo << " " << mode << " " << stage
              << " " << n_in << " " << n_out << " " << delta << "\n";
    std::cout.flush();
}

static void run_pipeline_pcd_sc(CloudData& c, const PipelineParams& p,
                                 const std::string& save_dir = "") {
    size_t N = c.x.size();
    std::vector<PointXYZ> vox(N), obj(N), sor_buf(N);
    std::vector<float> nx(N), ny(N), nz(N);
    float model[4];
    uint64_t t0, t1;

    t0 = read_instret();
    size_t n_vox = voxel_grid_downsamp_sc(c.aos.data(), N, vox.data(), p.leaf);
    t1 = read_instret();
    print_stage("pipeline_pcd", "sc", "VoxelGrid", N, n_vox, t1 - t0);
    save_stage(save_dir, "01_voxel_sc", vox.data(), n_vox);

    t0 = read_instret();
    ransac_plane_sc(vox.data(), n_vox, p.ransac_thresh, p.ransac_iters, model);
    t1 = read_instret();
    print_stage("pipeline_pcd", "sc", "RANSAC", n_vox, n_vox, t1 - t0);

    t0 = read_instret();
    size_t n_obj = 0;
    for (size_t j = 0; j < n_vox; ++j) {
        float d = model[0]*vox[j].x + model[1]*vox[j].y
                + model[2]*vox[j].z + model[3];
        if (d < -p.ransac_thresh || d > p.ransac_thresh) obj[n_obj++] = vox[j];
    }
    t1 = read_instret();
    print_stage("pipeline_pcd", "sc", "ExtractPlane", n_vox, n_obj, t1 - t0);
    save_stage(save_dir, "02_ransac_extract_sc", obj.data(), n_obj);

    t0 = read_instret();
    size_t n_sor = sor_sc(obj.data(), n_obj, sor_buf.data(), p.k, 1.f);
    t1 = read_instret();
    print_stage("pipeline_pcd", "sc", "SOR", n_obj, n_sor, t1 - t0);
    save_stage(save_dir, "03_sor_sc", sor_buf.data(), n_sor);

    t0 = read_instret();
    normal_estimation_sc(sor_buf.data(), n_sor,
                         nx.data(), ny.data(), nz.data(), p.k, p.norm_r);
    t1 = read_instret();
    print_stage("pipeline_pcd", "sc", "NormalEst", n_sor, n_sor, t1 - t0);
    save_stage(save_dir, "04_normals_sc", sor_buf.data(), n_sor);
}

static void run_pipeline_pcd_rvv(CloudData& c, const PipelineParams& p,
                                  const std::string& save_dir = "") {
    size_t N = c.x.size();
    std::vector<PointXYZ> vox(N), obj(N), sor_buf(N);
    std::vector<float> vx(N), vy(N), vz(N);
    std::vector<float> ox(N), oy(N), oz(N);
    std::vector<float> sx(N), sy(N), sz(N);
    std::vector<float> nx(N), ny(N), nz(N);
    float model[4];
    uint64_t t0, t1;

    t0 = read_instret();
    size_t n_vox = voxel_grid_downsamp_rvv_v2(c.soa, vox.data(), p.leaf);
    t1 = read_instret();
    print_stage("pipeline_pcd", "rvv", "VoxelGrid", N, n_vox, t1 - t0);
    save_stage(save_dir, "01_voxel_rvv", vox.data(), n_vox);

    for (size_t j = 0; j < n_vox; ++j) {
        vx[j]=vox[j].x; vy[j]=vox[j].y; vz[j]=vox[j].z;
    }
    PointCloudSoA vox_soa = {vx.data(), vy.data(), vz.data(), n_vox};

    t0 = read_instret();
    ransac_plane_rvv(vox_soa, p.ransac_thresh, p.ransac_iters, model);
    t1 = read_instret();
    print_stage("pipeline_pcd", "rvv", "RANSAC", n_vox, n_vox, t1 - t0);

    t0 = read_instret();
    size_t n_obj = extract_plane_outliers_rvv(vox_soa, model, p.ransac_thresh, obj.data());
    t1 = read_instret();
    print_stage("pipeline_pcd", "rvv", "ExtractPlane", n_vox, n_obj, t1 - t0);
    save_stage(save_dir, "02_ransac_extract_rvv", obj.data(), n_obj);

    for (size_t j = 0; j < n_obj; ++j) {
        ox[j]=obj[j].x; oy[j]=obj[j].y; oz[j]=obj[j].z;
    }
    PointCloudSoA obj_soa = {ox.data(), oy.data(), oz.data(), n_obj};

    t0 = read_instret();
    size_t n_sor = sor_rvv(obj_soa, sor_buf.data(), p.k, 1.f);
    t1 = read_instret();
    print_stage("pipeline_pcd", "rvv", "SOR", n_obj, n_sor, t1 - t0);
    save_stage(save_dir, "03_sor_rvv", sor_buf.data(), n_sor);

    for (size_t j = 0; j < n_sor; ++j) {
        sx[j]=sor_buf[j].x; sy[j]=sor_buf[j].y; sz[j]=sor_buf[j].z;
    }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_sor};

    t0 = read_instret();
    normal_estimation_rvv(sor_soa, nx.data(), ny.data(), nz.data(), p.k, p.norm_r);
    t1 = read_instret();
    print_stage("pipeline_pcd", "rvv", "NormalEst", n_sor, n_sor, t1 - t0);
    save_stage(save_dir, "04_normals_rvv", sor_buf.data(), n_sor);
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: benchmark_gem5 <algorithm> <N> [pcd_path]\n";
        std::cerr << "  synthetic: voxel_sc voxel_rvv sor_sc sor_rvv ransac_sc ransac_rvv\n";
        std::cerr << "             normal_sc normal_rvv pipeline_sc pipeline_rvv\n";
        std::cerr << "  real PCD:  pipeline_pcd_sc pipeline_pcd_rvv  (pcd_path required)\n";
        return 1;
    }

    std::string algo(argv[1]);
    // N=0 means "load full PCD without subsampling" (only valid for pcd algos)
    size_t N = (size_t)std::atoi(argv[2]);

    // Optional: --save-dir <path>  (pre-scan, directories created by shell before gem5)
    std::string save_dir;
    for (int i = 1; i < argc - 1; ++i) {
        if (std::string(argv[i]) == "--save-dir") {
            save_dir = argv[i + 1];
            break;
        }
    }

    if (N > 1000000) {
        std::cerr << "N must be <= 1000000 (use 0 for full PCD, no subsampling)\n";
        return 1;
    }

    size_t result = 0;

    // ── PCD algorithms: load real data, no synthetic generation ──────────────
    bool is_pcd_algo = (algo == "pipeline_pcd_sc"    || algo == "pipeline_pcd_rvv" ||
                        algo == "pipeline_pcd_bunny_sc" || algo == "pipeline_pcd_bunny_rvv");
    if (is_pcd_algo) {
        if (argc < 4) {
            std::cerr << "pipeline_pcd_* requires pcd_path as argv[3]\n";
            return 1;
        }
        std::string pcd_path(argv[3]);
        // Load (+ optional subsample) BEFORE gem5 cycle window
        CloudData cloud = load_pcd_cloud(pcd_path, N);
        result = cloud.x.size();
        bool is_bunny = (algo == "pipeline_pcd_bunny_sc" || algo == "pipeline_pcd_bunny_rvv");
        const PipelineParams& params = is_bunny ? P_BUNNY : P_LIDAR;
        if (algo == "pipeline_pcd_sc" || algo == "pipeline_pcd_bunny_sc")
            run_pipeline_pcd_sc(cloud, params, save_dir);
        else
            run_pipeline_pcd_rvv(cloud, params, save_dir);
    }
    // ── Synthetic algorithms ──────────────────────────────────────────────────
    else {
    if (N == 0) {
        std::cerr << "N=0 only valid for pcd algorithms\n";
        return 1;
    }
    // Generate data BEFORE gem5 starts measuring (startup is fast, algo dominates)
    CloudData cloud = make_cloud(N, /*with_ground=*/true);

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
    }

    // Print result so gem5_bench.sh can verify the run completed
    std::cout << "gem5_bench: " << algo << " N=" << N << " result=" << result << "\n";
    return 0;
}
