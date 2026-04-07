#include "../src/include/rvv_pcl.h"
#include "../src/include/simple_pcd_loader.h"
#include <vector>
#include <random>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <chrono>
#include <string>
#include <cstdlib>

using namespace rvv_pcl;

// ─── rdinstret timer ─────────────────────────────────────────────────────────
struct Timer {
    uint64_t start;
    void reset() {
        asm volatile("" ::: "memory");
        asm volatile("rdinstret %0" : "=r"(start));
        asm volatile("" ::: "memory");
    }
    uint64_t elapsed() {
        uint64_t end;
        asm volatile("" ::: "memory");
        asm volatile("rdinstret %0" : "=r"(end));
        asm volatile("" ::: "memory");
        return end - start;
    }
};

// ─── Dual output (stdout + file) ─────────────────────────────────────────────
struct DualStream {
    std::ofstream &file;
    template <typename T>
    DualStream &operator<<(const T &v) { std::cout << v; file << v; return *this; }
    DualStream &operator<<(std::ostream &(*m)(std::ostream &)) {
        std::cout << m; file << m; return *this;
    }
};

// ─── Per-run results ──────────────────────────────────────────────────────────
struct PipelineResult {
    size_t N;
    // scalar stages
    uint64_t sc_voxel, sc_ransac, sc_extract, sc_sor, sc_normal;
    // rvv stages (octree_build is RVV-only real cost)
    uint64_t rv_voxel, rv_ransac, rv_extract, rv_sor, rv_octree, rv_normal;
    // post-pipeline cloud sizes (for transparency)
    size_t n_sc_vox, n_sc_obj, n_sc_sor;
    size_t n_rv_vox, n_rv_obj, n_rv_sor;
};

// ─── Spatial index comparison result (Octree vs SpatialHash for normals) ─────
struct SpatialIndexResult {
    size_t N;
    uint64_t octree_build, octree_normals;
    uint64_t hash_build,   hash_normals;
};

// ─── PCD stage saver (outside rdinstret windows) ─────────────────────────────
static void save_stage_pcd(const std::string& save_dir, const std::string& subdir,
                            const std::vector<PointXYZ>& pts, size_t n) {
    if (save_dir.empty() || n == 0) return;
    std::string dir = save_dir + "/" + subdir;
    std::system(("mkdir -p \"" + dir + "\"").c_str());
    rvv_pcl::savePCD(dir + "/output.pcd",
                     std::vector<PointXYZ>(pts.begin(), pts.begin() + static_cast<std::ptrdiff_t>(n)));
}

// ─── Pipeline runner ──────────────────────────────────────────────────────────
//
// Design rules:
//  1. All data generation and buffer allocation happen BEFORE any rdinstret read.
//  2. AoS<->SoA conversions between stages are NOT timed (scaffolding).
//  3. Octree build IS timed for RVV (it is real pipeline cost).
//  4. Scalar extract outliers: inline scalar loop (no scalar function exists).
//  5. Both pipelines run on identical input data.
//  6. mode: "sc" | "rvv" | "both" — skips the other pipeline.
//  7. save_dir: if non-empty, saves per-stage PCD files (outside rdinstret).
//
PipelineResult run_pipeline(size_t N,
                            const std::string& mode = "both",
                            const std::string& save_dir = "") {
    const bool do_sc  = (mode != "rvv");
    const bool do_rvv = (mode != "sc");

    // ── Parameters ──────────────────────────────────────────────────────────
    const float LEAF         = 1.0f;   // 1% density for [0,100] range
    const float RANSAC_THRESH = 0.5f;  // ground at z=0, objects at z>1
    const int   RANSAC_ITERS  = 500;
    const int   SOR_K         = 10;
    const float SOR_ALPHA     = 1.0f;
    const int   NORM_K        = 10;
    const float NORM_R        = 2.0f;

    // ── Data generation — entirely outside measurement ───────────────────────
    const size_t N_OBJ    = N * 8 / 10;   // 80% object cloud
    const size_t N_GROUND = N - N_OBJ;    // 20% ground plane at z=0

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> rxy(0.0f, 100.0f);
    std::uniform_real_distribution<float> rz(1.0f, 100.0f); // object z > 1

    std::vector<float> hx(N), hy(N), hz(N);
    for (size_t i = 0; i < N_OBJ; ++i) {
        hx[i] = rxy(rng); hy[i] = rxy(rng); hz[i] = rz(rng);
    }
    for (size_t i = N_OBJ; i < N; ++i) {
        hx[i] = rxy(rng); hy[i] = rxy(rng); hz[i] = 0.0f;
    }

    // Prepare AoS (scalar) and SoA (RVV) from same data
    std::vector<PointXYZ> h_aos(N);
    for (size_t i = 0; i < N; ++i)
        h_aos[i] = {hx[i], hy[i], hz[i]};
    PointCloudSoA h_soa = {hx.data(), hy.data(), hz.data(), N};

    // ── Pre-allocate all output buffers (max N) ──────────────────────────────
    std::vector<PointXYZ> sc_vox(N), sc_obj(N), sc_sor_buf(N);
    std::vector<float>    sc_nx(N),  sc_ny(N),   sc_nz(N);
    float sc_model[4] = {};

    std::vector<PointXYZ> rv_vox(N), rv_obj(N), rv_sor_buf(N);
    std::vector<float>    rv_nx(N),  rv_ny(N),   rv_nz(N);
    float rv_model[4] = {};

    // Intermediate SoA buffers for RVV inter-stage data (not timed)
    std::vector<float> vx(N), vy(N), vz(N); // after voxel
    std::vector<float> ox(N), oy(N), oz(N); // after extract
    std::vector<float> sx(N), sy(N), sz(N); // after sor

    PipelineResult res{};
    res.N = N;
    Timer t;

    // ═══════════════════════════════════════════════════════════════════════
    //  SCALAR PIPELINE
    // ═══════════════════════════════════════════════════════════════════════
    size_t n_sc_vox = 0, n_sc_obj = 0, n_sc_sor = 0;
    if (do_sc) {

    // Stage 1 — Voxel Grid (scalar, AoS)
    t.reset();
    n_sc_vox = voxel_grid_downsamp_sc(h_aos.data(), N, sc_vox.data(), LEAF);
    res.sc_voxel = t.elapsed();
    save_stage_pcd(save_dir, "01_voxel_sc", sc_vox, n_sc_vox);

    // Stage 2 — RANSAC (scalar, AoS)
    t.reset();
    ransac_plane_sc(sc_vox.data(), n_sc_vox, RANSAC_THRESH, RANSAC_ITERS, sc_model);
    res.sc_ransac = t.elapsed();

    // Stage 3 — Extract plane outliers (scalar inline — no library function)
    t.reset();
    for (size_t j = 0; j < n_sc_vox; ++j) {
        float d = sc_model[0] * sc_vox[j].x + sc_model[1] * sc_vox[j].y
                + sc_model[2] * sc_vox[j].z + sc_model[3];
        if (d < -RANSAC_THRESH || d > RANSAC_THRESH)
            sc_obj[n_sc_obj++] = sc_vox[j];
    }
    res.sc_extract = t.elapsed();
    save_stage_pcd(save_dir, "02_ransac_extract_sc", sc_obj, n_sc_obj);

    // Stage 4 — SOR (scalar, AoS)
    t.reset();
    n_sc_sor = sor_sc(sc_obj.data(), n_sc_obj, sc_sor_buf.data(), SOR_K, SOR_ALPHA);
    res.sc_sor = t.elapsed();
    save_stage_pcd(save_dir, "03_sor_sc", sc_sor_buf, n_sc_sor);

    // Stage 5 — Octree build: not needed in scalar path
    // Stage 6 — Normal estimation (scalar brute-force O(n²), AoS)
    t.reset();
    normal_estimation_sc(sc_sor_buf.data(), n_sc_sor,
                         sc_nx.data(), sc_ny.data(), sc_nz.data(),
                         NORM_K, NORM_R);
    res.sc_normal = t.elapsed();
    save_stage_pcd(save_dir, "04_normals_sc", sc_sor_buf, n_sc_sor);

    } // end do_sc
    res.n_sc_vox = n_sc_vox;
    res.n_sc_obj = n_sc_obj;
    res.n_sc_sor = n_sc_sor;

    // ═══════════════════════════════════════════════════════════════════════
    //  RVV PIPELINE
    // ═══════════════════════════════════════════════════════════════════════
    size_t n_rv_vox = 0, n_rv_obj = 0, n_rv_sor = 0;
    if (do_rvv) {

    // Stage 1 — Voxel Grid (RVV v2, SoA input → AoS output)
    t.reset();
    n_rv_vox = voxel_grid_downsamp_rvv_v2(h_soa, rv_vox.data(), LEAF);
    res.rv_voxel = t.elapsed();
    save_stage_pcd(save_dir, "01_voxel_rvv", rv_vox, n_rv_vox);

    // [not timed] AoS → SoA for next stage
    for (size_t j = 0; j < n_rv_vox; ++j) {
        vx[j] = rv_vox[j].x; vy[j] = rv_vox[j].y; vz[j] = rv_vox[j].z;
    }
    PointCloudSoA vox_soa = {vx.data(), vy.data(), vz.data(), n_rv_vox};

    // Stage 2 — RANSAC (RVV, SoA)
    t.reset();
    ransac_plane_rvv(vox_soa, RANSAC_THRESH, RANSAC_ITERS, rv_model);
    res.rv_ransac = t.elapsed();

    // Stage 3 — Extract plane outliers (RVV, SoA → AoS)
    t.reset();
    n_rv_obj = extract_plane_outliers_rvv(vox_soa, rv_model, RANSAC_THRESH, rv_obj.data());
    res.rv_extract = t.elapsed();
    save_stage_pcd(save_dir, "02_ransac_extract_rvv", rv_obj, n_rv_obj);

    // [not timed] AoS → SoA for next stage
    for (size_t j = 0; j < n_rv_obj; ++j) {
        ox[j] = rv_obj[j].x; oy[j] = rv_obj[j].y; oz[j] = rv_obj[j].z;
    }
    PointCloudSoA obj_soa = {ox.data(), oy.data(), oz.data(), n_rv_obj};

    // Stage 4 — SOR (RVV, SoA → AoS)
    t.reset();
    n_rv_sor = sor_rvv(obj_soa, rv_sor_buf.data(), SOR_K, SOR_ALPHA);
    res.rv_sor = t.elapsed();
    save_stage_pcd(save_dir, "03_sor_rvv", rv_sor_buf, n_rv_sor);

    // [not timed] AoS → SoA for next stage
    for (size_t j = 0; j < n_rv_sor; ++j) {
        sx[j] = rv_sor_buf[j].x; sy[j] = rv_sor_buf[j].y; sz[j] = rv_sor_buf[j].z;
    }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_rv_sor};

    // Stage 5 — Octree Build (RVV — COUNTED: it is real pipeline cost)
    Octree octree;
    octree.setInputCloud(sor_soa);
    t.reset();
    octree.build();
    res.rv_octree = t.elapsed();

    // Stage 6 — Normal Estimation (RVV + Octree, SoA)
    t.reset();
    normal_estimation_rvv(sor_soa, octree,
                          rv_nx.data(), rv_ny.data(), rv_nz.data(),
                          NORM_K, NORM_R);
    res.rv_normal = t.elapsed();
    save_stage_pcd(save_dir, "04_normals_rvv", rv_sor_buf, n_rv_sor);

    } // end do_rvv
    res.n_rv_vox = n_rv_vox;
    res.n_rv_obj = n_rv_obj;
    res.n_rv_sor = n_rv_sor;

    return res;
}

// ─── Spatial index comparison ─────────────────────────────────────────────────
// Runs ONLY Stage 5+6 on the already-cleaned cloud from run_pipeline,
// comparing Octree vs SpatialHash build + normal estimation cost.
// Uses the same SoA cloud (sor_soa equivalent), regenerated here for isolation.
SpatialIndexResult bench_spatial_index(size_t N) {
    const float NORM_R = 2.0f;
    const int   NORM_K = 10;

    // Generate a clean object cloud (no ground — that's already been removed
    // in the real pipeline at this point)
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> rd(0.0f, 50.0f);
    std::vector<float> hx(N), hy(N), hz(N);
    for (size_t i = 0; i < N; ++i) {
        hx[i] = rd(rng); hy[i] = rd(rng); hz[i] = rd(rng);
    }
    PointCloudSoA soa = {hx.data(), hy.data(), hz.data(), N};

    std::vector<float> nx(N), ny(N), nz(N);
    Timer t;
    SpatialIndexResult r{};
    r.N = N;

    // ── Octree ──────────────────────────────────────────────────────────────
    {
        Octree octree;
        octree.setInputCloud(soa);
        t.reset(); octree.build();         r.octree_build   = t.elapsed();
        t.reset();
        normal_estimation_rvv(soa, octree, nx.data(), ny.data(), nz.data(), NORM_K, NORM_R);
        r.octree_normals = t.elapsed();
    }

    // ── SpatialHash ─────────────────────────────────────────────────────────
    // cell_size = NORM_R (search radius) is optimal: most queries touch ≤27 cells
    {
        SpatialHash hash;
        hash.setInputCloud(soa, NORM_R);
        t.reset(); hash.build();           r.hash_build     = t.elapsed();
        t.reset();
        normal_estimation_rvv(soa, hash, nx.data(), ny.data(), nz.data(), NORM_K, NORM_R);
        r.hash_normals   = t.elapsed();
    }

    return r;
}

// ─── Formatted row ────────────────────────────────────────────────────────────
static void print_row(DualStream &out, const std::string &stage,
                      uint64_t sc, uint64_t rv, bool no_ratio = false) {
    std::string sc_str = (sc == 0) ? "---" : std::to_string(sc);
    std::string rv_str = (rv == 0) ? "---" : std::to_string(rv);
    std::string ratio_str = "---";
    if (!no_ratio && sc > 0 && rv > 0) {
        std::ostringstream rs;
        rs << std::fixed << std::setprecision(2) << (double)sc / (double)rv << "x";
        ratio_str = rs.str();
    }
    std::ostringstream row;
    row << std::left << std::setw(21) << stage
        << "| " << std::left << std::setw(13) << sc_str
        << "| " << std::left << std::setw(13) << rv_str
        << "| " << ratio_str << "\n";
    out << row.str();
}

// ─── Real-data pipeline runner ────────────────────────────────────────────────
// Parameters tuned for metre-scale LiDAR data (e.g. Sick LMS400 table scene).
PipelineResult run_pipeline_real(const std::vector<PointXYZ>& input_pts,
                                 const std::string& mode = "both",
                                 const std::string& save_dir = "") {
    const bool do_sc  = (mode != "rvv");
    const bool do_rvv = (mode != "sc");

    const float LEAF          = 0.01f;  // 1 cm voxel for metre-scale clouds
    const float RANSAC_THRESH = 0.02f;  // 2 cm table-plane tolerance
    const int   RANSAC_ITERS  = 500;
    const int   SOR_K         = 10;
    const float SOR_ALPHA     = 1.0f;
    const int   NORM_K        = 10;
    const float NORM_R        = 0.05f;  // 5 cm radius for normals

    const size_t N = input_pts.size();

    // Build SoA from the AoS loaded by simple_pcd_loader (not timed)
    std::vector<float> hx(N), hy(N), hz(N);
    for (size_t i = 0; i < N; ++i) {
        hx[i] = input_pts[i].x;
        hy[i] = input_pts[i].y;
        hz[i] = input_pts[i].z;
    }
    PointCloudSoA h_soa = {hx.data(), hy.data(), hz.data(), N};

    // Pre-allocate output buffers
    std::vector<PointXYZ> sc_vox(N), sc_obj(N), sc_sor_buf(N);
    std::vector<float>    sc_nx(N),  sc_ny(N),  sc_nz(N);
    float sc_model[4] = {};

    std::vector<PointXYZ> rv_vox(N), rv_obj(N), rv_sor_buf(N);
    std::vector<float>    rv_nx(N),  rv_ny(N),  rv_nz(N);
    float rv_model[4] = {};

    std::vector<float> vx(N), vy(N), vz(N);
    std::vector<float> ox(N), oy(N), oz(N);
    std::vector<float> sx(N), sy(N), sz(N);

    PipelineResult res{};
    res.N = N;
    Timer t;

    // ═══ SCALAR PIPELINE ════════════════════════════════════════════════════
    size_t n_sc_vox = 0, n_sc_obj = 0, n_sc_sor = 0;
    if (do_sc) {

    t.reset();
    n_sc_vox = voxel_grid_downsamp_sc(input_pts.data(), N, sc_vox.data(), LEAF);
    res.sc_voxel = t.elapsed();
    save_stage_pcd(save_dir, "01_voxel_sc", sc_vox, n_sc_vox);

    t.reset();
    ransac_plane_sc(sc_vox.data(), n_sc_vox, RANSAC_THRESH, RANSAC_ITERS, sc_model);
    res.sc_ransac = t.elapsed();

    t.reset();
    for (size_t j = 0; j < n_sc_vox; ++j) {
        float d = sc_model[0]*sc_vox[j].x + sc_model[1]*sc_vox[j].y
                + sc_model[2]*sc_vox[j].z + sc_model[3];
        if (d < -RANSAC_THRESH || d > RANSAC_THRESH)
            sc_obj[n_sc_obj++] = sc_vox[j];
    }
    res.sc_extract = t.elapsed();
    save_stage_pcd(save_dir, "02_ransac_extract_sc", sc_obj, n_sc_obj);

    t.reset();
    n_sc_sor = sor_sc(sc_obj.data(), n_sc_obj, sc_sor_buf.data(), SOR_K, SOR_ALPHA);
    res.sc_sor = t.elapsed();
    save_stage_pcd(save_dir, "03_sor_sc", sc_sor_buf, n_sc_sor);

    t.reset();
    normal_estimation_sc(sc_sor_buf.data(), n_sc_sor,
                         sc_nx.data(), sc_ny.data(), sc_nz.data(),
                         NORM_K, NORM_R);
    res.sc_normal = t.elapsed();
    save_stage_pcd(save_dir, "04_normals_sc", sc_sor_buf, n_sc_sor);

    } // end do_sc
    res.n_sc_vox = n_sc_vox;
    res.n_sc_obj = n_sc_obj;
    res.n_sc_sor = n_sc_sor;

    // ═══ RVV PIPELINE ═══════════════════════════════════════════════════════
    size_t n_rv_vox = 0, n_rv_obj = 0, n_rv_sor = 0;
    if (do_rvv) {

    t.reset();
    n_rv_vox = voxel_grid_downsamp_rvv_v2(h_soa, rv_vox.data(), LEAF);
    res.rv_voxel = t.elapsed();
    save_stage_pcd(save_dir, "01_voxel_rvv", rv_vox, n_rv_vox);

    for (size_t j = 0; j < n_rv_vox; ++j) {
        vx[j] = rv_vox[j].x; vy[j] = rv_vox[j].y; vz[j] = rv_vox[j].z;
    }
    PointCloudSoA vox_soa = {vx.data(), vy.data(), vz.data(), n_rv_vox};

    t.reset();
    ransac_plane_rvv(vox_soa, RANSAC_THRESH, RANSAC_ITERS, rv_model);
    res.rv_ransac = t.elapsed();

    t.reset();
    n_rv_obj = extract_plane_outliers_rvv(vox_soa, rv_model, RANSAC_THRESH, rv_obj.data());
    res.rv_extract = t.elapsed();
    save_stage_pcd(save_dir, "02_ransac_extract_rvv", rv_obj, n_rv_obj);

    for (size_t j = 0; j < n_rv_obj; ++j) {
        ox[j] = rv_obj[j].x; oy[j] = rv_obj[j].y; oz[j] = rv_obj[j].z;
    }
    PointCloudSoA obj_soa = {ox.data(), oy.data(), oz.data(), n_rv_obj};

    t.reset();
    n_rv_sor = sor_rvv(obj_soa, rv_sor_buf.data(), SOR_K, SOR_ALPHA);
    res.rv_sor = t.elapsed();
    save_stage_pcd(save_dir, "03_sor_rvv", rv_sor_buf, n_rv_sor);

    for (size_t j = 0; j < n_rv_sor; ++j) {
        sx[j] = rv_sor_buf[j].x; sy[j] = rv_sor_buf[j].y; sz[j] = rv_sor_buf[j].z;
    }
    PointCloudSoA sor_soa = {sx.data(), sy.data(), sz.data(), n_rv_sor};

    Octree octree;
    octree.setInputCloud(sor_soa);
    t.reset();
    octree.build();
    res.rv_octree = t.elapsed();

    t.reset();
    normal_estimation_rvv(sor_soa, octree,
                          rv_nx.data(), rv_ny.data(), rv_nz.data(),
                          NORM_K, NORM_R);
    res.rv_normal = t.elapsed();
    save_stage_pcd(save_dir, "04_normals_rvv", rv_sor_buf, n_rv_sor);

    } // end do_rvv
    res.n_rv_vox = n_rv_vox;
    res.n_rv_obj = n_rv_obj;
    res.n_rv_sor = n_rv_sor;

    return res;
}

// ─── Print one PipelineResult block ──────────────────────────────────────────
static void print_pipeline_result(DualStream& out, const PipelineResult& r,
                                  const std::string& label) {
    uint64_t sc_total = r.sc_voxel + r.sc_ransac + r.sc_extract
                      + r.sc_sor   + r.sc_normal;
    uint64_t rv_total = r.rv_voxel + r.rv_ransac + r.rv_extract
                      + r.rv_sor   + r.rv_octree + r.rv_normal;

    out << "\n--- " << label << " (N = " << r.N << ") ---\n";
    out << "  After Voxel:   sc=" << r.n_sc_vox << "  rv=" << r.n_rv_vox << "\n";
    out << "  After Extract: sc=" << r.n_sc_obj << "  rv=" << r.n_rv_obj << "\n";
    out << "  After SOR:     sc=" << r.n_sc_sor << "  rv=" << r.n_rv_sor << "\n";
    out << "\n";
    out << "Stage                | Scalar(ins)  | RVV(ins)     | Ratio\n";
    out << "------------------------------------------------------------\n";
    print_row(out, "VoxelGrid",    r.sc_voxel,  r.rv_voxel);
    print_row(out, "RANSAC",       r.sc_ransac, r.rv_ransac);
    print_row(out, "ExtractPlane", r.sc_extract,r.rv_extract);
    print_row(out, "SOR",          r.sc_sor,    r.rv_sor);
    print_row(out, "OctreeBuild",  0,           r.rv_octree, true);
    print_row(out, "NormalEst",    r.sc_normal, r.rv_normal);
    out << "------------------------------------------------------------\n";
    print_row(out, "TOTAL (pipeline)", sc_total, rv_total);
    out << "============================================================\n";
}

// ─── Machine-readable stage output (for CSV construction by bench_run.sh) ────
// Prints tagged lines to stdout:
//   BENCH_STAGE <algo> <mode> <stage> <n_in> <n_out> <instructions>
//   BENCH_TOTAL <algo> <mode> <n_in> <n_out_final> <total_instructions>
static void print_bench_stages(const std::string& algo, const std::string& mode,
                               size_t n_in, const PipelineResult& r) {
    auto emit_sc = [&](const std::string& stage, size_t n_out, uint64_t ins) {
        if (ins > 0)
            std::cout << "BENCH_STAGE " << algo << " sc " << stage
                      << " " << n_in << " " << n_out << " " << ins << "\n";
    };
    auto emit_rv = [&](const std::string& stage, size_t n_out, uint64_t ins) {
        if (ins > 0)
            std::cout << "BENCH_STAGE " << algo << " rvv " << stage
                      << " " << n_in << " " << n_out << " " << ins << "\n";
    };

    if (mode != "rvv") {
        emit_sc("VoxelGrid",    r.n_sc_vox, r.sc_voxel);
        emit_sc("RANSAC",       r.n_sc_vox, r.sc_ransac);
        emit_sc("ExtractPlane", r.n_sc_obj, r.sc_extract);
        emit_sc("SOR",          r.n_sc_sor, r.sc_sor);
        emit_sc("NormalEst",    r.n_sc_sor, r.sc_normal);
        uint64_t tot = r.sc_voxel + r.sc_ransac + r.sc_extract + r.sc_sor + r.sc_normal;
        std::cout << "BENCH_TOTAL " << algo << " sc TOTAL "
                  << n_in << " " << r.n_sc_sor << " " << tot << "\n";
    }
    if (mode != "sc") {
        emit_rv("VoxelGrid",    r.n_rv_vox, r.rv_voxel);
        emit_rv("RANSAC",       r.n_rv_vox, r.rv_ransac);
        emit_rv("ExtractPlane", r.n_rv_obj, r.rv_extract);
        emit_rv("SOR",          r.n_rv_sor, r.rv_sor);
        emit_rv("OctreeBuild",  r.n_rv_sor, r.rv_octree);
        emit_rv("NormalEst",    r.n_rv_sor, r.rv_normal);
        uint64_t tot = r.rv_voxel + r.rv_ransac + r.rv_extract
                     + r.rv_sor + r.rv_octree + r.rv_normal;
        std::cout << "BENCH_TOTAL " << algo << " rvv TOTAL "
                  << n_in << " " << r.n_rv_sor << " " << tot << "\n";
    }
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[]) {
    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");

    std::system("mkdir -p results");
    std::string filename = "results/benchmark_pipeline_" + ss.str() + ".txt";
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "Error: cannot open " << filename << "\n";
        return 1;
    }
    DualStream out{outfile};

    // ── Argument parsing ─────────────────────────────────────────────────────
    std::string save_dir;
    std::string mode     = "both";  // sc | rvv | both
    std::string pcd_path;
    size_t      single_n = 0;       // 0 = use default size sweep

    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "--save-dir" && i + 1 < argc) { save_dir  = argv[++i]; }
        else if (a == "--mode" && i + 1 < argc) { mode     = argv[++i]; }
        else if (a == "--n"    && i + 1 < argc) { single_n = static_cast<size_t>(std::atol(argv[++i])); }
        else if (a[0] != '-')                   { pcd_path = a; }  // positional = PCD file
    }

    out << "Saving report to: " << filename << "\n\n";

    // ── Mode: real PCD file ──────────────────────────────────────────────────
    if (!pcd_path.empty()) {
        out << "============================================================\n";
        out << "  RVPoint End-to-End Pipeline Benchmark (rdinstret)\n";
        out << "  Mode: real LiDAR data from PCD file\n";
        out << "  File: " << pcd_path << "\n";
        out << "  Pipeline mode: " << mode << "\n";
        out << "  Params: leaf=0.01m  ransac_thresh=0.02m  norm_r=0.05m\n";
        out << "  Scalar: brute-force NN search\n";
        out << "  RVV:    vectorized kernels + Octree (build cost included)\n";
        out << "  Note: AoS<->SoA conversions between stages are NOT timed.\n";
        out << "============================================================\n";

        std::vector<PointXYZ> pts;
        int loaded = rvv_pcl::loadPCD(pcd_path, pts);
        if (loaded < 0) {
            std::cerr << "Error: failed to load " << pcd_path << "\n";
            return 1;
        }
        out << "\nLoaded " << loaded << " points from " << pcd_path << "\n";

        out << "\nRunning real-data pipeline (" << mode << ")...";
        PipelineResult r = run_pipeline_real(pts, mode, save_dir);
        out << " done.\n";
        print_pipeline_result(out, r, "Real LiDAR");
        print_bench_stages("pipeline", mode, static_cast<size_t>(loaded), r);

        outfile.close();
        return 0;
    }

    // ── Mode: synthetic benchmark (default) ──────────────────────────────────
    out << "============================================================\n";
    out << "  RVPoint End-to-End Pipeline Benchmark (rdinstret)\n";
    out << "  Synthetic cloud: 80% object (z>1) + 20% ground plane (z=0)\n";
    out << "  Pipeline mode: " << mode << "\n";
    out << "  Scalar: brute-force NN search\n";
    out << "  RVV:    vectorized kernels + Octree (build cost included)\n";
    out << "  Note: AoS<->SoA conversions between stages are NOT timed.\n";
    out << "============================================================\n";

    // Single N or default sweep (1024, 4096, 16384)
    std::vector<size_t> SIZES;
    if (single_n > 0) {
        SIZES.push_back(single_n);
    } else {
        SIZES = {1024, 4096, 16384};
    }

    for (size_t N : SIZES) {
        out << "\nRunning N=" << N << " (" << mode << ")...";
        PipelineResult r = run_pipeline(N, mode, save_dir);
        out << " done.\n";

        out << "  Input:        " << N << " pts"
            << "   (obj: " << N*8/10 << ", ground: " << N-N*8/10 << ")\n";
        print_pipeline_result(out, r, "Synthetic");
        print_bench_stages("pipeline", mode, N, r);
    }

    // ── Section 2: Spatial Index Comparison (Octree vs SpatialHash) ──────────
    out << "\n\n";
    out << "============================================================\n";
    out << "  Spatial Index Comparison: Octree vs SpatialHash\n";
    out << "  Stage 5+6 only (build + normal estimation)\n";
    out << "  Clean object cloud, no ground plane, seed=99\n";
    out << "  cell_size = search_radius (" << 2.0f << ") for SpatialHash\n";
    out << "============================================================\n";
    out << "\n";
    out << "N      | Phase         | Octree(ins)  | SpatialHash(ins) | Winner\n";
    out << "-----------------------------------------------------------------------\n";

    for (size_t N : SIZES) {
        out << "Running spatial index bench N=" << N << " ...";
        SpatialIndexResult sr = bench_spatial_index(N);
        out << " done.\n";

        auto winner = [](uint64_t a, uint64_t b, const char* na, const char* nb) {
            return (a <= b) ? na : nb;
        };

        uint64_t oct_total  = sr.octree_build + sr.octree_normals;
        uint64_t hash_total = sr.hash_build   + sr.hash_normals;

        auto fmt_row = [&](const std::string& phase, uint64_t oct, uint64_t sh) {
            std::ostringstream row;
            row << std::left << std::setw(7) << N
                << "| " << std::left << std::setw(14) << phase
                << "| " << std::left << std::setw(13) << oct
                << "| " << std::left << std::setw(17) << sh
                << "| " << winner(oct, sh, "Octree", "SpatialHash") << "\n";
            out << row.str();
        };

        fmt_row("Build",   sr.octree_build,   sr.hash_build);
        fmt_row("Normals", sr.octree_normals, sr.hash_normals);
        fmt_row("TOTAL",   oct_total,         hash_total);
        out << "-----------------------------------------------------------------------\n";
    }

    out << "\nNote: SpatialHash build is O(n); Octree build is O(n log n).\n";
    out << "      SpatialHash query is O(1) avg; Octree query is O(log n + k).\n";
    out << "      SpatialHash wins at uniform distributions and fixed-radius queries.\n";
    out << "      Octree wins when point density varies significantly across the cloud.\n";

    outfile.close();
    return 0;
}
