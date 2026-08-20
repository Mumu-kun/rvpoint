#include "simple_pcd_loader.h"
#include "fast_3d_spatial_grid.h"
#include "rvv_pcl.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <filesystem>
#include <algorithm>

using namespace rvv_pcl;

#define TEST_CHECK(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "  [FAIL] " << msg << " (line " << __LINE__ << ")\n"; \
            g_failures++; \
        } else { \
            std::cout << "  [PASS] " << msg << "\n"; \
        } \
    } while(0)

static int g_failures = 0;

// Test 1: ASCII PCD Loader Finite Filtering & Multi-Component COUNT Handling
static void test_ascii_loader_hardening() {
    std::cout << "[TEST 1] Testing ASCII PCD Loader Hardening..." << std::endl;
    std::string test_pcd_path = "/tmp/test_ascii_nan.pcd";
    {
        std::ofstream ofs(test_pcd_path);
        ofs << "# .PCD v.7 - Point Cloud Data file format\n"
            << "VERSION .7\n"
            << "FIELDS x y z rgb\n"
            << "SIZE 4 4 4 4\n"
            << "TYPE F F F F\n"
            << "COUNT 1 1 1 1\n"
            << "WIDTH 4\n"
            << "HEIGHT 1\n"
            << "POINTS 4\n"
            << "DATA ascii\n"
            << "1.0 2.0 3.0 12345\n"
            << "nan 2.0 3.0 12345\n"
            << "1.0 inf 3.0 12345\n"
            << "4.0 5.0 6.0 12345\n";
    }

    std::vector<PointXYZ> pts;
    int64_t count = loadPCD(test_pcd_path, pts);
    TEST_CHECK(count == 2, "loadPCD filtered out NaN and Inf points from ASCII PCD");
    TEST_CHECK(pts.size() == 2, "pts vector contains exactly 2 finite points");
    if (pts.size() == 2) {
        TEST_CHECK(pts[0].x == 1.0f && pts[0].y == 2.0f && pts[0].z == 3.0f, "First point coordinates match");
        TEST_CHECK(pts[1].x == 4.0f && pts[1].y == 5.0f && pts[1].z == 6.0f, "Second point coordinates match");
    }
    std::filesystem::remove(test_pcd_path);
}

// Test 2: Dynamic Spatial Grid Safety and Bounded Probing
static void test_dynamic_grid_safety() {
    std::cout << "[TEST 2] Testing Dynamic 3D Spatial Grid with 100k points..." << std::endl;
    const size_t N = 100000;
    std::vector<float> x(N), y(N), z(N);
    for (size_t i = 0; i < N; ++i) {
        x[i] = static_cast<float>(i % 100) * 0.05f;
        y[i] = static_cast<float>((i / 100) % 100) * 0.05f;
        z[i] = static_cast<float>(i / 10000) * 0.05f;
    }

    // Initialize with expected points to trigger dynamic sizing
    Fast3DSpatialGrid grid(0.25f, N);
    TEST_CHECK(grid.capacity_ >= N * 4, "Grid capacity is sized to >= 4x point count");
    bool build_ok = grid.build(x.data(), y.data(), z.data(), N);
    TEST_CHECK(build_ok, "Grid build completed without hang or crash");

    std::vector<int> nbrs;
    std::vector<float> d2;
    grid.radiusSearch(0.0f, 0.0f, 0.0f, 0.25f * 0.25f, nbrs, d2);
    TEST_CHECK(!nbrs.empty(), "radiusSearch found expected neighbor points within radius");
}

// Test 3: Save PCD boolean status and error propagation
static void test_io_error_propagation() {
    std::cout << "[TEST 3] Testing I/O Error Propagation..." << std::endl;
    std::vector<PointXYZ> pts = {{1.0f, 2.0f, 3.0f}};
    bool save_fail = savePCD("/invalid_dir_nonexistent/test.pcd", pts, true);
    TEST_CHECK(!save_fail, "savePCD returns false on invalid file path");

    std::vector<PointXYZRGB> rgb_pts = {{1.0f, 2.0f, 3.0f, 255, 0, 0}};
    bool rgb_fail = savePCDRGB("/invalid_dir_nonexistent/test_rgb.pcd", rgb_pts, true);
    TEST_CHECK(!rgb_fail, "savePCDRGB returns false on invalid file path");
}

int main() {
    std::cout << "========================================================" << std::endl;
    std::cout << "RUNNING PIPELINE ULTRA AUDIT DEFECTS VERIFICATION TEST" << std::endl;
    std::cout << "========================================================" << std::endl;

    test_ascii_loader_hardening();
    test_dynamic_grid_safety();
    test_io_error_propagation();

    std::cout << "========================================================" << std::endl;
    if (g_failures == 0) {
        std::cout << "ALL AUDIT DEFECT VERIFICATION TESTS PASSED SUCCESSFULLY!" << std::endl;
        return 0;
    } else {
        std::cerr << "FAILURES DETECTED: " << g_failures << std::endl;
        return 1;
    }
}
