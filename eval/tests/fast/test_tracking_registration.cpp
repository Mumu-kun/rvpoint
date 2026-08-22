// test_tracking_registration.cpp
//
// Fast unit tests for ICP Registration (Stages 2–5) & SE3Transform
// Tests:
//   1. Pure translation recovery
//   2. Small rotation (5 deg) + translation recovery
//   3. Identity transform (no motion) convergence
//   4. SE3Transform basic operations

#include "include/rvpoint.h"

#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>

using namespace rvpoint;

namespace {

void generateSyntheticCloud(std::vector<float>& x,
                            std::vector<float>& y,
                            std::vector<float>& z,
                            std::vector<float>& nx,
                            std::vector<float>& ny,
                            std::vector<float>& nz,
                            int grid_n = 30,
                            float spacing = 0.05f) {
    x.clear(); y.clear(); z.clear();
    nx.clear(); ny.clear(); nz.clear();

    // Plane 1: Ground plane at z = 0, normal = (0, 0, 1)
    for (int i = 0; i < grid_n; ++i) {
        for (int j = 0; j < grid_n; ++j) {
            x.push_back((i - grid_n / 2.0f) * spacing);
            y.push_back((j - grid_n / 2.0f) * spacing);
            z.push_back(0.0f);
            nx.push_back(0.0f);
            ny.push_back(0.0f);
            nz.push_back(1.0f);
        }
    }

    // Plane 2: Wall at x = 0.5, normal = (-1, 0, 0)
    for (int i = 0; i < grid_n / 2; ++i) {
        for (int j = 0; j < grid_n / 2; ++j) {
            x.push_back(0.5f);
            y.push_back((i - grid_n / 4.0f) * spacing);
            z.push_back(j * spacing);
            nx.push_back(-1.0f);
            ny.push_back(0.0f);
            nz.push_back(0.0f);
        }
    }

    // Plane 3: Wall at y = 0.5, normal = (0, -1, 0)
    for (int i = 0; i < grid_n / 2; ++i) {
        for (int j = 0; j < grid_n / 2; ++j) {
            x.push_back((i - grid_n / 4.0f) * spacing);
            y.push_back(0.5f);
            z.push_back(j * spacing);
            nx.push_back(0.0f);
            ny.push_back(-1.0f);
            nz.push_back(0.0f);
        }
    }
}

void applyTransform(const SE3Transform& T,
                    const std::vector<float>& in_x,
                    const std::vector<float>& in_y,
                    const std::vector<float>& in_z,
                    std::vector<float>& out_x,
                    std::vector<float>& out_y,
                    std::vector<float>& out_z) {
    std::size_t n = in_x.size();
    out_x.resize(n); out_y.resize(n); out_z.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        PointXYZ p = {in_x[i], in_y[i], in_z[i]};
        PointXYZ tp = T.apply(p);
        out_x[i] = tp.x;
        out_y[i] = tp.y;
        out_z[i] = tp.z;
    }
}

bool transformsClose(const SE3Transform& a, const SE3Transform& b,
                     float rot_tol_deg = 5.0f, float trans_tol_m = 0.05f) {
    float dtx = a.tx() - b.tx();
    float dty = a.ty() - b.ty();
    float dtz = a.tz() - b.tz();
    float trans_err = std::sqrt(dtx*dtx + dty*dty + dtz*dtz);

    // Rotation difference: trace(R_a * R_b^T)
    float trace = 0.0f;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            trace += a.R(i, j) * b.R(i, j);
        }
    }
    float cos_angle = std::max(-1.0f, std::min(1.0f, (trace - 1.0f) * 0.5f));
    float angle_deg = std::acos(cos_angle) * 180.0f / 3.14159265f;

    std::cout << "  Translation error: " << std::fixed << std::setprecision(4)
              << trans_err << " m (threshold: " << trans_tol_m << " m)" << std::endl;
    std::cout << "  Rotation error:    " << std::fixed << std::setprecision(4)
              << angle_deg << " deg (threshold: " << rot_tol_deg << " deg)" << std::endl;

    return (trans_err <= trans_tol_m) && (angle_deg <= rot_tol_deg);
}

} // anonymous namespace

int main() {
    std::cout << "=============================================" << std::endl;
    std::cout << "   ICP Registration Unit Test" << std::endl;
    std::cout << "=============================================" << std::endl;

    int pass_count = 0;
    int fail_count = 0;

    // Test 1: Pure translation recovery
    {
        std::cout << "\n--- Test 1: Pure Translation (0.1, 0.05, 0.0) ---" << std::endl;

        std::vector<float> ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz;
        generateSyntheticCloud(ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz, 30, 0.05f);
        std::size_t n = ref_x.size();

        PointCloudSoA ref_soa = {ref_x.data(), ref_y.data(), ref_z.data(), n};

        SpatialHash hash;
        hash.setInputCloud(ref_soa, 0.1f);
        hash.build();

        SE3Transform known_T = SE3Transform::fromAxisAngle(0, 0, 0, 0.1f, 0.05f, 0.0f);

        std::vector<float> src_x, src_y, src_z;
        applyTransform(known_T, ref_x, ref_y, ref_z, src_x, src_y, src_z);
        PointCloudSoA src_soa = {src_x.data(), src_y.data(), src_z.data(), n};

        TrackingConfig config;
        config.icp_max_iterations = 30;
        config.correspondence_max_dist = 0.5f;

        ICPRegistration icp;
        SE3Transform recovered_T;
        int iterations;
        float error;
        double t_corr, t_res, t_red, t_solve;

        icp.align(src_soa, ref_soa,
                  ref_nx.data(), ref_ny.data(), ref_nz.data(),
                  hash, config,
                  recovered_T, iterations, error,
                  t_corr, t_res, t_red, t_solve);

        std::cout << "  ICP iterations: " << iterations << std::endl;
        std::cout << "  Final error:    " << error << std::endl;
        std::cout << "  Recovered tx=" << recovered_T.tx()
                  << " ty=" << recovered_T.ty()
                  << " tz=" << recovered_T.tz() << std::endl;

        SE3Transform composed = recovered_T.compose(known_T);
        SE3Transform identity;
        if (transformsClose(composed, identity, 5.0f, 0.05f)) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // Test 2: Small rotation (5 deg about Z) + Translation
    {
        std::cout << "\n--- Test 2: Small Rotation (5 deg about Z) + Translation ---" << std::endl;

        std::vector<float> ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz;
        generateSyntheticCloud(ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz, 30, 0.05f);
        std::size_t n = ref_x.size();

        PointCloudSoA ref_soa = {ref_x.data(), ref_y.data(), ref_z.data(), n};

        SpatialHash hash;
        hash.setInputCloud(ref_soa, 0.1f);
        hash.build();

        float angle = 5.0f * 3.14159265f / 180.0f;
        SE3Transform known_T = SE3Transform::fromAxisAngle(0, 0, angle, 0.02f, 0.01f, 0.0f);

        std::vector<float> src_x, src_y, src_z;
        applyTransform(known_T, ref_x, ref_y, ref_z, src_x, src_y, src_z);
        PointCloudSoA src_soa = {src_x.data(), src_y.data(), src_z.data(), n};

        TrackingConfig config;
        config.icp_max_iterations = 30;
        config.correspondence_max_dist = 0.5f;

        ICPRegistration icp;
        SE3Transform recovered_T;
        int iterations;
        float error;
        double t_corr, t_res, t_red, t_solve;

        icp.align(src_soa, ref_soa,
                  ref_nx.data(), ref_ny.data(), ref_nz.data(),
                  hash, config,
                  recovered_T, iterations, error,
                  t_corr, t_res, t_red, t_solve);

        std::cout << "  ICP iterations: " << iterations << std::endl;
        std::cout << "  Final error:    " << error << std::endl;

        SE3Transform composed = recovered_T.compose(known_T);
        SE3Transform identity;
        if (transformsClose(composed, identity, 10.0f, 0.05f)) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // Test 3: Identity transform
    {
        std::cout << "\n--- Test 3: Identity Transform (no motion) ---" << std::endl;

        std::vector<float> ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz;
        generateSyntheticCloud(ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz, 20, 0.05f);
        std::size_t n = ref_x.size();

        PointCloudSoA soa = {ref_x.data(), ref_y.data(), ref_z.data(), n};

        SpatialHash hash;
        hash.setInputCloud(soa, 0.1f);
        hash.build();

        TrackingConfig config;
        config.icp_max_iterations = 15;
        config.correspondence_max_dist = 0.3f;

        ICPRegistration icp;
        SE3Transform recovered_T;
        int iterations;
        float error;
        double t_corr, t_res, t_red, t_solve;

        icp.align(soa, soa,
                  ref_nx.data(), ref_ny.data(), ref_nz.data(),
                  hash, config,
                  recovered_T, iterations, error,
                  t_corr, t_res, t_red, t_solve);

        SE3Transform identity;
        std::cout << "  ICP iterations: " << iterations << std::endl;
        std::cout << "  Final error:    " << error << std::endl;

        if (transformsClose(recovered_T, identity, 1.0f, 0.01f)) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    // Test 4: SE3Transform operations
    {
        std::cout << "\n--- Test 4: SE3Transform operations ---" << std::endl;

        SE3Transform T1 = SE3Transform::fromAxisAngle(0, 0, 0, 1.0f, 2.0f, 3.0f);
        PointXYZ p = {1.0f, 0.0f, 0.0f};
        PointXYZ tp = T1.apply(p);

        bool pass = (std::abs(tp.x - 2.0f) < 0.001f) &&
                    (std::abs(tp.y - 2.0f) < 0.001f) &&
                    (std::abs(tp.z - 3.0f) < 0.001f);

        std::cout << "  Applied (1,0,0) + translation(1,2,3) = ("
                  << tp.x << ", " << tp.y << ", " << tp.z << ")" << std::endl;

        if (pass) {
            std::cout << "  PASS" << std::endl;
            pass_count++;
        } else {
            std::cout << "  FAIL" << std::endl;
            fail_count++;
        }
    }

    std::cout << "\n=============================================" << std::endl;
    std::cout << "  Results: " << pass_count << " PASSED, " << fail_count << " FAILED" << std::endl;
    std::cout << "=============================================" << std::endl;

    return fail_count > 0 ? 1 : 0;
}
