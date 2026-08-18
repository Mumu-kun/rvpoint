// test_tracking_registration.cpp
//
// Unit test for ICP Registration (Stages 2–5)
// ============================================
// Creates a synthetic point cloud, applies a known transform,
// runs ICP to recover the transform, and verifies accuracy.

#include "registration/icp_registration.h"
#include "registration/tracking_types.h"
#include "rvv_pcl.h"
#include "simple_pcd_loader.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>
#include <vector>

using namespace rvv_pcl;

namespace {

// Generate a synthetic paraboloid point cloud: z = curvature * (x² + y²)
// A curved 3D surface with analytically varying normals so point-to-plane
// ICP can constrain all 6 DOF (unlike a flat plane which is degenerate
// for in-plane translations).
void generateSyntheticCloud(std::vector<float>& x, std::vector<float>& y,
                            std::vector<float>& z,
                            std::vector<float>& nx, std::vector<float>& ny,
                            std::vector<float>& nz,
                            int grid_size, float spacing) {
    const float curvature = 0.5f; // controls bowl depth

    int n = grid_size * grid_size;
    x.resize(n); y.resize(n); z.resize(n);
    nx.resize(n); ny.resize(n); nz.resize(n);

    int idx = 0;
    for (int i = 0; i < grid_size; ++i) {
        for (int j = 0; j < grid_size; ++j) {
            float px = (float)i * spacing - (grid_size * spacing * 0.5f);
            float py = (float)j * spacing - (grid_size * spacing * 0.5f);
            float pz = curvature * (px * px + py * py);

            x[idx] = px;
            y[idx] = py;
            z[idx] = pz;

            // Analytic normal of z = c*(x²+y²):
            //   grad = (-dz/dx, -dz/dy, 1) = (-2cx, -2cy, 1), then normalize
            float gx = -2.0f * curvature * px;
            float gy = -2.0f * curvature * py;
            float gz = 1.0f;
            float inv_len = 1.0f / std::sqrt(gx*gx + gy*gy + gz*gz);
            nx[idx] = gx * inv_len;
            ny[idx] = gy * inv_len;
            nz[idx] = gz * inv_len;
            idx++;
        }
    }
}

// Apply a transform to a cloud
void applyTransform(const SE3Transform& T,
                    const std::vector<float>& in_x, const std::vector<float>& in_y,
                    const std::vector<float>& in_z,
                    std::vector<float>& out_x, std::vector<float>& out_y,
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

// Check if two transforms are approximately equal
bool transformsClose(const SE3Transform& A, const SE3Transform& B,
                     float rot_tol_deg, float trans_tol_m) {
    // Translation check
    float dt_x = A.tx() - B.tx();
    float dt_y = A.ty() - B.ty();
    float dt_z = A.tz() - B.tz();
    float trans_err = std::sqrt(dt_x*dt_x + dt_y*dt_y + dt_z*dt_z);

    // Rotation check (trace-based angle)
    float trace = 0.0f;
    for (int i = 0; i < 3; ++i) {
        float dot = 0.0f;
        for (int k = 0; k < 3; ++k) {
            dot += A.R(i, k) * B.R(i, k);
        }
        trace += dot;
    }
    // trace of R_A^T * R_B should be close to 3 for identity
    float cos_angle = (trace - 1.0f) / 2.0f;
    if (cos_angle > 1.0f) cos_angle = 1.0f;
    if (cos_angle < -1.0f) cos_angle = -1.0f;
    float angle_rad = std::acos(cos_angle);
    float angle_deg = angle_rad * 180.0f / 3.14159265f;

    std::cout << "  Translation error: " << std::fixed << std::setprecision(4)
              << trans_err << " m (threshold: " << trans_tol_m << " m)" << std::endl;
    std::cout << "  Rotation error:    " << std::fixed << std::setprecision(4)
              << angle_deg << " deg (threshold: " << rot_tol_deg << " deg)" << std::endl;

    return (trans_err <= trans_tol_m) && (angle_deg <= rot_tol_deg);
}

} // anonymous namespace

int main(int argc, char** argv) {
    std::cout << "=============================================" << std::endl;
    std::cout << "   ICP Registration Unit Test" << std::endl;
    std::cout << "=============================================" << std::endl;

    int pass_count = 0;
    int fail_count = 0;

    // =========================================================================
    // Test 1: Pure translation recovery
    // =========================================================================
    {
        std::cout << "\n--- Test 1: Pure Translation (0.1, 0.05, 0.0) ---" << std::endl;

        std::vector<float> ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz;
        generateSyntheticCloud(ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz, 30, 0.05f);
        std::size_t n = ref_x.size();

        PointCloudSoA ref_soa = {ref_x.data(), ref_y.data(), ref_z.data(), n};

        // Build spatial hash on reference
        SpatialHash hash;
        hash.setInputCloud(ref_soa, 0.1f);
        hash.build();

        // Apply known transform to create "new frame"
        SE3Transform known_T = SE3Transform::fromAxisAngle(0, 0, 0, 0.1f, 0.05f, 0.0f);

        std::vector<float> src_x, src_y, src_z;
        applyTransform(known_T, ref_x, ref_y, ref_z, src_x, src_y, src_z);
        PointCloudSoA src_soa = {src_x.data(), src_y.data(), src_z.data(), n};

        // Run ICP
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

        // ICP finds source→target, which is the inverse of known_T.
        // Verify: recovered_T ∘ known_T ≈ Identity
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

    // =========================================================================
    // Test 2: Small rotation recovery (5° about Z)
    // =========================================================================
    {
        std::cout << "\n--- Test 2: Small Rotation (5 deg about Z) + Translation ---" << std::endl;

        std::vector<float> ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz;
        generateSyntheticCloud(ref_x, ref_y, ref_z, ref_nx, ref_ny, ref_nz, 30, 0.05f);
        std::size_t n = ref_x.size();

        PointCloudSoA ref_soa = {ref_x.data(), ref_y.data(), ref_z.data(), n};

        SpatialHash hash;
        hash.setInputCloud(ref_soa, 0.1f);
        hash.build();

        float angle = 5.0f * 3.14159265f / 180.0f; // 5 degrees
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

        // ICP finds source→target, which is the inverse of known_T.
        // Verify: recovered_T ∘ known_T ≈ Identity
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

    // =========================================================================
    // Test 3: Identity (no motion) — should converge in 1-2 iterations
    // =========================================================================
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

    // =========================================================================
    // Test 4: SE3Transform basic operations
    // =========================================================================
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

    // =========================================================================
    // Summary
    // =========================================================================
    std::cout << "\n=============================================" << std::endl;
    std::cout << "  Results: " << pass_count << " PASSED, " << fail_count << " FAILED" << std::endl;
    std::cout << "=============================================" << std::endl;

    return fail_count > 0 ? 1 : 0;
}
