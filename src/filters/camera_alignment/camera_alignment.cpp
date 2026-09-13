#include "filters/camera_alignment/camera_alignment.h"

#include <cmath>

#if defined(__riscv_vector)
#include <riscv_vector.h>
#endif

namespace rvpoint {

namespace {
constexpr float kDegToRad = 0.017453292519943295f; // pi / 180

static void apply_transform(const PointCloud& cam_cloud,
                            const float R[9],
                            const float t[3],
                            PointCloud& body_cloud) {
    const std::size_t n = cam_cloud.size();
    body_cloud.resize(n);
    if (n == 0) return;

#if defined(__riscv_vector)
    const float r00 = R[0], r01 = R[1], r02 = R[2], tx = t[0];
    const float r10 = R[3], r11 = R[4], r12 = R[5], ty = t[1];
    const float r20 = R[6], r21 = R[7], r22 = R[8], tz = t[2];

    std::size_t i = 0;
    while (i < n) {
        std::size_t vl = __riscv_vsetvl_e32m8(n - i);
        vfloat32m8_t vx = __riscv_vle32_v_f32m8(cam_cloud.x.data() + i, vl);
        vfloat32m8_t vy = __riscv_vle32_v_f32m8(cam_cloud.y.data() + i, vl);
        vfloat32m8_t vz = __riscv_vle32_v_f32m8(cam_cloud.z.data() + i, vl);

        vfloat32m8_t bx = __riscv_vfmv_v_f_f32m8(tx, vl);
        bx = __riscv_vfmacc_vf_f32m8(bx, r00, vx, vl);
        bx = __riscv_vfmacc_vf_f32m8(bx, r01, vy, vl);
        bx = __riscv_vfmacc_vf_f32m8(bx, r02, vz, vl);

        vfloat32m8_t by = __riscv_vfmv_v_f_f32m8(ty, vl);
        by = __riscv_vfmacc_vf_f32m8(by, r10, vx, vl);
        by = __riscv_vfmacc_vf_f32m8(by, r11, vy, vl);
        by = __riscv_vfmacc_vf_f32m8(by, r12, vz, vl);

        vfloat32m8_t bz = __riscv_vfmv_v_f_f32m8(tz, vl);
        bz = __riscv_vfmacc_vf_f32m8(bz, r20, vx, vl);
        bz = __riscv_vfmacc_vf_f32m8(bz, r21, vy, vl);
        bz = __riscv_vfmacc_vf_f32m8(bz, r22, vz, vl);

        __riscv_vse32_v_f32m8(body_cloud.x.data() + i, bx, vl);
        __riscv_vse32_v_f32m8(body_cloud.y.data() + i, by, vl);
        __riscv_vse32_v_f32m8(body_cloud.z.data() + i, bz, vl);
        i += vl;
    }
#else
    for (std::size_t i = 0; i < n; ++i) {
        const float cx = cam_cloud.x[i];
        const float cy = cam_cloud.y[i];
        const float cz = cam_cloud.z[i];
        body_cloud.x[i] = R[0] * cx + R[1] * cy + R[2] * cz + t[0];
        body_cloud.y[i] = R[3] * cx + R[4] * cy + R[5] * cz + t[1];
        body_cloud.z[i] = R[6] * cx + R[7] * cy + R[8] * cz + t[2];
    }
#endif
}
} // namespace

void CameraAlignment::compute_extrinsics(const float gravity[3], float R[9], float t[3]) const {
    float gx = gravity[0];
    float gy = gravity[1];
    float gz = gravity[2];
    float g_norm = std::sqrt(gx * gx + gy * gy + gz * gz);

    float uz_x = 0.0f, uz_y = 0.0f, uz_z = 1.0f; // Body Up vector in camera frame

    if (g_norm >= 1.0f) {
        // CoreMotion gravity points downward towards Earth; True UP is -g
        uz_x = -gx / g_norm;
        uz_y = -gy / g_norm;
        uz_z = -gz / g_norm;
    } else {
        // Fallback to nominal pitch tilt (pitched down towards floor)
        float pitch_rad = params_.camera_pitch_deg * kDegToRad;
        uz_x = 0.0f;
        uz_y = -std::cos(pitch_rad);
        uz_z = -std::sin(pitch_rad);
    }

    // Camera optical axis (+Z) projected onto horizontal plane perpendicular to uz
    float dot = uz_z; // v_opt . uz
    float fx = -dot * uz_x;
    float fy = -dot * uz_y;
    float fz = 1.0f - dot * uz_z;

    float f_len = std::sqrt(fx * fx + fy * fy + fz * fz);
    float ux_x = 0.0f, ux_y = 0.0f, ux_z = 1.0f;
    if (f_len > 1e-4f) {
        ux_x = fx / f_len;
        ux_y = fy / f_len;
        ux_z = fz / f_len;
    } else {
        ux_x = 0.0f;
        ux_y = 0.0f;
        ux_z = 1.0f;
    }

    // Body Left vector: uy = uz x ux
    float uy_x = uz_y * ux_z - uz_z * ux_y;
    float uy_y = uz_z * ux_x - uz_x * ux_z;
    float uy_z = uz_x * ux_y - uz_y * ux_x;

    R[0] = ux_x; R[1] = ux_y; R[2] = ux_z;
    R[3] = uy_x; R[4] = uy_y; R[5] = uy_z;
    R[6] = uz_x; R[7] = uz_y; R[8] = uz_z;

    t[0] = params_.mount_x_offset_m;
    t[1] = params_.mount_y_offset_m;
    t[2] = params_.mount_height_m;
}

bool CameraAlignment::is_ground_plane_valid(const float gravity[3],
                                            const PlaneModel& ground_plane,
                                            float* out_vertical_height) const {
    float gx = gravity[0], gy = gravity[1], gz = gravity[2];
    float g_norm = std::sqrt(gx * gx + gy * gy + gz * gz);
    float uz_x = 0.0f, uz_y = 0.0f, uz_z = 1.0f;
    if (g_norm >= 1.0f) {
        uz_x = -gx / g_norm;
        uz_y = -gy / g_norm;
        uz_z = -gz / g_norm;
    } else {
        float pitch_rad = params_.camera_pitch_deg * kDegToRad;
        uz_x = 0.0f;
        uz_y = -std::cos(pitch_rad);
        uz_z = -std::sin(pitch_rad);
    }

    float a = ground_plane.a, b = ground_plane.b, c = ground_plane.c, d = ground_plane.d;
    float p_norm = std::sqrt(a * a + b * b + c * c);
    if (p_norm < 1e-6f) return false;

    a /= p_norm; b /= p_norm; c /= p_norm; d /= p_norm;

    // Orient normal upwards (along uz)
    float dot_uz = a * uz_x + b * uz_y + c * uz_z;
    if (dot_uz < 0.0f) {
        a = -a; b = -b; c = -c; d = -d;
        dot_uz = -dot_uz;
    }

    // Check angular threshold: cos(delta_theta) >= cos(max_ground_angle_deg)
    float cos_max = std::cos(params_.max_ground_angle_deg * kDegToRad);
    if (dot_uz < cos_max) {
        return false; // Rejected (wall, ceiling, or steep obstacle)
    }

    // Vertical height along gravity: h_vert = d / dot_uz
    if (out_vertical_height) {
        *out_vertical_height = d / dot_uz;
    }
    return true;
}

void CameraAlignment::compute_extrinsics(const PlaneModel& ground_plane, float R[9], float t[3]) const {
    // When live gravity is not supplied, use nominal gravity from camera_pitch_deg
    float pitch_rad = params_.camera_pitch_deg * kDegToRad;
    float nominal_g[3] = {0.0f, 9.81f * std::cos(pitch_rad), 9.81f * std::sin(pitch_rad)};
    compute_extrinsics(nominal_g, ground_plane, R, t);
}

void CameraAlignment::compute_extrinsics(const float gravity[3],
                                         const PlaneModel& ground_plane,
                                         float R[9],
                                         float t[3]) const {
    // Rotation is ALWAYS strictly aligned with gravity (Z_body = Up = -g)
    compute_extrinsics(gravity, R, t);

    // Height offset: validate ground plane against gravity
    float h_vert = 0.0f;
    if (is_ground_plane_valid(gravity, ground_plane, &h_vert)) {
        t[2] = h_vert; // Valid ground detected: calibrate vertical height
    } else {
        t[2] = params_.mount_height_m; // Rejected: fall back to nominal mount height
    }
}

void CameraAlignment::transform_to_body(const PointCloud& cam_cloud,
                                        const float gravity[3],
                                        PointCloud& body_cloud) const {
    float R[9], t[3];
    compute_extrinsics(gravity, R, t);
    apply_transform(cam_cloud, R, t, body_cloud);
}

void CameraAlignment::transform_to_body(const PointCloud& cam_cloud,
                                        const PlaneModel& ground_plane,
                                        PointCloud& body_cloud) const {
    float R[9], t[3];
    compute_extrinsics(ground_plane, R, t);
    apply_transform(cam_cloud, R, t, body_cloud);
}

void CameraAlignment::transform_to_body(const PointCloud& cam_cloud,
                                        const float gravity[3],
                                        const PlaneModel& ground_plane,
                                        PointCloud& body_cloud) const {
    float R[9], t[3];
    compute_extrinsics(gravity, ground_plane, R, t);
    apply_transform(cam_cloud, R, t, body_cloud);
}

} // namespace rvpoint
