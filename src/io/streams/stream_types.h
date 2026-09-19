#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "core/point_types.h"

namespace rvpoint {

/**
 * @brief Binary Stream Protocol Constants.
 */
constexpr uint32_t kStreamMagic = 0x52565054; // "RVPT" (ASCII little-endian)
constexpr uint32_t kLdp1Magic   = 0x3150444c; // "LDP1" (iPhone TCP raw depth protocol v1)
constexpr uint32_t kLdp2Magic   = 0x3250444c; // "LDP2" (iPhone TCP raw depth + CoreMotion v2)

constexpr int kDefaultUdpPort = 8765;
constexpr int kDefaultTcpPort = 9000;
constexpr size_t kMaxUdpPayloadSize = 65507;

#pragma pack(push, 1)

/**
 * @brief Section 1: CoreMotion Telemetry (Raw iPhone IMU).
 */
struct CoreMotionData {
    float gyro[3] = {0.0f, 0.0f, 0.0f};      // Angular rates [wx, wy, wz] in rad/s
    float accel[3] = {0.0f, 0.0f, 0.0f};     // User linear acceleration [ax, ay, az] in m/s^2
    float gravity[3] = {0.0f, 0.0f, -9.81f}; // Gravity vector [gx, gy, gz] in camera frame
};

static_assert(sizeof(CoreMotionData) == 36, "CoreMotionData must be exactly 36 bytes");

/**
 * @brief Section 2: ARKit VIO Odometry (Camera 6-DoF Pose & Tracking State).
 */
struct ArKitVioData {
    float T_world_cam[16] = {             // 4x4 column-major transformation matrix
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    };
    uint32_t tracking_state = 2;          // 0: NotAvailable, 1: Limited, 2: Normal
};

static_assert(sizeof(ArKitVioData) == 68, "ArKitVioData must be exactly 68 bytes");

/**
 * @brief 132-byte packed binary header for the unified iPhone UDP stream.
 */
struct UdpPacketHeader {
    // Framing metadata
    uint32_t magic = kStreamMagic;        // 0x52565054 ("RVPT")
    uint32_t seq = 0;                     // Frame sequence number
    uint64_t timestamp_ns = 0;            // Monotonic timestamp (nanoseconds)

    // Section 1: CoreMotion
    CoreMotionData coremotion;             // 36 bytes

    // Section 2: ARKit VIO
    ArKitVioData vio;                     // 68 bytes

    // Section 3: LiDAR dToF Metadata
    uint32_t num_points = 0;              // Points in this packet
    uint16_t chunk_idx = 0;               // Chunk index (0-based)
    uint16_t total_chunks = 1;            // Total chunks for this frame
    uint32_t reserved = 0;                // Alignment padding
};

static_assert(sizeof(UdpPacketHeader) == 132, "UdpPacketHeader must be exactly 132 bytes");

/**
 * @brief Legacy iPhone app LDP1 header (88 bytes header + 4 bytes magic = 96 bytes).
 */
struct Ldp1Header {
    uint32_t magic;                       // "LDP1" (0x3150444c)
    uint32_t frame_idx;                   // Frame index
    uint32_t width;                       // 256
    uint32_t height;                      // 192
    float fx, fy, cx, cy;                 // Intrinsics
    float pose[16];                       // 4x4 camera pose (row-major with inverted Y/Z)
};

static_assert(sizeof(Ldp1Header) == 96, "Ldp1Header must be exactly 96 bytes");

/**
 * @brief Extended iPhone app LDP2 header (96 bytes LDP1 + 40 bytes telemetry = 136 bytes).
 */
struct Ldp2Header {
    uint32_t magic;                       // "LDP2" (0x3250444c)
    uint32_t frame_idx;                   // Frame index
    uint32_t width;                       // 256
    uint32_t height;                      // 192
    float fx, fy, cx, cy;                 // Intrinsics
    float pose[16];                       // 4x4 camera pose
    uint32_t tracking_state;              // 0: NotAvailable, 1: Limited, 2: Normal
    float gyro[3];                        // wx, wy, wz (rad/s)
    float accel[3];                       // ax, ay, az (m/s^2)
    float gravity[3];                     // gx, gy, gz (m/s^2)
};

static_assert(sizeof(Ldp2Header) == 136, "Ldp2Header must be exactly 136 bytes");

#pragma pack(pop)

/**
 * @brief Trajectory synthesis mode for MockPcdStreamSource.
 */
enum class MockTrajectoryMode {
    Stationary,       // Zero motion (identity pose)
    ForwardStraight,  // Constant forward velocity (vx)
    CircularArc       // Constant forward velocity (vx) and yaw rate (wz)
};

/**
 * @brief Unified container for a single incoming stream frame.
 */
struct StreamFrame {
    uint32_t seq = 0;
    uint64_t timestamp_ns = 0;

    // Section 1: CoreMotion (Raw IMU)
    CoreMotionData coremotion;

    // Section 2: ARKit VIO (6-DoF Pose)
    ArKitVioData vio;

    // Section 3: LiDAR dToF Point Cloud
    PointCloud cloud;

    void get_translation(float& x, float& y, float& z) const noexcept {
        x = vio.T_world_cam[12];
        y = vio.T_world_cam[13];
        z = vio.T_world_cam[14];
    }

    float get_yaw() const noexcept {
        return std::atan2(vio.T_world_cam[1], vio.T_world_cam[0]);
    }
};

} // namespace rvpoint

