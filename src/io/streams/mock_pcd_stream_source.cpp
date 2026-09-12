#include "io/streams/mock_pcd_stream_source.h"

#include <cmath>
#include <iostream>
#include <utility>

#include "io/simple_pcd_loader.h"

namespace rvpoint {

MockPcdStreamSource::MockPcdStreamSource(std::vector<std::string> pcd_paths,
                                         double fps,
                                         bool loop,
                                         MockTrajectoryMode mode,
                                         float linear_speed_mps,
                                         float yaw_rate_radps)
    : paths_(std::move(pcd_paths)),
      fps_(fps),
      loop_(loop),
      mode_(mode),
      vx_(linear_speed_mps),
      wz_(yaw_rate_radps) {
    frame_interval_ns_ = (fps_ > 0.0) ? static_cast<uint64_t>(1e9 / fps_) : 0;
}

MockPcdStreamSource::MockPcdStreamSource(const std::string& single_pcd_path,
                                         double fps,
                                         bool loop,
                                         MockTrajectoryMode mode,
                                         float linear_speed_mps,
                                         float yaw_rate_radps)
    : MockPcdStreamSource(std::vector<std::string>{single_pcd_path},
                          fps, loop, mode, linear_speed_mps, yaw_rate_radps) {}

bool MockPcdStreamSource::poll_frame(StreamFrame& out) {
    if (paths_.empty()) return false;
    if (current_idx_ >= paths_.size()) {
        if (!loop_) return false;
        current_idx_ = 0;
    }

    const std::string& path = paths_[current_idx_++];
    if (!loadPCD(path, out.cloud)) {
        std::cerr << "[MockPcdStreamSource] Warning: Failed to load " << path << std::endl;
        return false;
    }

    out.seq = seq_counter_++;
    out.timestamp_ns = current_time_ns_;
    synthesize_telemetry(out, current_time_ns_ * 1e-9);

    current_time_ns_ += frame_interval_ns_;
    return true;
}

void MockPcdStreamSource::reset() {
    current_idx_ = 0;
    seq_counter_ = 0;
    current_time_ns_ = 0;
    sim_x_ = 0.0f;
    sim_y_ = 0.0f;
    sim_yaw_ = 0.0f;
}

void MockPcdStreamSource::synthesize_telemetry(StreamFrame& frame, double t_sec) {
    float* T = frame.vio.T_world_cam;
    for (int i = 0; i < 16; ++i) T[i] = 0.0f;
    T[0] = 1.0f; T[5] = 1.0f; T[10] = 1.0f; T[15] = 1.0f;
    frame.vio.tracking_state = 2; // Normal

    // Section 1: CoreMotion
    frame.coremotion.gyro[0] = 0.0f;
    frame.coremotion.gyro[1] = 0.0f;
    frame.coremotion.gyro[2] = 0.0f;
    frame.coremotion.accel[0] = 0.0f;
    frame.coremotion.accel[1] = 0.0f;
    frame.coremotion.accel[2] = 0.0f;
    frame.coremotion.gravity[0] = 0.0f;
    frame.coremotion.gravity[1] = 0.0f;
    frame.coremotion.gravity[2] = -9.81f; // Earth gravity downwards

    if (mode_ == MockTrajectoryMode::Stationary) {
        return;
    }

    if (mode_ == MockTrajectoryMode::ForwardStraight) {
        sim_x_ = static_cast<float>(vx_ * t_sec);
        sim_y_ = 0.0f;
        sim_yaw_ = 0.0f;
        frame.coremotion.gyro[2] = 0.0f;
    } else if (mode_ == MockTrajectoryMode::CircularArc) {
        sim_yaw_ = static_cast<float>(wz_ * t_sec);
        frame.coremotion.gyro[2] = wz_; // Raw yaw rate from gyro
        if (std::abs(wz_) > 1e-5f) {
            float R = vx_ / wz_;
            sim_x_ = R * std::sin(sim_yaw_);
            sim_y_ = R * (1.0f - std::cos(sim_yaw_));
            frame.coremotion.accel[1] = vx_ * wz_;
        } else {
            sim_x_ = static_cast<float>(vx_ * t_sec);
            sim_y_ = 0.0f;
        }
    }

    float c = std::cos(sim_yaw_);
    float s = std::sin(sim_yaw_);

    T[0] = c;   T[4] = -s;  T[8]  = 0.0f; T[12] = sim_x_;
    T[1] = s;   T[5] = c;   T[9]  = 0.0f; T[13] = sim_y_;
    T[2] = 0.0f;T[6] = 0.0f;T[10] = 1.0f; T[14] = 0.0f;
    T[3] = 0.0f;T[7] = 0.0f;T[11] = 0.0f; T[15] = 1.0f;
}

} // namespace rvpoint

