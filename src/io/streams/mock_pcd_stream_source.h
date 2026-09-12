#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "io/streams/stream_source.h"

namespace rvpoint {

/**
 * @brief Replay adapter reading local PCD files with synthetic CoreMotion & ARKit VIO.
 *
 * Provides deterministic offline regression and testing for perception pipelines.
 */
class MockPcdStreamSource : public StreamSource {
public:
    MockPcdStreamSource(std::vector<std::string> pcd_paths,
                        double fps = 30.0,
                        bool loop = true,
                        MockTrajectoryMode mode = MockTrajectoryMode::ForwardStraight,
                        float linear_speed_mps = 0.4f,
                        float yaw_rate_radps = 0.2f);

    explicit MockPcdStreamSource(const std::string& single_pcd_path,
                                 double fps = 30.0,
                                 bool loop = true,
                                 MockTrajectoryMode mode = MockTrajectoryMode::ForwardStraight,
                                 float linear_speed_mps = 0.4f,
                                 float yaw_rate_radps = 0.2f);

    bool poll_frame(StreamFrame& out) override;

    void reset();

    size_t file_count() const noexcept { return paths_.size(); }
    size_t current_index() const noexcept { return current_idx_; }

private:
    std::vector<std::string> paths_;
    double fps_ = 30.0;
    bool loop_ = true;
    MockTrajectoryMode mode_ = MockTrajectoryMode::ForwardStraight;
    float vx_ = 0.4f;
    float wz_ = 0.2f;

    uint64_t frame_interval_ns_ = 33333333; // ~30 Hz
    size_t current_idx_ = 0;
    uint32_t seq_counter_ = 0;
    uint64_t current_time_ns_ = 0;

    float sim_x_ = 0.0f;
    float sim_y_ = 0.0f;
    float sim_yaw_ = 0.0f;

    void synthesize_telemetry(StreamFrame& frame, double t_sec);
};

} // namespace rvpoint

