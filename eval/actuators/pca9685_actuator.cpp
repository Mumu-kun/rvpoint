#include "pca9685_actuator.h"

#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <cstring>
#include <algorithm>

#if defined(__linux__)
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#endif

namespace rvpoint {

#ifndef I2C_SLAVE
#define I2C_SLAVE 0x0703
#endif

uint64_t PCA9685Actuator::get_current_time_ns() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
}

static float clamp_f(float val, float min_v, float max_v) {
    return std::max(min_v, std::min(val, max_v));
}

bool PCA9685Config::load_from_json(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;
    std::string content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());

    auto extract_float = [&](const std::string& key, float def) -> float {
        auto pos = content.find("\"" + key + "\"");
        if (pos == std::string::npos) return def;
        auto colon = content.find(':', pos);
        if (colon == std::string::npos) return def;
        return std::stof(content.substr(colon + 1));
    };

    auto extract_int = [&](const std::string& key, int def) -> int {
        auto pos = content.find("\"" + key + "\"");
        if (pos == std::string::npos) return def;
        auto colon = content.find(':', pos);
        if (colon == std::string::npos) return def;
        return std::stoi(content.substr(colon + 1));
    };

    bus = extract_int("bus", 4);
    deadband = extract_float("deadband", 0.12f);
    freq_hz = extract_float("freq_hz", 200.0f);
    return true;
}

bool PCA9685Config::save_to_json(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) return false;
    file << "{\n"
         << "  \"bus\": " << bus << ",\n"
         << "  \"address\": \"" << address << "\",\n"
         << "  \"freq_hz\": " << freq_hz << ",\n"
         << "  \"mapping\": \"" << mapping << "\",\n"
         << "  \"deadband\": " << deadband << ",\n"
         << "  \"enable_watchdog\": " << (enable_watchdog ? "true" : "false") << ",\n"
         << "  \"watchdog_timeout_ms\": " << watchdog_timeout_ms << ",\n"
         << "  \"wheels\": {\n"
         << "    \"FL\": { \"motor\": " << fl.motor << ", \"invert\": " << (fl.invert ? "true" : "false") << ", \"trim\": " << fl.trim << " },\n"
         << "    \"FR\": { \"motor\": " << fr.motor << ", \"invert\": " << (fr.invert ? "true" : "false") << ", \"trim\": " << fr.trim << " },\n"
         << "    \"RL\": { \"motor\": " << rl.motor << ", \"invert\": " << (rl.invert ? "true" : "false") << ", \"trim\": " << rl.trim << " },\n"
         << "    \"RR\": { \"motor\": " << rr.motor << ", \"invert\": " << (rr.invert ? "true" : "false") << ", \"trim\": " << rr.trim << " }\n"
         << "  }\n"
         << "}\n";
    return true;
}

PCA9685Actuator::PCA9685Actuator(const PCA9685Config& config)
    : config_(config) {
    initialize_hardware();
}

PCA9685Actuator::~PCA9685Actuator() {
    shutdown_hardware();
}

bool PCA9685Actuator::initialize_hardware() {
#if defined(__linux__)
    std::string dev_path = "/dev/i2c-" + std::to_string(config_.bus);
    i2c_fd_ = open(dev_path.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
        // Try fallback bus 1
        i2c_fd_ = open("/dev/i2c-1", O_RDWR);
    }

    if (i2c_fd_ >= 0) {
        int addr = 0x60;
        try {
            addr = std::stoi(config_.address, nullptr, 0);
        } catch (...) {}
        if (ioctl(i2c_fd_, I2C_SLAVE, addr) >= 0) {
            is_simulated_ = false;
            // Mode 1: Auto-increment + allcall
            uint8_t init_mode[2] = {0x00, 0x21};
            write(i2c_fd_, init_mode, 2);
            // Mode 2: Totem pole
            uint8_t out_mode[2] = {0x01, 0x04};
            write(i2c_fd_, out_mode, 2);
        } else {
            close(i2c_fd_);
            i2c_fd_ = -1;
            is_simulated_ = true;
        }
    } else {
        is_simulated_ = true;
    }
#else
    is_simulated_ = true;
#endif

    running_ = true;
    worker_thread_ = std::thread(&PCA9685Actuator::worker_loop, this);
    return true;
}

void PCA9685Actuator::shutdown_hardware() {
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }
    emergency_brake();
    if (i2c_fd_ >= 0) {
        close(i2c_fd_);
        i2c_fd_ = -1;
    }
}

void PCA9685Actuator::hw_set_duty(int channel, float duty) {
    if (is_simulated_ || i2c_fd_ < 0 || channel < 0 || channel > 15) return;
    duty = clamp_f(duty, 0.0f, 1.0f);
    uint8_t reg_base = 0x06 + 4 * channel;

    if (duty <= 0.0f) {
        uint8_t payload[5] = {reg_base, 0, 0, 0, 0x10};
        (void)write(i2c_fd_, payload, 5);
    } else if (duty >= 1.0f) {
        uint8_t payload[5] = {reg_base, 0, 0x10, 0, 0};
        (void)write(i2c_fd_, payload, 5);
    } else {
        uint16_t off_count = static_cast<uint16_t>(duty * 4095.0f);
        uint8_t payload[5] = {
            reg_base,
            0, 0,
            static_cast<uint8_t>(off_count & 0xFF),
            static_cast<uint8_t>((off_count >> 8) & 0x0F)
        };
        (void)write(i2c_fd_, payload, 5);
    }
}

void PCA9685Actuator::hw_drive_motor(int motor_num, float duty, const std::string& dir) {
    int pwm_ch = 8, in1_ch = 10, in2_ch = 9;
    switch (motor_num) {
        case 1: pwm_ch = 8; in2_ch = 9; in1_ch = 10; break;
        case 2: pwm_ch = 13; in2_ch = 12; in1_ch = 11; break;
        case 3: pwm_ch = 2; in2_ch = 3; in1_ch = 4; break;
        case 4: pwm_ch = 7; in2_ch = 6; in1_ch = 5; break;
        default: return;
    }

    if (duty <= 0.01f) {
        hw_set_duty(pwm_ch, 0.0f);
        hw_set_duty(in1_ch, 0.0f);
        hw_set_duty(in2_ch, 0.0f);
        return;
    }

    if (dir == "forward") {
        hw_set_duty(in1_ch, 1.0f);
        hw_set_duty(in2_ch, 0.0f);
        hw_set_duty(pwm_ch, duty);
    } else {
        hw_set_duty(in1_ch, 0.0f);
        hw_set_duty(in2_ch, 1.0f);
        hw_set_duty(pwm_ch, duty);
    }
}

void PCA9685Actuator::emergency_brake() {
    emergency_stop_ = true;
    for (int m = 1; m <= 4; ++m) {
        hw_drive_motor(m, 0.0f, "forward");
    }
}

bool PCA9685Actuator::is_emergency_stopped() const noexcept {
    return emergency_stop_.load();
}

void PCA9685Actuator::reset_emergency_stop() {
    emergency_stop_ = false;
    watchdog_tripped_ = false;
    last_command_time_ns_ = get_current_time_ns();
}

void PCA9685Actuator::set_wheel_duties(float fl, float fr, float rl, float rr) {
    if (emergency_stop_) return;
    last_command_time_ns_ = get_current_time_ns();
    watchdog_tripped_ = false;

    auto apply_wheel = [&](WheelId wid, float raw) {
        const auto& cfg = config_.get_wheel_config(wid);
        bool pos = (raw >= 0.0f);
        if (cfg.invert) pos = !pos;
        float mag = std::abs(raw);
        float duty = (mag < 0.01f) ? 0.0f : clamp_f(cfg.deadband_forward + (1.0f - cfg.deadband_forward) * mag * cfg.trim, 0.0f, 1.0f);
        hw_drive_motor(cfg.motor, duty, pos ? "forward" : "reverse");
    };

    apply_wheel(WheelId::FL, fl);
    apply_wheel(WheelId::FR, fr);
    apply_wheel(WheelId::RL, rl);
    apply_wheel(WheelId::RR, rr);
}

void PCA9685Actuator::set_duty_cycles(float duty_left, float duty_right) {
    set_wheel_duties(duty_left, duty_right, duty_left, duty_right);
}

void PCA9685Actuator::worker_loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        if (!watchdog_enabled_ || emergency_stop_) continue;

        uint64_t elapsed_ms = (get_current_time_ns() - last_command_time_ns_) / 1000000;
        if (elapsed_ms > config_.watchdog_timeout_ms && !watchdog_tripped_) {
            watchdog_tripped_ = true;
            for (int m = 1; m <= 4; ++m) {
                hw_drive_motor(m, 0.0f, "forward");
            }
        }
    }
}

} // namespace rvpoint
