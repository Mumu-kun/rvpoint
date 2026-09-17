#include "dual_l298n_actuator.h"

#include <cmath>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>

#if defined(__linux__)
#include <sys/types.h>
#endif

namespace rvpoint {

static inline float clamp_val(float v, float min_v, float max_v) {
    return (v < min_v) ? min_v : (v > max_v) ? max_v : v;
}

uint64_t DualL298NActuator::get_current_time_ns() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

// ---------------------------------------------------------------------------
// Configuration JSON Serialization / Deserialization (Zero External Deps)
// ---------------------------------------------------------------------------

static std::string extract_json_str(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    size_t colon = json.find(':', pos);
    if (colon == std::string::npos) return "";
    size_t quote1 = json.find('"', colon);
    if (quote1 == std::string::npos) return "";
    size_t quote2 = json.find('"', quote1 + 1);
    if (quote2 == std::string::npos) return "";
    return json.substr(quote1 + 1, quote2 - quote1 - 1);
}

static double extract_json_num(const std::string& json, const std::string& key, double default_val) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return default_val;
    size_t colon = json.find(':', pos);
    if (colon == std::string::npos) return default_val;
    size_t val_start = json.find_first_not_of(" \t\r\n", colon + 1);
    if (val_start == std::string::npos) return default_val;
    size_t val_end = json.find_first_of(",}\r\n", val_start);
    std::string token = json.substr(val_start, (val_end == std::string::npos) ? std::string::npos : (val_end - val_start));
    try {
        return std::stod(token);
    } catch (...) {
        return default_val;
    }
}

static bool extract_json_bool(const std::string& json, const std::string& key, bool default_val) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return default_val;
    size_t colon = json.find(':', pos);
    if (colon == std::string::npos) return default_val;
    size_t val_start = json.find_first_not_of(" \t\r\n", colon + 1);
    if (val_start == std::string::npos) return default_val;
    if (json.compare(val_start, 4, "true") == 0) return true;
    if (json.compare(val_start, 5, "false") == 0) return false;
    return default_val;
}

static std::string extract_subobject(const std::string& json, const std::string& obj_name) {
    size_t pos = json.find("\"" + obj_name + "\"");
    if (pos == std::string::npos) return "";
    size_t open_brace = json.find('{', pos);
    if (open_brace == std::string::npos) return "";
    int depth = 1;
    size_t i = open_brace + 1;
    while (i < json.size() && depth > 0) {
        if (json[i] == '{') depth++;
        else if (json[i] == '}') depth--;
        i++;
    }
    return json.substr(open_brace, i - open_brace);
}

static void parse_wheel_block(const std::string& block, MotorPinConfig& cfg) {
    cfg.channel_index = static_cast<int>(extract_json_num(block, "channel", cfg.channel_index));
    cfg.pwm_pin = static_cast<int>(extract_json_num(block, "pwm_pin", cfg.pwm_pin));
    cfg.sysfs_pwm_path = extract_json_str(block, "sysfs_pwm_path");
    cfg.in1_pin = static_cast<int>(extract_json_num(block, "in1_pin", cfg.in1_pin));
    cfg.in2_pin = static_cast<int>(extract_json_num(block, "in2_pin", cfg.in2_pin));
    cfg.invert = extract_json_bool(block, "invert", cfg.invert);
    cfg.trim = static_cast<float>(extract_json_num(block, "trim", cfg.trim));
}

bool DualL298NConfig::load_from_json(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) return false;

    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();

    use_hardware_pwm = extract_json_bool(content, "use_hardware_pwm", use_hardware_pwm);
    pwm_frequency_hz = static_cast<int>(extract_json_num(content, "pwm_frequency_hz", pwm_frequency_hz));
    deadband = static_cast<float>(extract_json_num(content, "deadband", deadband));
    enable_watchdog = extract_json_bool(content, "enable_watchdog", enable_watchdog);
    watchdog_timeout_ms = static_cast<uint64_t>(extract_json_num(content, "watchdog_timeout_ms", watchdog_timeout_ms));

    std::string fl_block = extract_subobject(content, "FL");
    if (!fl_block.empty()) parse_wheel_block(fl_block, fl);

    std::string fr_block = extract_subobject(content, "FR");
    if (!fr_block.empty()) parse_wheel_block(fr_block, fr);

    std::string rl_block = extract_subobject(content, "RL");
    if (!rl_block.empty()) parse_wheel_block(rl_block, rl);

    std::string rr_block = extract_subobject(content, "RR");
    if (!rr_block.empty()) parse_wheel_block(rr_block, rr);

    return true;
}

bool DualL298NConfig::save_to_json(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) return false;

    file << "{\n";
    file << "  \"use_hardware_pwm\": " << (use_hardware_pwm ? "true" : "false") << ",\n";
    file << "  \"pwm_frequency_hz\": " << pwm_frequency_hz << ",\n";
    file << "  \"deadband\": " << deadband << ",\n";
    file << "  \"enable_watchdog\": " << (enable_watchdog ? "true" : "false") << ",\n";
    file << "  \"watchdog_timeout_ms\": " << watchdog_timeout_ms << ",\n";
    file << "  \"wheels\": {\n";

    auto dump_wheel = [&](const std::string& name, const MotorPinConfig& w, bool is_last) {
        file << "    \"" << name << "\": {\n";
        file << "      \"channel\": " << w.channel_index << ",\n";
        file << "      \"pwm_pin\": " << w.pwm_pin << ",\n";
        file << "      \"sysfs_pwm_path\": \"" << w.sysfs_pwm_path << "\",\n";
        file << "      \"in1_pin\": " << w.in1_pin << ",\n";
        file << "      \"in2_pin\": " << w.in2_pin << ",\n";
        file << "      \"invert\": " << (w.invert ? "true" : "false") << ",\n";
        file << "      \"trim\": " << w.trim << "\n";
        file << "    }" << (is_last ? "\n" : ",\n");
    };

    dump_wheel("FL", fl, false);
    dump_wheel("FR", fr, false);
    dump_wheel("RL", rl, false);
    dump_wheel("RR", rr, true);

    file << "  }\n";
    file << "}\n";
    return true;
}

// ---------------------------------------------------------------------------
// DualL298NActuator Implementation
// ---------------------------------------------------------------------------

DualL298NActuator::DualL298NActuator(const DualL298NConfig& config)
    : config_(config), watchdog_enabled_(config.enable_watchdog) {
    for (int i = 0; i < 16; ++i) gpio_fds_[i] = -1;
    last_command_time_ns_ = get_current_time_ns();
    initialize_hardware();
}

DualL298NActuator::~DualL298NActuator() {
    shutdown_hardware();
}

bool DualL298NActuator::export_gpio(int pin) {
    if (pin < 0) return false;
    char path[64];
    std::snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d", pin);
    struct stat st;
    if (stat(path, &st) == 0) return true; // Already exported

    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) return false;
    char buf[16];
    int len = std::snprintf(buf, sizeof(buf), "%d", pin);
    ssize_t written = write(fd, buf, len);
    close(fd);
    return (written > 0);
}

bool DualL298NActuator::set_gpio_direction(int pin, const std::string& dir) {
    if (pin < 0) return false;
    char path[64];
    std::snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
    int fd = open(path, O_WRONLY);
    if (fd < 0) return false;
    ssize_t written = write(fd, dir.c_str(), dir.size());
    close(fd);
    return (written > 0);
}

int DualL298NActuator::open_gpio_value_fd(int pin) {
    if (pin < 0) return -1;
    char path[64];
    std::snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    return open(path, O_WRONLY);
}

void DualL298NActuator::write_gpio_fast(int fd, int value) {
    if (fd >= 0) {
        pwrite(fd, value ? "1" : "0", 1, 0);
    }
}

void DualL298NActuator::write_gpio(int pin, int value) {
    if (pin < 0) return;
    char path[64];
    std::snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
    int fd = open(path, O_WRONLY);
    if (fd >= 0) {
        write(fd, value ? "1" : "0", 1);
        close(fd);
    }
}

bool DualL298NActuator::init_sysfs_pwm(const std::string& pwm_path, int freq_hz) {
    if (pwm_path.empty()) return false;
    std::string period_path = pwm_path + "/period";
    int fd = open(period_path.c_str(), O_WRONLY);
    if (fd < 0) return false;

    uint64_t period_ns = 1000000000ULL / static_cast<uint64_t>(freq_hz);
    std::string s = std::to_string(period_ns);
    write(fd, s.c_str(), s.size());
    close(fd);

    std::string enable_path = pwm_path + "/enable";
    fd = open(enable_path.c_str(), O_WRONLY);
    if (fd >= 0) {
        write(fd, "1", 1);
        close(fd);
    }
    return true;
}

void DualL298NActuator::set_sysfs_pwm_duty(const std::string& pwm_path, float duty, int freq_hz) {
    if (pwm_path.empty()) return;
    float abs_duty = clamp_val(std::abs(duty), 0.0f, 1.0f);
    uint64_t period_ns = 1000000000ULL / static_cast<uint64_t>(freq_hz);
    uint64_t duty_ns = static_cast<uint64_t>(abs_duty * static_cast<float>(period_ns));

    std::string duty_path = pwm_path + "/duty_cycle";
    int fd = open(duty_path.c_str(), O_WRONLY);
    if (fd >= 0) {
        std::string s = std::to_string(duty_ns);
        write(fd, s.c_str(), s.size());
        close(fd);
    }
}

bool DualL298NActuator::initialize_hardware() {
    shutdown_hardware();

    // Check if sysfs GPIO is present and writable
    bool gpio_accessible = (access("/sys/class/gpio", W_OK) == 0);
    if (!gpio_accessible) {
        is_simulated_ = true;
        std::cout << "[DualL298NActuator] Hardware GPIO inaccessible. Running in SIMULATION / MOCK mode." << std::endl;
    } else {
        is_simulated_ = false;
        std::cout << "[DualL298NActuator] Initializing physical Linux sysfs GPIO / PWM..." << std::endl;

        const MotorPinConfig* wheels[4] = {&config_.fl, &config_.fr, &config_.rl, &config_.rr};
        for (int i = 0; i < 4; ++i) {
            const auto& w = *wheels[i];
            // Export and setup direction pins
            if (export_gpio(w.in1_pin)) set_gpio_direction(w.in1_pin, "out");
            if (export_gpio(w.in2_pin)) set_gpio_direction(w.in2_pin, "out");
            gpio_fds_[i * 3 + 0] = open_gpio_value_fd(w.in1_pin);
            gpio_fds_[i * 3 + 1] = open_gpio_value_fd(w.in2_pin);

            // Active brake on init
            write_gpio_fast(gpio_fds_[i * 3 + 0], 0);
            write_gpio_fast(gpio_fds_[i * 3 + 1], 0);

            // Setup PWM
            if (config_.use_hardware_pwm && !w.sysfs_pwm_path.empty()) {
                init_sysfs_pwm(w.sysfs_pwm_path, config_.pwm_frequency_hz);
            } else if (w.pwm_pin >= 0) {
                if (export_gpio(w.pwm_pin)) set_gpio_direction(w.pwm_pin, "out");
                gpio_fds_[i * 3 + 2] = open_gpio_value_fd(w.pwm_pin);
                write_gpio_fast(gpio_fds_[i * 3 + 2], 0);
            }
        }
    }

    // Start background software PWM and watchdog thread
    running_ = true;
    worker_thread_ = std::thread(&DualL298NActuator::worker_loop, this);
    return true;
}

void DualL298NActuator::shutdown_hardware() {
    running_ = false;
    if (worker_thread_.joinable()) {
        worker_thread_.join();
    }

    // Active brake and close fds
    for (int i = 0; i < 16; ++i) {
        if (gpio_fds_[i] >= 0) {
            write_gpio_fast(gpio_fds_[i], 0);
            close(gpio_fds_[i]);
            gpio_fds_[i] = -1;
        }
    }
}

void DualL298NActuator::emergency_brake() {
    emergency_stop_.store(true, std::memory_order_release);
    for (int i = 0; i < 4; ++i) {
        target_duties_[i].store(0.0f, std::memory_order_release);
    }
    // Instant active brake to zero
    for (int i = 0; i < 4; ++i) {
        if (gpio_fds_[i * 3 + 0] >= 0) write_gpio_fast(gpio_fds_[i * 3 + 0], 0);
        if (gpio_fds_[i * 3 + 1] >= 0) write_gpio_fast(gpio_fds_[i * 3 + 1], 0);
        if (gpio_fds_[i * 3 + 2] >= 0) write_gpio_fast(gpio_fds_[i * 3 + 2], 0);
    }
}

bool DualL298NActuator::is_emergency_stopped() const noexcept {
    return emergency_stop_.load(std::memory_order_acquire);
}

void DualL298NActuator::reset_emergency_stop() {
    emergency_stop_.store(false, std::memory_order_release);
    watchdog_tripped_.store(false, std::memory_order_release);
    last_command_time_ns_ = get_current_time_ns();
}

void DualL298NActuator::set_raw_channel_duty(int channel_index, float duty) {
    if (emergency_stop_.load(std::memory_order_acquire)) return;
    if (channel_index < 0 || channel_index >= 4) return;

    last_command_time_ns_ = get_current_time_ns();
    target_duties_[channel_index].store(clamp_val(duty, -1.0f, 1.0f), std::memory_order_release);
}

void DualL298NActuator::set_wheel_duties(float fl, float fr, float rl, float rr) {
    if (emergency_stop_.load(std::memory_order_acquire)) return;

    last_command_time_ns_ = get_current_time_ns();

    // Map each wheel to its assigned driver channel
    auto assign_wheel = [&](const MotorPinConfig& cfg, float duty) {
        if (cfg.channel_index >= 0 && cfg.channel_index < 4) {
            float applied = duty * cfg.trim;
            if (cfg.invert) applied = -applied;
            target_duties_[cfg.channel_index].store(clamp_val(applied, -1.0f, 1.0f), std::memory_order_release);
        }
    };

    assign_wheel(config_.fl, fl);
    assign_wheel(config_.fr, fr);
    assign_wheel(config_.rl, rl);
    assign_wheel(config_.rr, rr);
}

void DualL298NActuator::set_duty_cycles(float duty_left, float duty_right) {
    if (emergency_stop_.load(std::memory_order_acquire)) return;

    float dl = clamp_val(duty_left, -1.0f, 1.0f);
    float dr = clamp_val(duty_right, -1.0f, 1.0f);

    // Stiction deadband injection
    if (std::abs(dl) > 1e-3f) {
        dl += (dl > 0.0f ? config_.deadband : -config_.deadband);
    }
    if (std::abs(dr) > 1e-3f) {
        dr += (dr > 0.0f ? config_.deadband : -config_.deadband);
    }

    dl = clamp_val(dl, -1.0f, 1.0f);
    dr = clamp_val(dr, -1.0f, 1.0f);

    // Omni-Tank: Left bank = FL & RL; Right bank = FR & RR
    set_wheel_duties(dl, dr, dl, dr);
}

void DualL298NActuator::worker_loop() {
    const uint64_t period_us = 1000000ULL / static_cast<uint64_t>(config_.pwm_frequency_hz);
    const uint64_t period_ns = period_us * 1000ULL;

    while (running_) {
        uint64_t loop_start_ns = get_current_time_ns();

        // 1. Watchdog evaluation
        if (watchdog_enabled_.load(std::memory_order_acquire)) {
            uint64_t elapsed_ms = (loop_start_ns - last_command_time_ns_.load(std::memory_order_acquire)) / 1000000ULL;
            if (elapsed_ms > config_.watchdog_timeout_ms) {
                if (!watchdog_tripped_.load(std::memory_order_acquire)) {
                    watchdog_tripped_.store(true, std::memory_order_release);
                }
                for (int i = 0; i < 4; ++i) {
                    target_duties_[i].store(0.0f, std::memory_order_release);
                }
            }
        }

        bool is_e_stopped = emergency_stop_.load(std::memory_order_acquire);

        // 2. Fetch commanded duties for all 4 channels
        float duties[4];
        for (int i = 0; i < 4; ++i) {
            duties[i] = is_e_stopped ? 0.0f : target_duties_[i].load(std::memory_order_acquire);
        }

        // 3. Update outputs: Direct-IN PWM mode (4-wire per board) or 3-pin mode
        if (!is_simulated_) {
            const MotorPinConfig* wheels[4] = {&config_.fl, &config_.fr, &config_.rl, &config_.rr};
            int pulse_fds[4] = {-1, -1, -1, -1};
            uint64_t on_time_ns[4] = {0, 0, 0, 0};

            for (int i = 0; i < 4; ++i) {
                int ch = wheels[i]->channel_index;
                float d = (ch >= 0 && ch < 4) ? duties[ch] : 0.0f;
                int in1_fd = gpio_fds_[i * 3 + 0];
                int in2_fd = gpio_fds_[i * 3 + 1];
                int pwm_fd = gpio_fds_[i * 3 + 2];
                uint64_t ot = static_cast<uint64_t>(std::abs(d) * static_cast<float>(period_ns));

                if (wheels[i]->pwm_pin < 0) {
                    // Direct-IN PWM Mode (ENA/ENB jumper caps ON)
                    if (std::abs(d) < 1e-3f) {
                        write_gpio_fast(in1_fd, 0);
                        write_gpio_fast(in2_fd, 0);
                    } else if (d > 0.0f) {
                        // Forward: IN2 is LOW, IN1 pulses
                        write_gpio_fast(in2_fd, 0);
                        if (ot > 0) {
                            pulse_fds[i] = in1_fd;
                            on_time_ns[i] = ot;
                        } else {
                            write_gpio_fast(in1_fd, 0);
                        }
                    } else {
                        // Reverse: IN1 is LOW, IN2 pulses
                        write_gpio_fast(in1_fd, 0);
                        if (ot > 0) {
                            pulse_fds[i] = in2_fd;
                            on_time_ns[i] = ot;
                        } else {
                            write_gpio_fast(in2_fd, 0);
                        }
                    }
                } else {
                    // 3-Pin Mode (Dedicated PWM on ENA/ENB)
                    if (std::abs(d) < 1e-3f) {
                        write_gpio_fast(in1_fd, 0);
                        write_gpio_fast(in2_fd, 0);
                    } else if (d > 0.0f) {
                        write_gpio_fast(in1_fd, 1);
                        write_gpio_fast(in2_fd, 0);
                    } else {
                        write_gpio_fast(in1_fd, 0);
                        write_gpio_fast(in2_fd, 1);
                    }

                    if (config_.use_hardware_pwm && !wheels[i]->sysfs_pwm_path.empty()) {
                        set_sysfs_pwm_duty(wheels[i]->sysfs_pwm_path, d, config_.pwm_frequency_hz);
                    } else if (ot > 0) {
                        pulse_fds[i] = pwm_fd;
                        on_time_ns[i] = ot;
                    } else {
                        write_gpio_fast(pwm_fd, 0);
                    }
                }
            }

            // Set pulsing pins HIGH at start of period
            for (int i = 0; i < 4; ++i) {
                if (pulse_fds[i] >= 0 && on_time_ns[i] > 0) {
                    write_gpio_fast(pulse_fds[i], 1);
                }
            }

            // Wait until period expires, dropping pins to LOW as their duty threshold expires
            while (true) {
                uint64_t cur_ns = get_current_time_ns();
                uint64_t elapsed_ns = cur_ns - loop_start_ns;
                if (elapsed_ns >= period_ns) break;

                for (int i = 0; i < 4; ++i) {
                    if (pulse_fds[i] >= 0 && on_time_ns[i] > 0 && elapsed_ns >= on_time_ns[i]) {
                        write_gpio_fast(pulse_fds[i], 0);
                        on_time_ns[i] = 0;
                    }
                }
                std::this_thread::sleep_for(std::chrono::microseconds(50));
            }

            // Ensure all pulse pins are LOW at period boundary
            for (int i = 0; i < 4; ++i) {
                if (pulse_fds[i] >= 0) {
                    write_gpio_fast(pulse_fds[i], 0);
                }
            }
        } else {
            // Simulated mode: sleep the full period
            std::this_thread::sleep_for(std::chrono::microseconds(period_us));
        }
    }
}

} // namespace rvpoint
