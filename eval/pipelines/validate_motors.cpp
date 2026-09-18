#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <csignal>
#include <cstdlib>
#include <iomanip>

#if defined(__linux__) || defined(__APPLE__)
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#include "../actuators/dual_l298n_actuator.h"
// Include implementation for single-translation-unit executable build outside librvpoint.a
#include "../actuators/dual_l298n_actuator.cpp"

using namespace rvpoint;

static DualL298NActuator* g_actuator = nullptr;

#if defined(__linux__) || defined(__APPLE__)
static struct termios g_orig_termios;
static bool g_raw_mode_active = false;

static void disable_raw_mode() {
    if (g_raw_mode_active) {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &g_orig_termios);
        g_raw_mode_active = false;
    }
}

static void enable_raw_mode() {
    if (!g_raw_mode_active && isatty(STDIN_FILENO)) {
        tcgetattr(STDIN_FILENO, &g_orig_termios);
        atexit(disable_raw_mode);
        struct termios raw = g_orig_termios;
        raw.c_lflag &= ~(ECHO | ICANON | ISIG);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 1; // 100ms timeout
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        g_raw_mode_active = true;
    }
}

static int read_key_nonblocking() {
    char c = 0;
    ssize_t n = read(STDIN_FILENO, &c, 1);
    return (n > 0) ? static_cast<unsigned char>(c) : -1;
}
#else
static void enable_raw_mode() {}
static void disable_raw_mode() {}
static int read_key_nonblocking() { return -1; }
#endif

static void handle_signal(int sig) {
    if (g_actuator) {
        std::cout << "\n[SAFETY] Interrupted by signal " << sig << ". Active E-Brake engaged!\n";
        g_actuator->emergency_brake();
    }
#if defined(__linux__) || defined(__APPLE__)
    disable_raw_mode();
#endif
    std::_Exit(sig);
}

static void print_banner() {
    std::cout << "\n===============================================================\n"
              << "   RVPoint Axle-Partitioned Dual L298N Motor Validation Tool   \n"
              << "    Ch A/B Front Board + Ch A/B Rear Board Omni-Tank Driver    \n"
              << "===============================================================\n\n";
}

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n\n"
              << "Modes:\n"
              << "  --wizard              Interactive guided channel & polarity calibration\n"
              << "  --test individual     Sequential test on FL -> RL -> FR -> RR\n"
              << "  --test directional    Drive Forward -> Reverse -> Pivot Left -> Pivot Right\n"
              << "  --test sweep          Stiction/deadband ramp calibration sweep (5% to 50%)\n"
              << "  --teleop              Interactive WASD terminal keyboard driving\n\n"
              << "Options:\n"
              << "  --config <path>       Pin configuration JSON path (default: eval/actuators/l298n_pins.json)\n"
              << "  --duty <val>          Test duty cycle ratio [0.1, 1.0] (default: 0.30)\n"
              << "  --hw-pwm              Force Linux sysfs hardware PWM mode\n"
              << "  --sw-pwm              Force high-resolution software timer GPIO PWM mode\n"
              << "  --help                Display this help message\n\n";
}

// ---------------------------------------------------------------------------
// Mode 1: Calibration Wizard
// ---------------------------------------------------------------------------
static void run_wizard(DualL298NActuator& actuator, const std::string& config_path, float test_duty) {
    print_banner();
    std::cout << ">>> GUIDED CHANNEL & POLARITY CALIBRATION WIZARD <<<\n\n"
              << "This wizard will pulse each of the 4 physical H-bridge channels\n"
              << "one by one so you can identify which motor is connected to which channel\n"
              << "and whether the wiring polarity is correct.\n\n"
              << "SAFETY NOTICE: Put vehicle on a stand so wheels can spin freely!\n\n";

    std::cout << "Press [ENTER] to begin...";
    std::string dummy;
    std::getline(std::cin, dummy);

    actuator.set_watchdog_enabled(false); // Disable timeout during prompts

    const char* ch_names[4] = {
        "Channel 1 (Board 1 Front, Channel A)",
        "Channel 2 (Board 1 Front, Channel B)",
        "Channel 3 (Board 2 Rear,  Channel A)",
        "Channel 4 (Board 2 Rear,  Channel B)"
    };

    DualL298NConfig& cfg = actuator.config();

    for (int ch = 0; ch < 4; ++ch) {
        std::cout << "\n---------------------------------------------------------------\n";
        std::cout << ">>> Pulsing " << ch_names[ch] << " at " << static_cast<int>(test_duty * 100) << "% duty...\n";
        std::cout << "---------------------------------------------------------------\n";

        // Pulse forward
        actuator.set_raw_channel_duty(ch, test_duty);

        std::cout << "\nWhich wheel is currently spinning?\n"
                  << "  [1] Front-Left  (FL)\n"
                  << "  [2] Front-Right (FR)\n"
                  << "  [3] Rear-Left   (RL)\n"
                  << "  [4] Rear-Right  (RR)\n"
                  << "  [0] None / Not moving\n"
                  << "Selection [0-4]: ";
        int choice = 0;
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(1000, '\n');
            choice = 0;
        }

        // Stop channel immediately
        actuator.set_raw_channel_duty(ch, 0.0f);

        if (choice < 1 || choice > 4) {
            std::cout << "(!) Channel " << ch + 1 << " skipped or not connected.\n";
            continue;
        }

        std::cout << "\nWas the wheel spinning FORWARD or REVERSE?\n"
                  << "  [f] Forward\n"
                  << "  [r] Reverse\n"
                  << "Direction [f/r]: ";
        char dir = 'f';
        std::cin >> dir;

        WheelId target_wheel = static_cast<WheelId>(choice - 1);
        MotorPinConfig& w = cfg.get_wheel_config(target_wheel);
        w.channel_index = ch;
        w.invert = (dir == 'r' || dir == 'R');

        const char* w_names[4] = {"Front-Left (FL)", "Front-Right (FR)", "Rear-Left (RL)", "Rear-Right (RR)"};
        std::cout << "==> Assigned " << ch_names[ch] << " -> " << w_names[choice - 1]
                  << " (Invert: " << (w.invert ? "YES" : "NO") << ")\n";
    }

    std::cout << "\n===============================================================\n"
              << ">>> CALIBRATION SUMMARY <<<\n"
              << "===============================================================\n";
    std::cout << "  FL: Channel " << cfg.fl.channel_index << ", Invert: " << (cfg.fl.invert ? "true" : "false") << "\n";
    std::cout << "  FR: Channel " << cfg.fr.channel_index << ", Invert: " << (cfg.fr.invert ? "true" : "false") << "\n";
    std::cout << "  RL: Channel " << cfg.rl.channel_index << ", Invert: " << (cfg.rl.invert ? "true" : "false") << "\n";
    std::cout << "  RR: Channel " << cfg.rr.channel_index << ", Invert: " << (cfg.rr.invert ? "true" : "false") << "\n\n";

    if (cfg.save_to_json(config_path)) {
        std::cout << "Successfully saved updated configuration to: " << config_path << "\n";
    } else {
        std::cerr << "Failed to save configuration to: " << config_path << "\n";
    }

    std::cout << "\nRun simultaneous 1.5s forward verification spin on all wheels? [Y/n]: ";
    char confirm = 'y';
    std::cin >> confirm;
    if (confirm == 'y' || confirm == 'Y') {
        std::cout << "\n[TEST] Spinning all 4 wheels FORWARD for 1.5s...\n";
        actuator.set_wheel_duties(test_duty, test_duty, test_duty, test_duty);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        actuator.set_wheel_duties(0.0f, 0.0f, 0.0f, 0.0f);
        std::cout << "[TEST] Done. Active brake engaged.\n";
    }

    actuator.set_watchdog_enabled(true);
}

// ---------------------------------------------------------------------------
// Mode 2: Individual Wheel Polarity Test
// ---------------------------------------------------------------------------
// Mode 2: Individual Wheel Polarity Test
// ---------------------------------------------------------------------------
static void run_individual_test(DualL298NActuator& actuator, float duty) {
    print_banner();
    std::cout << ">>> INDIVIDUAL WHEEL POLARITY TEST <<<\n\n"
              << "Testing each wheel: Forward (1.5s) -> Active Brake -> Pause (1.0s) -> Reverse (1.5s)\n\n";

    actuator.set_watchdog_enabled(false);
    const WheelId wheels[4] = {WheelId::FL, WheelId::FR, WheelId::RL, WheelId::RR};
    const char* names[4] = {"Front-Left (FL)", "Front-Right (FR)", "Rear-Left (RL)", "Rear-Right (RR)"};
    for (int i = 0; i < 4; ++i) {
        std::cout << "\n=== Testing " << names[i] << " ===\n";

        // Forward
        std::cout << "  -> FORWARD (+" << static_cast<int>(duty * 100) << "%)..." << std::flush;
        float d[4] = {0, 0, 0, 0};
        d[i] = duty;
        actuator.set_wheel_duties(d[0], d[1], d[2], d[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        // Active Brake & Settle Delay
        actuator.emergency_brake();
        std::cout << " [ACTIVE BRAKE]..." << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        actuator.reset_emergency_stop();
        std::cout << " STOP." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // Reverse
        std::cout << "  -> REVERSE (-" << static_cast<int>(duty * 100) << "%)..." << std::flush;
        d[i] = -duty;
        actuator.set_wheel_duties(d[0], d[1], d[2], d[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        // Active Brake & Settle Delay
        actuator.emergency_brake();
        std::cout << " [ACTIVE BRAKE]..." << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        actuator.reset_emergency_stop();
        std::cout << " STOP.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    actuator.set_watchdog_enabled(true);
    std::cout << "\n[DONE] Individual wheel test completed.\n";
}

// ---------------------------------------------------------------------------
// Mode 3: Directional Motions Test
// ---------------------------------------------------------------------------
static void run_directional_test(DualL298NActuator& actuator, float duty) {
    print_banner();
    std::cout << ">>> DIRECTIONAL MOTIONS TEST (Omni-Tank 2-DoF) <<<\n\n"
              << "Executing: Forward -> Reverse -> Pivot Left -> Pivot Right (1.5s each)\n"
              << "Active dynamic braking and 1.5s settle delay between moves.\n\n";

    actuator.set_watchdog_enabled(false);
    auto execute_move = [&](const std::string& label, float dl, float dr) {
        std::cout << ">>> " << label << " (Left: " << dl << ", Right: " << dr << ")..." << std::flush;
        actuator.set_duty_cycles(dl, dr);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));

        // 1. Active dynamic brake to halt all wheels immediately
        std::cout << " [ACTIVE BRAKE]..." << std::flush;
        actuator.emergency_brake();
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        actuator.reset_emergency_stop();

        // 2. Settling delay between directional moves
        std::cout << " [PAUSE 1.5s] STOP." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
    };

    execute_move("FORWARD", duty, duty);
    execute_move("REVERSE", -duty, -duty);
    execute_move("PIVOT LEFT", -duty, duty);
    execute_move("PIVOT RIGHT", duty, -duty);

    actuator.set_watchdog_enabled(true);
    std::cout << "\n[DONE] Directional motion test completed.\n";
}

// ---------------------------------------------------------------------------
// Mode 4: Stiction / Deadband Calibration Sweep
// ---------------------------------------------------------------------------
static void run_sweep_test(DualL298NActuator& actuator, const std::string& config_path) {
    print_banner();
    std::cout << ">>> STICTION DEADBAND CALIBRATION SWEEP <<<\n\n"
              << "Ramping forward duty from 5% to 50% in steps of 5%.\n"
              << "Observe wheels and indicate when motor rotation begins.\n\n";

    actuator.set_watchdog_enabled(false);
    float found_deadband = 0.15f;
    bool detected = false;

    for (int pct = 5; pct <= 50; pct += 5) {
        float d = static_cast<float>(pct) / 100.0f;
        std::cout << "Testing duty: " << pct << "% (" << std::fixed << std::setprecision(2) << d << ")... " << std::flush;

        actuator.set_wheel_duties(d, d, d, d);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        actuator.set_wheel_duties(0, 0, 0, 0);

        std::cout << "\nDid all wheels start turning? [y/N/q]: ";
        char ans = 'n';
        std::cin >> ans;
        if (ans == 'q' || ans == 'Q') break;
        if (ans == 'y' || ans == 'Y') {
            found_deadband = d;
            detected = true;
            std::cout << "\n==> Stiction break threshold detected at: " << pct << "% (" << d << " duty)!\n";
            break;
        }
    }

    if (detected) {
        actuator.config().deadband = found_deadband;
        actuator.config().save_to_json(config_path);
        std::cout << "Updated configuration deadband to " << found_deadband << " and saved to " << config_path << "\n";
    }

    actuator.set_watchdog_enabled(true);
}

// ---------------------------------------------------------------------------
// Mode 5: Interactive Terminal Teleop
// ---------------------------------------------------------------------------
static void run_teleop(DualL298NActuator& actuator) {
    print_banner();
    std::cout << ">>> INTERACTIVE TERMINAL TELEOPERATION <<<\n\n"
              << "Controls:\n"
              << "  [W] Drive Forward\n"
              << "  [S] Drive Reverse\n"
              << "  [A] Pivot Left\n"
              << "  [D] Pivot Right\n"
              << "  [SPACE] Instant Active Emergency Brake\n"
              << "  [R] Reset Emergency Brake (Arm Controls)\n"
              << "  [+] Increase Speed Step\n"
              << "  [-] Decrease Speed Step\n"
              << "  [Q] Exit Teleoperation\n\n"
              << "Notice: Driving keys (W/A/S/D) or [R] automatically clear active brake.\n"
              << "Safety Watchdog: Releases auto-brake within 200 ms if no key is pressed.\n"
              << "Starting teleop loop...\n\n";

    float base_speed = 0.55f;

#if defined(__linux__) || defined(__APPLE__)
    enable_raw_mode();
#endif

    bool running = true;
    while (running) {
        int key = read_key_nonblocking();

        if (key != -1) {
            char c = static_cast<char>(key);
            if (c == 'q' || c == 'Q') {
                running = false;
                break;
            } else if (c == 'w' || c == 'W' || c == 's' || c == 'S' ||
                       c == 'a' || c == 'A' || c == 'd' || c == 'D' ||
                       c == 'r' || c == 'R') {
                if (actuator.is_emergency_stopped()) {
                    actuator.reset_emergency_stop();
                    std::cout << "\r[BRAKE CLEARED] Controls re-armed.          " << std::flush;
                }

                if (c == 'w' || c == 'W') {
                    actuator.set_duty_cycles(base_speed, base_speed);
                } else if (c == 's' || c == 'S') {
                    actuator.set_duty_cycles(-base_speed, -base_speed);
                } else if (c == 'a' || c == 'A') {
                    actuator.set_duty_cycles(-base_speed, base_speed);
                } else if (c == 'd' || c == 'D') {
                    actuator.set_duty_cycles(base_speed, -base_speed);
                }
            } else if (c == ' ') {
                actuator.emergency_brake();
                std::cout << "\r[ACTIVE BRAKE ENGAGED] (Press W/A/S/D or R to resume)" << std::flush;
            } else if (c == '+' || c == '=') {
                base_speed = clamp_val(base_speed + 0.05f, 0.15f, 1.0f);
                std::cout << "\rSpeed: " << std::fixed << std::setprecision(2) << base_speed << "                              " << std::flush;
            } else if (c == '-' || c == '_') {
                base_speed = clamp_val(base_speed - 0.05f, 0.15f, 1.0f);
                std::cout << "\rSpeed: " << std::fixed << std::setprecision(2) << base_speed << "                              " << std::flush;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

#if defined(__linux__) || defined(__APPLE__)
    disable_raw_mode();
#endif
    actuator.emergency_brake();
    std::cout << "\nTeleoperation terminated safely.\n";
}

// ---------------------------------------------------------------------------
// Main Entrypoint
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    std::string config_path = "eval/actuators/l298n_pins.json";
    std::string mode = "wizard"; // Default to wizard for fast bring-up
    float duty = 0.55f;
    int force_pwm_mode = 0; // 0: config, 1: hw, 2: sw

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--wizard") {
            mode = "wizard";
        } else if (arg == "--test" && i + 1 < argc) {
            mode = argv[++i];
        } else if (arg == "--teleop") {
            mode = "teleop";
        } else if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--duty" && i + 1 < argc) {
            duty = std::stof(argv[++i]);
        } else if (arg == "--hw-pwm") {
            force_pwm_mode = 1;
        } else if (arg == "--sw-pwm") {
            force_pwm_mode = 2;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // Load configuration
    DualL298NConfig config;
    if (config.load_from_json(config_path)) {
        std::cout << "[Config] Loaded configuration from " << config_path << "\n";
    } else {
        std::cout << "[Config] Configuration file not found at " << config_path
                  << ". Using default pin definitions.\n";
    }

    if (force_pwm_mode == 1) config.use_hardware_pwm = true;
    if (force_pwm_mode == 2) config.use_hardware_pwm = false;

    // Initialize Actuator
    DualL298NActuator actuator(config);
    g_actuator = &actuator;

    // Register POSIX signal handlers
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (mode == "wizard") {
        run_wizard(actuator, config_path, duty);
    } else if (mode == "individual") {
        run_individual_test(actuator, duty);
    } else if (mode == "directional") {
        run_directional_test(actuator, duty);
    } else if (mode == "sweep") {
        run_sweep_test(actuator, config_path);
    } else if (mode == "teleop") {
        run_teleop(actuator);
    } else {
        std::cerr << "Unknown test mode: " << mode << "\n";
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}
