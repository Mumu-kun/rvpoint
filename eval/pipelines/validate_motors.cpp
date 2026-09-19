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

#include "../actuators/pca9685_actuator.h"
#include "../actuators/pca9685_actuator.cpp"

using namespace rvpoint;

static PCA9685Actuator* g_actuator = nullptr;

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
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);
        g_raw_mode_active = true;
    }
}

static char read_char_nonblocking() {
    char c = 0;
    if (read(STDIN_FILENO, &c, 1) > 0) {
        return c;
    }
    return 0;
}
#else
static void disable_raw_mode() {}
static void enable_raw_mode() {}
static char read_char_nonblocking() { return 0; }
#endif

static void handle_signal(int sig) {
    std::cout << "\n[!] Caught signal " << sig << ". Emergency stop...\n";
    disable_raw_mode();
    if (g_actuator) {
        g_actuator->emergency_brake();
        g_actuator->shutdown_hardware();
    }
    std::exit(sig);
}

static void print_banner() {
    std::cout << "\n===============================================================\n"
              << "      RVPoint PCA9685 I2C Hardware Motor Validation Tool      \n"
              << "===============================================================\n\n";
}

static void print_usage(const char* prog_name) {
    print_banner();
    std::cout << "Usage: " << prog_name << " [options]\n\n"
              << "Modes:\n"
              << "  --wizard              Interactive guided motor calibration\n"
              << "  --test individual     Sequential test on FL -> FR -> RL -> RR\n"
              << "  --test directional    Drive Forward -> Reverse -> Pivot Left -> Pivot Right\n"
              << "  --teleop              Interactive WASD terminal keyboard driving\n\n"
              << "Options:\n"
              << "  --config <path>       Configuration JSON path (default: eval/actuators/pca9685_pins.json)\n"
              << "  --duty <val>          Test duty cycle ratio [0.1, 1.0] (default: 0.40)\n"
              << "  --help                Display this help message\n\n";
}

static void run_wizard(PCA9685Actuator& actuator, const std::string& config_path, float test_duty) {
    print_banner();
    std::cout << ">>> GUIDED PCA9685 MOTOR & POLARITY CALIBRATION WIZARD <<<\n\n"
              << "SAFETY NOTICE: Put vehicle on a stand so wheels can spin freely!\n\n"
              << "Press [ENTER] to begin...";
    std::string dummy;
    std::getline(std::cin, dummy);

    actuator.set_watchdog_enabled(false);
    PCA9685Config& cfg = actuator.config();

    for (int m = 1; m <= 4; ++m) {
        std::cout << "\n---------------------------------------------------------------\n"
                  << ">>> Pulsing Motor " << m << " at " << static_cast<int>(test_duty * 100) << "% duty for 1.5s...\n";
        actuator.set_wheel_duties((m == 1 ? test_duty : 0), (m == 2 ? test_duty : 0),
                                  (m == 3 ? test_duty : 0), (m == 4 ? test_duty : 0));
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        actuator.emergency_brake();
        actuator.reset_emergency_stop();

        std::cout << "\nWhich wheel just spun?\n"
                  << "  [1] Front-Left  (FL)\n"
                  << "  [2] Front-Right (FR)\n"
                  << "  [3] Rear-Left   (RL)\n"
                  << "  [4] Rear-Right  (RR)\n"
                  << "  [0] Skip\n"
                  << "Selection [1-4, 0]: ";
        int choice = 0;
        std::cin >> choice;
        if (choice < 1 || choice > 4) continue;

        std::cout << "Did it rotate FORWARD or REVERSE? [f/r]: ";
        std::string dir;
        std::cin >> dir;
        bool is_rev = (dir == "r" || dir == "R" || dir == "reverse");

        WheelId wid = static_cast<WheelId>(choice - 1);
        cfg.get_wheel_config(wid).motor = m;
        cfg.get_wheel_config(wid).invert = is_rev;
        std::cout << "==> Configured wheel " << choice << " -> Motor " << m << " (Invert: " << (is_rev ? "YES" : "NO") << ")\n";
    }

    cfg.save_to_json(config_path);
    std::cout << "\nSaved calibration to " << config_path << "\n";
    actuator.set_watchdog_enabled(true);
}

static void run_individual_test(PCA9685Actuator& actuator, float duty) {
    print_banner();
    std::cout << ">>> INDIVIDUAL WHEEL POLARITY TEST <<<\n";
    actuator.set_watchdog_enabled(false);

    WheelId wheels[4] = {WheelId::FL, WheelId::FR, WheelId::RL, WheelId::RR};
    const char* names[4] = {"Front-Left (FL)", "Front-Right (FR)", "Rear-Left (RL)", "Rear-Right (RR)"};

    for (int i = 0; i < 4; ++i) {
        std::cout << "\n=== Testing " << names[i] << " ===\n";
        float d[4] = {0, 0, 0, 0};
        d[i] = duty;
        std::cout << "  -> FORWARD (+ " << static_cast<int>(duty * 100) << "%)... " << std::flush;
        actuator.set_wheel_duties(d[0], d[1], d[2], d[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        actuator.emergency_brake();
        actuator.reset_emergency_stop();
        std::cout << "[BRAKE]\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        d[i] = -duty;
        std::cout << "  -> REVERSE (- " << static_cast<int>(duty * 100) << "%)... " << std::flush;
        actuator.set_wheel_duties(d[0], d[1], d[2], d[3]);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        actuator.emergency_brake();
        actuator.reset_emergency_stop();
        std::cout << "[BRAKE]\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

static void run_directional_test(PCA9685Actuator& actuator, float duty) {
    print_banner();
    std::cout << ">>> 4-WHEEL DIRECTIONAL MOTIONS TEST <<<\n";
    actuator.set_watchdog_enabled(false);

    auto pulse = [&](float fl, float fr, float rl, float rr, const char* name) {
        std::cout << "\n>>> " << name << " (1.5s)...\n";
        actuator.set_wheel_duties(fl, fr, rl, rr);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        actuator.emergency_brake();
        actuator.reset_emergency_stop();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    };

    pulse(duty, duty, duty, duty, "FORWARD");
    pulse(-duty, -duty, -duty, -duty, "REVERSE");
    pulse(-duty, duty, -duty, duty, "PIVOT LEFT");
    pulse(duty, -duty, duty, -duty, "PIVOT RIGHT");
}

static void run_teleop(PCA9685Actuator& actuator) {
    print_banner();
    std::cout << ">>> TERMINAL TELEOPERATION (WASD) <<<\n"
              << "Controls: [W] Fwd, [S] Rev, [A] Left, [D] Right, [SPACE] Brake, [Q] Quit\n";
    enable_raw_mode();
    float speed = 0.50f;

    while (true) {
        char c = read_char_nonblocking();
        if (c == 'q' || c == 'Q' || c == 27) break;
        if (c == 'w' || c == 'W') actuator.set_wheel_duties(speed, speed, speed, speed);
        else if (c == 's' || c == 'S') actuator.set_wheel_duties(-speed, -speed, -speed, -speed);
        else if (c == 'a' || c == 'A') actuator.set_wheel_duties(-speed, speed, -speed, speed);
        else if (c == 'd' || c == 'D') actuator.set_wheel_duties(speed, -speed, speed, -speed);
        else if (c == ' ') actuator.emergency_brake();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    disable_raw_mode();
    actuator.emergency_brake();
}

int main(int argc, char** argv) {
    std::string config_path = "eval/actuators/pca9685_pins.json";
    std::string mode = "wizard";
    float duty = 0.40f;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--wizard") mode = "wizard";
        else if (arg == "--test" && i + 1 < argc) mode = argv[++i];
        else if (arg == "--teleop") mode = "teleop";
        else if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (arg == "--duty" && i + 1 < argc) duty = std::stof(argv[++i]);
        else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            return 0;
        }
    }

    PCA9685Config config;
    config.load_from_json(config_path);

    PCA9685Actuator actuator(config);
    g_actuator = &actuator;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    if (mode == "wizard") run_wizard(actuator, config_path, duty);
    else if (mode == "individual") run_individual_test(actuator, duty);
    else if (mode == "directional") run_directional_test(actuator, duty);
    else if (mode == "teleop") run_teleop(actuator);

    return 0;
}
