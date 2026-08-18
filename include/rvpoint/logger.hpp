#pragma once

#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace rvpoint {

/// Logger utility for RVPoint library
class Logger {
public:
    /// Get the singleton logger instance
    static std::shared_ptr<spdlog::logger>& get() {
        static auto logger = []() {
            auto console_logger = spdlog::stdout_color_mt("rvpoint");
            console_logger->set_level(spdlog::level::info);
            console_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%^%l%$] %v");
            return console_logger;
        }();
        return logger;
    }

    /// Set log level
    static void set_level(spdlog::level::level_enum level) { get()->set_level(level); }
};

} // namespace rvpoint

// Convenience macros for logging
#define RVPOINT_TRACE(...) rvpoint::Logger::get()->trace(__VA_ARGS__)
#define RVPOINT_DEBUG(...) rvpoint::Logger::get()->debug(__VA_ARGS__)
#define RVPOINT_INFO(...) rvpoint::Logger::get()->info(__VA_ARGS__)
#define RVPOINT_WARN(...) rvpoint::Logger::get()->warn(__VA_ARGS__)
#define RVPOINT_ERROR(...) rvpoint::Logger::get()->error(__VA_ARGS__)
#define RVPOINT_CRITICAL(...) rvpoint::Logger::get()->critical(__VA_ARGS__)
