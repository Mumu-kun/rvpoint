#pragma once

#include <string>
#include <tl/expected.hpp>

namespace rvpoint {

/// Error codes for point cloud operations
enum class ErrorCode {
    Success = 0,
    InvalidInput,
    FileNotFound,
    ParseError,
    OutOfMemory,
    InvalidDimensions,
    EmptyPointCloud
};

/// Convert error code to human-readable string
inline std::string error_to_string(ErrorCode code) {
    switch (code) {
        case ErrorCode::Success:
            return "Success";
        case ErrorCode::InvalidInput:
            return "Invalid input";
        case ErrorCode::FileNotFound:
            return "File not found";
        case ErrorCode::ParseError:
            return "Parse error";
        case ErrorCode::OutOfMemory:
            return "Out of memory";
        case ErrorCode::InvalidDimensions:
            return "Invalid dimensions";
        case ErrorCode::EmptyPointCloud:
            return "Empty point cloud";
        default:
            return "Unknown error";
    }
}

/// Result type for operations that can fail
template <typename T> using Result = tl::expected<T, ErrorCode>;

/// Convenience function to create an error result
template <typename T> inline Result<T> make_error(ErrorCode code) {
    return tl::unexpected(code);
}

} // namespace rvpoint
