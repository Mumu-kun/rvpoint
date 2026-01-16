#include <iostream>
#include <rvpoint/error.hpp>
#include <rvpoint/logger.hpp>

using namespace rvpoint;

// Example function that returns a Result
Result<int> divide(int a, int b) {
    if (b == 0) {
        RVPOINT_ERROR("Division by zero: {} / {}", a, b);
        return make_error<int>(ErrorCode::InvalidInput);
    }
    RVPOINT_DEBUG("Dividing {} by {}", a, b);
    return a / b;
}

// Example function demonstrating error propagation
Result<double> complex_calculation(int x, int y) {
    RVPOINT_INFO("Starting complex calculation with x={}, y={}", x, y);

    // Early return on error
    auto result1 = divide(x, y);
    if (!result1) {
        RVPOINT_WARN("First division failed");
        return make_error<double>(result1.error());
    }

    auto result2 = divide(*result1, 2);
    if (!result2) {
        RVPOINT_WARN("Second division failed");
        return make_error<double>(result2.error());
    }

    RVPOINT_INFO("Calculation successful");
    return static_cast<double>(*result2);
}

int main() {
    // Set log level to debug
    Logger::set_level(spdlog::level::debug);

    RVPOINT_INFO("RVPoint Error Handling & Logging Example");
    RVPOINT_INFO("==========================================");

    // Example 1: Successful operation
    auto result = divide(10, 2);
    if (result) {
        std::cout << "Result: " << *result << std::endl;
    } else {
        std::cout << "Error: " << error_to_string(result.error()) << std::endl;
    }

    // Example 2: Error handling
    auto error_result = divide(10, 0);
    if (!error_result) {
        std::cout << "Error: " << error_to_string(error_result.error()) << std::endl;
    }

    // Example 3: Using and_then for chaining
    auto chained = divide(100, 5)
                       .and_then([](int val) -> Result<int> {
                           RVPOINT_DEBUG("Chaining: got value {}", val);
                           return divide(val, 4);
                       })
                       .and_then([](int val) -> Result<int> {
                           RVPOINT_DEBUG("Chaining: got value {}", val);
                           return val * 2;
                       });

    if (chained) {
        std::cout << "Chained result: " << *chained << std::endl;
    }

    // Example 4: Complex calculation
    auto complex = complex_calculation(100, 10);
    if (complex) {
        std::cout << "Complex result: " << *complex << std::endl;
    } else {
        std::cout << "Complex calculation failed: " << error_to_string(complex.error())
                  << std::endl;
    }

    RVPOINT_INFO("Example completed");

    return 0;
}
