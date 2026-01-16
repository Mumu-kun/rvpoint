# Catch2 Testing Guide

## Quick Start

Catch2 is now set up in the tests directory. To add tests:

1. **Create test files** in `tests/` (e.g., `test_point_cloud.cpp`)
2. **Add them to** `tests/CMakeLists.txt` in the `add_executable()` list
3. **Build and run:**
   ```bash
   cmake -B build -DBUILD_TESTS=ON
   cmake --build build
   cd build && ctest --output-on-failure
   ```

## Writing Tests

### Basic Test
```cpp
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Description of what's tested", "[tag]") {
    REQUIRE(actual == expected);
    CHECK(condition);  // Continues on failure
}
```

### Sections (Shared Setup)
```cpp
TEST_CASE("Point cloud operations") {
    PointCloud cloud;  // Setup runs for each SECTION
    
    SECTION("adding points") {
        cloud.add({1, 2, 3});
        REQUIRE(cloud.size() == 1);
    }
    
    SECTION("clearing") {
        cloud.clear();
        REQUIRE(cloud.empty());
    }
}
```

### Floating Point Comparisons
```cpp
#include <catch2/catch_approx.hpp>

REQUIRE(result == Catch::Approx(0.3));
REQUIRE(result == Catch::Approx(0.3).epsilon(0.01));
```

### Parameterized Tests
```cpp
#include <catch2/generators/catch_generators.hpp>

TEST_CASE("Vector operations work for different sizes") {
    auto size = GENERATE(4, 8, 16, 32);
    
    std::vector<float> v(size);
    REQUIRE(v.size() == size);
}
```

### Test Fixtures
```cpp
class PointCloudFixture {
protected:
    PointCloud cloud;
    
    PointCloudFixture() {
        // Setup
        cloud.reserve(100);
    }
    
    ~PointCloudFixture() {
        // Teardown
    }
};

TEST_CASE_METHOD(PointCloudFixture, "Operations on cloud") {
    REQUIRE(cloud.capacity() >= 100);
}
```

## Running Tests

### All tests
```bash
ctest --output-on-failure
```

### Specific test
```bash
./rvpoint_tests "test name"
```

### By tag
```bash
./rvpoint_tests [tag]
./rvpoint_tests [scalar]
./rvpoint_tests [rvv]
```

### With verbose output
```bash
./rvpoint_tests -s  # Show successful assertions
```

### List tests
```bash
./rvpoint_tests --list-tests
./rvpoint_tests --list-tags
```

## Suggested Tags

- `[scalar]` - Scalar backend tests
- `[rvv]` - RISC-V Vector tests
- `[point_cloud]` - Point cloud structure tests
- `[transform]` - Transformation tests
- `[filter]` - Filter operation tests
- `[performance]` - Performance-critical tests

## VS Code Integration

The test adapter extension will automatically discover and run tests in the Testing panel. You can:
- Run individual tests
- Debug tests
- See test results inline

## CI Integration

Tests are automatically run in CI via:
```bash
ctest --output-on-failure --verbose
```
