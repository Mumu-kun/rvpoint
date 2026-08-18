.PHONY: all build test quick benchmark compare clean help

# Default target
all: test

# Build the project
build:
	@./test_voxel.sh build

# Run all tests
test:
	@./test_voxel.sh all

# Quick test (build + unit tests + basic checks)
quick:
	@./test_voxel.sh quick

# Run unit tests only
unit:
	@./test_voxel.sh unit

# Run CTest
ctest:
	@./test_voxel.sh ctest

# Run benchmarks
benchmark:
	@./test_voxel.sh benchmark

# Compare scalar vs RVV performance
compare:
	@./test_voxel.sh compare

# Verify correctness
verify:
	@./test_voxel.sh verify

# Stress test
stress:
	@./test_voxel.sh stress

# Test standalone executables
standalone:
	@./test_voxel.sh standalone

# Clean build directory
clean:
	@echo "Cleaning build directory..."
	@rm -rf build
	@echo "✓ Clean complete"

# Show help
help:
	@echo "Voxel Downsampling Test Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  make           - Run all tests (default)"
	@echo "  make quick     - Quick test (build + unit tests)"
	@echo "  make build     - Build the project only"
	@echo "  make unit      - Run unit tests only"
	@echo "  make benchmark - Run performance benchmarks"
	@echo "  make compare   - Compare scalar vs RVV performance"
	@echo "  make verify    - Verify correctness"
	@echo "  make stress    - Run stress test (1M points)"
	@echo "  make clean     - Clean build directory"
	@echo "  make help      - Show this help"
	@echo ""
	@echo "Examples:"
	@echo "  make quick     # Fast verification"
	@echo "  make benchmark # Performance testing"
	@echo "  make compare   # See speedup numbers"
