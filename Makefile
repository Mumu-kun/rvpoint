CC = /opt/riscv/bin/riscv64-unknown-elf-gcc
CFLAGS = -march=rv64gcv -mabi=lp64d
RUNNER = /opt/riscv/bin/qemu-riscv64
RUNNER_FLAGS = -cpu max

SRC_DIR = src
TEST_DIR = tests
BUILD_DIR = build

# Ensure build directory exists
$(shell mkdir -p $(BUILD_DIR))

# Targets
all: $(BUILD_DIR)/test_scalar $(BUILD_DIR)/test_vector

$(BUILD_DIR)/test_scalar: $(TEST_DIR)/test_scalar.c
	$(CC) $(CFLAGS) -o $@ $<

$(BUILD_DIR)/test_vector: $(TEST_DIR)/test_vector.c
	$(CC) $(CFLAGS) -o $@ $<

run_scalar: $(BUILD_DIR)/test_scalar
	@echo "Running Scalar Test..."
	$(RUNNER) $(RUNNER_FLAGS) $(BUILD_DIR)/test_scalar

run_vector: $(BUILD_DIR)/test_vector
	@echo "Running Vector Test..."
	$(RUNNER) $(RUNNER_FLAGS) $(BUILD_DIR)/test_vector

clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean run_scalar run_vector
