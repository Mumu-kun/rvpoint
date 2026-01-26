#!/bin/bash
set -e

# ==============================================================================
# Styling Constants
# ==============================================================================
BOLD="\033[1m"
RESET="\033[0m"
RED="\033[1;31m"
GREEN="\033[1;32m"
YELLOW="\033[1;33m"
BLUE="\033[1;34m"
CYAN="\033[1;36m"
MAGENTA="\033[1;35m"

# ==============================================================================
# Helper Functions
# ==============================================================================
log_header() {
    echo -e "\n${BLUE}${BOLD}============================================================${RESET}"
    echo -e "${BLUE}${BOLD}  $1${RESET}"
    echo -e "${BLUE}${BOLD}============================================================${RESET}"
}

log_step() {
    echo -e "\n${CYAN}${BOLD}>> $1${RESET}"
}

log_success() {
    echo -e "${GREEN}${BOLD}✔ $1${RESET}"
}

log_error() {
    echo -e "${RED}${BOLD}✘ $1${RESET}"
}

log_info() {
    echo -e "${RESET}$1"
}

print_banner() {
    clear
    echo -e "${MAGENTA}${BOLD}"
    echo "  _____  _    __   ___      _       _   "
    echo " |  __ \| |   \ \ / / |    (_)     | |  "
    echo " | |__) | |__  \ V /| |____ _ _ __ | |_ "
    echo " |  _  /| '_ \  > < | |__  | | '_ \| __|"
    echo " | | \ \| | \ \/ . \| |  | | | | | | |_ "
    echo " |_|  \_\_|  \_\_/  |_|  |_|_|_| |_|\__|"
    echo " RISC-V Optimization Verification Suite"
    echo -e "${RESET}"
}

# ==============================================================================
# Main Script
# ==============================================================================

print_banner
log_header "Starting Verification Suite"

# 0. Workflow Checks
log_step "0. Running Workflow Checks"
if [ -f scripts/check_format.sh ]; then
    bash scripts/check_format.sh || { log_error "Format check failed"; exit 1; }
    log_success "Formatting OK"
else
    log_info "Skipping format check (script not found)"
fi

if [ -f scripts/lint.sh ]; then
    bash scripts/lint.sh || { log_error "Lint check failed"; exit 1; }
    log_success "Linting OK"
else
    log_info "Skipping lint check (script not found)"
fi

# 1. Configuration
log_step "1. Configuring CMake"
rm -rf build_cmake
mkdir -p build_cmake
cd build_cmake || exit 1
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/riscv.cmake > /dev/null
log_success "CMake Configured"

# 2. Building
log_step "2. Building Project"
make -j$(nproc) > /dev/null
log_success "Build Complete"
cd .. || exit 1

# Helper: Find QEMU
if [ -f "${RISCV_PATH:-/opt/riscv}/bin/qemu-riscv64" ]; then
    QEMU_BIN="${RISCV_PATH:-/opt/riscv}/bin/qemu-riscv64"
else
    QEMU_BIN="qemu-riscv64"
fi

# CPU Flags (Use specific config instead of 'max' for better compatibility with older QEMU)
QEMU_FLAGS="-cpu rv64,v=true,vlen=128"

# 3. Scalar Test
log_step "3. Running Scalar Test"
if "$QEMU_BIN" $QEMU_FLAGS bin/test_scalar | grep -q "verification"; then
    log_success "Scalar Test Passed"
else
    log_error "Scalar Test Failed"
    exit 1
fi

# 4. Vector Test
log_step "4. Running Vector Test"
if "$QEMU_BIN" $QEMU_FLAGS bin/test_vector | grep -q "verification"; then
    log_success "Vector Test Passed"
else
    log_error "Vector Test Failed"
    exit 1
fi

# 5. Voxel Grid
log_step "5. Verifying Voxel Grid"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_voxel_grid)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Voxel Grid OK"
else
    log_error "Voxel Grid Failed"
    exit 1
fi

# 6. SOR
log_step "6. Verifying SOR"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_sor)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "SOR OK"
else
    log_error "SOR Failed"
    exit 1
fi

# 7. Normal Estimation
log_step "7. Verifying Normal Estimation"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_normal)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Normal Estimation OK"
else
    log_error "Normal Estimation Failed"
    exit 1
fi

# 8. Radius Search
log_step "8. Verifying Radius Search"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_radius)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Radius Search OK"
else
    log_error "Radius Search Failed"
    exit 1
fi

# 9. RANSAC
log_step "9. Verifying RANSAC"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_ransac)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "RANSAC OK"
else
    log_error "RANSAC Failed"
    exit 1
fi

log_header "All Verification Steps Passed! 🚀"

# 10. Linux Toolchain Verification (Optional)
if [ -f "scripts/verify_linux.sh" ] && [ -f "${RISCV_PATH:-/opt/riscv}/bin/riscv64-unknown-linux-gnu-gcc" ]; then
    log_header "10. Linux Toolchain Detected - Verifying..."
    bash scripts/verify_linux.sh || { log_error "Linux Verification Failed"; exit 1; }
    log_success "Linux Toolchain Verified"
else
    log_info "Skipping Linux Toolchain verification (toolchain or script not found)"
fi

exit 0
