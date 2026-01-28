#!/bin/bash
set -e

# --- Auto-launch in Docker if not inside a container ---
SCRIPT_DIR_EARLY="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT_EARLY="$(cd "$SCRIPT_DIR_EARLY/.." && pwd)"
if [ ! -f /.dockerenv ] && [ -z "$IN_RVPOINT_CONTAINER" ]; then
    IMAGE="${RVPOINT_IMAGE:-rvpoint}"
    if ! docker image inspect "$IMAGE" &> /dev/null; then
        echo "Docker image '$IMAGE' not found. Building from .devcontainer/Dockerfile..."
        docker build -f "$PROJECT_ROOT_EARLY/.devcontainer/Dockerfile" -t "$IMAGE" "$PROJECT_ROOT_EARLY"
    fi
    exec docker run --rm -e TERM="$TERM" -e IN_RVPOINT_CONTAINER=1 \
        -v "$PROJECT_ROOT_EARLY:/workspace" -w /workspace \
        "$IMAGE" bash scripts/verify_container.sh "$@"
fi

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

# 1. Build
log_step "1. Building Project"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
"$SCRIPT_DIR/build.sh" --clean --toolchain linux > /dev/null
log_success "Build Complete"

# Helper: Find QEMU
if command -v qemu-riscv64 &> /dev/null; then
    QEMU_BIN="qemu-riscv64"
elif [ -f "/opt/qemu/bin/qemu-riscv64" ]; then
    QEMU_BIN="/opt/qemu/bin/qemu-riscv64"
else
    log_error "qemu-riscv64 not found"
    exit 1
fi

QEMU_FLAGS="-L ${RISCV_PATH:-/opt/riscv}/sysroot -cpu rv64,v=true,vlen=128"

# 2. Scalar Test
log_step "2. Running Scalar Test"
if "$QEMU_BIN" $QEMU_FLAGS bin/test_scalar | grep -q "verification"; then
    log_success "Scalar Test Passed"
else
    log_error "Scalar Test Failed"
    exit 1
fi

# 3. Vector Test
log_step "3. Running Vector Test"
if "$QEMU_BIN" $QEMU_FLAGS bin/test_vector | grep -q "verification"; then
    log_success "Vector Test Passed"
else
    log_error "Vector Test Failed"
    exit 1
fi

# 4. Voxel Grid
log_step "4. Verifying Voxel Grid"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_voxel_grid)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Voxel Grid OK"
else
    log_error "Voxel Grid Failed"
    exit 1
fi

# 5. SOR
log_step "5. Verifying SOR"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_sor)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "SOR OK"
else
    log_error "SOR Failed"
    exit 1
fi

# 6. Normal Estimation
log_step "6. Verifying Normal Estimation"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_normal)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Normal Estimation OK"
else
    log_error "Normal Estimation Failed"
    exit 1
fi

# 7. Radius Search
log_step "7. Verifying Radius Search"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_radius)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "Radius Search OK"
else
    log_error "Radius Search Failed"
    exit 1
fi

# 8. RANSAC
log_step "8. Verifying RANSAC"
OUT=$("$QEMU_BIN" $QEMU_FLAGS bin/test_ransac)
echo "$OUT"
if echo "$OUT" | grep -q "PASS"; then
    log_success "RANSAC OK"
else
    log_error "RANSAC Failed"
    exit 1
fi

log_header "All Verification Steps Passed!"

exit 0
