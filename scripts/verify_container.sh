#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/lib/common.sh"
wsl_bootstrap "scripts/verify_container.sh" "$@"

source "${PROJECT_ROOT}/env/activate.sh"

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
    echo -e "${BLUE}${BOLD}============================================================${RESET}"
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

# Runs every test_* / rvv_test binary produced by the build under QEMU.
# Excludes benchmark/pipeline_export (not self-checking tests) and gives
# test_pcd_radius (a parameterized CLI tool, not a plain test) sample args.
run_tests() {
    local build_dir bin_dir qemu_bin
    build_dir="$(get_build_dir)"
    bin_dir="${build_dir}/rvv/bin/rvv"
    qemu_bin="$(find_qemu)"
    # shellcheck disable=SC2054  # comma is part of the -cpu value, not an array separator
    local qemu_flags=(-cpu "rv64,v=true,vlen=128" -L "${RISCV}/sysroot")

    if [ ! -d "$bin_dir" ]; then
        log_error "Build output directory not found: $bin_dir"
        return 1
    fi

    local total=0
    local failed=0
    local log_file

    for bin in "$bin_dir"/*; do
        [ -f "$bin" ] && [ -x "$bin" ] || continue
        local name
        name="$(basename "$bin")"

        case "$name" in
            benchmark|pipeline_export)
                continue
                ;;
        esac
        case "$name" in
            test_*|rvv_test|benchmark_pointer_octree_real) ;;
            *) continue ;;
        esac

        total=$((total + 1))
        log_file="$(mktemp)"

        local rc=0
        if [ "$name" = "test_pcd_radius" ]; then
            local sample_pcd="${PROJECT_ROOT}/data/table_scene_lms400.pcd"
            if [ ! -f "$sample_pcd" ]; then
                log_info "Skipping $name (sample PCD not found: $sample_pcd)"
                total=$((total - 1))
                rm -f "$log_file"
                continue
            fi
            log_info "Running $name..."
            "$qemu_bin" "${qemu_flags[@]}" "$bin" "$sample_pcd" octree 0.5 3 0 >"$log_file" 2>&1 || rc=$?
        else
            log_info "Running $name..."
            "$qemu_bin" "${qemu_flags[@]}" "$bin" >"$log_file" 2>&1 || rc=$?
        fi

        if [ "$rc" -eq 0 ]; then
            log_success "$name passed"
        else
            failed=$((failed + 1))
            log_error "$name FAILED"
            cat "$log_file"
        fi
        rm -f "$log_file"
    done

    log_info ""
    log_info "Ran ${total} test binaries, ${failed} failed."
    [ "$failed" -eq 0 ]
}

# Parse arguments
BUILD_ONLY=true
RUN_TESTS=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-only|-b)
            BUILD_ONLY=true
            RUN_TESTS=false
            shift
            ;;
        --test|--tests|-t|--all)
            BUILD_ONLY=false
            RUN_TESTS=true
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--build-only|-b] [--test|-t]"
            echo "  Default mode is --build-only."
            exit 0
            ;;
        *)
            echo "Unknown argument: $1"
            echo "Usage: $0 [--build-only|-b] [--test|-t]"
            exit 1
            ;;
    esac
done

print_banner
log_header "Starting Verification Suite"

# 1. Build
log_step "1. Building Project"
"$SCRIPT_DIR/build.sh" --clean --toolchain linux
log_success "Build Complete"

if [ "$BUILD_ONLY" = true ] && [ "$RUN_TESTS" = false ]; then
    log_header "Build Verification Passed!"
    exit 0
fi

# 2. Running Test Suite
log_step "2. Running Test Suite"
if run_tests; then
    log_success "All Tests Passed"
else
    log_error "Test Suite Failed"
    exit 1
fi

log_header "All Verification Steps Passed!"

exit 0