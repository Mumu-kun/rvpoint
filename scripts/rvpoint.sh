#!/bin/bash
# =============================================================================
#  RVPoint Interactive Development CLI
#  Auto-manages Docker container, provides menu interface for all dev tasks.
#
#  Usage: ./scripts/rvpoint.sh
# =============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN_DIR="$PROJECT_ROOT/bin"
RESULTS_DIR="$PROJECT_ROOT/results"

IMAGE_NAME="${RVPOINT_IMAGE:-rvpoint}"
CONTAINER_NAME="${RVPOINT_CONTAINER:-rvpoint-dev}"

# ─── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; BOLD='\033[1m'; RESET='\033[0m'

info()    { echo -e "${CYAN}==>${RESET} $*"; }
success() { echo -e "${GREEN}[OK]${RESET} $*"; }
warn()    { echo -e "${YELLOW}[WARN]${RESET} $*"; }
error()   { echo -e "${RED}[ERR]${RESET} $*" >&2; }

# ─── Docker helpers ───────────────────────────────────────────────────────────

ensure_image() {
    if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
        info "Docker image '$IMAGE_NAME' not found — building from .devcontainer/Dockerfile"
        info "This will take ~15-20 minutes on first run (downloads toolchains + QEMU)..."
        docker build \
            -f "$PROJECT_ROOT/.devcontainer/Dockerfile" \
            -t "$IMAGE_NAME" \
            "$PROJECT_ROOT/.devcontainer"
        success "Image '$IMAGE_NAME' built."
    fi
}

# Returns the container ID if running, empty otherwise
running_container_id() {
    docker ps -q -f "name=^${CONTAINER_NAME}$" 2>/dev/null || true
}

# Returns the container ID if it exists (running or stopped), empty otherwise
existing_container_id() {
    docker ps -aq -f "name=^${CONTAINER_NAME}$" 2>/dev/null || true
}

ensure_container() {
    ensure_image

    local cid
    cid=$(running_container_id)

    if [ -n "$cid" ]; then
        return 0   # Already running — reuse it
    fi

    local existing
    existing=$(existing_container_id)

    if [ -n "$existing" ]; then
        # Check runtime and privileged mode match expected config.
        # Old containers may have wrong runtime or lack --privileged (needed for swapon).
        local runtime privileged
        runtime=$(docker inspect --format '{{.HostConfig.Runtime}}' "$CONTAINER_NAME" 2>/dev/null || echo "unknown")
        privileged=$(docker inspect --format '{{.HostConfig.Privileged}}' "$CONTAINER_NAME" 2>/dev/null || echo "false")
        if [ "$runtime" != "runc" ] || [ "$privileged" != "true" ]; then
            info "Existing container config outdated (runtime=$runtime, privileged=$privileged). Recreating..."
            docker rm -f "$CONTAINER_NAME" > /dev/null 2>&1 || true
            existing=""
        else
            info "Starting existing container '$CONTAINER_NAME'..."
            docker start "$CONTAINER_NAME" > /dev/null
        fi
    fi

    if [ -z "$existing" ]; then
        info "Creating new persistent container '$CONTAINER_NAME'..."
        # Mount host /opt/gem5 if it exists (built on host to avoid container RAM limits)
        GEM5_MOUNT=()
        [ -d /opt/gem5 ] && GEM5_MOUNT=(-v /opt/gem5:/opt/gem5)

        docker run -d \
            --runtime=runc \
            --privileged \
            --name "$CONTAINER_NAME" \
            -v "$PROJECT_ROOT:/workspace" \
            "${GEM5_MOUNT[@]}" \
            -w /workspace \
            -e IN_RVPOINT_CONTAINER=1 \
            "$IMAGE_NAME" \
            tail -f /dev/null > /dev/null
        success "Container '$CONTAINER_NAME' created."
    fi
}

# Run a shell command inside the container
# Pass -i for interactive, -t for tty (default: both)
run_in_container() {
    ensure_container
    docker exec -it "$CONTAINER_NAME" bash -c "$1"
}

# ─── Task runners ─────────────────────────────────────────────────────────────

do_build() {
    local toolchain="${1:-linux}"
    ensure_container

    # Sync source files into the container.
    # Docker bind mounts don't propagate atomic file replacements (write+rename
    # creates a new inode that the container's overlay keeps stale). docker cp
    # always pushes the current host version regardless of inode changes.
    info "Syncing source files into container..."
    docker cp "$PROJECT_ROOT/src"          "$CONTAINER_NAME:/workspace/"
    docker cp "$PROJECT_ROOT/tests"        "$CONTAINER_NAME:/workspace/"
    docker cp "$PROJECT_ROOT/CMakeLists.txt" "$CONTAINER_NAME:/workspace/CMakeLists.txt"

    info "Building (toolchain: $toolchain)..."
    docker exec -it "$CONTAINER_NAME" bash -c "bash scripts/build.sh --toolchain $toolchain"
}

do_run_all_tests() {
    info "Running all tests..."
    run_in_container "bash scripts/run.sh test"
}

do_run_specific_test() {
    echo ""
    echo "Available tests:"
    echo "  scalar  vector  voxel_grid  sor  normal  radius  ransac  octree"
    echo "  spatial_hash_comparison  pipeline_walkthrough"
    echo ""
    read -rp "Test name(s) (space-separated): " test_names
    if [ -z "$test_names" ]; then
        warn "No test name entered."
        return
    fi
    info "Running: $test_names"
    run_in_container "bash scripts/run.sh test $test_names"
}

do_bench_per_algo() {
    info "Running per-algorithm rdinstret benchmark (N=1024)..."
    mkdir -p "$RESULTS_DIR"
    run_in_container "bash scripts/run.sh bench"
    echo ""
    latest=$(ls -t "$RESULTS_DIR"/benchmark_report_*.txt 2>/dev/null | head -1)
    if [ -n "$latest" ]; then
        success "Latest report: $latest"
    fi
}

do_bench_pipeline() {
    if ! docker exec "$CONTAINER_NAME" test -f /workspace/bin/benchmark_pipeline 2>/dev/null; then
        warn "bin/benchmark_pipeline not found. Building first..."
        do_build
    fi

    info "Running end-to-end pipeline benchmark..."
    mkdir -p "$RESULTS_DIR"
    run_in_container "
        QEMU=\$(command -v qemu-riscv64 2>/dev/null || echo /opt/qemu/bin/qemu-riscv64)
        mkdir -p results
        cd /workspace
        \"\$QEMU\" -cpu rv64,v=true,vlen=128 -L /opt/riscv/sysroot bin/benchmark_pipeline
    "
    echo ""
    latest=$(ls -t "$RESULTS_DIR"/benchmark_pipeline_*.txt 2>/dev/null | head -1)
    if [ -n "$latest" ]; then
        success "Latest report: $latest"
    fi
}

do_gem5_setup() {
    info "Setting up gem5 inside container (30-45 min build)..."
    warn "This downloads ~1 GB and compiles ~2 GB. Only needed once."
    read -rp "  Continue? [y/N] " confirm
    [[ "$confirm" =~ ^[Yy]$ ]] || { info "Cancelled."; return; }
    ensure_container
    docker cp "$PROJECT_ROOT/scripts/gem5_setup.sh" "$CONTAINER_NAME:/workspace/scripts/gem5_setup.sh"
    docker exec -it "$CONTAINER_NAME" bash scripts/gem5_setup.sh
}

do_gem5_bench() {
    ensure_container
    # Check gem5 exists inside container
    if ! docker exec "$CONTAINER_NAME" bash -c "test -f /opt/gem5/build/RISCV/gem5.opt || test -f /opt/gem5-25/build/RISCV/gem5.opt" 2>/dev/null; then
        warn "gem5 not found inside container. Run option 6 (gem5 Setup) first."
        return
    fi
    info "Running gem5 cycle-accurate benchmark (N=${GEM5_N:-512}, cpu=${GEM5_CPU:-o3})..."
    warn "Each algorithm pair takes ~2-5 minutes. Total: ~20-30 minutes."
    mkdir -p "$RESULTS_DIR"
    # Sync scripts and source into container
    docker cp "$PROJECT_ROOT/scripts/gem5_bench.sh"  "$CONTAINER_NAME:/workspace/scripts/gem5_bench.sh"
    docker cp "$PROJECT_ROOT/scripts/gem5_se.py"     "$CONTAINER_NAME:/workspace/scripts/gem5_se.py"
    docker cp "$PROJECT_ROOT/tests/benchmark_gem5.cpp" "$CONTAINER_NAME:/workspace/tests/benchmark_gem5.cpp"
    docker cp "$PROJECT_ROOT/src"                    "$CONTAINER_NAME:/workspace/"
    docker exec -it "$CONTAINER_NAME" bash -c \
        "GEM5_N=${GEM5_N:-512} GEM5_CPU=${GEM5_CPU:-o3} bash scripts/gem5_bench.sh"
    echo ""
    latest=$(ls -t "$RESULTS_DIR"/gem5_report_*.txt 2>/dev/null | head -1)
    [ -n "$latest" ] && success "Latest report: $latest"
}

do_shell() {
    info "Opening interactive shell in container '$CONTAINER_NAME'..."
    info "Type 'exit' to return to the menu."
    ensure_container
    docker exec -it "$CONTAINER_NAME" bash || true
}

do_rebuild_image() {
    echo ""
    warn "This will remove the existing image and rebuild from scratch (~15-20 min)."
    read -rp "Are you sure? [y/N] " confirm
    if [[ "$confirm" =~ ^[Yy]$ ]]; then
        # Stop and remove the running container first
        local existing
        existing=$(existing_container_id)
        if [ -n "$existing" ]; then
            info "Stopping and removing container '$CONTAINER_NAME'..."
            docker stop "$CONTAINER_NAME" &>/dev/null || true
            docker rm "$CONTAINER_NAME" &>/dev/null || true
        fi
        docker rmi "$IMAGE_NAME" &>/dev/null || true
        ensure_image
    else
        info "Cancelled."
    fi
}

do_clean_build() {
    info "Cleaning build directory..."
    run_in_container "rm -rf /workspace/build /workspace/bin && echo 'Build directory cleaned.'"
}

# ─── Status bar ───────────────────────────────────────────────────────────────

print_status() {
    local img_status container_status
    if docker image inspect "$IMAGE_NAME" &>/dev/null; then
        img_status="${GREEN}built${RESET}"
    else
        img_status="${RED}not built${RESET}"
    fi

    local cid
    cid=$(running_container_id)
    if [ -n "$cid" ]; then
        container_status="${GREEN}running (${cid:0:12})${RESET}"
    elif [ -n "$(existing_container_id)" ]; then
        container_status="${YELLOW}stopped${RESET}"
    else
        container_status="${RED}none${RESET}"
    fi

    local bin_status
    if [ -d "$BIN_DIR" ] && [ "$(ls -A "$BIN_DIR" 2>/dev/null)" ]; then
        bin_status="${GREEN}present${RESET}"
    else
        bin_status="${RED}not built${RESET}"
    fi

    echo -e "  Image: $img_status  |  Container: $container_status  |  Binaries: $bin_status"
}

# ─── Main menu ────────────────────────────────────────────────────────────────

print_menu() {
    clear
    echo -e "${BOLD}"
    echo "╔══════════════════════════════════════════════════════════════╗"
    echo "║         RVPoint — RISC-V Vector PCL Development CLI          ║"
    echo "╠══════════════════════════════════════════════════════════════╣"
    echo "║                                                              ║"
    echo "║  Build & Test                                                ║"
    echo "║    1)  Build  (default: linux toolchain)                     ║"
    echo "║    2)  Run All Tests                                         ║"
    echo "║    3)  Run Specific Test(s)                                  ║"
    echo "║                                                              ║"
    echo "║  Benchmarks                                                  ║"
    echo "║    4)  Per-Algorithm Benchmark  (rdinstret, N=1024)          ║"
    echo "║    5)  End-to-End Pipeline Benchmark  (all stages, multi-N)  ║"
    echo "║    6)  gem5 Setup  (build cycle-accurate simulator, ~40 min) ║"
    echo "║    7)  gem5 Benchmark  (hardware-accurate cycles, N=512)     ║"
    echo "║                                                              ║"
    echo "║  Container                                                   ║"
    echo "║    8)  Open Shell in Container                               ║"
    echo "║    9)  Rebuild Docker Image  (slow, only if env changed)     ║"
    echo "║    0)  Clean Build Directory                                 ║"
    echo "║                                                              ║"
    echo "║    q)  Quit                                                  ║"
    echo "║                                                              ║"
    echo "║  GEM5_N=<val> GEM5_CPU=<o3|minor|timing> to override defaults║"
    echo "║                                                              ║"
    echo "╚══════════════════════════════════════════════════════════════╝"
    echo -e "${RESET}"
    print_status
    echo ""
}

main() {
    # If called with arguments, skip menu and run directly
    if [ $# -gt 0 ]; then
        case "$1" in
            build)           do_build "${2:-linux}" ;;
            test)            shift; do_run_all_tests ;;
            bench)           do_bench_per_algo ;;
            bench-pipeline)  do_bench_pipeline ;;
            gem5-setup)      do_gem5_setup ;;
            gem5-bench)      do_gem5_bench ;;
            shell)           do_shell ;;
            *)               error "Unknown command: $1"; exit 1 ;;
        esac
        exit 0
    fi

    # Interactive menu loop
    while true; do
        print_menu
        read -rp "  Select option: " choice
        echo ""
        case "$choice" in
            1) do_build ;;
            2) do_run_all_tests ;;
            3) do_run_specific_test ;;
            4) do_bench_per_algo ;;
            5) do_bench_pipeline ;;
            6) do_gem5_setup ;;
            7) do_gem5_bench ;;
            8) do_shell ;;
            9) do_rebuild_image ;;
            0) do_clean_build ;;
            q|Q) info "Goodbye."; exit 0 ;;
            *) warn "Invalid option: '$choice'" ;;
        esac
        echo ""
        read -rp "  Press Enter to return to menu..."
    done
}

main "$@"
