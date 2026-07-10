#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN_DIR="$PROJECT_ROOT/bin"

# --- Auto-launch in Docker if not inside a container ---
if [ ! -f /.dockerenv ] && [ -z "$IN_RVPOINT_CONTAINER" ]; then
    IMAGE="${RVPOINT_IMAGE:-rvpoint}"
    if ! docker image inspect "$IMAGE" &> /dev/null; then
        echo "Docker image '$IMAGE' not found. Building from .devcontainer/Dockerfile..."
        docker build -f "$PROJECT_ROOT/.devcontainer/Dockerfile" -t "$IMAGE" "$PROJECT_ROOT"
    fi
    exec docker run --rm -e TERM="$TERM" -e IN_RVPOINT_CONTAINER=1 \
        -v "$PROJECT_ROOT:/workspace" -w /workspace \
        "$IMAGE" bash scripts/run.sh "$@"
fi

# All available test binaries (order: basic -> algorithm-specific -> integration)
# Auto-discovered from bin/ at runtime; kept here for listing and deterministic order.
ALL_TESTS=(
    test_scalar
    test_loader
    test_normal
    test_ransac
    test_sor
    test_octree
    test_radius
    test_voxel_grid
    test_voxel_linux
    test_pipeline_walkthrough
    test_spatial_hash_comparison
    # RVV-only tests (built when RVV is enabled)
    test_vector
    rvv_test
    test_rvv_features
    test_tuples
)

# Defaults
TOOLCHAIN="linux"
MODE=""
TARGETS=()
QEMU_FLAGS=(-cpu "rv64,v=true,vlen=128")

# --- Helpers ---
usage() {
    cat <<'EOF'
Usage: run.sh [options] <mode> [targets...]

Modes:
  test   [names...]    Run tests. No names = all tests.
    bench                Run legacy rdinstret benchmark (all algorithms).

Options:
  --toolchain <elf|linux>   Toolchain for build (default: linux)
  --list                    List available test and benchmark targets
  --help                    Show this help

Examples:
  ./run.sh test                        # all tests
  ./run.sh test voxel_grid sor         # only voxel_grid and sor
    ./run.sh bench                       # legacy benchmark (use ./bench for gem5)
  ./run.sh --toolchain linux test      # build with linux toolchain, run tests
EOF
    exit 0
}

list_targets() {
    echo "Available tests:"
    for t in "${ALL_TESTS[@]}"; do
        echo "  $t"
    done
    echo ""
    echo "Benchmark: legacy rdinstret runner; use ./bench <mode> <kernel> <size> for gem5"
    exit 0
}

find_qemu() {
    if command -v qemu-riscv64 &> /dev/null; then
        echo "qemu-riscv64"
    elif [ -f "/opt/qemu/bin/qemu-riscv64" ]; then
        echo "/opt/qemu/bin/qemu-riscv64"
    else
        echo "Error: qemu-riscv64 not found" >&2
        exit 1
    fi
}

# --- Parse arguments ---
while [[ $# -gt 0 ]]; do
    case "$1" in
        --toolchain)
            TOOLCHAIN="$2"
            shift 2
            ;;
        --list)
            list_targets
            ;;
        --help|-h)
            usage
            ;;
        test|bench)
            MODE="$1"
            shift
            # Remaining positional args are targets
            while [[ $# -gt 0 && ! "$1" =~ ^-- ]]; do
                TARGETS+=("$1")
                shift
            done
            ;;
        *)
            echo "Unknown argument: $1"
            usage
            ;;
    esac
done

if [ -z "$MODE" ]; then
    echo "Error: no mode specified."
    echo ""
    usage
fi

# --- Build ---
echo "==> Building (toolchain: $TOOLCHAIN)..."
"$SCRIPT_DIR/build.sh" --toolchain "$TOOLCHAIN"
echo ""

QEMU_BIN=$(find_qemu)

# Linux toolchain binaries need sysroot for the dynamic linker
if [ "$TOOLCHAIN" = "linux" ]; then
    QEMU_FLAGS=(-L "${RISCV_PATH:-/opt/riscv}/sysroot" "${QEMU_FLAGS[@]}")
fi

# --- Run tests ---
if [ "$MODE" = "test" ]; then
    if [ ${#TARGETS[@]} -eq 0 ]; then
        TARGETS=("${ALL_TESTS[@]}")
    else
        # Normalize: allow "voxel_grid" or "test_voxel_grid"
        NORMALIZED=()
        for t in "${TARGETS[@]}"; do
            if [[ "$t" != test_* && "$t" != rvv_test ]]; then
                NORMALIZED+=("test_$t")
            else
                NORMALIZED+=("$t")
            fi
        done
        TARGETS=("${NORMALIZED[@]}")
    fi

    PASSED=0
    FAILED=0
    FAILURES=()

    echo "==> Running ${#TARGETS[@]} test(s)"
    echo "------------------------------------------------------------"

    for t in "${TARGETS[@]}"; do
        BINARY="$BIN_DIR/$t"
        if [ ! -f "$BINARY" ]; then
            echo "  SKIP  $t  (binary not found)"
            continue
        fi

        printf "  %-35s " "$t"
        if OUTPUT=$("$QEMU_BIN" "${QEMU_FLAGS[@]}" "$BINARY" 2>&1); then
            if echo "$OUTPUT" | grep -qi "PASS\|verification"; then
                echo "PASS"
                PASSED=$((PASSED + 1))
            else
                echo "WARN (exit 0, no PASS marker)"
                echo "$OUTPUT" | head -5
                PASSED=$((PASSED + 1))
            fi
        else
            echo "FAIL"
            echo "$OUTPUT" | tail -5
            FAILED=$((FAILED + 1))
            FAILURES+=("$t")
        fi
    done

    echo "------------------------------------------------------------"
    echo "Results: $PASSED passed, $FAILED failed"
    if [ $FAILED -gt 0 ]; then
        echo "Failed: ${FAILURES[*]}"
        exit 1
    fi
fi

# --- Run benchmarks ---
if [ "$MODE" = "bench" ]; then
    mkdir -p "$PROJECT_ROOT/results"

    echo "==> Running rdinstret benchmark (all algorithms, N=1024)"
    echo ""

    # The benchmark binary uses rdinstret to count instructions internally.
    # It outputs a formatted table to stdout AND saves a timestamped report
    # to results/benchmark_report_<timestamp>.txt
    pushd "$PROJECT_ROOT" > /dev/null
    "$QEMU_BIN" "${QEMU_FLAGS[@]}" "$BIN_DIR/benchmark"
    popd > /dev/null

    echo ""
    echo "==> Timestamped report saved to results/"
fi
