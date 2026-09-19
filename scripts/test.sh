#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$SCRIPT_DIR/lib/common.sh"
wsl_bootstrap "scripts/test.sh" "$@"

BUILD_DIR="$(get_build_dir)"
source "${PROJECT_ROOT}/env/activate.sh"

TOOLCHAIN="linux"
BACKEND="rvv"
RUN_ALL=false

while [[ $# -gt 0 ]]; do
    case "$1" in
        --all|--experimental)
            RUN_ALL=true
            shift
            ;;
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --toolchain)
            TOOLCHAIN="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 [--all|--experimental] [--backend rvv|scalar] [--toolchain linux|elf]"
            exit 1
            ;;
    esac
done

echo "============================================================"
echo " RVPoint Test Suite Runner [$BACKEND]"
echo " Mode: $([ "$RUN_ALL" = true ] && echo "All (Fast + Experimental)" || echo "Fast Essential Tests")"
echo "============================================================"

# Ensure build is up-to-date
"$SCRIPT_DIR/build.sh" --toolchain "$TOOLCHAIN" --backend "$BACKEND"

QEMU_BIN="$(find_qemu)"
QEMU_FLAGS=(${QEMU_SYSROOT_FLAGS:-})
if [ "$BACKEND" = "rvv" ]; then
    QEMU_FLAGS+=("-cpu" "rv64,v=true,vlen=128")
else
    QEMU_FLAGS+=("-cpu" "rv64")
fi

BIN_DIR="${BUILD_DIR}/${BACKEND}/bin/${BACKEND}"

# List of fast tests
FAST_TESTS=(
    "test_voxel_grid"
    "test_sor"
    "test_normal"
    "test_ransac"
    "test_octree"
    "test_radius"
    "test_euclidean_clustering"
    "test_pipeline_walkthrough"
    "test_scaling"
    "test_loader"
)
if [ "$BACKEND" = "rvv" ]; then
    FAST_TESTS+=("test_rvv_features")
fi

TESTS_TO_RUN=("${FAST_TESTS[@]}")

if [ "$RUN_ALL" = true ]; then
    for exp_file in "${PROJECT_ROOT}"/eval/tests/experimental/*.cpp; do
        t_name="$(basename "$exp_file" .cpp)"
        TESTS_TO_RUN+=("$t_name")
    done
fi

PASSED=0
FAILED=0
TOTAL=0

for test_name in "${TESTS_TO_RUN[@]}"; do
    test_bin="$BIN_DIR/$test_name"
    if [ ! -f "$test_bin" ]; then
        test_bin="${BUILD_DIR}/${BACKEND}/bin/$test_name"
    fi
    if [ ! -f "$test_bin" ]; then
        continue
    fi

    TOTAL=$((TOTAL + 1))
    echo -n "[$TOTAL] Running $test_name... "
    if "$QEMU_BIN" "${QEMU_FLAGS[@]}" "$test_bin" > /dev/null 2>&1; then
        echo "PASS"
        PASSED=$((PASSED + 1))
    else
        echo "FAIL"
        FAILED=$((FAILED + 1))
    fi
done

echo "============================================================"
echo " Summary: $PASSED / $TOTAL passed ($FAILED failed)"
echo "============================================================"

if [ "$FAILED" -ne 0 ]; then
    exit 1
fi
