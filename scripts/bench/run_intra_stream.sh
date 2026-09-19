#!/usr/bin/env bash
# ==============================================================================
# scripts/bench/run_intra_stream.sh
# ==============================================================================
# Continuous Stream Benchmark Runner for RVPoint pipeline_3d_intra
# Runs scalable 4-core and 8-core spatial slab streaming benchmarks.
# ==============================================================================

set -e

# Default settings
INPUT_DIR="${1:-data/pcd_compressed/}"
THREADS_LIST=("4" "8")
MAX_FRAMES="${2:-}"
LEAF_SIZE="0.10"
CLUSTER_TOL="0.15"
MIN_CLUSTER="50"
MAX_CLUSTER="100000"
ROR_MIN_PTS="3"
ROR_RADIUS="0.25"
RANSAC_ITERS="100"

# Find binary
BINARY=""
for cand in "./build/rvv/bin/rvv/pipeline_3d_intra" \
            "/root/.cache/rvpoint/build/rvv/bin/rvv/pipeline_3d_intra" \
            "build/rvv/bin/rvv/pipeline_3d_intra"; do
    if [ -x "$cand" ]; then
        BINARY="$cand"
        break
    fi
done

if [ -z "$BINARY" ]; then
    echo "[ERROR] pipeline_3d_intra binary not found. Run ./scripts/build.sh --target pipeline_3d_intra first."
    exit 1
fi

echo "========================================================================"
echo "  RVPoint Scalable Threading Benchmark: Intra-Frame Continuous Stream   "
echo "========================================================================"
echo "  Binary        : $BINARY"
echo "  Input Path    : $INPUT_DIR"
echo "  Core Counts   : ${THREADS_LIST[*]}"
[ -n "$MAX_FRAMES" ] && echo "  Max Frames    : $MAX_FRAMES"
echo "========================================================================"

EXTRA_ARGS=()
if [ -n "$MAX_FRAMES" ]; then
    EXTRA_ARGS+=(--max-frames "$MAX_FRAMES")
fi

for T in "${THREADS_LIST[@]}"; do
    echo ""
    echo ">>> EXECUTING $T-CORE INTRA-FRAME STREAM BENCHMARK <<<"
    "$BINARY" "$INPUT_DIR" \
        --threads "$T" \
        --leaf-size "$LEAF_SIZE" \
        --ror-radius "$ROR_RADIUS" \
        --ror-min-pts "$ROR_MIN_PTS" \
        --ransac-iters "$RANSAC_ITERS" \
        --cluster-tolerance "$CLUSTER_TOL" \
        --min-cluster "$MIN_CLUSTER" \
        --max-cluster "$MAX_CLUSTER" \
        --no-write \
        "${EXTRA_ARGS[@]}"
done

echo ""
echo "========================================================================"
echo "  BENCHMARK RUNS COMPLETE"
echo "========================================================================"
