#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BIN_DIR="$PROJECT_ROOT/bin"

if [ ! -f /.dockerenv ] && [ -z "$IN_RVPOINT_CONTAINER" ]; then
    IMAGE="${RVPOINT_IMAGE:-rvpoint}"
    if ! docker image inspect "$IMAGE" &> /dev/null; then
        docker build -f "$PROJECT_ROOT/.devcontainer/Dockerfile" -t "$IMAGE" "$PROJECT_ROOT"
    fi
    exec docker run --rm -e TERM="$TERM" -e IN_RVPOINT_CONTAINER=1 \
        -v "$PROJECT_ROOT:/workspace" -w /workspace \
        "$IMAGE" bash scripts/export_pipeline.sh "$@"
fi

usage() {
    echo "Usage: $0 [--backend riscv|rvv] [--progress] <input.pcd> [output_dir]"
    exit 1
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

BACKEND="rvv"
PROGRESS=false
POSITIONAL=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --progress|--timings)
            PROGRESS=true
            shift
            ;;
        *)
            POSITIONAL+=("$1")
            shift
            ;;
    esac
done

set -- "${POSITIONAL[@]}"
if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    usage
fi

"$SCRIPT_DIR/build.sh" --toolchain linux --backend "$BACKEND" --target pipeline_export

QEMU_BIN="$(find_qemu)"
if [ "$BACKEND" = "rvv" ]; then
    QEMU_FLAGS=(-L "${RISCV_PATH:-/opt/riscv}/sysroot" -cpu "rv64,v=true,vlen=128")
else
    QEMU_FLAGS=(-L "${RISCV_PATH:-/opt/riscv}/sysroot" -cpu "rv64")
fi

PIPELINE_ARGS=()
if [ "$PROGRESS" = true ]; then
    PIPELINE_ARGS+=(--progress)
fi
PIPELINE_ARGS+=("$@")

exec "$QEMU_BIN" "${QEMU_FLAGS[@]}" "$BIN_DIR/pipeline_export" "${PIPELINE_ARGS[@]}"
