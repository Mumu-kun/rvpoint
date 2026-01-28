#!/bin/bash
set -e

# Resolve project root (parent of scripts/)
PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"

# --- Auto-launch in Docker if not inside a container ---
if [ ! -f /.dockerenv ] && [ -z "$IN_RVPOINT_CONTAINER" ]; then
    IMAGE="${RVPOINT_IMAGE:-rvpoint}"
    if ! docker image inspect "$IMAGE" &> /dev/null; then
        echo "Docker image '$IMAGE' not found. Building from .devcontainer/Dockerfile..."
        docker build -f "$PROJECT_ROOT/.devcontainer/Dockerfile" -t "$IMAGE" "$PROJECT_ROOT"
    fi
    exec docker run --rm -e IN_RVPOINT_CONTAINER=1 \
        -v "$PROJECT_ROOT:/workspace" -w /workspace \
        "$IMAGE" bash scripts/build.sh "$@"
fi

# Defaults
TOOLCHAIN="linux"
CLEAN=false

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --toolchain)
            TOOLCHAIN="$2"
            shift 2
            ;;
        --clean)
            CLEAN=true
            shift
            ;;
        *)
            echo "Usage: $0 [--toolchain linux|elf] [--clean]"
            exit 1
            ;;
    esac
done

# Map toolchain name to cmake file
case "$TOOLCHAIN" in
    linux)
        TOOLCHAIN_FILE="${PROJECT_ROOT}/cmake/riscv_linux.cmake"
        ;;
    elf)
        TOOLCHAIN_FILE="${PROJECT_ROOT}/cmake/riscv.cmake"
        ;;
    *)
        echo "Error: unknown toolchain '$TOOLCHAIN'. Use 'linux' or 'elf'."
        exit 1
        ;;
esac

# Clean if requested
if [ "$CLEAN" = true ]; then
    echo "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Auto-clean on toolchain mismatch
MARKER="$BUILD_DIR/.toolchain"
if [ -f "$MARKER" ] && [ "$(cat "$MARKER")" != "$TOOLCHAIN" ]; then
    echo "Toolchain changed ($(cat "$MARKER") -> $TOOLCHAIN), reconfiguring..."
    rm -rf "$BUILD_DIR"
fi

# Configure if needed
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring CMake (toolchain: $TOOLCHAIN)..."
    cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE"
    echo "$TOOLCHAIN" > "$MARKER"
fi

# Build
echo "Building..."
cmake --build "$BUILD_DIR" -j"$(nproc)"

echo "Build complete. Binaries in: ${PROJECT_ROOT}/bin/"
