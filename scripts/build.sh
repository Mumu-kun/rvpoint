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
BACKEND="rvv"
TARGET=""

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
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --target)
            TARGET="$2"
            shift 2
            ;;
        *)
            echo "Usage: $0 [--toolchain linux|elf] [--backend rvv|riscv] [--target <cmake-target>] [--clean]"
            exit 1
            ;;
    esac
done

case "$BACKEND" in
    rvv)
        RISCV_ARCH="rv64gcv"
        RVV_CMAKE="ON"
        ;;
    riscv|scalar|normal)
        RISCV_ARCH="rv64gc"
        RVV_CMAKE="OFF"
        BACKEND="riscv"
        ;;
    *)
        echo "Error: unknown backend '$BACKEND'. Use 'riscv' or 'rvv'."
        exit 1
        ;;
esac

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
BACKEND_MARKER="$BUILD_DIR/.backend"
if [ -f "$MARKER" ] && [ "$(cat "$MARKER")" != "$TOOLCHAIN" ]; then
    echo "Toolchain changed ($(cat "$MARKER") -> $TOOLCHAIN), reconfiguring..."
    rm -rf "$BUILD_DIR"
fi
if [ -f "$BACKEND_MARKER" ] && [ "$(cat "$BACKEND_MARKER")" != "$BACKEND" ]; then
    echo "Backend changed ($(cat "$BACKEND_MARKER") -> $BACKEND), reconfiguring..."
    rm -rf "$BUILD_DIR"
fi

# Auto-clean if CMake cache is pinned to a non-RISC-V compiler.
CACHE_FILE="$BUILD_DIR/CMakeCache.txt"
if [ -f "$CACHE_FILE" ]; then
    CXX_COMPILER="$(sed -n 's/^CMAKE_CXX_COMPILER:FILEPATH=//p' "$CACHE_FILE" | head -n 1)"
    CACHE_ARCH="$(sed -n 's/^RISCV_ARCH:STRING=//p' "$CACHE_FILE" | head -n 1)"
    CACHE_RVV="$(sed -n 's/^RVV_PCL_USE_RVV:BOOL=//p' "$CACHE_FILE" | head -n 1)"
    if [ -n "$CXX_COMPILER" ] && [[ "$CXX_COMPILER" != *riscv64* ]]; then
        echo "Detected stale host compiler in CMake cache ($CXX_COMPILER), reconfiguring..."
        rm -rf "$BUILD_DIR"
    elif [ -n "$CACHE_ARCH" ] && [ "$CACHE_ARCH" != "$RISCV_ARCH" ]; then
        echo "Detected stale ISA in CMake cache ($CACHE_ARCH -> $RISCV_ARCH), reconfiguring..."
        rm -rf "$BUILD_DIR"
    elif [ -n "$CACHE_RVV" ] && [ "$CACHE_RVV" != "$RVV_CMAKE" ]; then
        echo "Detected stale RVV setting in CMake cache ($CACHE_RVV -> $RVV_CMAKE), reconfiguring..."
        rm -rf "$BUILD_DIR"
    fi
fi

# Configure if needed
if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
    echo "Configuring CMake (toolchain: $TOOLCHAIN, backend: $BACKEND)..."
    cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
        -DRISCV_ARCH="$RISCV_ARCH" \
        -DRISCV_ABI="lp64d" \
        -DRVV_PCL_USE_RVV="$RVV_CMAKE"
    echo "$TOOLCHAIN" > "$MARKER"
    echo "$BACKEND" > "$BACKEND_MARKER"
fi

# Build
if [ -n "$TARGET" ]; then
    echo "Building target '$TARGET'..."
    cmake --build "$BUILD_DIR" --target "$TARGET" -j"$(nproc)"
else
    echo "Building..."
    cmake --build "$BUILD_DIR" -j"$(nproc)"
fi

echo "Build complete. Binaries in: ${PROJECT_ROOT}/bin/"
