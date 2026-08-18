#!/usr/bin/env bash
# === RVPoint Gem5 Docker Wrapper ===
# Runs the precompiled gem5.opt inside the manuel313/gem5_v25 docker container.
# It automatically mounts the project root and build directory so all file paths resolve correctly.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Load common utils to resolve the correct build directory
source "$PROJECT_ROOT/scripts/lib/common.sh"
BUILD_DIR="$(get_build_dir)"

# Define container paths and image
GEM5_IMAGE="manuel313/gem5_v25"
GEM5_BIN_IN_CONTAINER="/gem5/build/RISCV/gem5.opt"

# Check if docker command is available
if ! command -v docker &> /dev/null; then
  echo "Error: docker command not found. Please install Docker." >&2
  exit 1
fi

# Run gem5 inside Docker
# Map host UID/GID so generated stats/logs have the correct permissions.
# Mount both the project root and the build directory at their identical absolute paths.
exec docker run --rm \
  -u "$(id -u):$(id -g)" \
  -v "$PROJECT_ROOT:$PROJECT_ROOT" \
  -v "$BUILD_DIR:$BUILD_DIR" \
  -w "$PROJECT_ROOT" \
  "$GEM5_IMAGE" \
  "$GEM5_BIN_IN_CONTAINER" "$@"
