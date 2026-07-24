#!/usr/bin/env bash
set -euo pipefail

# === RVPoint Gem5 Installation Script (Docker-based) ===
# Pulls the prebuilt manuel313/gem5_v25 image and sets up execution wrappers.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Ensure docker is installed
if ! command -v docker &> /dev/null; then
  echo "Error: docker command not found. Please install Docker in WSL." >&2
  exit 1
fi

echo "Pulling Docker image 'manuel313/gem5_v25'..."
docker pull manuel313/gem5_v25

echo ""
echo "gem5 Docker integration setup completed successfully."
echo "Set the following environment variables (or source env/activate.sh):"
echo "  export GEM5_BIN=$PROJECT_ROOT/env/extras/gem5_docker.sh"
echo "  export GEM5_CONFIG=/gem5/configs/deprecated/example/se.py"