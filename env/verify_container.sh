#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$PROJECT_ROOT/scripts/lib/common.sh"
wsl_bootstrap "env/verify_container.sh" "$@"

exec "$PROJECT_ROOT/scripts/verify_container.sh" "$@"
