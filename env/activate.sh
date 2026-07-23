#!/bin/bash
# Source this file to set up environment variables

ACTIVATE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$ACTIVATE_DIR/.." && pwd)}"

source "${ACTIVATE_DIR}/common.sh"
setup_env_paths