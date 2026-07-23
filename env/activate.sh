#!/bin/bash
# Source this file to set up environment variables

# Load default user bashrc first if interactive
if [ -n "$PS1" ] && [ -f "$HOME/.bashrc" ]; then
    source "$HOME/.bashrc" 2>/dev/null
fi

ACTIVATE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$ACTIVATE_DIR/.." && pwd)}"

source "${ACTIVATE_DIR}/common.sh"
setup_env_paths

if [ -n "$PS1" ]; then
    echo "=== RVPoint Environment Activated ==="
fi