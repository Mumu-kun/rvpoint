#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/sync_pi.sh
# PURPOSE: Smart rsync from Windows host to Orange Pi RV2
#          Supports .gitignore filtering and relative path file syncing.
# ==============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
HOST_TARGET="${HOST_TARGET:-orangepi@orangepirv2:~/projects/rvpoint/}"

SSH_CMD="ssh"
if command -v cygpath &>/dev/null || [[ "${OSTYPE:-}" == "msys"* ]]; then
    if [ -f "C:/Windows/System32/OpenSSH/ssh.exe" ]; then
        SSH_CMD="C:/Windows/System32/OpenSSH/ssh.exe"
    fi
fi

if [ $# -eq 0 ]; then
    echo "==> Syncing entire repo to $HOST_TARGET (strictly adhering to .gitignore)..."
    rsync -avz \
        -e "$SSH_CMD" \
        --filter=':- .gitignore' \
        --exclude='.git/' \
        --delete-excluded \
        "$PROJECT_ROOT/" "$HOST_TARGET"
else
    echo "==> Syncing specific target(s): $@ to $HOST_TARGET ..."
    cd "$PROJECT_ROOT"
    rsync -avz -R -e "$SSH_CMD" "$@" "$HOST_TARGET"
fi

echo "==> Sync complete!"
