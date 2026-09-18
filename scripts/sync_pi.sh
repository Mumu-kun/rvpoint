#!/usr/bin/env bash
# ==============================================================================
# SCRIPT: scripts/sync_pi.sh
# PURPOSE: Smart rsync / scp from Windows host to Orange Pi RV2
#          Supports automatic alias resolution (.local / mDNS / router DNS),
#          custom hosts/IPs, .gitignore filtering, and file syncing.
# ==============================================================================

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
PI_HOST_FILE="$PROJECT_ROOT/.pi_host"
DEFAULT_REMOTE_DIR="/home/orangepi/projects/rvpoint"

probe_host() {
    local host="$1"
    if command -v ping.exe &>/dev/null; then
        ping.exe -n 1 -w 700 "$host" >/dev/null 2>&1
    else
        ping -c 1 -W 1 "$host" >/dev/null 2>&1
    fi
}

PI_HOST=""
TARGET_FILES=()

# 1. Parse CLI flags and targets
while [[ $# -gt 0 ]]; do
    case "$1" in
        --set-host)
            if [[ $# -lt 2 ]]; then
                echo "[-] Error: --set-host requires an IP or hostname argument." >&2
                exit 1
            fi
            PI_HOST="$2"
            echo "$PI_HOST" > "$PI_HOST_FILE"
            echo "[+] Saved default Pi host '$PI_HOST' to .pi_host"
            shift 2
            ;;
        --host|-h)
            if [[ $# -lt 2 ]]; then
                echo "[-] Error: --host requires an IP or hostname argument." >&2
                exit 1
            fi
            PI_HOST="$2"
            shift 2
            ;;
        *)
            # Clean up path (strip leading ./)
            clean_arg="${1#./}"
            TARGET_FILES+=("$clean_arg")
            shift
            ;;
    esac
done

# 2. Check environment variable
if [[ -z "$PI_HOST" && -n "${HOST_TARGET:-}" ]]; then
    PI_HOST="$HOST_TARGET"
fi

# 3. Check persistent .pi_host file
if [[ -z "$PI_HOST" && -f "$PI_HOST_FILE" ]]; then
    SAVED_HOST="$(head -n 1 "$PI_HOST_FILE" | tr -d '\r\n[:space:]')"
    if [[ -n "$SAVED_HOST" ]]; then
        PI_HOST="$SAVED_HOST"
    fi
fi

# 4. Auto-detect alias (mDNS vs local router DNS) if still not set
if [[ -z "$PI_HOST" ]]; then
    echo "[*] Auto-detecting Orange Pi on current network..."
    for candidate in "orangepirv2.local" "orangepirv2" "orangepi.local" "orangepi"; do
        if probe_host "$candidate"; then
            echo "[+] Discovered active Pi at '$candidate'!"
            PI_HOST="$candidate"
            break
        fi
    done
fi

# 5. Final fallback
if [[ -z "$PI_HOST" ]]; then
    PI_HOST="orangepirv2.local"
fi

# Parse remote user, host, and path
if [[ "$PI_HOST" == *:* ]]; then
    REMOTE_HOST="${PI_HOST%%:*}"
    REMOTE_PATH="${PI_HOST#*:}"
else
    if [[ "$PI_HOST" == *@* ]]; then
        REMOTE_HOST="$PI_HOST"
    else
        REMOTE_HOST="orangepi@$PI_HOST"
    fi
    REMOTE_PATH="$DEFAULT_REMOTE_DIR"
fi

SSH_CMD="ssh"

# Ensure remote base directory exists
$SSH_CMD "$REMOTE_HOST" "mkdir -p '$REMOTE_PATH'" >/dev/null 2>&1 || true

cd "$PROJECT_ROOT"

if [ ${#TARGET_FILES[@]} -eq 0 ]; then
    echo "==> Syncing entire repo to $REMOTE_HOST:$REMOTE_PATH (strictly adhering to .gitignore)..."
    rsync -avz \
        --rsync-path="mkdir -p '$REMOTE_PATH' && rsync" \
        -e "$SSH_CMD" \
        --filter=':- .gitignore' \
        --exclude='.git/' \
        --delete-excluded \
        "$PROJECT_ROOT/" "${REMOTE_HOST}:${REMOTE_PATH}/"
else
    echo "==> Syncing specific target(s): ${TARGET_FILES[*]} to $REMOTE_HOST:$REMOTE_PATH ..."
    for file in "${TARGET_FILES[@]}"; do
        if [[ ! -e "$file" ]]; then
            echo "[-] Error: Local file '$file' does not exist." >&2
            exit 1
        fi
        
        parent_dir="$(dirname "$file")"
        target_dir="$REMOTE_PATH"
        if [[ "$parent_dir" != "." ]]; then
            target_dir="$REMOTE_PATH/$parent_dir"
            $SSH_CMD "$REMOTE_HOST" "mkdir -p '$target_dir'"
        fi

        echo "[*] Transferring $file -> $target_dir/$(basename "$file") ..."
        scp "$file" "${REMOTE_HOST}:${target_dir}/$(basename "$file")"
    done
fi

echo "==> Sync complete! Files verified on Pi at: $REMOTE_PATH/"
