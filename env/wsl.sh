#!/bin/bash
# Quick launcher to enter rvpoint WSL distro in current working directory and source env/activate.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

if [ "$(detect_platform)" != "wsl2" ] && is_windows_gitbash; then
    if command -v cygpath &>/dev/null; then
        TARGET_PATH="$(cygpath -w "$(pwd)")"
    elif pwd -W &>/dev/null; then
        TARGET_PATH="$(pwd -W)"
    else
        TARGET_PATH="$(pwd)"
    fi
    
    WSL_BIN="/c/Windows/System32/wsl.exe"
    if [ ! -x "$WSL_BIN" ]; then
        WSL_BIN="wsl.exe"
    fi

    if [ $# -gt 0 ]; then
        exec "$WSL_BIN" -d rvpoint --cd "$TARGET_PATH" bash -c "source env/activate.sh; exec \"\$@\"" bash "$@"
    else
        exec "$WSL_BIN" -d rvpoint --cd "$TARGET_PATH" bash --rcfile env/activate.sh -i
    fi
else
    # Already inside WSL / Linux
    if [ -f "${SCRIPT_DIR}/activate.sh" ]; then
        source "${SCRIPT_DIR}/activate.sh"
    fi
    if [ $# -gt 0 ]; then
        exec "$@"
    fi
fi
