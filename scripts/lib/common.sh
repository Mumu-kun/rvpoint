#!/bin/bash
# Common library for RVPoint scripts

WSL_DISTRO="rvpoint"

LIB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd "${LIB_DIR}/../.." && pwd)}"
source "${PROJECT_ROOT}/env/common.sh"

# --- Build Directory Resolution ---
get_build_dir() {
    if [ -n "${RVPOINT_BUILD_DIR:-}" ]; then
        echo "$RVPOINT_BUILD_DIR"
    elif [ -n "${HOME:-}" ] && [ -d "$HOME" ]; then
        echo "${HOME}/.cache/rvpoint/build"
    else
        echo "/tmp/rvpoint_build"
    fi
}

# --- WSL2 bootstrap ---
wsl_bootstrap() {
    local script_path="$1"
    shift
    local script_args=("$@")
    
    # Only route to WSL2 when on Windows Git Bash and WSL2 is available
    if is_windows_gitbash; then
        if ! is_wsl_available "$WSL_DISTRO"; then
            echo "Error: WSL2 distribution '${WSL_DISTRO}' not found." >&2
            echo "Please run: ./env/setup.sh" >&2
            exit 1
        fi
        
        # Convert Windows path to WSL path
        local linux_path
        if command -v wslpath &>/dev/null; then
            linux_path=$(wslpath -u "$PROJECT_ROOT")
        else
            linux_path=$(wslpath_to_wsl "$PROJECT_ROOT")
        fi
        
        exec wsl -d "$WSL_DISTRO" -u root -e bash -c "cd '$linux_path' && exec bash '$script_path' ${script_args[*]}"
    fi
}