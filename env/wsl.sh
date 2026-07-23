#!/bin/bash
# Quick launcher to enter rvpoint WSL distro in current working directory and source env/activate.sh
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

CURRENT_DIR="$(pwd)"
WSL_PATH=$(wslpath_to_wsl "$CURRENT_DIR")

if [ $# -gt 0 ]; then
    exec wsl -d rvpoint --cd "$WSL_PATH" bash -c "if [ -f env/activate.sh ]; then source env/activate.sh; fi; exec \"\$@\"" bash "$@"
else
    exec wsl -d rvpoint --cd "$WSL_PATH" bash -c "if [ -f ~/.bashrc ]; then source ~/.bashrc 2>/dev/null; fi; if [ -f env/activate.sh ]; then source env/activate.sh; fi; exec bash -i"
fi
