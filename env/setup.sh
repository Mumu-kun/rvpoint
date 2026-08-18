#!/bin/bash
# RVPoint Setup - Single entry point for Windows Git Bash or WSL
set -e

# Get absolute path to this script's directory and project root
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Source common utilities
source "${SCRIPT_DIR}/common.sh"

echo "=== RVPoint Environment Setup ==="

# --- Windows Git Bash Path ---
if is_windows_gitbash; then
    # Check for Administrator privileges (required for wsl --import)
    if ! wsl --status >/dev/null 2>&1; then
        echo "Error: Administrator privileges required for WSL2 import." >&2
        echo "Please run Git Bash as Administrator." >&2
        exit 1
    fi

    DISTRO_NAME="rvpoint"
    IMAGE_URL="https://cloud-images.ubuntu.com/wsl/releases/24.04/current/ubuntu-noble-wsl-amd64-wsl.rootfs.tar.gz"
    WSL_ROOT="$PROJECT_ROOT/wsl"
    INSTALL_DIR="$WSL_ROOT/$DISTRO_NAME"
    ROOTFS_TAR="$WSL_ROOT/ubuntu-noble-rootfs.tar.gz"

    mkdir -p "$WSL_ROOT"

    # Check if rvpoint is already installed
    if [ ! -f "$INSTALL_DIR/ext4.vhdx" ] && ! is_wsl_available "$DISTRO_NAME"; then
        # Download only if the rootfs isn't already present
        if [ ! -f "$ROOTFS_TAR" ]; then
            echo "Downloading Ubuntu 24.04 WSL image..."
            curl -L \
                 --progress-bar \
                 --retry 5 \
                 --retry-delay 5 \
                 -o "$ROOTFS_TAR" \
                 "$IMAGE_URL"
        else
            echo "Using existing Ubuntu WSL image: $ROOTFS_TAR"
        fi

        echo "Importing rvpoint WSL..."
        mkdir -p "$INSTALL_DIR"
        wsl --import "$DISTRO_NAME" "$INSTALL_DIR" "$ROOTFS_TAR" --version 2
        echo "rvpoint WSL imported successfully."
    fi

    # Run Linux setup inside WSL2
    # Use wslpath if available, otherwise inline conversion
    if command -v wslpath &>/dev/null; then
        LINUX_PATH=$(wslpath -u "$PROJECT_ROOT")
        wsl -d "$DISTRO_NAME" -u root -e bash -c "cd '$LINUX_PATH' && bash env/linux/install.sh"
    else
        # Fallback: WSL auto-mounts Windows drives at /mnt/<drive>
        # Git Bash msys-style path: /e/path -> WSL: /mnt/e/path
        LINUX_PATH=$(wslpath_to_wsl "$PROJECT_ROOT")
        wsl -d "$DISTRO_NAME" -u root -e bash -c "cd '$LINUX_PATH' && bash env/linux/install.sh"
    fi

# --- WSL or Linux Path ---
else
    # Direct execution (already in WSL or on Linux)
    if [ "$(id -u)" -ne 0 ]; then
        if command -v sudo &>/dev/null; then
            exec sudo bash "${SCRIPT_DIR}/linux/install.sh"
        else
            echo "Error: root privileges required to run setup." >&2
            echo "Please run: sudo bash env/setup.sh" >&2
            exit 1
        fi
    else
        bash "${SCRIPT_DIR}/linux/install.sh"
    fi
fi

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To enter rvpoint WSL in current directory:"
echo "    ./env/wsl.sh  (or: wsl -d rvpoint)"
echo ""
echo "To activate RVPoint environment inside WSL:"
echo "    source env/activate.sh"
echo ""
echo "To verify the installation:"
echo "    env/verify_container.sh"