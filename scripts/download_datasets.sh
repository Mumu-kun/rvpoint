#!/bin/bash
set -e

DATA_DIR="/workspace/data"
mkdir -p "$DATA_DIR"

# 1. PCL Table Scene (Compressed PCD)
# -----------------------------------------------------------------------------
PCD_FILE="$DATA_DIR/table_scene_lms400.pcd"
PCD_URL="https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd"

if [ ! -f "$PCD_FILE" ]; then
    echo "Downloading PCL Table Scene..."
    curl -L -o "$PCD_FILE" "$PCD_URL"
    echo "Downloaded: $PCD_FILE"
else
    echo "Exists: $PCD_FILE"
fi

# 2. Stanford Bunny (Optional - defined in plan but user prioritized table scene)
# -----------------------------------------------------------------------------
# Uncomment to enable auto-download of Bunny
# BUNNY_FILE="$DATA_DIR/bunny.ply"
# if [ ! -f "$BUNNY_FILE" ]; then ... fi

echo "Dataset check complete."
