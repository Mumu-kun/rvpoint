#!/usr/bin/env bash
# scripts/get_data.sh
# Automated downloader for RANSAC & Euclidean Clustering Benchmark Datasets

set -e

DATA_DIR="data"
mkdir -p "${DATA_DIR}"
echo "==> Downloading RANSAC & Clustering datasets into ${DATA_DIR}/..."

# 1. Official Tabletop Scene (Laser Scanner: Table + Cup + Bowl + Box)
echo "--> [1/3] Downloading 01_table_scene_lms400.pcd..."
wget -q --show-progress -O "${DATA_DIR}/01_table_scene_lms400.pcd" \
  https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_lms400.pcd

# 2. Noisy Stereo Tabletop Scene (Stereo Camera: Table + Mug + Carton)
echo "--> [2/3] Downloading 02_table_stereo_objects.pcd..."
wget -q --show-progress -O "${DATA_DIR}/02_table_stereo_objects.pcd" \
  https://raw.githubusercontent.com/PointCloudLibrary/data/master/tutorials/table_scene_mug_stereo_textured.pcd

# 3. Automotive 64-Beam LiDAR Scan (Road Plane + Vehicles/Pedestrians)
echo "--> [3/3] Downloading and converting 03_kitti_drive_clusters.pcd..."
wget -q --show-progress -O "${DATA_DIR}/kitti_frame.bin" \
  https://raw.githubusercontent.com/PRBonn/semantic-kitti-api/master/content/000000.bin

# Inline Python converter: KITTI binary -> Standard PCD format
python3 - << 'EOF'
import numpy as np

bin_file = "data/kitti_frame.bin"
pcd_file = "data/03_kitti_drive_clusters.pcd"

scan = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
points = scan[:, :3]
n_points = points.shape[0]

header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n_points}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n_points}
DATA ascii
"""

with open(pcd_file, "w") as f:
    f.write(header)
    for pt in points:
        f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")

print(f"==> Successfully generated {pcd_file} ({n_points:,} points).")
EOF

# Clean up temporary binary
rm -f "${DATA_DIR}/kitti_frame.bin"

echo ""
echo "=========================================================="
echo " Datasets ready in data/:"
ls -lh "${DATA_DIR}"/*.pcd
echo "=========================================================="