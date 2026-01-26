#!/bin/bash
set -e

DATA_DIR="data"
mkdir -p $DATA_DIR

echo "Downloading Stanford Bunny..."
curl -o $DATA_DIR/bunny.pcd https://raw.githubusercontent.com/PointCloudLibrary/pcl/master/test/bunny.pcd

echo "Checking datasets..."
ls -lh $DATA_DIR
