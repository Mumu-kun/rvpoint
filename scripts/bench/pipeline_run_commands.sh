#!/usr/bin/env bash
# ==============================================================================
# RVPoint vs PCL 1.14 Baseline Perception Pipelines: Execution Commands Reference
# ==============================================================================
# This script lists all 4 pipeline targets across both execution environments:
#   - QEMU User Emulation (Functional & Quick Verification)
#   - gem5 Microarchitectural Simulation (Cycle-Accurate SpacemiT K1 / MinorCPU)
#
# Target Definitions:
#   1. pipeline_3d_ultimate   : RVPoint Hand-Crafted RVV 1.0 Pipeline
#   2. pcl_native_pipeline    : Standard PCL 1.14 KdTree Pipeline Baseline
#   3. pcl_octree_pipeline    : Standard PCL 1.14 Octree Pipeline Baseline
#   4. pcl_optimized_pipeline : Optimized Standard PCL Pipeline (Dense + Unsorted FLANN)
#
# Common Benchmark Arguments:
#   DATASET="data/pcd_compressed/0000000090.pcd"
#   PARAMS="--progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write"
# ==============================================================================

set -euo pipefail

DATASET="${1:-data/pcd_compressed/0000000090.pcd}"
COMMON_ARGS="--progress --leaf-size 0.10 --cluster-tolerance 0.15 --min-cluster 50 --max-cluster 100000 --no-write"

echo "======================================================================"
echo "                  4 x 2 RUN COMMANDS REFERENCE"
echo "======================================================================"

# ==============================================================================
# 1. QEMU EXECUTION (via ./scripts/run.sh)
# ==============================================================================

# [1/4] RVPoint RVV 1.0 Pipeline (QEMU)
echo "Running: RVPoint RVV 1.0 Pipeline under QEMU..."
./scripts/run.sh pipeline_3d_ultimate "${DATASET}" ${COMMON_ARGS}

# [2/4] Standard PCL KdTree Baseline (QEMU)
echo "Running: PCL KdTree Pipeline under QEMU..."
./scripts/run.sh pcl_native_pipeline "${DATASET}" ${COMMON_ARGS}

# [3/4] Standard PCL Octree Baseline (QEMU)
echo "Running: PCL Octree Pipeline under QEMU..."
./scripts/run.sh pcl_octree_pipeline "${DATASET}" ${COMMON_ARGS}

# [4/4] Optimized PCL Baseline (QEMU)
echo "Running: Optimized PCL Pipeline under QEMU..."
./scripts/run.sh pcl_optimized_pipeline "${DATASET}" ${COMMON_ARGS}

# ==============================================================================
# 2. GEM5 MICROARCHITECTURAL SIMULATION (via ./scripts/gem5/run_sim.sh)
# ==============================================================================

# [1/4] RVPoint RVV 1.0 Pipeline (gem5)
echo "Running: RVPoint RVV 1.0 Pipeline under gem5..."
./scripts/gem5/run_sim.sh pipeline_3d_ultimate "${DATASET}" ${COMMON_ARGS}

# [2/4] Standard PCL KdTree Baseline (gem5)
echo "Running: PCL KdTree Pipeline under gem5..."
./scripts/gem5/run_sim.sh pcl_native_pipeline "${DATASET}" ${COMMON_ARGS}

# [3/4] Standard PCL Octree Baseline (gem5)
echo "Running: PCL Octree Pipeline under gem5..."
./scripts/gem5/run_sim.sh pcl_octree_pipeline "${DATASET}" ${COMMON_ARGS}

# [4/4] Optimized PCL Baseline (gem5)
echo "Running: Optimized PCL Pipeline under gem5..."
./scripts/gem5/run_sim.sh pcl_optimized_pipeline "${DATASET}" ${COMMON_ARGS}
