#!/bin/bash
# =============================================================================
#  run_batch_pcd_compressed.sh — Process all PCDs in data/pcd_compressed serially
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

DATA_DIR="${PROJECT_ROOT}/data/pcd_compressed"
DEFAULT_OUTPUT_DIR="${PROJECT_ROOT}/output/pcd_compressed_pipeline"

OUTPUT_DIR="$DEFAULT_OUTPUT_DIR"
SKIP_SOR=""
LEAF_SIZE=""
MAX_FRAMES=""
BACKEND="rvv"
TOOLCHAIN="linux"

usage() {
    cat <<EOF
Usage: $0 [options]

Options:
  --output-dir <path>   Directory to save outputs (default: output/pcd_compressed_pipeline)
  --skip-sor            Skip statistical outlier removal stage for faster processing
  --leaf-size <val>     Voxel leaf size for downsampling stage (e.g. 0.2)
  --max-frames <N>      Limit execution to the first N PCD files
  --backend <rvv|scalar> Target backend (default: rvv)
  --help, -h            Show this help message
EOF
    exit 0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --output-dir)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        --skip-sor)
            SKIP_SOR="--skip-sor"
            shift
            ;;
        --leaf-size)
            LEAF_SIZE="--leaf-size $2"
            shift 2
            ;;
        --max-frames)
            MAX_FRAMES="$2"
            shift 2
            ;;
        --backend)
            BACKEND="$2"
            shift 2
            ;;
        --help|-h)
            usage
            ;;
        *)
            echo "Error: Unknown option '$1'"
            usage
            ;;
    esac
done

if [ ! -d "$DATA_DIR" ]; then
    echo "Error: Directory '$DATA_DIR' does not exist."
    exit 1
fi

# Build pipeline_export target once
echo "==> Building pipeline_export ($BACKEND)..."
"$SCRIPT_DIR/build.sh" --toolchain "$TOOLCHAIN" --backend "$BACKEND" --target "pipeline_export"

# Create output directory structure
BY_FRAME_DIR="${OUTPUT_DIR}/by_frame"
BY_STAGE_DIR="${OUTPUT_DIR}/by_stage"

STAGES=("00_input" "01_downsampled" "02_sor_filtered" "04_ransac_inliers" "05_ground_plane_removed" "06_clusters")

for stage in "${STAGES[@]}"; do
    mkdir -p "${BY_STAGE_DIR}/${stage}"
done

# Get sorted list of PCD files
PCD_FILES=($(ls -1 "$DATA_DIR"/*.pcd | sort))
TOTAL_FILES=${#PCD_FILES[@]}

if [ "$TOTAL_FILES" -eq 0 ]; then
    echo "Error: No PCD files found in '$DATA_DIR'."
    exit 1
fi

if [ -n "$MAX_FRAMES" ] && [ "$MAX_FRAMES" -lt "$TOTAL_FILES" ]; then
    TOTAL_FILES="$MAX_FRAMES"
    PCD_FILES=("${PCD_FILES[@]:0:$TOTAL_FILES}")
fi

echo "=========================================================================="
echo " Starting batch processing of $TOTAL_FILES PCD files serially"
echo " Input directory:  $DATA_DIR"
echo " Output directory: $OUTPUT_DIR"
if [ -n "$SKIP_SOR" ]; then
    echo " Mode:             --skip-sor enabled"
fi
if [ -n "$LEAF_SIZE" ]; then
    echo " Mode:             $LEAF_SIZE"
fi
echo "=========================================================================="

COUNTER=0
START_TIME=$(date +%s)

for pcd_path in "${PCD_FILES[@]}"; do
    COUNTER=$((COUNTER + 1))
    filename=$(basename "$pcd_path")
    stem="${filename%.pcd}"
    
    frame_out_dir="${BY_FRAME_DIR}/${stem}"
    mkdir -p "$frame_out_dir"
    
    echo -n "[$COUNTER/$TOTAL_FILES] Processing $filename ... "
    
    # Run pipeline_export via run.sh
    "$SCRIPT_DIR/run.sh" --backend "$BACKEND" pipeline_export $SKIP_SOR $LEAF_SIZE "$pcd_path" "$frame_out_dir" > "${frame_out_dir}/pipeline.log" 2>&1
    
    # Organize outputs into by_stage directory
    for stage in "${STAGES[@]}"; do
        stage_file="${frame_out_dir}/${stage}.pcd"
        if [ -f "$stage_file" ]; then
            cp "$stage_file" "${BY_STAGE_DIR}/${stage}/${stem}.pcd"
        fi
    done
    
    echo "Done."
done

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo "=========================================================================="
echo " Batch processing completed successfully!"
echo " Total frames processed: $TOTAL_FILES"
echo " Total time elapsed:     ${ELAPSED} seconds"
echo " Results stored in:"
echo "   • By Stage: $BY_STAGE_DIR"
echo "   • By Frame: $BY_FRAME_DIR"
echo "=========================================================================="
