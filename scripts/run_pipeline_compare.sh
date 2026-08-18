#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Run pipeline_compare_bench via run.sh
"$SCRIPT_DIR/run.sh" pipeline_compare_bench "$@"
