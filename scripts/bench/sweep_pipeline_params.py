#!/usr/bin/env python3
"""
Parameter Sweep Benchmark for pipeline_3d_ultra.
Systematically tests parameter configurations (with and without normals)
to determine the Pareto-optimal operating points for latency vs object detection quality.
"""

import itertools
import json
import os
import re
import subprocess
import sys
import time

BINARY = "/root/.cache/rvpoint/build/rvv/bin/rvv/pipeline_3d_ultra"
QEMU = "/usr/bin/qemu-riscv64"
QEMU_CPU = "rv64,v=true,vlen=256,vext_spec=v1.0"
LIB_DIR = "/opt/riscv/sysroot/lib"

def run_config(pcd_path, leaf_size, tol, min_c, max_c, ror_r, ror_min, with_normals, ransac_iters=250):
    cmd = [
        QEMU, "-cpu", QEMU_CPU, "-L", "/opt/riscv/sysroot",
        BINARY, pcd_path,
        "--leaf-size", str(leaf_size),
        "--cluster-tolerance", str(tol),
        "--min-cluster", str(min_c),
        "--max-cluster", str(max_c),
        "--ror-radius", str(ror_r),
        "--ror-min-pts", str(ror_min),
        "--ransac-iters", str(ransac_iters),
        "--no-write",
        "--progress"
    ]
    if not with_normals:
        cmd.append("--no-normals")

    try:
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=30)
        output = res.stdout + res.stderr
        
        # Parse timing and point counts
        total_match = re.search(r"Total:\s+([\d\.]+)\s+ms", output)
        down_match = re.search(r"\[\s*3/10\]\s+Downsampling:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)
        ror_match = re.search(r"\[\s*5/10\]\s+Radius outlier removal.*?:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)
        ransac_match = re.search(r"\[\s*8/10\]\s+RANSAC primitive fitting:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)
        clust_match = re.search(r"\[\s*9/10\]\s+Euclidean clustering:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)

        if total_match and clust_match:
            return {
                "total_ms": float(total_match.group(1)),
                "down_ms": float(down_match.group(1)) if down_match else 0.0,
                "down_pts": int(down_match.group(2)) if down_match else 0,
                "ror_ms": float(ror_match.group(1)) if ror_match else 0.0,
                "ror_pts": int(ror_match.group(2)) if ror_match else 0,
                "ransac_ms": float(ransac_match.group(1)) if ransac_match else 0.0,
                "outlier_pts": int(ransac_match.group(2)) if ransac_match else 0,
                "clust_ms": float(clust_match.group(1)) if clust_match else 0.0,
                "num_clusters": int(clust_match.group(2)) if clust_match else 0,
                "success": True
            }
    except Exception as e:
        pass
    return {"success": False}

def main():
    pcd = "data/pcd_compressed/0000000005.pcd"
    if len(sys.argv) > 1:
        pcd = sys.argv[1]

    print(f"==========================================================================================")
    print(f" PIPELINE_3D_ULTRA PARAMETER SWEEP & OPTIMAL OPERATING POINT SEARCH")
    print(f" Target Frame: {pcd}")
    print(f"==========================================================================================")

    # Grid search parameters
    leaf_sizes = [0.10, 0.12, 0.15, 0.18, 0.20]
    tolerances = [0.15, 0.20, 0.25, 0.30]
    min_clusters = [30, 50, 80]
    normal_modes = [False, True]  # Without normals vs With normals

    results = []

    for with_normals in normal_modes:
        mode_str = "WITH NORMALS" if with_normals else "WITHOUT NORMALS (--no-normals)"
        print(f"\n>>> EVALUATING MODE: {mode_str} <<<")
        print(f"{'Leaf(m)':<8} {'Tol(m)':<8} {'MinCl':<8} {'Total(ms)':<10} {'Down(ms)':<10} {'ROR(ms)':<10} {'RANSAC':<10} {'Clust':<10} {'Clusters':<10}")
        print("-" * 88)

        mode_results = []
        for leaf, tol, min_c in itertools.product(leaf_sizes, tolerances, min_clusters):
            res = run_config(pcd, leaf, tol, min_c, 100000, 0.25, 2, with_normals)
            if res["success"]:
                res.update({
                    "leaf_size": leaf,
                    "cluster_tolerance": tol,
                    "min_cluster": min_c,
                    "with_normals": with_normals
                })
                mode_results.append(res)
                print(f"{leaf:<8.2f} {tol:<8.2f} {min_c:<8} {res['total_ms']:<10.2f} {res['down_ms']:<10.2f} {res['ror_ms']:<10.2f} {res['ransac_ms']:<10.2f} {res['clust_ms']:<10.2f} {res['num_clusters']:<10}")

        results.extend(mode_results)

    # Summary Analysis
    print("\n" + "=" * 90)
    print(" PARETO OPTIMAL CONFIGURATION RECOMMENDATIONS")
    print("=" * 90)

    for with_normals in [False, True]:
        mode_name = "WITH NORMALS" if with_normals else "WITHOUT NORMALS (--no-normals)"
        subset = [r for r in results if r["with_normals"] == with_normals and r["num_clusters"] > 0]
        if not subset:
            continue

        fastest = min(subset, key=lambda x: x["total_ms"])
        balanced = min([s for s in subset if s["leaf_size"] <= 0.15 and s["min_cluster"] == 50], key=lambda x: x["total_ms"], default=fastest)
        high_res = min([s for s in subset if s["leaf_size"] <= 0.10], key=lambda x: x["total_ms"], default=fastest)

        print(f"\n--- {mode_name} ---")
        print(f" 1. ULTRA HIGH-SPEED PROFILE (Lowest Latency):")
        print(f"    --leaf-size {fastest['leaf_size']:.2f} --cluster-tolerance {fastest['cluster_tolerance']:.2f} --min-cluster {fastest['min_cluster']}")
        print(f"    Latency: {fastest['total_ms']:.2f} ms | Clusters: {fastest['num_clusters']} | Downsampled pts: {fastest['down_pts']}")

        print(f"\n 2. BALANCED AUTONOMOUS DRIVING PROFILE (Recommended Default):")
        print(f"    --leaf-size {balanced['leaf_size']:.2f} --cluster-tolerance {balanced['cluster_tolerance']:.2f} --min-cluster {balanced['min_cluster']}")
        print(f"    Latency: {balanced['total_ms']:.2f} ms | Clusters: {balanced['num_clusters']} | Downsampled pts: {balanced['down_pts']}")

        print(f"\n 3. HIGH-DENSITY GEOMETRIC PROFILE (Maximum Resolution):")
        print(f"    --leaf-size {high_res['leaf_size']:.2f} --cluster-tolerance {high_res['cluster_tolerance']:.2f} --min-cluster {high_res['min_cluster']}")
        print(f"    Latency: {high_res['total_ms']:.2f} ms | Clusters: {high_res['num_clusters']} | Downsampled pts: {high_res['down_pts']}")

if __name__ == "__main__":
    main()
