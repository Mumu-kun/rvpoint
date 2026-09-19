#!/usr/bin/env python3
"""
scripts/bench/bench_intra_scaling.py
================================================================================
Comprehensive Scalable Threading Benchmark for RVPoint pipeline_3d_intra
Supports 4-Core, 8-Core (or arbitrary thread scaling) on all PCD files.

Computes:
  - Full FPS (total wall-clock latency including PCD loading & overhead)
  - Computational FPS (pure in-memory point cloud processing: Stages 2 through 6)
  - Stage-by-stage latency breakdowns (Downsample, RANSAC, Grid, ROR, Clustering)
  - Multi-core scaling speedup and parallel efficiency

Usage Examples:
  # Run directly on Orange Pi RV2 board across all PCD files for 4 and 8 cores:
  python3 scripts/bench/bench_intra_scaling.py --cores 4 8

  # Run from host targeting Orange Pi RV2 via SSH:
  python3 scripts/bench/bench_intra_scaling.py --remote orangepi@100.94.165.126 --cores 4 8

  # Quick test on first 5 frames:
  python3 scripts/bench/bench_intra_scaling.py --cores 4 8 --limit 5
================================================================================
"""

import argparse
import glob
import json
import os
import platform
import re
import subprocess
import sys
import time
from pathlib import Path

# Ensure UTF-8 output
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Scalable Threading Benchmark for pipeline_3d_intra (Full FPS & Computational FPS)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--cores", "--threads",
        dest="cores",
        nargs="+",
        type=int,
        default=[4, 8],
        help="List of core / thread counts to evaluate (e.g. 4 8 or 1 2 4 8)",
    )
    parser.add_argument(
        "--pcd-dir",
        type=str,
        default="data/pcd_compressed",
        help="Directory containing input PCD files",
    )
    parser.add_argument(
        "--pattern",
        type=str,
        default="*.pcd",
        help="Glob pattern for PCD files",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Limit number of PCD files to evaluate (0 for all)",
    )
    parser.add_argument(
        "--files",
        nargs="*",
        default=None,
        help="Explicit list of PCD files to evaluate (overrides --pcd-dir)",
    )
    parser.add_argument(
        "--binary",
        type=str,
        default="./build/rvv/bin/rvv/pipeline_3d_intra",
        help="Path to pipeline_3d_intra executable",
    )
    parser.add_argument(
        "--leaf-size",
        type=float,
        default=0.10,
        help="Voxel downsampling leaf size (m)",
    )
    parser.add_argument(
        "--ror-radius",
        type=float,
        default=0.25,
        help="Radius outlier removal search radius (m)",
    )
    parser.add_argument(
        "--ror-min-pts",
        type=int,
        default=3,
        help="Radius outlier removal minimum neighbors",
    )
    parser.add_argument(
        "--ransac-iters",
        type=int,
        default=100,
        help="RANSAC ground removal max iterations",
    )
    parser.add_argument(
        "--cluster-tolerance",
        type=float,
        default=0.15,
        help="Euclidean clustering distance tolerance (m)",
    )
    parser.add_argument(
        "--min-cluster",
        type=int,
        default=50,
        help="Minimum cluster size",
    )
    parser.add_argument(
        "--max-cluster",
        type=int,
        default=100000,
        help="Maximum cluster size",
    )
    parser.add_argument(
        "--no-write",
        action="store_true",
        default=True,
        help="Disable PCD disk writing during benchmark",
    )
    parser.add_argument(
        "--remote",
        type=str,
        default="",
        help="Optional SSH host (e.g. orangepi@100.94.165.126)",
    )
    parser.add_argument(
        "--remote-dir",
        type=str,
        default="projects/rvpoint",
        help="Working directory on remote target",
    )
    parser.add_argument(
        "--password",
        type=str,
        default="orangepi",
        help="SSH password for remote target",
    )
    parser.add_argument(
        "--output-json",
        type=str,
        default="output/intra_scaling_bench.json",
        help="File path to save JSON results",
    )
    parser.add_argument(
        "--output-md",
        type=str,
        default="output/intra_scaling_bench.md",
        help="File path to save Markdown summary report",
    )
    parser.add_argument(
        "--qemu",
        action="store_true",
        default=False,
        help="Force execution through QEMU wrapper (./scripts/run.sh)",
    )
    return parser.parse_args()


def resolve_binary(args):
    """Resolve executable path, handling local vs remote vs QEMU paths."""
    if args.remote:
        return args.binary

    # Local checks
    candidates = [
        args.binary,
        "./build/rvv/bin/rvv/pipeline_3d_intra",
        "/root/.cache/rvpoint/build/rvv/bin/rvv/pipeline_3d_intra",
        "build/rvv/bin/rvv/pipeline_3d_intra",
    ]
    for c in candidates:
        if os.path.exists(c) and os.access(c, os.X_OK):
            return os.path.abspath(c)

    return args.binary


def find_pcd_files(args):
    """Collect sorted list of PCD files."""
    if args.files:
        pcds = sorted(args.files)
    elif args.remote:
        # Query remote directory
        cmd = [
            "sshpass", "-p", args.password,
            "ssh", "-o", "StrictHostKeyChecking=no",
            args.remote,
            f"ls -1 {args.remote_dir}/{args.pcd_dir}/{args.pattern} 2>/dev/null"
        ]
        try:
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
            pcds = [line.strip() for line in res.stdout.strip().split("\n") if line.strip().endswith(".pcd")]
            # Strip remote_dir prefix so it's relative to project root
            prefix = args.remote_dir.rstrip("/") + "/"
            clean_pcds = []
            for p in pcds:
                if p.startswith(prefix):
                    clean_pcds.append(p[len(prefix):])
                else:
                    clean_pcds.append(p)
            pcds = sorted(clean_pcds)
        except Exception as e:
            print(f"[WARN] Failed to query remote PCDs via SSH: {e}. Falling back to local search.")
            pcds = sorted(glob.glob(os.path.join(args.pcd_dir, args.pattern)))
    else:
        pcds = sorted(glob.glob(os.path.join(args.pcd_dir, args.pattern)))

    if args.limit > 0:
        pcds = pcds[:args.limit]

    return pcds


def execute_pipeline(pcd_path, core_count, binary, args):
    """Execute pipeline_3d_intra with the specified core count on a PCD file."""
    cmd_args = [
        binary,
        pcd_path,
        "--progress",
        "--json",
        "--threads", str(core_count),
        "--leaf-size", str(args.leaf_size),
        "--ror-radius", str(args.ror_radius),
        "--ror-min-pts", str(args.ror_min_pts),
        "--ransac-iters", str(args.ransac_iters),
        "--cluster-tolerance", str(args.cluster_tolerance),
        "--min-cluster", str(args.min_cluster),
        "--max-cluster", str(args.max_cluster),
    ]
    if args.no_write:
        cmd_args.append("--no-write")

    env_prefix = f"OMP_NUM_THREADS={core_count}"

    t0 = time.perf_counter()
    if args.remote:
        joined_args = " ".join(cmd_args)
        remote_cmd = f"cd {args.remote_dir} && {env_prefix} {joined_args}"
        full_cmd = [
            "sshpass", "-p", args.password,
            "ssh", "-o", "StrictHostKeyChecking=no",
            args.remote,
            remote_cmd,
        ]
        res = subprocess.run(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    elif args.qemu or (platform.machine() not in ("riscv64", "riscv")):
        # On non-RISC-V host, use run.sh or qemu wrapper
        run_sh = os.path.abspath("./scripts/run.sh")
        if os.path.exists(run_sh):
            full_cmd = [run_sh, "pipeline_3d_intra", pcd_path] + cmd_args[2:]
            res = subprocess.run(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                 env=dict(os.environ, OMP_NUM_THREADS=str(core_count)))
        else:
            qemu_cmd = ["qemu-riscv64", "-cpu", "rv64,v=true,vlen=256,vext_spec=v1.0", "-L", "/opt/riscv/sysroot"]
            full_cmd = qemu_cmd + cmd_args
            res = subprocess.run(full_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                 env=dict(os.environ, OMP_NUM_THREADS=str(core_count)))
    else:
        # Native RISC-V execution
        res = subprocess.run(cmd_args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                             env=dict(os.environ, OMP_NUM_THREADS=str(core_count)))

    wall_ms = (time.perf_counter() - t0) * 1000.0
    output = res.stdout + res.stderr

    parsed = parse_output(output, wall_ms)
    parsed["exit_code"] = res.returncode
    return parsed


def parse_output(output, fallback_wall_ms):
    """Extract metrics from [metrics_json] or regex fallback."""
    result = {
        "success": False,
        "total_ms": fallback_wall_ms,
        "compute_ms": 0.0,
        "load_ms": 0.0,
        "downsample_ms": 0.0,
        "ransac_ms": 0.0,
        "grid_ms": 0.0,
        "ror_ms": 0.0,
        "clustering_ms": 0.0,
        "clusters": 0,
        "input_pts": 0,
        "full_fps": 0.0,
        "compute_fps": 0.0,
    }

    # 1. Try JSON line
    json_match = re.search(r"\[metrics_json\]\s*(\{.*\})", output)
    if json_match:
        try:
            data = json.loads(json_match.group(1))
            result["total_ms"] = float(data.get("total_ms", fallback_wall_ms))
            result["compute_ms"] = float(data.get("compute_ms", 0.0))
            result["load_ms"] = float(data.get("load_ms", 0.0))
            result["downsample_ms"] = float(data.get("downsample_ms", 0.0))
            result["ransac_ms"] = float(data.get("ransac_ms", 0.0))
            result["grid_ms"] = float(data.get("grid_ms", 0.0))
            result["ror_ms"] = float(data.get("ror_ms", 0.0))
            result["clustering_ms"] = float(data.get("clustering_ms", 0.0))
            result["clusters"] = int(data.get("clusters", 0))
            result["full_fps"] = float(data.get("full_fps", 0.0))
            result["compute_fps"] = float(data.get("compute_fps", 0.0))
            result["success"] = True
        except Exception:
            pass

    # 2. Regex fallback / augmentation
    if not result["success"]:
        load_m = re.search(r"\[\s*1/7\]\s+Load input cloud:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)
        down_m = re.search(r"\[\s*2/7\]\s+Downsampling.*?:\s+([\d\.]+)\s+ms", output)
        ransac_m = re.search(r"\[\s*3/7\]\s+RANSAC ground removal.*?:\s+([\d\.]+)\s+ms", output)
        grid_m = re.search(r"\[\s*4/7\]\s+Build search grid.*?:\s+([\d\.]+)\s+ms", output)
        ror_m = re.search(r"\[\s*5/7\]\s+Radius outlier removal.*?:\s+([\d\.]+)\s+ms", output)
        clust_m = re.search(r"\[\s*6/7\]\s+Euclidean clustering.*?:\s+([\d\.]+)\s+ms.*?pts=(\d+)", output)
        total_m = re.search(r"Total:\s+([\d\.]+)\s+ms", output)
        clusters_m = re.search(r"Total clusters found:\s+(\d+)", output)

        if total_m:
            result["total_ms"] = float(total_m.group(1))
            result["success"] = True
        if load_m:
            result["load_ms"] = float(load_m.group(1))
            result["input_pts"] = int(load_m.group(2))
        if down_m: result["downsample_ms"] = float(down_m.group(1))
        if ransac_m: result["ransac_ms"] = float(ransac_m.group(1))
        if grid_m: result["grid_ms"] = float(grid_m.group(1))
        if ror_m: result["ror_ms"] = float(ror_m.group(1))
        if clust_m: result["clustering_ms"] = float(clust_m.group(1))
        if clusters_m:
            result["clusters"] = int(clusters_m.group(1))
        elif clust_m:
            result["clusters"] = int(clust_m.group(2))

        result["compute_ms"] = (
            result["downsample_ms"] + result["ransac_ms"] +
            result["grid_ms"] + result["ror_ms"] + result["clustering_ms"]
        )
        if result["total_ms"] > 0:
            result["full_fps"] = 1000.0 / result["total_ms"]
        if result["compute_ms"] > 0:
            result["compute_fps"] = 1000.0 / result["compute_ms"]

    # Extract input_pts if missing
    if result["input_pts"] == 0:
        pts_m = re.search(r"pts=(\d+)", output)
        if pts_m:
            result["input_pts"] = int(pts_m.group(1))

    return result


def compute_statistics(records):
    """Compute summary stats across a list of frame records."""
    if not records:
        return {}

    n = len(records)
    computes = [r["compute_ms"] for r in records if r["compute_ms"] > 0]
    totals = [r["total_ms"] for r in records if r["total_ms"] > 0]
    c_fps = [r["compute_fps"] for r in records if r["compute_fps"] > 0]
    f_fps = [r["full_fps"] for r in records if r["full_fps"] > 0]

    sum_compute_s = sum(computes) / 1000.0 if computes else 1.0
    sum_total_s = sum(totals) / 1000.0 if totals else 1.0

    overall_compute_fps = n / sum_compute_s if sum_compute_s > 0 else 0.0
    overall_full_fps = n / sum_total_s if sum_total_s > 0 else 0.0

    def stats(vals):
        if not vals: return {"mean": 0.0, "min": 0.0, "max": 0.0, "std": 0.0}
        m = sum(vals) / len(vals)
        var = sum((x - m) ** 2 for x in vals) / len(vals)
        return {
            "mean": round(m, 2),
            "min": round(min(vals), 2),
            "max": round(max(vals), 2),
            "std": round(var ** 0.5, 2),
        }

    return {
        "frames_evaluated": n,
        "compute_latency_ms": stats(computes),
        "total_latency_ms": stats(totals),
        "overall_compute_fps": round(overall_compute_fps, 2),
        "overall_full_fps": round(overall_full_fps, 2),
        "mean_compute_fps": round(sum(c_fps) / len(c_fps), 2) if c_fps else 0.0,
        "mean_full_fps": round(sum(f_fps) / len(f_fps), 2) if f_fps else 0.0,
        "stages_mean_ms": {
            "load": round(sum(r["load_ms"] for r in records) / n, 2),
            "downsample": round(sum(r["downsample_ms"] for r in records) / n, 2),
            "ransac": round(sum(r["ransac_ms"] for r in records) / n, 2),
            "grid": round(sum(r["grid_ms"] for r in records) / n, 2),
            "ror": round(sum(r["ror_ms"] for r in records) / n, 2),
            "clustering": round(sum(r["clustering_ms"] for r in records) / n, 2),
        },
        "mean_clusters": round(sum(r["clusters"] for r in records) / n, 1),
    }


def main():
    args = parse_arguments()
    binary = resolve_binary(args)
    pcds = find_pcd_files(args)

    if not pcds:
        print(f"[ERROR] No PCD files found in {args.pcd_dir} with pattern {args.pattern}")
        sys.exit(1)

    target_desc = f"Remote SSH ({args.remote})" if args.remote else (
        "Host (QEMU)" if args.qemu or platform.machine() not in ("riscv64", "riscv") else "Native RISC-V Hardware"
    )

    print("=" * 90)
    print("  RVPoint pipeline_3d_intra Scalable Threading Benchmark")
    print("=" * 90)
    print(f"  Target Mode       : {target_desc}")
    print(f"  Binary            : {binary}")
    print(f"  Core Counts       : {args.cores}")
    print(f"  PCD Files Total   : {len(pcds)}")
    print(f"  Pipeline Params   : leaf={args.leaf_size}m, ror_r={args.ror_radius}m, ror_pts={args.ror_min_pts},")
    print(f"                      ransac_iters={args.ransac_iters}, tol={args.cluster_tolerance}m, min_c={args.min_cluster}")
    print("=" * 90)

    all_results = {}

    for cores in args.cores:
        print(f"\n>>> Benchmarking {cores}-Core Scalable Spatial Slab Decomposition ({len(pcds)} frames) <<<")
        print(f"{'Frame':<16} {'Compute(ms)':<13} {'Comp FPS':<11} {'Total(ms)':<12} {'Full FPS':<11} {'Clusters':<9} {'Down':<8} {'Clust':<8}")
        print("-" * 90)

        core_records = []
        for idx, pcd in enumerate(pcds):
            fname = Path(pcd).name
            res = execute_pipeline(pcd, cores, binary, args)
            res["frame"] = fname
            res["pcd_path"] = pcd
            res["cores"] = cores
            core_records.append(res)

            print(
                f"[{idx + 1:3d}/{len(pcds):3d}] {fname:<12} "
                f"{res['compute_ms']:>8.2f} ms   "
                f"{res['compute_fps']:>7.2f} fps  "
                f"{res['total_ms']:>8.2f} ms   "
                f"{res['full_fps']:>7.2f} fps  "
                f"{res['clusters']:>6d}   "
                f"{res['downsample_ms']:>6.2f} "
                f"{res['clustering_ms']:>6.2f}"
            )

        summary = compute_statistics(core_records)
        all_results[cores] = {
            "summary": summary,
            "records": core_records,
        }

        print("-" * 90)
        print(
            f"SUMMARY ({cores} Cores) | "
            f"Mean Compute: {summary['compute_latency_ms']['mean']} ms ({summary['overall_compute_fps']} FPS) | "
            f"Mean Total: {summary['total_latency_ms']['mean']} ms ({summary['overall_full_fps']} FPS) | "
            f"Mean Clusters: {summary['mean_clusters']}"
        )

    # Comparison summary table across tested core counts
    print("\n" + "=" * 90)
    print("  SCALABLE THREADING PERFORMANCE COMPARISON & SPEEDUP SUMMARY")
    print("=" * 90)
    print(
        f"{'Cores':<7} | {'Compute(ms)':<13} | {'Comp FPS':<11} | {'Total(ms)':<12} | "
        f"{'Full FPS':<11} | {'Down(ms)':<10} | {'Clust(ms)':<10} | {'Speedup':<9} | {'Efficiency':<10}"
    )
    print("-" * 90)

    base_cores = args.cores[0]
    base_compute = all_results[base_cores]["summary"]["compute_latency_ms"]["mean"]

    for cores in args.cores:
        s = all_results[cores]["summary"]
        c_mean = s["compute_latency_ms"]["mean"]
        t_mean = s["total_latency_ms"]["mean"]
        c_fps = s["overall_compute_fps"]
        f_fps = s["overall_full_fps"]
        down_ms = s["stages_mean_ms"]["downsample"]
        clust_ms = s["stages_mean_ms"]["clustering"]

        if c_mean > 0:
            speedup = base_compute / c_mean
            rel_cores = cores / base_cores
            eff = (speedup / rel_cores) * 100.0 if rel_cores > 0 else 100.0
            sp_str = f"{speedup:.2f}x"
            eff_str = f"{eff:.1f}%"
        else:
            sp_str = "1.00x"
            eff_str = "100.0%"

        print(
            f"{cores:<7} | {c_mean:>9.2f} ms   | {c_fps:>7.2f} fps  | {t_mean:>8.2f} ms   | "
            f"{f_fps:>7.2f} fps  | {down_ms:>7.2f} ms | {clust_ms:>7.2f} ms | {sp_str:>8} | {eff_str:>9}"
        )
    print("=" * 90)

    # Save JSON output
    os.makedirs(os.path.dirname(os.path.abspath(args.output_json)), exist_ok=True)
    with open(args.output_json, "w", encoding="utf-8") as f:
        # Format JSON cleanly
        json_dump_data = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "target": target_desc,
            "binary": binary,
            "config": {
                "leaf_size": args.leaf_size,
                "ror_radius": args.ror_radius,
                "ror_min_pts": args.ror_min_pts,
                "ransac_iters": args.ransac_iters,
                "cluster_tolerance": args.cluster_tolerance,
                "min_cluster": args.min_cluster,
                "max_cluster": args.max_cluster,
            },
            "pcd_count": len(pcds),
            "scaling_results": {
                str(c): {
                    "summary": all_results[c]["summary"],
                    "records": all_results[c]["records"],
                }
                for c in args.cores
            },
        }
        json.dump(json_dump_data, f, indent=2)
    print(f"\n[export] Detailed benchmark JSON written to: {args.output_json}")

    # Save Markdown report
    if args.output_md:
        os.makedirs(os.path.dirname(os.path.abspath(args.output_md)), exist_ok=True)
        with open(args.output_md, "w", encoding="utf-8") as f:
            f.write("# RVPoint Intra-Frame Pipeline Scalable Threading Benchmark\n\n")
            f.write(f"- **Target**: {target_desc}\n")
            f.write(f"- **Evaluated PCD Files**: {len(pcds)} frames\n")
            f.write(f"- **Parameters**: leaf={args.leaf_size}m, ror_r={args.ror_radius}m, ror_min_pts={args.ror_min_pts}, "
                    f"tol={args.cluster_tolerance}m, min_cluster={args.min_cluster}\n\n")

            f.write("## 1. Summary Performance & FPS Comparison\n\n")
            f.write("| Cores | Pure Compute (ms) | Computational FPS | Total Wall-Clock (ms) | Full FPS | Downsampling (ms) | Clustering (ms) | Speedup | Scaling Efficiency |\n")
            f.write("|:---|---:|---:|---:|---:|---:|---:|---:|---:|\n")
            for cores in args.cores:
                s = all_results[cores]["summary"]
                c_mean = s["compute_latency_ms"]["mean"]
                t_mean = s["total_latency_ms"]["mean"]
                c_fps = s["overall_compute_fps"]
                f_fps = s["overall_full_fps"]
                down_ms = s["stages_mean_ms"]["downsample"]
                clust_ms = s["stages_mean_ms"]["clustering"]
                speedup = base_compute / c_mean if c_mean > 0 else 1.0
                rel_cores = cores / base_cores
                eff = (speedup / rel_cores) * 100.0 if rel_cores > 0 else 100.0
                f.write(f"| **{cores} Cores** | **{c_mean:.2f} ms** | **{c_fps:.2f} FPS** | {t_mean:.2f} ms | {f_fps:.2f} FPS | {down_ms:.2f} ms | {clust_ms:.2f} ms | {speedup:.2f}x | {eff:.1f}% |\n")

            f.write("\n## 2. Stage-by-Stage Latency Breakdown (Mean ms)\n\n")
            f.write("| Cores | [1] Load | [2] Downsample | [3] RANSAC | [4] Grid Build | [5] ROR | [6] Clustering | **Pure Compute** |\n")
            f.write("|:---|---:|---:|---:|---:|---:|---:|---:|\n")
            for cores in args.cores:
                s = all_results[cores]["summary"]
                st = s["stages_mean_ms"]
                c_mean = s["compute_latency_ms"]["mean"]
                f.write(f"| **{cores} Cores** | {st['load']:.2f} ms | {st['downsample']:.2f} ms | {st['ransac']:.2f} ms | {st['grid']:.2f} ms | {st['ror']:.2f} ms | {st['clustering']:.2f} ms | **{c_mean:.2f} ms** |\n")

        print(f"[export] Formatted Markdown summary written to: {args.output_md}")


if __name__ == "__main__":
    main()
