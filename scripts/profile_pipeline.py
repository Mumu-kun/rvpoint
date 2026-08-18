#!/usr/bin/env python3
"""
profile_pipeline.py — Automated Two-Phase Pipeline Profiler & Ablation Runner

Phase 1: Executes broad pipeline runs across representative leaf sizes (0.10, 0.20)
         and SOR modes (enabled, bypassed) to identify critical path bottlenecks.
Phase 2: Runs targeted ablation benchmarks (ablation_bench) for identified bottleneck operations.
"""

import argparse
import csv
import json
import os
import subprocess
import sys
from pathlib import Path


def run_command(cmd, cwd=None):
    print(f"==> Executing: {' '.join(cmd)}")
    res = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    if res.returncode != 0:
        print(f"Error executing command: {res.stderr}")
    return res.stdout, res.returncode


def run_phase1_profiling(project_root, input_pcd, output_dir, backend):
    print("\n==========================================================================")
    print(" PHASE 1: Broad Pipeline Profiling (4 Representative Benchmark Runs)")
    print("==========================================================================")

    runs = [
        {"name": "leaf_0.10_sor_on", "leaf_size": 0.10, "skip_sor": False},
        {"name": "leaf_0.10_sor_off", "leaf_size": 0.10, "skip_sor": True},
        {"name": "leaf_0.20_sor_on", "leaf_size": 0.20, "skip_sor": False},
        {"name": "leaf_0.20_sor_off", "leaf_size": 0.20, "skip_sor": True},
    ]

    phase1_results = []

    for r in runs:
        run_out_dir = output_dir / "phase1" / r["name"]
        run_out_dir.mkdir(parents=True, exist_ok=True)

        cmd = ["./scripts/run.sh", "--backend", backend, "pipeline_export", "--progress", "--json",
               "--leaf-size", str(r["leaf_size"])]
        if r["skip_sor"]:
            cmd.append("--skip-sor")
        cmd.extend([str(input_pcd), str(run_out_dir)])

        stdout, ret = run_command(cmd, cwd=project_root)

        metrics_file = run_out_dir / "pipeline_metrics.json"
        if metrics_file.exists():
            with open(metrics_file, "r") as f:
                data = json.load(f)
                data["run_name"] = r["name"]
                phase1_results.append(data)
                print(f"  [✓] Completed {r['name']}: total_ms = {data['total_ms']:.2f} ms")
        else:
            print(f"  [✗] Failed {r['name']}")

    return phase1_results


def run_phase2_ablations(project_root, input_pcd, output_dir, backend):
    print("\n==========================================================================")
    print(" PHASE 2: Targeted Implementation Ablations (ablation_bench)")
    print("==========================================================================")

    ablation_out_dir = output_dir / "phase2_ablation"
    ablation_out_dir.mkdir(parents=True, exist_ok=True)

    cmd = ["./scripts/run.sh", "--backend", backend, "ablation_bench", str(input_pcd), str(ablation_out_dir)]
    stdout, ret = run_command(cmd, cwd=project_root)

    metrics_file = ablation_out_dir / "ablation_metrics.json"
    ablation_data = []
    if metrics_file.exists():
        with open(metrics_file, "r") as f:
            ablation_data = json.load(f).get("ablation_benchmarks", [])
            print(f"  [✓] Completed ablation benchmarks ({len(ablation_data)} measurements)")

    return ablation_data


def generate_summary(phase1_data, ablation_data, output_dir):
    print("\n==========================================================================")
    print(" PROFILING SUMMARY & BOTTLENECK ANALYSIS")
    print("==========================================================================")

    # 1. Save consolidated JSON
    summary_json = {
        "phase1_pipeline_profiling": phase1_data,
        "phase2_ablation_benchmarks": ablation_data,
    }
    with open(output_dir / "consolidated_profiling_results.json", "w") as f:
        json.dump(summary_json, f, indent=2)

    # 2. Save CSV summary
    csv_file = output_dir / "summary_results.csv"
    with open(csv_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["Run Name", "Leaf Size", "SOR Mode", "Total Time (ms)", "Stage Index", "Stage Label", "Stage Time (ms)", "Stage %", "Points"])

        for run in phase1_data:
            name = run.get("run_name", "")
            leaf = run.get("leaf_size", 0.0)
            sor_str = "Bypassed" if run.get("skip_sor", False) else "Enabled"
            total_ms = run.get("total_ms", 0.0)

            for st in run.get("stages", []):
                writer.writerow([name, leaf, sor_str, f"{total_ms:.2f}", st["index"], st["label"], f"{st['ms']:.2f}", f"{st['pct']:.2f}", st["point_count"]])

    # 3. Print Console Tables
    print("\nPhase 1 Stage Breakdown Summary:")
    header = f"{'Run':<20} | {'Leaf':<5} | {'SOR':<8} | {'Total (ms)':<10} | {'Top Bottleneck':<30}"
    print(header)
    print("-" * len(header))

    for run in phase1_data:
        name = run.get("run_name", "")
        leaf = run.get("leaf_size", 0.0)
        sor_str = "OFF" if run.get("skip_sor", False) else "ON"
        total_ms = run.get("total_ms", 0.0)

        top_stage = max(run.get("stages", []), key=lambda x: x["ms"]) if run.get("stages") else {"label": "N/A", "pct": 0.0}
        bottleneck_str = f"{top_stage['label']} ({top_stage['pct']:.1f}%)"
        print(f"{name:<20} | {leaf:<5.2f} | {sor_str:<8} | {total_ms:<10.2f} | {bottleneck_str:<30}")

    if ablation_data:
        print("\nPhase 2 Ablation Speedup Highlights (vs Scalar):")
        ablation_header = f"{'Category':<28} | {'Implementation':<22} | {'Time (ms)':<10} | {'Speedup':<8}"
        print(ablation_header)
        print("-" * len(ablation_header))
        for ab in ablation_data:
            print(f"{ab['category']:<28} | {ab['implementation']:<22} | {ab['time_ms']:<10.2f} | {ab['speedup']:<8.2f}x")

    print(f"\nResults saved to:")
    print(f"  • JSON: {output_dir / 'consolidated_profiling_results.json'}")
    print(f"  • CSV:  {csv_file}")


def main():
    parser = argparse.ArgumentParser(description="Automated Two-Phase Pipeline Profiler & Ablation Harness")
    parser.add_argument("--input", default="data/0000000000.pcd", help="Input PCD file path")
    parser.add_argument("--output-dir", default="output/profiling_results", help="Output directory")
    parser.add_argument("--backend", default="rvv", choices=["rvv", "scalar"], help="Target backend")
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parent.parent
    input_pcd = project_root / args.input
    output_dir = project_root / args.output_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    if not input_pcd.exists():
        print(f"Error: Input PCD file not found at {input_pcd}")
        sys.exit(1)

    phase1_data = run_phase1_profiling(project_root, input_pcd, output_dir, args.backend)
    ablation_data = run_phase2_ablations(project_root, input_pcd, output_dir, args.backend)
    generate_summary(phase1_data, ablation_data, output_dir)


if __name__ == "__main__":
    main()
