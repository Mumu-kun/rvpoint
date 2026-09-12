#!/usr/bin/env python3
"""
==============================================================================
SCRIPT: scripts/gem5/compare_stats.py
PURPOSE: Standalone results comparator for gem5 simulation metrics.
         Parses stats.txt files, computes speedup, instruction reduction,
         IPC, and L1D cache miss rates, and formats clean comparison tables.

USAGE:
  python3 scripts/gem5/compare_stats.py <run_dir_1> <run_dir_2> [...]
  python3 scripts/gem5/compare_stats.py results/gem5/pcl_* results/gem5/rvpoint_* --json results/compare.json

OUTPUT:
  - Formatted terminal table
  - GitHub Flavored Markdown table
  - Optional JSON export
==============================================================================
"""

import sys
import os
import re
import json
import argparse
from typing import Dict, Any, List, Optional

def parse_stats_file(filepath: str) -> Dict[str, float]:
    stats: Dict[str, float] = {}
    if not os.path.isfile(filepath):
        # Check if directory was passed
        if os.path.isdir(filepath):
            filepath = os.path.join(filepath, "stats.txt")
        if not os.path.isfile(filepath):
            raise FileNotFoundError(f"stats.txt not found at: {filepath}")

    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("---"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0]
                try:
                    val = float(parts[1])
                    stats[key] = val
                except ValueError:
                    pass

    return stats

def extract_key_metrics(stats: Dict[str, float], label: str) -> Dict[str, Any]:
    sim_sec = stats.get("simSeconds", 0.0)
    sim_ticks = stats.get("simTicks", 0.0)
    insts = stats.get("simInsts", stats.get("system.cpu.numInsts", 0.0))
    cycles = stats.get("system.cpu.numCycles", 0.0)
    ipc = stats.get("system.cpu.ipc", (insts / cycles if cycles > 0 else 0.0))
    cpi = stats.get("system.cpu.cpi", (1.0 / ipc if ipc > 0 else 0.0))
    
    # L1D Miss Rate
    dcache_miss_rate = 0.0
    for k, v in stats.items():
        if "dcache.overallMissRate" in k or "dcache.overall_miss_rate" in k:
            dcache_miss_rate = v * 100.0
            break

    return {
        "label": label,
        "sim_sec": sim_sec,
        "sim_ticks": sim_ticks,
        "insts": int(insts),
        "cycles": int(cycles),
        "ipc": ipc,
        "cpi": cpi,
        "dcache_miss_rate": dcache_miss_rate
    }

def print_markdown_table(runs: List[Dict[str, Any]]) -> str:
    if not runs:
        return "No runs to display."

    baseline = runs[0]
    base_time = baseline["sim_sec"] if baseline["sim_sec"] > 0 else 1.0
    base_insts = baseline["insts"] if baseline["insts"] > 0 else 1.0

    lines = []
    lines.append("\n### gem5 Microarchitectural Comparison (SpacemiT K1 Preset)\n")
    lines.append("| Metric | " + " | ".join(r["label"] for r in runs) + " | Speedup / Reduction |")
    lines.append("| :--- | " + " | ".join(":---:" for _ in runs) + " | :---: |")

    # Time / Latency
    row = ["**Simulated CPU Time**"]
    for r in runs:
        row.append(f"{r['sim_sec']:.6f} s" if r["sim_sec"] > 0 else "N/A")
    speedup = (base_time / runs[-1]["sim_sec"]) if len(runs) > 1 and runs[-1]["sim_sec"] > 0 else 1.0
    row.append(f"**{speedup:.2f}× faster**" if len(runs) > 1 else "1.00× (Baseline)")
    lines.append("| " + " | ".join(row) + " |")

    # Committed Instructions
    row = ["**Committed Insts**"]
    for r in runs:
        row.append(f"{r['insts']:,}")
    reduction = (base_insts / runs[-1]["insts"]) if len(runs) > 1 and runs[-1]["insts"] > 0 else 1.0
    row.append(f"**{reduction:.2f}× fewer insts**" if len(runs) > 1 else "Baseline")
    lines.append("| " + " | ".join(row) + " |")

    # Clock Cycles
    row = ["**Simulated Cycles**"]
    for r in runs:
        row.append(f"{r['cycles']:,}")
    row.append("-")
    lines.append("| " + " | ".join(row) + " |")

    # IPC
    row = ["**IPC (Insts/Cycle)**"]
    for r in runs:
        row.append(f"{r['ipc']:.4f}")
    ipc_delta = (runs[-1]["ipc"] - baseline["ipc"]) if len(runs) > 1 else 0.0
    row.append(f"{'+' if ipc_delta >= 0 else ''}{ipc_delta:.4f}" if len(runs) > 1 else "Baseline")
    lines.append("| " + " | ".join(row) + " |")

    # L1D Cache Miss Rate
    row = ["**L1D Miss Rate**"]
    for r in runs:
        row.append(f"{r['dcache_miss_rate']:.2f}%")
    miss_delta = (runs[-1]["dcache_miss_rate"] - baseline["dcache_miss_rate"]) if len(runs) > 1 else 0.0
    row.append(f"{'+' if miss_delta >= 0 else ''}{miss_delta:.2f}%" if len(runs) > 1 else "Baseline")
    lines.append("| " + " | ".join(row) + " |")

    table_str = "\n".join(lines)
    return table_str

def main():
    parser = argparse.ArgumentParser(description="RVPoint gem5 Simulation Results Comparator")
    parser.add_argument("runs", nargs="+", help="Paths to gem5 results directories or stats.txt files")
    parser.add_argument("--json", "-j", help="Optional path to output comparison JSON")
    parser.add_argument("--markdown", "-m", action="store_true", default=True, help="Print GitHub Markdown table (default: True)")

    args = parser.parse_args()

    parsed_runs = []
    for rpath in args.runs:
        label = os.path.basename(os.path.normpath(rpath))
        if label == "stats.txt":
            label = os.path.basename(os.path.dirname(os.path.abspath(rpath)))
        try:
            stats = parse_stats_file(rpath)
            metrics = extract_key_metrics(stats, label)
            parsed_runs.append(metrics)
        except Exception as e:
            print(f"Warning: Could not parse '{rpath}': {e}", file=sys.stderr)

    if not parsed_runs:
        print("Error: No valid gem5 stats files were parsed.", file=sys.stderr)
        sys.exit(1)

    table = print_markdown_table(parsed_runs)
    print(table)

    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(parsed_runs, f, indent=2)
        print(f"\nComparison JSON exported to: {args.json}")

if __name__ == "__main__":
    main()
