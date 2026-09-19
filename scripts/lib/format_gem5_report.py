#!/usr/bin/env python3
"""
==============================================================================
SCRIPT: scripts/lib/format_gem5_report.py
PURPOSE: Simple, sleek reporter for gem5 microarchitectural simulation statistics.
==============================================================================
"""

import sys
import os
from typing import Dict

# ANSI styling
BOLD = "\033[1m"
CYAN = "\033[36m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
RESET = "\033[0m"
DIM = "\033[2m"

def parse_stats(stats_path: str) -> Dict[str, float]:
    stats: Dict[str, float] = {}
    if not os.path.exists(stats_path):
        return stats
    with open(stats_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("---"):
                continue
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0]
                try:
                    stats[key] = float(parts[1])
                except ValueError:
                    pass
    return stats

def render_report(stats_file: str, target: str, backend: str, wall_sec: float) -> None:
    stats = parse_stats(stats_file)
    if not stats:
        print("Error: Could not parse stats from", stats_file, file=sys.stderr)
        return

    sim_sec = stats.get("simSeconds", 0.0)
    insts = int(stats.get("simInsts", stats.get("system.cpu.numInsts", 0)))
    cycles = int(stats.get("system.cpu.numCycles", 0))
    ipc = stats.get("system.cpu.ipc", (insts / cycles if cycles > 0 else 0.0))
    cpi = stats.get("system.cpu.cpi", (1.0 / ipc if ipc > 0 else 0.0))

    # L1D Cache Miss Rate
    dcache_miss = None
    for k, v in stats.items():
        if "dcache.overallMissRate" in k or "dcache.overall_miss_rate" in k:
            dcache_miss = v * 100.0
            break

    # Format timing
    if sim_sec < 1.0:
        time_str = f"{sim_sec * 1000.0:.2f} ms ({sim_sec:.6f} s)"
    else:
        time_str = f"{sim_sec:.4f} s"

    print("")
    print(f"{CYAN}============================================================{RESET}")
    print(f"{BOLD} gem5 Simulation Results (SpacemiT K1 / MinorCPU){RESET}")
    print(f"{CYAN}============================================================{RESET}")
    print(f"  {BOLD}Target Executable{RESET}       : {GREEN}{target}{RESET} [{CYAN}{backend.lower()}{RESET}]")
    print(f"  {BOLD}Simulated CPU Time{RESET}      : {YELLOW}{time_str}{RESET}")
    print(f"  {BOLD}Host Wall-Clock Time{RESET}    : {wall_sec:.1f}s")
    print(f"  {BOLD}Committed Instructions{RESET}  : {insts:,}")
    print(f"  {BOLD}Simulated Clock Cycles{RESET}  : {cycles:,}")
    print(f"  {BOLD}Instructions / Cycle{RESET}    : {GREEN}{ipc:.4f}{RESET} (IPC)")
    print(f"  {BOLD}Cycles / Instruction{RESET}    : {cpi:.4f} (CPI)")
    if dcache_miss is not None:
        print(f"  {BOLD}L1D Cache Miss Rate{RESET}     : {dcache_miss:.2f}%")
    print(f"{CYAN}============================================================{RESET}")
    print(f"  {DIM}Raw stats: {stats_file}{RESET}")
    print("")

def main():
    if len(sys.argv) < 2:
        print("Usage: format_gem5_report.py <stats_file> [target] [backend] [wall_sec]")
        sys.exit(1)

    stats_file = sys.argv[1]
    target = sys.argv[2] if len(sys.argv) > 2 else "Target"
    backend = sys.argv[3] if len(sys.argv) > 3 else "rvv"
    wall_sec = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

    render_report(stats_file, target, backend, wall_sec)

if __name__ == "__main__":
    main()
