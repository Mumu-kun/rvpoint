"""
gem5 SE (Syscall Emulation) mode configuration for RVPoint benchmarks.

Models a realistic RISC-V edge SoC:
  - CPU:  O3CPU (out-of-order) at 1 GHz  — default, best ILP modelling
          MinorCPU (in-order) is available via --cpu=minor for lower-end cores
  - L1 I/D: 32 KB, 4-way, 2-cycle hit
  - L2:     256 KB, 8-way, 20-cycle hit
  - Memory: DDR4-2400, 1 GB

Usage (called by gem5.opt, NOT run directly):
  gem5.opt scripts/gem5_se.py \
      --cmd=bin/benchmark_gem5_static \
      --options="voxel_rvv 512" \
      [--cpu=o3|minor|timing]

gem5 version: tested on v23.0 / v24.0 (m5.objects API).
"""

import m5
from m5.objects import (
    System, SrcClockDomain, VoltageDomain, AddrRange,
    O3CPU, MinorCPU, TimingSimpleCPU,
    Cache, L2XBar, SystemXBar,
    MemCtrl, DDR4_2400_8x8,
    Process, Root, SEWorkload,
)
import argparse
import os

# ─── Argument parsing ─────────────────────────────────────────────────────────
# parse_known_args ignores gem5's own flags that get mixed in
parser = argparse.ArgumentParser(description="RVPoint gem5 SE config")
parser.add_argument("--cmd",     required=True,  help="Path to binary")
parser.add_argument("--options", default="",     help="Arguments for the binary")
parser.add_argument("--cpu",     default="o3",   choices=["o3", "minor", "timing"],
                    help="CPU model: o3 (out-of-order), minor (in-order), timing (simple)")
args, _ = parser.parse_known_args()

# ─── System ───────────────────────────────────────────────────────────────────
system = System()
system.clk_domain = SrcClockDomain(
    clock="1GHz",
    voltage_domain=VoltageDomain()
)
system.mem_mode   = "timing"
system.mem_ranges = [AddrRange("1GB")]

# ─── CPU ──────────────────────────────────────────────────────────────────────
if args.cpu == "o3":
    system.cpu = O3CPU()
elif args.cpu == "minor":
    system.cpu = MinorCPU()
else:
    system.cpu = TimingSimpleCPU()

# ─── Cache hierarchy ──────────────────────────────────────────────────────────
# L1 I-cache
system.cpu.icache = Cache(
    size="32kB", assoc=4,
    tag_latency=2, data_latency=2, response_latency=2,
    mshrs=4, tgts_per_mshr=20,
)
# L1 D-cache
system.cpu.dcache = Cache(
    size="32kB", assoc=4,
    tag_latency=2, data_latency=2, response_latency=2,
    mshrs=4, tgts_per_mshr=20,
)

# L2 cache + bus
system.l2bus = L2XBar()
system.cpu.icache_port = system.cpu.icache.cpu_side
system.cpu.dcache_port = system.cpu.dcache.cpu_side
system.cpu.icache.mem_side = system.l2bus.cpu_side_ports
system.cpu.dcache.mem_side = system.l2bus.cpu_side_ports

system.l2cache = Cache(
    size="256kB", assoc=8,
    tag_latency=20, data_latency=20, response_latency=20,
    mshrs=20, tgts_per_mshr=12,
)
system.l2bus.mem_side_ports = system.l2cache.cpu_side

# ─── Memory bus + controller ──────────────────────────────────────────────────
system.membus = SystemXBar()
system.l2cache.mem_side      = system.membus.cpu_side_ports
system.system_port           = system.membus.cpu_side_ports

system.mem_ctrl      = MemCtrl()
system.mem_ctrl.dram = DDR4_2400_8x8(range=system.mem_ranges[0])
system.mem_ctrl.port = system.membus.mem_side_ports

# ─── Interrupt controller (required for SE mode) ──────────────────────────────
system.cpu.createInterruptController()

# ─── Workload (SE binary) ─────────────────────────────────────────────────────
# gem5 v23+: SEWorkload must be set on system, not just cpu
system.workload = SEWorkload.init_compatible(args.cmd)

process = Process()
cmd = [args.cmd]
if args.options.strip():
    cmd += args.options.split()
process.cmd = cmd
system.cpu.workload = process
system.cpu.createThreads()

# ─── Run ──────────────────────────────────────────────────────────────────────
root = Root(full_system=False, system=system)
m5.instantiate()

print(f"[gem5_se.py] Simulating: {' '.join(cmd)}")
print(f"[gem5_se.py] CPU model:  {args.cpu.upper()}  |  Clock: 1 GHz  |  L1: 32KB  |  L2: 256KB  |  Mem: DDR4-2400")

exit_event = m5.simulate()

print(f"[gem5_se.py] Exit at tick {m5.curTick():,} — reason: {exit_event.getCause()}")
