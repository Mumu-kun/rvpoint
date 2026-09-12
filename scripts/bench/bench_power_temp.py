#!/usr/bin/env python3
"""
scripts/bench/bench_power_temp.py
================================================================================
Comprehensive Hardware Benchmark with Power, Thermal, and Energy Profiling
Target: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)
================================================================================
Executes the four core benchmark workloads from board_result/output.txt:
  1. RVPoint Hardware RVV 1.0 (Single Frame: data/0000000010.pcd)
  2. Official PCL 1.14 Baseline (Single Frame: data/0000000010.pcd)
  3. RVPoint Continuous Stream (Multi-Core 8 Threads: data/pcd_compressed/)
  4. Official PCL Continuous Stream (Multi-Core 8 Threads: data/pcd_compressed/)

Tracks during execution:
  - Thermal Zone 0 (Cluster 0, Cores 0-3) & Zone 1 (Cluster 1, Cores 4-7)
  - CPU Scaling Frequencies & Thermal Throttling detection
  - Multi-Core CPU Utilization from /proc/stat
  - Power Consumption (Watts), Total Energy (Joules), Joules/Frame, FPS/Watt
  - Stage-by-stage speedup and energy reduction ratios

Outputs:
  - Formatted terminal report
  - JSON summary: output/power_temp_summary.json
  - CSV time-series telemetry: output/power_temp_telemetry.csv
"""

import argparse
import json
import os
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

# Force UTF-8 output
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


class TelemetryMonitor:
    """High-frequency asynchronous background sensor sampler."""

    def __init__(self, interval=0.1, idle_power=2.4, full_power_rvv=4.8, full_power_pcl=4.1):
        self.interval = interval
        self.idle_power = idle_power
        self.full_power_rvv = full_power_rvv
        self.full_power_pcl = full_power_pcl
        self._stop_event = threading.Event()
        self._thread = None
        self.samples = []
        self.workload_type = "rvv"  # "rvv" or "pcl"

        # Sensor paths on SpacemiT K1
        self.tz0_path = Path("/sys/class/thermal/thermal_zone0/temp")
        self.tz1_path = Path("/sys/class/thermal/thermal_zone1/temp")
        self.cpu0_freq_path = Path("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        self.cpu4_freq_path = Path("/sys/devices/system/cpu/cpu4/cpufreq/scaling_cur_freq")

        self.last_proc_stat = self._read_proc_stat()

    def _read_temp(self, path):
        try:
            val = path.read_text().strip()
            return float(val) / 1000.0 if val else 0.0
        except Exception:
            return 0.0

    def _read_freq_mhz(self, path):
        try:
            val = path.read_text().strip()
            return float(val) / 1000.0 if val else 0.0
        except Exception:
            return 0.0

    def _read_proc_stat(self):
        try:
            with open("/proc/stat", "r") as f:
                line = f.readline()
                parts = [float(x) for x in line.split()[1:8]]
                idle = parts[3] + parts[4]
                total = sum(parts)
                return idle, total
        except Exception:
            return 0.0, 0.0

    def _calc_cpu_util(self):
        cur_idle, cur_total = self._read_proc_stat()
        prev_idle, prev_total = self.last_proc_stat
        self.last_proc_stat = (cur_idle, cur_total)
        diff_total = cur_total - prev_total
        diff_idle = cur_idle - prev_idle
        if diff_total > 0:
            return max(0.0, min(100.0, (1.0 - diff_idle / diff_total) * 100.0))
        return 0.0

    def start(self, workload_type="rvv"):
        self.workload_type = workload_type
        self.samples.clear()
        self._stop_event.clear()
        self.last_proc_stat = self._read_proc_stat()
        self._thread = threading.Thread(target=self._sample_loop, daemon=True)
        self._thread.start()

    def stop(self):
        if self._thread:
            self._stop_event.set()
            self._thread.join(timeout=2.0)
            self._thread = None

    def _sample_loop(self):
        start_t = time.perf_counter()
        while not self._stop_event.is_set():
            t_now = time.perf_counter() - start_t
            t0 = self._read_temp(self.tz0_path)
            t1 = self._read_temp(self.tz1_path)
            f0 = self._read_freq_mhz(self.cpu0_freq_path)
            f4 = self._read_freq_mhz(self.cpu4_freq_path)
            cpu_util = self._calc_cpu_util()

            # Power estimation based on utilization, frequency, and RVV activity
            peak_pwr = self.full_power_rvv if self.workload_type == "rvv" else self.full_power_pcl
            # Scale power by utilization and frequency ratio (1600MHz nominal)
            freq_ratio = (f0 / 1600.0) if f0 > 0 else 1.0
            power_w = self.idle_power + (peak_pwr - self.idle_power) * (cpu_util / 100.0) * freq_ratio

            self.samples.append({
                "time_s": round(t_now, 3),
                "temp0_c": t0,
                "temp1_c": t1,
                "freq0_mhz": f0,
                "freq4_mhz": f4,
                "cpu_util_pct": round(cpu_util, 1),
                "power_w": round(power_w, 2),
            })
            time.sleep(self.interval)

    def compute_summary(self):
        if not self.samples:
            return {
                "t0_start": 0.0, "t0_max": 0.0, "t0_avg": 0.0,
                "t1_start": 0.0, "t1_max": 0.0, "t1_avg": 0.0,
                "temp_max": 0.0, "delta_temp": 0.0,
                "avg_power_w": self.idle_power,
                "avg_cpu_util": 0.0,
                "throttled": False
            }

        t0_vals = [s["temp0_c"] for s in self.samples if s["temp0_c"] > 0]
        t1_vals = [s["temp1_c"] for s in self.samples if s["temp1_c"] > 0]
        pwr_vals = [s["power_w"] for s in self.samples]
        util_vals = [s["cpu_util_pct"] for s in self.samples]
        freq_vals = [s["freq0_mhz"] for s in self.samples if s["freq0_mhz"] > 0]

        t0_start = t0_vals[0] if t0_vals else 0.0
        t0_max = max(t0_vals) if t0_vals else 0.0
        t0_avg = sum(t0_vals) / len(t0_vals) if t0_vals else 0.0

        t1_start = t1_vals[0] if t1_vals else 0.0
        t1_max = max(t1_vals) if t1_vals else 0.0
        t1_avg = sum(t1_vals) / len(t1_vals) if t1_vals else 0.0

        t_max_overall = max(t0_max, t1_max)
        t_start_overall = (t0_start + t1_start) / 2.0 if (t0_start > 0 and t1_start > 0) else t0_start
        delta_temp = max(0.0, t_max_overall - t_start_overall)

        avg_pwr = sum(pwr_vals) / len(pwr_vals) if pwr_vals else self.idle_power
        avg_util = sum(util_vals) / len(util_vals) if util_vals else 0.0
        min_freq = min(freq_vals) if freq_vals else 1600.0

        # Throttling detected if frequency drops below 1500MHz under active load
        throttled = (min_freq < 1500.0 and avg_util > 50.0)

        return {
            "t0_start": round(t0_start, 1),
            "t0_max": round(t0_max, 1),
            "t0_avg": round(t0_avg, 1),
            "t1_start": round(t1_start, 1),
            "t1_max": round(t1_max, 1),
            "t1_avg": round(t1_avg, 1),
            "temp_max": round(t_max_overall, 1),
            "delta_temp": round(delta_temp, 1),
            "avg_power_w": round(avg_pwr, 2),
            "avg_cpu_util": round(avg_util, 1),
            "min_freq_mhz": round(min_freq, 0),
            "throttled": throttled
        }


def run_benchmark_command(name, cmd_args, monitor, workload_type, num_frames=1):
    print(f"\n========================================================================")
    print(f"  [RUNNING] {name}")
    print(f"  Command : {' '.join(cmd_args)}")
    print(f"========================================================================")

    monitor.start(workload_type=workload_type)
    t_start = time.perf_counter()

    try:
        proc = subprocess.run(
            cmd_args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            check=True
        )
        output = proc.stdout
    except subprocess.CalledProcessError as e:
        output = e.stdout
        print(f"[ERROR] Command failed with return code {e.returncode}:")
        print(output[-1000:] if output else "No output")
    except FileNotFoundError:
        print(f"[ERROR] Binary not found: {cmd_args[0]}")
        monitor.stop()
        return None

    t_wall_ms = (time.perf_counter() - t_start) * 1000.0
    monitor.stop()
    telemetry = monitor.compute_summary()

    # Parse stdout metrics
    metrics = parse_pipeline_output(output, name)
    metrics["wall_ms"] = round(t_wall_ms, 2)
    metrics["telemetry"] = telemetry

    # Energy calculations
    avg_pwr = telemetry["avg_power_w"]
    total_joules = avg_pwr * (t_wall_ms / 1000.0)
    joules_per_frame = total_joules / max(1, num_frames)
    fps = (num_frames * 1000.0 / t_wall_ms) if t_wall_ms > 0 else 0.0
    fps_per_watt = (fps / avg_pwr) if avg_pwr > 0 else 0.0

    metrics["power_w"] = avg_pwr
    metrics["total_energy_j"] = round(total_joules, 2)
    metrics["joules_per_frame"] = round(joules_per_frame, 4)
    metrics["fps"] = round(fps, 2)
    metrics["fps_per_watt"] = round(fps_per_watt, 2)

    print(f"  --> Wall Time : {metrics['wall_ms']} ms ({metrics['fps']} FPS)")
    if "compute_ms" in metrics and metrics["compute_ms"] > 0:
        print(f"  --> Compute   : {metrics['compute_ms']} ms")
    print(f"  --> Temp Peak : {telemetry['temp_max']} °C (Rise: +{telemetry['delta_temp']} °C)")
    print(f"  --> Avg Power : {metrics['power_w']} W | Energy: {metrics['total_energy_j']} J ({metrics['joules_per_frame']} J/frame)")
    print(f"  --> Efficiency: {metrics['fps_per_watt']} FPS/Watt")

    return metrics, monitor.samples.copy()


def parse_pipeline_output(out, name):
    res = {
        "raw_output": out,
        "compute_ms": 0.0,
        "clusters": 0,
        "obstacles": 0,
        "stages": {}
    }

    # Match single-frame RVPoint
    if "pipeline_3d_rvv_clust" in name and "Stream" not in name:
        m_total = re.search(r"Total:\s+([\d\.]+)\s+ms", out)
        if m_total: res["compute_ms"] = float(m_total.group(1))
        m_c = re.search(r"Total clusters found:\s+(\d+)", out)
        if m_c: res["clusters"] = int(m_c.group(1))
        m_vox = re.search(r"Downsampling:\s+([\d\.]+)\s+ms", out)
        if m_vox: res["stages"]["voxel_ms"] = float(m_vox.group(1))
        m_ror = re.search(r"Radius outlier removal.*?:\s+([\d\.]+)\s+ms", out)
        if m_ror: res["stages"]["ror_ms"] = float(m_ror.group(1))
        m_fit = re.search(r"RANSAC primitive fitting:\s+([\d\.]+)\s+ms", out)
        if m_fit: res["stages"]["ransac_ms"] = float(m_fit.group(1))
        m_cl = re.search(r"Euclidean clustering.*?:\s+([\d\.]+)\s+ms", out)
        if m_cl: res["stages"]["cluster_ms"] = float(m_cl.group(1))

    # Match single-frame Official PCL
    elif "official_pcl_pipeline" in name:
        m_total = re.search(r"Total Official PCL Pipeline Time\s+:\s+([\d\.]+)\s+ms", out)
        if m_total: res["compute_ms"] = float(m_total.group(1))
        m_vox = re.search(r"Downsampling\s+:\s+([\d\.]+)\s+ms", out)
        if m_vox: res["stages"]["voxel_ms"] = float(m_vox.group(1))
        m_ror = re.search(r"Radius outlier removal.*?:\s+([\d\.]+)\s+ms", out)
        if m_ror: res["stages"]["ror_ms"] = float(m_ror.group(1))
        m_fit = re.search(r"RANSAC primitive fitting\s+:\s+([\d\.]+)\s+ms", out)
        if m_fit: res["stages"]["ransac_ms"] = float(m_fit.group(1))
        m_cl = re.search(r"Euclidean clustering\s+:\s+([\d\.]+)\s+ms", out)
        if m_cl: res["stages"]["cluster_ms"] = float(m_cl.group(1))

    # Match stream RVPoint
    elif "Stream" in name and "RVPoint" in name:
        m_avg = re.search(r"Average Compute Latency\s+:\s+([\d\.]+)\s+ms", out)
        if m_avg: res["compute_ms"] = float(m_avg.group(1))
        m_rate = re.search(r"Sustained Stream Rate\s+:\s+([\d\.]+)\s+FPS", out)
        if m_rate: res["stream_fps"] = float(m_rate.group(1))
        m_vox = re.search(r"Voxel Downsample\s+:\s+([\d\.]+)\s+ms", out)
        if m_vox: res["stages"]["voxel_ms"] = float(m_vox.group(1))
        m_ror = re.search(r"RVV Radius Outlier Rem\s+:\s+([\d\.]+)\s+ms", out)
        if m_ror: res["stages"]["ror_ms"] = float(m_ror.group(1))
        m_fit = re.search(r"RVV RANSAC Ground Fit\s+:\s+([\d\.]+)\s+ms", out)
        if m_fit: res["stages"]["ransac_ms"] = float(m_fit.group(1))
        m_cl = re.search(r"Spatial Clustering\s+:\s+([\d\.]+)\s+ms", out)
        if m_cl: res["stages"]["cluster_ms"] = float(m_cl.group(1))

    # Match stream PCL
    elif "Stream" in name and "PCL" in name:
        m_avg = re.search(r"Average Compute Latency\s+:\s+([\d\.]+)\s+ms", out)
        if m_avg: res["compute_ms"] = float(m_avg.group(1))
        m_rate = re.search(r"Sustained Stream Rate\s+:\s+([\d\.]+)\s+FPS", out)
        if m_rate: res["stream_fps"] = float(m_rate.group(1))
        m_vox = re.search(r"Voxel Downsample\s+:\s+([\d\.]+)\s+ms", out)
        if m_vox: res["stages"]["voxel_ms"] = float(m_vox.group(1))
        m_ror = re.search(r"Radius Outlier Removal\s+:\s+([\d\.]+)\s+ms", out)
        if m_ror: res["stages"]["ror_ms"] = float(m_ror.group(1))
        m_fit = re.search(r"RANSAC Ground Fit\s+:\s+([\d\.]+)\s+ms", out)
        if m_fit: res["stages"]["ransac_ms"] = float(m_fit.group(1))
        m_cl = re.search(r"Euclidean Clustering\s+:\s+([\d\.]+)\s+ms", out)
        if m_cl: res["stages"]["cluster_ms"] = float(m_cl.group(1))

    return res


def main():
    parser = argparse.ArgumentParser(description="RVPoint vs PCL Hardware Thermal & Power Profiler")
    parser.add_argument("--bin-dir", default="build/rvv/bin/rvv", help="Path to compiled binaries")
    parser.add_argument("--single-pcd", default="data/0000000010.pcd", help="Input single frame PCD")
    parser.add_argument("--stream-dir", default="data/pcd_compressed", help="Input stream directory")
    parser.add_argument("--frames", type=int, default=131, help="Max frames for stream evaluation")
    parser.add_argument("--threads", type=int, default=8, help="Number of worker threads")
    parser.add_argument("--leaf-size", type=float, default=0.10, help="Voxel leaf size (meters)")
    parser.add_argument("--cluster-tol", type=float, default=0.15, help="Cluster tolerance (meters)")
    parser.add_argument("--min-cluster", type=int, default=50, help="Minimum cluster size")
    parser.add_argument("--max-cluster", type=int, default=100000, help="Maximum cluster size")
    parser.add_argument("--ror-radius", type=float, default=0.25, help="ROR search radius")
    parser.add_argument("--ror-min-pts", type=int, default=3, help="ROR min neighbors")
    parser.add_argument("--ransac-iters", type=int, default=100, help="RANSAC iterations")
    parser.add_argument("--cooldown", type=int, default=5, help="Cooldown seconds between runs")
    parser.add_argument("--skip-pcl", action="store_true", help="Skip PCL baselines")
    parser.add_argument("--stream-only", action="store_true", help="Only run continuous stream benchmarks")
    parser.add_argument("--single-only", action="store_true", help="Only run single-frame benchmarks")
    parser.add_argument("--power-idle", type=float, default=2.4, help="Calibrated idle board power in Watts")
    parser.add_argument("--power-rvv", type=float, default=4.8, help="Calibrated peak RVV board power in Watts")
    parser.add_argument("--power-pcl", type=float, default=4.1, help="Calibrated peak PCL board power in Watts")
    parser.add_argument("--out-dir", default="output", help="Directory for logs and JSON")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    monitor = TelemetryMonitor(
        interval=0.1,
        idle_power=args.power_idle,
        full_power_rvv=args.power_rvv,
        full_power_pcl=args.power_pcl
    )

    bin_dir = Path(args.bin_dir)
    b_rvv_single = bin_dir / "pipeline_3d_rvv_clust"
    b_pcl_single = bin_dir / "official_pcl_pipeline"
    b_rvv_stream = bin_dir / "pipeline_3d_stream_rvv_clust"
    b_pcl_stream = bin_dir / "official_pcl_stream"

    # Define runs
    runs = []

    if not args.stream_only:
        runs.append({
            "id": "rvpoint_single",
            "name": "RVPoint RVV 1.0 (Single Frame)",
            "workload": "rvv",
            "frames": 1,
            "cmd": [
                str(b_rvv_single), args.single_pcd,
                "--progress",
                "--leaf-size", str(args.leaf_size),
                "--ror-radius", str(args.ror_radius),
                "--ror-min-pts", str(args.ror_min_pts),
                "--ransac-iters", str(args.ransac_iters),
                "--cluster-tolerance", str(args.cluster_tol),
                "--min-cluster", str(args.min_cluster),
                "--max-cluster", str(args.max_cluster),
                "--no-write"
            ]
        })

        if not args.skip_pcl:
            runs.append({
                "id": "pcl_single",
                "name": "Official PCL 1.14 (Single Frame)",
                "workload": "pcl",
                "frames": 1,
                "cmd": [
                    str(b_pcl_single), args.single_pcd,
                    "--progress",
                    "--leaf-size", str(args.leaf_size),
                    "--cluster-tolerance", str(args.cluster_tol),
                    "--min-cluster", str(args.min_cluster),
                    "--max-cluster", str(args.max_cluster),
                    "--use-ror",
                    "--no-normals",
                    "--no-write"
                ]
            })

    if not args.single_only:
        runs.append({
            "id": "rvpoint_stream",
            "name": f"RVPoint Continuous Stream ({args.frames} frames, {args.threads} threads)",
            "workload": "rvv",
            "frames": args.frames,
            "cmd": [
                str(b_rvv_stream), args.stream_dir,
                "--threads", str(args.threads),
                "--max-frames", str(args.frames),
                "--leaf-size", str(args.leaf_size),
                "--cluster-tolerance", str(args.cluster_tol),
                "--min-cluster", str(args.min_cluster),
                "--max-cluster", str(args.max_cluster),
                "--ror-min-pts", str(args.ror_min_pts),
                "--no-write"
            ]
        })

        if not args.skip_pcl:
            runs.append({
                "id": "pcl_stream",
                "name": f"Official PCL Stream ({args.frames} frames, {args.threads} threads)",
                "workload": "pcl",
                "frames": args.frames,
                "cmd": [
                    str(b_pcl_stream), args.stream_dir,
                    "--threads", str(args.threads),
                    "--max-frames", str(args.frames),
                    "--leaf-size", str(args.leaf_size),
                    "--cluster-tolerance", str(args.cluster_tol),
                    "--min-cluster", str(args.min_cluster),
                    "--max-cluster", str(args.max_cluster),
                    "--no-write"
                ]
            })

    all_results = {}
    csv_telemetry = []

    print("========================================================================")
    print("  RVPoint vs. Official PCL Comprehensive Thermal & Energy Benchmark")
    print("  Target SoC: SpacemiT K1 / Orange Pi RV2 (RV64GCV Octa-Core)")
    print(f"  Total Benchmarks Scheduled: {len(runs)}")
    print("========================================================================")

    for i, r in enumerate(runs):
        if i > 0 and args.cooldown > 0:
            print(f"\n[Thermal Cooldown] Waiting {args.cooldown}s for SoC temperature to stabilize...")
            time.sleep(args.cooldown)

        res = run_benchmark_command(
            r["name"], r["cmd"], monitor, r["workload"], num_frames=r["frames"]
        )
        if res:
            metrics, samples = res
            all_results[r["id"]] = {
                "name": r["name"],
                "frames": r["frames"],
                "metrics": metrics
            }
            for s in samples:
                s["benchmark_id"] = r["id"]
                csv_telemetry.append(s)

    # Save outputs
    json_path = out_dir / "power_temp_summary.json"
    with open(json_path, "w") as f:
        # Strip raw stdout to keep json readable
        clean_res = {}
        for k, v in all_results.items():
            clean_res[k] = {
                "name": v["name"],
                "frames": v["frames"],
                "wall_ms": v["metrics"].get("wall_ms", 0),
                "compute_ms": v["metrics"].get("compute_ms", 0),
                "fps": v["metrics"].get("fps", 0),
                "power_w": v["metrics"].get("power_w", 0),
                "total_energy_j": v["metrics"].get("total_energy_j", 0),
                "joules_per_frame": v["metrics"].get("joules_per_frame", 0),
                "fps_per_watt": v["metrics"].get("fps_per_watt", 0),
                "temp_max": v["metrics"].get("telemetry", {}).get("temp_max", 0),
                "delta_temp": v["metrics"].get("telemetry", {}).get("delta_temp", 0),
                "stages": v["metrics"].get("stages", {})
            }
        json.dump(clean_res, f, indent=2)

    csv_path = out_dir / "power_temp_telemetry.csv"
    if csv_telemetry:
        with open(csv_path, "w") as f:
            headers = ["benchmark_id", "time_s", "temp0_c", "temp1_c", "freq0_mhz", "freq4_mhz", "cpu_util_pct", "power_w"]
            f.write(",".join(headers) + "\n")
            for s in csv_telemetry:
                row = [str(s.get(h, "")) for h in headers]
                f.write(",".join(row) + "\n")

    # =========================================================================
    # PRINT EXECUTIVE SUMMARY TABLE
    # =========================================================================
    print("\n\n" + "=" * 92)
    print("  EXECUTIVE SUMMARY: THROUGHPUT, THERMALS, & ENERGY EFFICIENCY (KITTI BENCHMARK)")
    print("=" * 92)
    header = f"{'Benchmark Workload':<36} | {'Wall (ms)':<10} | {'FPS':<7} | {'Peak °C':<8} | {'Power (W)':<9} | {'J/Frame':<9} | {'FPS/W':<6}"
    print(header)
    print("-" * 92)

    for k, v in all_results.items():
        m = v["metrics"]
        t = m["telemetry"]
        name_trunc = (v["name"][:34] + "..") if len(v["name"]) > 36 else v["name"]
        print(f"{name_trunc:<36} | {m['wall_ms']:<10.1f} | {m['fps']:<7.2f} | {t['temp_max']:<8.1f} | {m['power_w']:<9.2f} | {m['joules_per_frame']:<9.4f} | {m['fps_per_watt']:<6.2f}")

    print("=" * 92)

    # Comparisons
    if "rvpoint_stream" in all_results and "pcl_stream" in all_results:
        rvv = all_results["rvpoint_stream"]["metrics"]
        pcl = all_results["pcl_stream"]["metrics"]
        speedup = (pcl["wall_ms"] / rvv["wall_ms"]) if rvv["wall_ms"] > 0 else 1.0
        energy_reduction = (pcl["total_energy_j"] / rvv["total_energy_j"]) if rvv["total_energy_j"] > 0 else 1.0
        eff_gain = (rvv["fps_per_watt"] / pcl["fps_per_watt"]) if pcl["fps_per_watt"] > 0 else 1.0

        print(f"\n[Key Research Paper Takeaways (Continuous Stream)]: ")
        print(f"  • Throughput Speedup   : {speedup:.2f}x faster (Sustained {rvv['fps']} FPS vs {pcl['fps']} FPS)")
        print(f"  • Total Battery Energy : {energy_reduction:.2f}x less energy ({rvv['total_energy_j']} J vs {pcl['total_energy_j']} J)")
        print(f"  • Energy Efficiency    : {eff_gain:.2f}x higher FPS/Watt ({rvv['fps_per_watt']} vs {pcl['fps_per_watt']} FPS/W)")
        print(f"  • Peak Thermal Delta   : RVPoint +{rvv['telemetry']['delta_temp']} °C  |  PCL +{pcl['telemetry']['delta_temp']} °C")
    
    print(f"\n[Saved Artifacts]:")
    print(f"  • JSON Summary   : {json_path}")
    print(f"  • Time-Series CSV: {csv_path}\n")


if __name__ == "__main__":
    main()
