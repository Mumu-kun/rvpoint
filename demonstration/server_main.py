#!/usr/bin/env python3
"""
server_main.py — Real-time LiDAR Perception Stream Server for Orange Pi RV2 (RVV 1.0).

Receives iPhone LiDAR depth frames over TCP, periodically captures frames at a
configurable rate (default 500ms / 2 Hz), executes the native hardware-vectorized
perception pipeline (pipeline_3d_rvv_clust), and exports the resulting clustered
point clouds directly into the output directory (default: main_scans/).

Usage Examples:
  # Standard Orange Pi RV2 native execution (defaults to 500ms periodic capture):
  python3 demonstration/server_main.py

  # Custom capture interval (e.g. 250ms) and custom output folder:
  python3 demonstration/server_main.py --interval 0.25 --out main_scans

  # Custom clustering & voxel parameters:
  python3 demonstration/server_main.py --leaf-size 0.08 --cluster-tolerance 0.20

  # Synthetic test without an iPhone:
  python3 demonstration/server_main.py --fake
"""

import argparse
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

MAGIC = b"LDP1"
HEADER = struct.Struct("<III4f16f")  # frame_idx, w, h, fx, fy, cx, cy, pose(4x4)


def get_local_ips() -> List[str]:
    """Discover active LAN IPs to display to the user for iPhone connection."""
    ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ips.append(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if ip not in ips and not ip.startswith("127."):
                ips.append(ip)
    except OSError:
        pass
    return ips or ["127.0.0.1"]


def find_pipeline_binary(specified_path: Optional[str] = None) -> Path:
    """Locate the pipeline_3d_rvv_clust binary."""
    candidates = []
    if specified_path:
        candidates.append(Path(specified_path))

    repo_root = Path(__file__).resolve().parent.parent
    candidates.extend([
        repo_root / "build" / "rvv" / "bin" / "rvv" / "pipeline_3d_rvv_clust",
        repo_root / "build" / "bin" / "rvv" / "pipeline_3d_rvv_clust",
        repo_root / "build" / "rvv" / "bin" / "pipeline_3d_rvv_clust",
        repo_root / "build" / "bin" / "pipeline_3d_rvv_clust",
        repo_root / "build_cmake" / "eval" / "pipelines" / "pipeline_3d_rvv_clust",
        repo_root / "eval" / "pipelines" / "pipeline_3d_rvv_clust",
        Path("pipeline_3d_rvv_clust"),
    ])

    which_bin = shutil.which("pipeline_3d_rvv_clust")
    if which_bin:
        candidates.append(Path(which_bin))

    for cand in candidates:
        if cand.is_file() and os.access(cand, os.X_OK):
            return cand.resolve()

    # If not yet built, return the most canonical expected location
    return repo_root / "build" / "rvv" / "bin" / "rvv" / "pipeline_3d_rvv_clust"


def recv_frame(conn: socket.socket, buf: bytearray) -> Optional[Dict]:
    """Parse next complete LiDAR frame from TCP buffer; returns dict or None on timeout."""
    while True:
        i = buf.find(MAGIC)
        if i == -1:
            del buf[:max(0, len(buf) - 3)]
        elif i > 0:
            del buf[:i]

        if len(buf) >= 4 + HEADER.size:
            frame_idx, w, h, fx, fy, cx, cy, *pose = HEADER.unpack_from(buf, 4)
            need = w * h * 5  # float32 depth (w*h*4) + uint8 confidence (w*h*1)
            if len(buf) >= 4 + HEADER.size + need:
                start = 4 + HEADER.size
                depth = np.frombuffer(buf, "<f4", w * h, start).reshape(h, w).copy()
                conf = np.frombuffer(buf, np.uint8, w * h, start + w * h * 4).reshape(h, w).copy()
                del buf[: start + need]
                return {
                    "idx": frame_idx,
                    "w": w,
                    "h": h,
                    "fx": fx,
                    "fy": fy,
                    "cx": cx,
                    "cy": cy,
                    "pose": np.asarray(pose, np.float64).reshape(4, 4),
                    "depth": depth,
                    "conf": conf,
                }

        try:
            chunk = conn.recv(262144)
        except socket.timeout:
            return None
        if not chunk:
            raise ConnectionError("iPhone client disconnected")
        buf += chunk


def unproject_points(frame: Dict, args: argparse.Namespace) -> np.ndarray:
    """Project depth map to 3D point cloud (Nx3 float32)."""
    depth, conf = frame["depth"], frame["conf"]
    h, w = depth.shape
    fx, fy, cx, cy = frame["fx"], frame["fy"], frame["cx"], frame["cy"]

    u = np.arange(w, dtype=np.float32)[None, :]
    v = np.arange(h, dtype=np.float32)[:, None]
    z = depth
    x = (u - np.float32(cx)) * z / np.float32(fx)
    y = (v - np.float32(cy)) * z / np.float32(fy)

    valid = (conf >= args.min_conf) & (z > args.min_range) & (z < args.max_range) & np.isfinite(z)
    if not valid.any():
        return np.zeros((0, 3), np.float32)

    pts = np.stack([x[valid], y[valid], z[valid]], axis=1)

    if args.rot90 == 1:
        pts = np.stack([-pts[:, 1], pts[:, 0], pts[:, 2]], axis=1)
    elif args.rot90 == 2:
        pts = np.stack([-pts[:, 0], -pts[:, 1], pts[:, 2]], axis=1)
    elif args.rot90 == 3:
        pts = np.stack([pts[:, 1], -pts[:, 0], pts[:, 2]], axis=1)

    if args.flip_x:
        pts[:, 0] *= -1
    if args.flip_y:
        pts[:, 1] *= -1
    if args.flip_z:
        pts[:, 2] *= -1

    if not args.cam_frame:
        pose = frame["pose"]
        pts = pts @ pose[:3, :3].T + pose[:3, 3]

    return pts.astype(np.float32)


def write_binary_pcd(path: str, pts: np.ndarray) -> None:
    """Fast write uncompressed binary PCD to RAM disk for zero-overhead pipeline ingestion."""
    n = pts.shape[0]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


def fake_frame(t: float) -> Dict:
    """Generate synthetic 3D depth frame for pipeline testing without an iPhone."""
    w, h = 256, 192
    u = np.arange(w, dtype=np.float32)[None, :].repeat(h, 0)
    v = np.arange(h, dtype=np.float32)[:, None].repeat(w, 1)
    z = 1.5 + 0.3 * np.sin((u + v) / 25.0 + t)
    return {
        "idx": int(t * 10),
        "w": w,
        "h": h,
        "fx": 230.0,
        "fy": 230.0,
        "cx": w / 2,
        "cy": h / 2,
        "pose": np.eye(4),
        "depth": z.astype(np.float32),
        "conf": np.full((h, w), 2, np.uint8),
    }


class FrameProcessor:
    """Processes captured frames through the native RVV pipeline every interval."""

    def __init__(self, args: argparse.Namespace, pipeline_bin: Path):
        self.args = args
        self.pipeline_bin = pipeline_bin
        self.latest_frame: Optional[Dict] = None
        self.frame_lock = threading.Lock()
        self.last_capture_time = 0.0
        self.processed_count = 0
        self.running = True

        # Use /dev/shm (shared memory RAM-disk) on Linux to eliminate flash storage wear
        shm_candidate = Path("/dev/shm")
        if shm_candidate.is_dir() and os.access(shm_candidate, os.W_OK):
            self.temp_base = shm_candidate / f"rvpoint_{os.getpid()}"
        else:
            self.temp_base = Path(tempfile.gettempdir()) / f"rvpoint_{os.getpid()}"
        self.temp_base.mkdir(parents=True, exist_ok=True)

        self.in_pcd_temp = str(self.temp_base / "stream_in.pcd")
        self.out_temp_dir = self.temp_base / "out"
        self.out_temp_dir.mkdir(parents=True, exist_ok=True)

    def cleanup(self):
        shutil.rmtree(self.temp_base, ignore_errors=True)

    def submit_frame(self, frame: Dict):
        """Called whenever a network packet arrives from the iPhone."""
        with self.frame_lock:
            self.latest_frame = frame

    def process_live_loop(self):
        """Main periodic processing loop running at the specified interval (default: 500ms)."""
        while self.running:
            now = time.time()
            elapsed = now - self.last_capture_time
            sleep_needed = self.args.interval - elapsed
            if sleep_needed > 0:
                time.sleep(min(sleep_needed, 0.02))
                continue

            frame_to_process = None
            with self.frame_lock:
                if self.latest_frame is not None:
                    frame_to_process = self.latest_frame
                    self.latest_frame = None  # Consume frame

            if frame_to_process is None:
                time.sleep(0.01)
                continue

            self.last_capture_time = time.time()
            self._execute_pipeline(frame_to_process)

    def _execute_pipeline(self, frame: Dict):
        frame_idx = frame["idx"]
        pts = unproject_points(frame, self.args)
        if len(pts) == 0:
            print(f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx}: No valid points in range.")
            return

        # Cap max points if specified
        if len(pts) > self.args.max_pts:
            sel = np.linspace(0, len(pts) - 1, self.args.max_pts).astype(np.int64)
            pts = pts[sel]

        # 1. Write temporary input PCD in RAM disk (not counted in algorithm process time)
        write_binary_pcd(self.in_pcd_temp, pts)

        # 2. Assemble native command for pipeline_3d_rvv_clust
        cmd = []
        if self.args.qemu:
            cmd.extend(["qemu-riscv64", "-cpu", "rv64,v=true,vlen=128"])

        cmd.extend([
            str(self.pipeline_bin),
            self.in_pcd_temp,
            str(self.out_temp_dir),
            "--json",
            "--leaf-size", str(self.args.leaf_size),
            "--cluster-tolerance", str(self.args.cluster_tolerance),
            "--min-cluster", str(self.args.min_cluster),
            "--max-cluster", str(self.args.max_cluster),
            "--ransac-iters", str(self.args.ransac_iters),
            "--ror-radius", str(self.args.ror_radius),
            "--ror-min-pts", str(self.args.ror_min_pts),
            "--ground-angle-thresh", str(self.args.ground_angle_thresh),
        ])

        if self.args.skip_sor:
            cmd.append("--skip-sor")
        else:
            cmd.append("--use-ror")

        if self.args.no_ground_prior:
            cmd.append("--no-ground-prior")
        if self.args.optical_frame:
            cmd.append("--optical-frame")

        # 3. Measure native pipeline execution
        t_start = time.perf_counter()
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        t_end = time.perf_counter()
        wall_process_ms = (t_end - t_start) * 1000.0

        if res.returncode != 0:
            print(f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx} pipeline failed (code {res.returncode}):")
            print(res.stderr or res.stdout)
            return

        # 4. Extract stage breakdown from metrics.json to isolate pure compute from I/O write time
        metrics_file = self.out_temp_dir / "metrics.json"
        compute_ms = None
        io_write_ms = 0.0
        clusters_found = 0

        if metrics_file.is_file():
            try:
                with open(metrics_file, "r") as f:
                    data = json.load(f)
                    clusters_found = data.get("clusters_found", 0)
                    stages = data.get("stages", [])

                    # Compute stages: 3 (Downsample), 4 (Search Index), 5 (ROR), 8 (RANSAC), 9 (Euclidean Clust)
                    # Stage 1 is Load PCD and Stage 10 is Write cluster PCD
                    compute_stage_indices = {3, 4, 5, 6, 7, 8, 9}
                    stage_compute_times = [s["time_ms"] for s in stages if s.get("stage") in compute_stage_indices]
                    if stage_compute_times:
                        compute_ms = sum(stage_compute_times)

                    write_stage_times = [s["time_ms"] for s in stages if s.get("stage") in {2, 10}]
                    if write_stage_times:
                        io_write_ms = sum(write_stage_times)
            except Exception as e:
                pass

        # If metrics parsing was unavailable, fallback to wall process time minus file writing
        effective_process_ms = compute_ms if compute_ms is not None else max(0.0, wall_process_ms - io_write_ms)

        # 5. Move/save processed colored clusters PCD to main_scans/
        produced_clust_pcd = self.out_temp_dir / "06_clusters.pcd"
        if produced_clust_pcd.is_file():
            self.processed_count += 1
            dest_filename = f"processed_frame_{self.processed_count:06d}_{time.strftime('%H%M%S')}.pcd"
            dest_path = os.path.join(self.args.out, dest_filename)
            shutil.copyfile(produced_clust_pcd, dest_path)

            print(
                f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx} -> "
                f"Processing time: {effective_process_ms:6.2f} ms "
                f"(pure compute, writing time excluded) | "
                f"Clusters: {clusters_found:2d} | "
                f"Input pts: {len(pts):,} | "
                f"Saved: {dest_path}"
            )
        else:
            print(
                f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx} -> "
                f"Processing time: {effective_process_ms:6.2f} ms | "
                f"No clusters detected / no output written."
            )


def main():
    ap = argparse.ArgumentParser(
        description="RVPoint Real-Time LiDAR Stream Server (Orange Pi RV2 Hardware RVV 1.0)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Server Networking & Cadence
    ap.add_argument("--port", type=int, default=9000, help="TCP listen port for iPhone app")
    ap.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Periodic capture interval in seconds (default: 500ms / 0.5s)",
    )
    ap.add_argument(
        "--out",
        default="main_scans",
        help="Destination directory for processed per-frame clustered PCDs",
    )
    ap.add_argument(
        "--bin",
        default=None,
        help="Path to compiled pipeline_3d_rvv_clust executable (auto-discovered if omitted)",
    )

    # Algorithm & Pipeline Tuning
    ap.add_argument("--leaf-size", type=float, default=0.10, help="Voxel downsample leaf size (m)")
    ap.add_argument("--cluster-tolerance", type=float, default=0.15, help="Euclidean clustering radius (m)")
    ap.add_argument("--min-cluster", type=int, default=50, help="Minimum points per cluster")
    ap.add_argument("--max-cluster", type=int, default=100000, help="Maximum points per cluster")
    ap.add_argument("--ransac-iters", type=int, default=100, help="RANSAC plane fit iterations")
    ap.add_argument("--ror-radius", type=float, default=0.25, help="Radius outlier removal radius (m)")
    ap.add_argument("--ror-min-pts", type=int, default=2, help="Minimum neighbor count for ROR")
    ap.add_argument("--skip-sor", action="store_true", help="Bypass outlier removal filtering stage")
    ap.add_argument("--ground-angle-thresh", type=float, default=45.0, help="Max ground plane normal tilt (deg)")
    ap.add_argument("--no-ground-prior", action="store_true", help="Disable ground normal orientation prior")
    ap.add_argument("--optical-frame", action="store_true", help="Set camera optical frame (+Y down)")

    # Pinhole & Point Cloud Range
    ap.add_argument("--min-conf", type=int, default=1, choices=(0, 1, 2), help="0=low, 1=medium, 2=high")
    ap.add_argument("--min-range", type=float, default=0.1, help="Min depth range (m)")
    ap.add_argument("--max-range", type=float, default=6.0, help="Max depth range (m)")
    ap.add_argument("--max-pts", type=int, default=500_000, help="Point count ceiling")
    ap.add_argument("--cam-frame", action="store_true", help="Keep points in camera frame instead of world")
    ap.add_argument("--rot90", type=int, default=0, choices=(0, 1, 2, 3), help="Rotate cloud by 90deg steps")
    ap.add_argument("--flip-x", action="store_true", help="Flip X axis")
    ap.add_argument("--flip-y", action="store_true", help="Flip Y axis")
    ap.add_argument("--flip-z", action="store_true", help="Flip Z axis")

    # Simulation & Test Modes
    ap.add_argument("--fake", action="store_true", help="Generate synthetic test frames (no iPhone needed)")
    ap.add_argument("--qemu", action="store_true", help="Run executable through qemu-riscv64 for cross-testing")

    args = ap.parse_args()

    # Ensure output directory exists
    os.makedirs(args.out, exist_ok=True)

    # Locate executable
    pipeline_bin = find_pipeline_binary(args.bin)
    if not args.fake and not pipeline_bin.is_file():
        print(f"\n[WARNING] Pipeline binary not found at: {pipeline_bin}")
        print("  On Orange Pi RV2, compile it with:")
        print("    cmake -B build/rvv -DRISCV_ARCH=rv64gcv")
        print("    cmake --build build/rvv -j8 --target pipeline_3d_rvv_clust\n")

    print("=" * 72)
    print("  RVPoint Real-Time LiDAR Stream Server (Hardware RVV 1.0)")
    print("=" * 72)
    print(f"Target Binary       : {pipeline_bin}")
    print(f"Periodic Capture    : Every {args.interval * 1000.0:.0f} ms ({1.0 / args.interval:.1f} Hz)")
    print(f"Output Directory    : {args.out}/ (Processed PCD per frame)")
    print(f"Voxel Leaf Size     : {args.leaf_size} m | Tolerance: {args.cluster_tolerance} m")
    print(f"RANSAC Iterations   : {args.ransac_iters} | Min Cluster: {args.min_cluster}")
    print("-" * 72)
    print("Connect your iPhone LiDAR Streamer app to one of these IP addresses:")
    for ip in get_local_ips():
        print(f"    --> {ip}:{args.port}")
    print("=" * 72)

    processor = FrameProcessor(args, pipeline_bin)

    # Start background processing thread that enforces the 500ms capture interval
    proc_thread = threading.Thread(target=processor.process_live_loop, daemon=True)
    proc_thread.start()

    # Fake mode for verification without physical iPhone
    if args.fake:
        print("\n[FAKE MODE] Generating synthetic point clouds at ~10 Hz...")
        t = 0.0
        try:
            while True:
                processor.submit_frame(fake_frame(t))
                t += 0.1
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nShutting down server...")
        finally:
            processor.running = False
            processor.cleanup()
        return

    # Network socket server
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)

    try:
        while True:
            print(f"\nWaiting for iPhone LiDAR stream on port {args.port}...")
            conn, addr = srv.accept()
            conn.settimeout(0.2)
            print(f"iPhone connected from {addr[0]}:{addr[1]}")
            buf = bytearray()

            try:
                while True:
                    frame = recv_frame(conn, buf)
                    if frame is not None:
                        processor.submit_frame(frame)
            except OSError as e:
                print(f"Connection closed ({e}). Waiting for reconnect...")
            finally:
                conn.close()

    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        processor.running = False
        processor.cleanup()
        srv.close()
        print("Server shutdown complete.")


if __name__ == "__main__":
    main()
