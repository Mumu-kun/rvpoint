#!/usr/bin/env python3
"""
server_main.py — Real-Time iPhone LiDAR Perception Stream Server for Orange Pi RV2 (RVV 1.0).

Receives iPhone LiDAR depth frames over TCP, periodically captures frames at a
configurable rate (default: 500ms / 2 Hz), and saves two distinct streams:
  1. main_scans/       --> Raw, unprojected 3D point cloud frames captured from iPhone.
  2. processed_scans/  --> Processed point cloud frames with ground plane extracted
                           and obstacles clustered with unique RGB colors via hardware RVV 1.0.

Compatible with both:
  - pipeline_3d_ultra      (Recommended for demonstrations: robust fallback, full 10 stages)
  - pipeline_3d_rvv_clust  (Minimal standalone benchmark target)

Usage Examples:
  # Standard Orange Pi RV2 execution (auto-detects pipeline_3d_ultra or pipeline_3d_rvv_clust):
  python3 demonstration/server_main.py

  # Explicitly select pipeline_3d_ultra:
  python3 demonstration/server_main.py --pipeline ultra

  # Save intermediate stages (ground plane & obstacle clouds) in processed_scans/:
  python3 demonstration/server_main.py --save-stages

  # Custom capture interval (e.g. 250ms) and tuning parameters:
  python3 demonstration/server_main.py --interval 0.25 --leaf-size 0.08 --cluster-tolerance 0.18

  # Synthetic offline test without an iPhone:
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


def find_pipeline_binary(pipeline_choice: str = "auto", specified_path: Optional[str] = None) -> Tuple[Path, str]:
    """
    Locate the perception pipeline binary (pipeline_3d_ultra or pipeline_3d_rvv_clust).
    Returns (Path, pipeline_type).
    """
    repo_root = Path(__file__).resolve().parent.parent

    if specified_path:
        p = Path(specified_path)
        if p.is_file() and os.access(p, os.X_OK):
            p_type = "ultra" if "ultra" in p.name else "rvv_clust"
            return p.resolve(), p_type

    build_dirs = [
        repo_root / "build" / "rvv" / "bin" / "rvv",
        repo_root / "build" / "bin" / "rvv",
        repo_root / "build" / "rvv" / "bin",
        repo_root / "build" / "bin",
        repo_root / "build_cmake" / "eval" / "pipelines",
        repo_root / "eval" / "pipelines",
    ]

    names_to_try = []
    if pipeline_choice == "ultra":
        names_to_try = ["pipeline_3d_ultra"]
    elif pipeline_choice == "rvv_clust":
        names_to_try = ["pipeline_3d_rvv_clust"]
    else:  # auto: prefer pipeline_3d_ultra for demonstrations
        names_to_try = ["pipeline_3d_ultra", "pipeline_3d_rvv_clust"]

    for name in names_to_try:
        for bdir in build_dirs:
            cand = bdir / name
            if cand.is_file() and os.access(cand, os.X_OK):
                p_type = "ultra" if "ultra" in name else "rvv_clust"
                return cand.resolve(), p_type

        which_bin = shutil.which(name)
        if which_bin:
            p_type = "ultra" if "ultra" in name else "rvv_clust"
            return Path(which_bin).resolve(), p_type

    # Default fallback path
    chosen_name = names_to_try[0]
    p_type = "ultra" if "ultra" in chosen_name else "rvv_clust"
    return repo_root / "build" / "rvv" / "bin" / "rvv" / chosen_name, p_type


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
    """Fast write uncompressed binary PCD for point cloud storage and pipeline ingestion."""
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


def pack_rgb_float(r: int, g: int, b: int) -> float:
    """Pack 8-bit RGB into IEEE-754 float32 for PCD viewer compatibility."""
    val = (int(r) << 16) | (int(g) << 8) | int(b)
    return np.frombuffer(np.uint32(val).tobytes(), dtype=np.float32)[0]


def save_pcd_xyzrgb(path: str, x: np.ndarray, y: np.ndarray, z: np.ndarray, rgb: np.ndarray) -> None:
    """Write binary XYZRGB PCD."""
    n = len(x)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z rgb\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    ).encode("ascii")
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
    arr = np.empty(n, dtype=dt)
    arr["x"] = x
    arr["y"] = y
    arr["z"] = z
    arr["rgb"] = rgb
    with open(path, "wb") as f:
        f.write(header)
        f.write(arr.tobytes())


def load_pcd_binary_data(path: Path) -> Tuple[List[str], np.ndarray]:
    """Read binary PCD into structured numpy array."""
    with open(path, "rb") as f:
        fields = []
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("FIELDS"):
                fields = line.split()[1:]
            elif line.startswith("DATA binary"):
                break
        if "rgb" in fields:
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        else:
            dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
        return fields, np.frombuffer(f.read(), dtype=dt)


def fake_frame(t: float) -> Dict:
    """Generate synthetic 3D room/obstacle depth frame for testing without an iPhone."""
    w, h = 256, 192
    u = np.arange(w, dtype=np.float32)[None, :].repeat(h, 0)
    v = np.arange(h, dtype=np.float32)[:, None].repeat(w, 1)

    # Simulated ground plane at bottom half, obstacle block in center
    z = np.full((h, w), 2.0, dtype=np.float32)
    z[80:120, 100:150] = 1.2 + 0.1 * np.sin(t)  # obstacle closer to camera
    z[140:, :] = 1.5 + (v[140:, :] - 140) * 0.01  # slanted floor plane

    return {
        "idx": int(t * 10),
        "w": w,
        "h": h,
        "fx": 230.0,
        "fy": 230.0,
        "cx": w / 2,
        "cy": h / 2,
        "pose": np.eye(4),
        "depth": z,
        "conf": np.full((h, w), 2, np.uint8),
    }


class FrameProcessor:
    """Processes captured frames and writes both raw frames and clustered outputs."""

    def __init__(self, args: argparse.Namespace, pipeline_bin: Path, pipeline_type: str):
        self.args = args
        self.pipeline_bin = pipeline_bin
        self.pipeline_type = pipeline_type
        self.latest_frame: Optional[Dict] = None
        self.frame_lock = threading.Lock()
        self.last_capture_time = 0.0
        self.processed_count = 0
        self.running = True

        # Use /dev/shm (RAM-disk) for pipeline scratch buffers to eliminate SSD/SD wear
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
            print(f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx}: 0 valid 3D points in range.")
            return

        # Cap max points if specified
        if len(pts) > self.args.max_pts:
            sel = np.linspace(0, len(pts) - 1, self.args.max_pts).astype(np.int64)
            pts = pts[sel]

        self.processed_count += 1
        time_tag = time.strftime("%H%M%S")
        frame_tag = f"frame_{self.processed_count:06d}_{time_tag}.pcd"

        # ── 1. SAVE RAW ORIGINAL FRAME TO main_scans/ ─────────────────────────────
        raw_dest_path = os.path.join(self.args.raw_dir, frame_tag)
        write_binary_pcd(raw_dest_path, pts)

        # Also write to temporary scratch file for pipeline execution
        write_binary_pcd(self.in_pcd_temp, pts)

        # ── 2. ASSEMBLE COMMAND FOR THE PERCEPTION PIPELINE ────────────────────────
        if not self.pipeline_bin.is_file():
            print(
                f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx:04d} -> "
                f"Raw: {len(pts):,} pts saved to {self.args.raw_dir}/ | "
                f"[Pipeline binary not found at {self.pipeline_bin}]"
            )
            return

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

        # Ground plane orientation prior for iPhone LiDAR:
        # In ARKit world coordinates, +Y is vertical (ground normal is [0, 1, 0]).
        # Passing --optical-frame tells RANSAC the floor is perpendicular to +Y,
        # perfectly isolating the horizontal floor and preventing diagonal cuts.
        if self.args.unconstrained_plane:
            cmd.append("--no-ground-prior")
        elif self.args.vehicle_frame:
            pass  # Vehicle frame (+Z up) is default in C++
        else:
            # Default for iPhone LiDAR: camera optical frame (+Y vertical)
            cmd.append("--optical-frame")

        if self.args.skip_sor:
            cmd.append("--skip-sor")
        else:
            cmd.append("--use-ror")

        # Clean out scratch directory before running
        for f in self.out_temp_dir.glob("*.pcd"):
            f.unlink(missing_ok=True)
        (self.out_temp_dir / "metrics.json").unlink(missing_ok=True)

        # ── 3. EXECUTE NATIVE HARDWARE-ACCELERATED PIPELINE ─────────────────────────
        t_start = time.perf_counter()
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        t_end = time.perf_counter()
        wall_process_ms = (t_end - t_start) * 1000.0

        if res.returncode != 0:
            print(f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx} pipeline failed (exit code {res.returncode}):")
            print(res.stderr or res.stdout)
            return

        # ── 4. PARSE METRICS (COMPUTE ONLY, EXCLUDING I/O WRITE TIME) ─────────────
        metrics_file = self.out_temp_dir / "metrics.json"
        compute_ms = None
        io_write_ms = 0.0
        clusters_found = 0
        ground_inliers = 0
        obstacle_points = 0

        if metrics_file.is_file():
            try:
                with open(metrics_file, "r") as f:
                    data = json.load(f)
                    clusters_found = data.get("clusters_found", 0)
                    ground_inliers = data.get("ground_inliers", 0)
                    obstacle_points = data.get("non_ground_outliers", 0)
                    stages = data.get("stages", [])

                    # Compute stages: 3 (Downsample), 4 (Search Index), 5 (ROR), 8 (RANSAC), 9 (Euclidean Clust)
                    # Excludes Stage 1 (Load PCD) and Stage 10 / Stage 2 (Disk Write)
                    compute_stage_indices = {3, 4, 5, 6, 7, 8, 9}
                    stage_compute_times = [s["time_ms"] for s in stages if s.get("stage") in compute_stage_indices]
                    if stage_compute_times:
                        compute_ms = sum(stage_compute_times)

                    write_stage_times = [s["time_ms"] for s in stages if s.get("stage") in {2, 10}]
                    if write_stage_times:
                        io_write_ms = sum(write_stage_times)
            except Exception:
                pass

        effective_process_ms = compute_ms if compute_ms is not None else max(0.0, wall_process_ms - io_write_ms)

        # ── 5. SAVE PROCESSED CLUSTERED PCD TO processed_scans/ ──────────────────
        produced_clust_pcd = self.out_temp_dir / "06_clusters.pcd"
        ground_src = self.out_temp_dir / "04_ransac_inliers.pcd"
        obs_src = self.out_temp_dir / "05_ground_plane_removed.pcd"
        processed_dest_path = os.path.join(self.args.processed_dir, f"processed_{frame_tag}")

        merged_saved = False
        # If pipeline_3d_ultra was used, 04_ransac_inliers.pcd exists: create a unified full-scene visualization
        if ground_src.is_file() and ground_src.stat().st_size > 200:
            try:
                g_fields, g_data = load_pcd_binary_data(ground_src)
                gx, gy, gz = g_data["x"], g_data["y"], g_data["z"]
                # Color ground plane in muted slate-gray (RGB: 75, 85, 95)
                g_rgb = np.full(len(gx), pack_rgb_float(75, 85, 95), dtype=np.float32)

                all_x, all_y, all_z, all_rgb = [gx], [gy], [gz], [g_rgb]

                if produced_clust_pcd.is_file() and produced_clust_pcd.stat().st_size > 200:
                    c_fields, c_data = load_pcd_binary_data(produced_clust_pcd)
                    if "rgb" in c_fields and len(c_data) > 0:
                        all_x.append(c_data["x"])
                        all_y.append(c_data["y"])
                        all_z.append(c_data["z"])
                        all_rgb.append(c_data["rgb"])

                fx = np.concatenate(all_x)
                fy = np.concatenate(all_y)
                fz = np.concatenate(all_z)
                frgb = np.concatenate(all_rgb)

                save_pcd_xyzrgb(processed_dest_path, fx, fy, fz, frgb)
                merged_saved = True

                # Also save the isolated clusters as clusters_only_*.pcd
                if produced_clust_pcd.is_file():
                    shutil.copyfile(produced_clust_pcd, os.path.join(self.args.processed_dir, f"clusters_only_{frame_tag}"))
            except Exception as e:
                pass

        if not merged_saved:
            if produced_clust_pcd.is_file() and produced_clust_pcd.stat().st_size > 200:
                shutil.copyfile(produced_clust_pcd, processed_dest_path)
            else:
                down_src = self.out_temp_dir / "01_downsampled.pcd"
                if down_src.is_file():
                    shutil.copyfile(down_src, processed_dest_path)

        # Optionally save raw intermediate stages
        if self.args.save_stages:
            if ground_src.is_file():
                shutil.copyfile(ground_src, os.path.join(self.args.processed_dir, f"ground_{frame_tag}"))
            if obs_src.is_file():
                shutil.copyfile(obs_src, os.path.join(self.args.processed_dir, f"obstacles_{frame_tag}"))

        print(
            f"[{time.strftime('%H:%M:%S')}] Frame #{frame_idx:04d} -> "
            f"Raw: {len(pts):,} pts ({self.args.raw_dir}/) | "
            f"Compute: {effective_process_ms:5.2f} ms | "
            f"Ground: {ground_inliers:,} pts | "
            f"Obstacles: {obstacle_points:,} pts | "
            f"Clusters: {clusters_found:2d} | "
            f"Saved: {processed_dest_path}"
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
        "--raw-dir",
        default="main_scans",
        help="Destination directory for raw captured PCD frames from iPhone",
    )
    ap.add_argument(
        "--processed-dir",
        default="processed_scans",
        help="Destination directory for processed clustered PCD frames",
    )

    # Pipeline Selection
    ap.add_argument(
        "--pipeline",
        choices=["auto", "ultra", "rvv_clust"],
        default="auto",
        help="Pipeline binary to run (ultra recommended for demonstrations)",
    )
    ap.add_argument(
        "--bin",
        default=None,
        help="Explicit path to pipeline executable (overrides --pipeline discovery)",
    )
    ap.add_argument(
        "--save-stages",
        action="store_true",
        help="Also export ground plane and obstacle-only PCDs into processed_scans/",
    )

    # Algorithm & Pipeline Tuning (Optimized defaults for indoor handheld LiDAR)
    ap.add_argument("--leaf-size", type=float, default=0.03, help="Voxel downsample leaf size in meters (default: 0.03 = 3cm)")
    ap.add_argument("--cluster-tolerance", type=float, default=0.12, help="Euclidean clustering radius in meters (default: 0.12 = 12cm)")
    ap.add_argument("--min-cluster", type=int, default=20, help="Minimum points per cluster (default: 20)")
    ap.add_argument("--max-cluster", type=int, default=100000, help="Maximum points per cluster")
    ap.add_argument("--ransac-iters", type=int, default=250, help="RANSAC plane fit iterations (default: 250)")
    ap.add_argument("--ror-radius", type=float, default=0.25, help="Radius outlier removal radius (m)")
    ap.add_argument("--ror-min-pts", type=int, default=2, help="Minimum neighbor count for ROR")
    ap.add_argument("--skip-sor", action="store_true", help="Bypass outlier removal filtering stage")
    ap.add_argument("--ground-angle-thresh", type=float, default=35.0, help="Max ground plane normal tilt in degrees (default: 35.0)")
    ap.add_argument(
        "--unconstrained-plane",
        action="store_true",
        help="Accept any planar orientation without horizontal ground constraint",
    )
    ap.add_argument(
        "--vehicle-frame",
        action="store_true",
        help="Force vehicle +Z ground normal prior (for automotive/KITTI datasets)",
    )

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

    # Ensure output directories exist
    os.makedirs(args.raw_dir, exist_ok=True)
    os.makedirs(args.processed_dir, exist_ok=True)

    # Locate executable
    pipeline_bin, pipeline_type = find_pipeline_binary(args.pipeline, args.bin)
    if not args.fake and not pipeline_bin.is_file():
        print(f"\n[WARNING] Pipeline binary not found at: {pipeline_bin}")
        print("  To build on Orange Pi RV2:")
        print("    cmake -B build/rvv -DRISCV_ARCH=rv64gcv")
        print(f"    cmake --build build/rvv -j8 --target {pipeline_bin.name}\n")

    print("=" * 76)
    print("  RVPoint Real-Time LiDAR Stream Server (Hardware RVV 1.0)")
    print("=" * 76)
    print(f"Pipeline Binary      : {pipeline_bin} [{pipeline_type}]")
    print(f"Periodic Capture     : Every {args.interval * 1000.0:.0f} ms ({1.0 / args.interval:.1f} Hz)")
    print(f"Raw Frames Output    : {args.raw_dir}/ (Original 3D iPhone scans)")
    print(f"Processed Output     : {args.processed_dir}/ (Segmented & Clustered scans)")
    print(f"Voxel Leaf Size      : {args.leaf_size} m | Tolerance: {args.cluster_tolerance} m")
    prior_str = "Unconstrained (Any Angle)" if args.unconstrained_plane else ("Vehicle +Z" if args.vehicle_frame else "iPhone Optical Frame (+Y Vertical)")
    print(f"Plane Prior          : {prior_str}")
    print("-" * 76)
    print("Connect your iPhone LiDAR Streamer app to one of these IP addresses:")
    for ip in get_local_ips():
        print(f"    --> {ip}:{args.port}")
    print("=" * 76)

    processor = FrameProcessor(args, pipeline_bin, pipeline_type)

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
