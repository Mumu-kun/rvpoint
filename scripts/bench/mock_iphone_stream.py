#!/usr/bin/env python3
"""
mock_iphone_stream.py — Replay point clouds as binary UDP packets simulating iPhone LiDAR & VIO.

Transmits unified packets with 3 distinct sections:
  1. CoreMotion: Raw 6-axis IMU + Gravity (gyro[3], accel[3], gravity[3])
  2. ARKit VIO: 6-DoF transformation matrix (T_world_cam[16]) & tracking_state
  3. LiDAR dToF: 3D point cloud coordinates (x, y, z)

Target: UDP port 8765.

Usage:
  # Stream PCD sequence at 30 FPS with forward motion:
  python3 scripts/bench/mock_iphone_stream.py --pcd-dir data/pcd_compressed --fps 30

  # Replay a single PCD repeatedly with circular arc motion:
  python3 scripts/bench/mock_iphone_stream.py --pcd data/0000000000.pcd --motion circle --fps 20

  # High-rate loopback test:
  python3 scripts/bench/mock_iphone_stream.py --pcd data/pcd_compressed/0000000000.pcd --max-frames 100 --fps 60
"""

import argparse
import math
import os
import socket
import struct
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

MAGIC = 0x52565054  # "RVPT"
# Wire format: magic(4B) + seq(4B) + timestamp_ns(8B) + gyro[3](12B) + accel[3](12B) + gravity[3](12B) + T[16](64B) + tracking_state(4B) + num_pts(4B) + chunk_idx(2B) + total_chunks(2B) + reserved(4B) = 132 bytes
HEADER_FORMAT = "<IIQ9f16fIIHH4x"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
assert HEADER_SIZE == 132, f"HEADER_SIZE is {HEADER_SIZE}, expected 132"
MAX_UDP_PAYLOAD = 65000  # Safe UDP payload limit below 65507


def parse_pcd(filepath: Path) -> Tuple[List[float], List[float], List[float]]:
    """Parse ASCII, binary, or binary_compressed PCD into separate x, y, z lists."""
    with open(filepath, "rb") as f:
        header_lines = []
        data_type = None
        num_points = 0
        fields = []
        sizes = []

        while True:
            line = f.readline().decode("latin1", errors="ignore").strip()
            if not line or line.startswith("#"):
                continue
            header_lines.append(line)
            parts = line.split()
            tag = parts[0].upper()
            if tag == "FIELDS":
                fields = parts[1:]
            elif tag == "SIZE":
                sizes = [int(p) for p in parts[1:]]
            elif tag == "POINTS":
                num_points = int(parts[1])
            elif tag == "DATA":
                data_type = parts[1].lower()
                break

        if "x" not in fields or "y" not in fields or "z" not in fields:
            raise ValueError(f"PCD file {filepath} does not contain x, y, z fields.")

        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")

        if data_type == "ascii":
            xs, ys, zs = [], [], []
            for _ in range(num_points):
                line = f.readline().decode("latin1", errors="ignore").strip()
                if not line:
                    break
                toks = line.split()
                if len(toks) >= len(fields):
                    xs.append(float(toks[x_idx]))
                    ys.append(float(toks[y_idx]))
                    zs.append(float(toks[z_idx]))
            return xs, ys, zs

        elif data_type == "binary":
            point_step = sum(sizes)
            buf = f.read(point_step * num_points)
            xs, ys, zs = [], [], []
            x_offset = sum(sizes[:x_idx])
            y_offset = sum(sizes[:y_idx])
            z_offset = sum(sizes[:z_idx])
            for i in range(num_points):
                base = i * point_step
                xs.append(struct.unpack_from("<f", buf, base + x_offset)[0])
                ys.append(struct.unpack_from("<f", buf, base + y_offset)[0])
                zs.append(struct.unpack_from("<f", buf, base + z_offset)[0])
            return xs, ys, zs

        elif data_type == "binary_compressed":
            c_size, u_size = struct.unpack("<II", f.read(8))
            compressed_data = f.read(c_size)

            uncompressed = bytearray(u_size)
            ip, op = 0, 0
            c_len = len(compressed_data)
            while ip < c_len:
                ctrl = compressed_data[ip]
                ip += 1
                if ctrl < 32:
                    ctrl += 1
                    uncompressed[op : op + ctrl] = compressed_data[ip : ip + ctrl]
                    op += ctrl
                    ip += ctrl
                else:
                    length = ctrl >> 5
                    ref = op - ((ctrl & 0x1F) << 8) - 1
                    if length == 7:
                        length += compressed_data[ip]
                        ip += 1
                    ref -= compressed_data[ip]
                    ip += 1
                    length += 2
                    for k in range(length):
                        uncompressed[op + k] = uncompressed[ref + k]
                    op += length

            field_total = num_points * 4
            x_bytes = uncompressed[x_idx * field_total : (x_idx + 1) * field_total]
            y_bytes = uncompressed[y_idx * field_total : (y_idx + 1) * field_total]
            z_bytes = uncompressed[z_idx * field_total : (z_idx + 1) * field_total]

            xs = list(struct.unpack(f"<{num_points}f", x_bytes))
            ys = list(struct.unpack(f"<{num_points}f", y_bytes))
            zs = list(struct.unpack(f"<{num_points}f", z_bytes))
            return xs, ys, zs

        else:
            raise ValueError(f"Unsupported PCD DATA type: {data_type}")


def synthesize_motion(motion: str, t_sec: float, speed: float, yaw_rate: float):
    """Synthesize CoreMotion (gyro, accel, gravity) and ARKit VIO (T_world_cam, tracking_state)."""
    T = [0.0] * 16
    T[0] = 1.0
    T[5] = 1.0
    T[10] = 1.0
    T[15] = 1.0
    tracking_state = 2  # Normal

    gyro = [0.0, 0.0, 0.0]
    accel = [0.0, 0.0, 0.0]
    gravity = [0.0, 0.0, -9.81]

    if motion == "stationary":
        return gyro, accel, gravity, T, tracking_state

    sim_yaw = 0.0
    sim_x = 0.0
    sim_y = 0.0

    if motion == "forward":
        sim_x = speed * t_sec
    elif motion == "circle":
        sim_yaw = yaw_rate * t_sec
        gyro[2] = yaw_rate
        if abs(yaw_rate) > 1e-5:
            R = speed / yaw_rate
            sim_x = R * math.sin(sim_yaw)
            sim_y = R * (1.0 - math.cos(sim_yaw))
            accel[1] = speed * yaw_rate
        else:
            sim_x = speed * t_sec

    c = math.cos(sim_yaw)
    s = math.sin(sim_yaw)

    T[0] = c
    T[4] = -s
    T[8] = 0.0
    T[12] = sim_x
    T[1] = s
    T[5] = c
    T[9] = 0.0
    T[13] = sim_y
    T[2] = 0.0
    T[6] = 0.0
    T[10] = 1.0
    T[14] = 0.0
    T[3] = 0.0
    T[7] = 0.0
    T[11] = 0.0
    T[15] = 1.0

    return gyro, accel, gravity, T, tracking_state


def send_frame(
    sock: socket.socket,
    target: Tuple[str, int],
    seq: int,
    timestamp_ns: int,
    gyro: List[float],
    accel: List[float],
    gravity: List[float],
    T_world_cam: List[float],
    tracking_state: int,
    xs: List[float],
    ys: List[float],
    zs: List[float],
):
    """Send point cloud frame partitioned into UDP chunks if necessary."""
    total_points = len(xs)
    max_pts_per_chunk = (MAX_UDP_PAYLOAD - HEADER_SIZE) // (3 * 4)
    total_chunks = (
        max(1, math.ceil(total_points / max_pts_per_chunk)) if total_points > 0 else 1
    )

    for chunk_idx in range(total_chunks):
        start = chunk_idx * max_pts_per_chunk
        end = min(start + max_pts_per_chunk, total_points)
        chunk_pts = end - start

        # Header: magic, seq, timestamp_ns, gyro(3), accel(3), gravity(3), T(16), tracking_state, chunk_pts, chunk_idx, total_chunks
        header = struct.pack(
            HEADER_FORMAT,
            MAGIC,
            seq,
            timestamp_ns,
            gyro[0],
            gyro[1],
            gyro[2],
            accel[0],
            accel[1],
            accel[2],
            gravity[0],
            gravity[1],
            gravity[2],
            *T_world_cam,
            tracking_state,
            chunk_pts,
            chunk_idx,
            total_chunks,
        )

        if chunk_pts > 0:
            payload = (
                struct.pack(f"<{chunk_pts}f", *xs[start:end])
                + struct.pack(f"<{chunk_pts}f", *ys[start:end])
                + struct.pack(f"<{chunk_pts}f", *zs[start:end])
            )
            packet = header + payload
        else:
            packet = header

        sock.sendto(packet, target)


def main():
    parser = argparse.ArgumentParser(
        description="Mock iPhone LiDAR, VIO & CoreMotion UDP Streamer"
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Target host (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=8765, help="Target UDP port (default: 8765)"
    )
    parser.add_argument(
        "--pcd-dir", default=None, help="Directory containing PCD files"
    )
    parser.add_argument("--pcd", default=None, help="Single PCD file to replay")
    parser.add_argument(
        "--fps", type=float, default=30.0, help="Stream framerate (default: 30)"
    )
    parser.add_argument(
        "--motion", choices=["forward", "circle", "stationary"], default="forward"
    )
    parser.add_argument(
        "--speed", type=float, default=0.4, help="Linear speed in m/s (default: 0.4)"
    )
    parser.add_argument(
        "--yaw-rate", type=float, default=0.2, help="Yaw rate in rad/s (default: 0.2)"
    )
    parser.add_argument(
        "--loop", action="store_true", default=True, help="Loop replay indefinitely"
    )
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument(
        "--max-frames", type=int, default=0, help="Stop after N frames (0 = unlimited)"
    )

    args = parser.parse_args()

    files: List[Path] = []
    if args.pcd:
        files.append(Path(args.pcd))
    elif args.pcd_dir:
        p = Path(args.pcd_dir)
        files = sorted(list(p.glob("*.pcd")))
    else:
        repo_root = Path(__file__).resolve().parent.parent.parent
        default_dir = repo_root / "data" / "pcd_compressed"
        if default_dir.exists():
            files = sorted(list(default_dir.glob("*.pcd")))
        else:
            default_dir = repo_root / "data"
            files = sorted(list(default_dir.glob("*.pcd")))

    if not files:
        print("Error: No PCD files found to stream.", file=sys.stderr)
        sys.exit(1)

    print(
        f"[*] Found {len(files)} PCD file(s). Target: {args.host}:{args.port} @ {args.fps} FPS (Motion: {args.motion})"
    )

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    frame_interval = 1.0 / args.fps if args.fps > 0 else 0.033
    seq = 0
    t0 = time.time()

    try:
        while True:
            for filepath in files:
                t_start = time.time()
                sim_time = t_start - t0

                try:
                    xs, ys, zs = parse_pcd(filepath)
                except Exception as e:
                    print(f"Warning: Failed to parse {filepath.name}: {e}")
                    continue

                gyro, accel, gravity, T_world_cam, tracking_state = synthesize_motion(
                    args.motion, sim_time, args.speed, args.yaw_rate
                )
                timestamp_ns = int(sim_time * 1e9)

                send_frame(
                    sock,
                    (args.host, args.port),
                    seq,
                    timestamp_ns,
                    gyro,
                    accel,
                    gravity,
                    T_world_cam,
                    tracking_state,
                    xs,
                    ys,
                    zs,
                )

                if seq % 30 == 0:
                    print(
                        f"[Frame {seq:05d}] Sent {len(xs):5d} pts | "
                        f"Pose: x={T_world_cam[12]:.2f}m, y={T_world_cam[13]:.2f}m | "
                        f"GyroZ: {gyro[2]:.2f} rad/s | GravZ: {gravity[2]:.2f} m/s²"
                    )

                seq += 1
                if args.max_frames > 0 and seq >= args.max_frames:
                    print(f"Reached max frames ({args.max_frames}). Done.")
                    return

                elapsed = time.time() - t_start
                sleep_time = frame_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

            if not args.loop:
                break

    except KeyboardInterrupt:
        print("\nStream stopped by user.")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
