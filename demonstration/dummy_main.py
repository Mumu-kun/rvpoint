#!/usr/bin/env python3
"""
dummy_main.py — Lightweight Test Server for iPhone LiDAR Streaming (LDP2 & LDP1 Protocols).

Verifies that the iOS LiDARStreamer app is transmitting packets correctly,
unpacks the 136-byte LDP2 header, displays live telemetry (CoreMotion, ARKit VIO,
Depth, Confidence), and validates 3D unprojected point coordinates.

Usage:
    python3 demonstration/dummy_main.py
    python3 demonstration/dummy_main.py --port 9000
    python3 demonstration/dummy_main.py --save-sample-pcd
"""

import argparse
import array
import math
import os
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# ANSI styling for crisp terminal diagnostics
C_CYAN = "\033[96m"
C_GREEN = "\033[92m"
C_YELLOW = "\033[93m"
C_RED = "\033[91m"
C_BOLD = "\033[1m"
C_DIM = "\033[2m"
C_RESET = "\033[0m"

# Protocols
MAGIC_LDP1 = b"LDP1"
MAGIC_LDP2 = b"LDP2"

# Header structures (excluding the 4-byte magic)
# LDP1: frame_idx(I), w(I), h(I), fx(f), fy(f), cx(f), cy(f), pose(16f) = 92 bytes
HEADER_LDP1 = struct.Struct("<III4f16f")
assert HEADER_LDP1.size == 92, f"Expected 92 bytes, got {HEADER_LDP1.size}"

# LDP2: LDP1(92B) + tracking_state(I, 4B) + gyro(3f, 12B) + accel(3f, 12B) + gravity(3f, 12B) = 132 bytes
HEADER_LDP2 = struct.Struct("<III4f16fI9f")
assert HEADER_LDP2.size == 132, f"Expected 132 bytes, got {HEADER_LDP2.size}"


def get_local_ips() -> List[str]:
    """Discover all active non-loopback IPv4 addresses."""
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


def tracking_state_to_str(state: int) -> str:
    if state == 0:
        return f"{C_RED}NOT_AVAILABLE (0){C_RESET}"
    elif state == 1:
        return f"{C_YELLOW}LIMITED (1){C_RESET}"
    elif state == 2:
        return f"{C_GREEN}NORMAL (2){C_RESET}"
    return f"{C_DIM}UNKNOWN ({state}){C_RESET}"


def parse_next_frame(conn: socket.socket, buf: bytearray) -> Optional[Dict]:
    """
    Scans buffer for LDP2 or LDP1 magic, unmarshals the header and arrays,
    and returns a structured frame dictionary.
    """
    while True:
        # Search for either magic in buffer
        idx1 = buf.find(MAGIC_LDP1)
        idx2 = buf.find(MAGIC_LDP2)

        # Pick earliest occurrence
        pos = -1
        protocol = None
        if idx1 != -1 and idx2 != -1:
            if idx1 < idx2:
                pos, protocol = idx1, "LDP1"
            else:
                pos, protocol = idx2, "LDP2"
        elif idx1 != -1:
            pos, protocol = idx1, "LDP1"
        elif idx2 != -1:
            pos, protocol = idx2, "LDP2"

        if pos == -1:
            # Drop obsolete bytes while keeping potential partial magic at tail
            del buf[: max(0, len(buf) - 3)]
        elif pos > 0:
            # Discard junk before magic
            del buf[:pos]

        if protocol is not None:
            header_struct = HEADER_LDP2 if protocol == "LDP2" else HEADER_LDP1
            header_size = 4 + header_struct.size  # includes magic

            if len(buf) >= header_size:
                unpacked = header_struct.unpack_from(buf, 4)
                frame_idx = unpacked[0]
                w = unpacked[1]
                h = unpacked[2]
                fx, fy, cx, cy = unpacked[3:7]
                pose = unpacked[7:23]

                telemetry = {}
                if protocol == "LDP2":
                    track_state = unpacked[23]
                    gyro = unpacked[24:27]
                    accel = unpacked[27:30]
                    gravity = unpacked[30:33]
                    telemetry = {
                        "track_state": track_state,
                        "gyro": gyro,
                        "accel": accel,
                        "gravity": gravity,
                    }

                payload_len = w * h * 5  # 4 bytes float32 depth + 1 byte uint8 confidence
                total_packet_len = header_size + payload_len

                if len(buf) >= total_packet_len:
                    # Extract depth and confidence
                    depth_offset = header_size
                    conf_offset = header_size + (w * h * 4)

                    if HAS_NUMPY:
                        depth = np.frombuffer(buf, "<f4", w * h, depth_offset).reshape(h, w).copy()
                        conf = np.frombuffer(buf, np.uint8, w * h, conf_offset).reshape(h, w).copy()
                        pose_mat = np.asarray(pose, np.float64).reshape(4, 4)
                    else:
                        depth_arr = array.array("f")
                        depth_arr.frombytes(buf[depth_offset : depth_offset + w * h * 4])
                        conf_arr = array.array("B")
                        conf_arr.frombytes(buf[conf_offset : conf_offset + w * h])
                        depth = depth_arr
                        conf = conf_arr
                        pose_mat = pose

                    del buf[:total_packet_len]

                    return {
                        "protocol": protocol,
                        "seq": frame_idx,
                        "w": w,
                        "h": h,
                        "fx": fx,
                        "fy": fy,
                        "cx": cx,
                        "cy": cy,
                        "pose": pose_mat,
                        "depth": depth,
                        "conf": conf,
                        "telemetry": telemetry,
                        "packet_bytes": total_packet_len,
                    }

        # Need more network data
        try:
            chunk = conn.recv(262144)
        except socket.timeout:
            return None
        if not chunk:
            raise ConnectionError("iPhone disconnected.")
        buf += chunk


def unproject_sample_points(frame: Dict, min_conf: int = 1) -> List[Tuple[float, float, float]]:
    """Unprojects (u, v) pixels + depth into 3D (x, y, z) camera coordinates."""
    w, h = frame["w"], frame["h"]
    fx, fy, cx, cy = frame["fx"], frame["fy"], frame["cx"], frame["cy"]

    pts = []
    if HAS_NUMPY:
        depth = frame["depth"]
        conf = frame["conf"]
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        mask = (conf >= min_conf) & (depth > 0.05) & (depth < 6.0) & np.isfinite(depth)
        z = depth[mask]
        x = (u[mask] - cx) * z / fx
        y = (v[mask] - cy) * z / fy
        stacked = np.column_stack((x, y, z))
        return stacked
    else:
        depth = frame["depth"]
        conf = frame["conf"]
        for row in range(0, h, 2):  # step by 2 for lightweight pure-python
            for col in range(0, w, 2):
                idx = row * w + col
                d = depth[idx]
                c = conf[idx]
                if c >= min_conf and 0.05 < d < 6.0 and math.isfinite(d):
                    z = d
                    x = (col - cx) * z / fx
                    y = (row - cy) * z / fy
                    pts.append((x, y, z))
        return pts


def align_optical_to_body(pts, gravity, mount_height=0.0):
    """
    Implements rvpoint::CameraAlignment closed-form orthonormal rotation
    (src/filters/camera_alignment/camera_alignment.cpp) using the live
    CoreMotion gravity vector [gx, gy, gz] from LDP2.

    Transforms camera optical frame (+X right, +Y down, +Z forward) into the
    ISO 8855 vehicle/body frame (+X forward, +Y left, +Z up, ground at Z=0).
    Body +Z is strictly anti-parallel to gravity.
    """
    gx, gy, gz = gravity
    g_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
    if g_norm < 1.0:
        return pts

    # CoreMotion gravity points towards Earth; True UP is uz = -g / ||g||
    uz_x = -gx / g_norm
    uz_y = -gy / g_norm
    uz_z = -gz / g_norm

    # Camera optical forward [0, 0, 1] projected onto horizontal plane perpendicular to uz
    dot = uz_z  # v_opt . uz
    fx = -dot * uz_x
    fy = -dot * uz_y
    fz = 1.0 - dot * uz_z

    f_len = math.sqrt(fx * fx + fy * fy + fz * fz)
    if f_len > 1e-4:
        ux_x = fx / f_len
        ux_y = fy / f_len
        ux_z = fz / f_len
    else:
        ux_x, ux_y, ux_z = 0.0, 0.0, 1.0

    # Body Left vector: uy = uz x ux
    uy_x = uz_y * ux_z - uz_z * ux_y
    uy_y = uz_z * ux_x - uz_x * ux_z
    uy_z = uz_x * ux_y - uz_y * ux_x

    if HAS_NUMPY and isinstance(pts, np.ndarray):
        R = np.array([
            [ux_x, ux_y, ux_z],
            [uy_x, uy_y, uy_z],
            [uz_x, uz_y, uz_z],
        ], dtype=np.float32)
        return pts @ R.T + np.array([0.0, 0.0, mount_height], dtype=np.float32)
    else:
        aligned = []
        for px, py, pz in pts:
            bx = ux_x * px + ux_y * py + ux_z * pz
            by = uy_x * px + uy_y * py + uy_z * pz
            bz = uz_x * px + uz_y * py + uz_z * pz + mount_height
            aligned.append((bx, by, bz))
        return aligned


def align_with_pose(pts, pose):
    """
    Transforms points into ARKit World coordinate frame using 4x4 camera pose.
    Matches demonstration/server_main.py unproject() logic.
    """
    if HAS_NUMPY and isinstance(pts, np.ndarray):
        pose_mat = np.asarray(pose, np.float32).reshape(4, 4)
        return pts @ pose_mat[:3, :3].T + pose_mat[:3, 3]
    else:
        aligned = []
        r00, r01, r02, tx = pose[0], pose[1], pose[2], pose[3]
        r10, r11, r12, ty = pose[4], pose[5], pose[6], pose[7]
        r20, r21, r22, tz = pose[8], pose[9], pose[10], pose[11]
        for px, py, pz in pts:
            wx = px * r00 + py * r01 + pz * r02 + tx
            wy = px * r10 + py * r11 + pz * r12 + ty
            wz = px * r20 + py * r21 + pz * r22 + tz
            aligned.append((wx, wy, wz))
        return aligned


def save_pcd_ascii(filename: Path, pts):
    """Write an ASCII PCD file for visual inspection."""
    if HAS_NUMPY and isinstance(pts, np.ndarray):
        n = pts.shape[0]
    else:
        n = len(pts)

    header = f"""# .PCD v0.7 - Point Cloud Data
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {n}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {n}
DATA ascii
"""
    with open(filename, "w") as f:
        f.write(header)
        for i in range(n):
            if HAS_NUMPY and isinstance(pts, np.ndarray):
                f.write(f"{pts[i, 0]:.4f} {pts[i, 1]:.4f} {pts[i, 2]:.4f}\n")
            else:
                p = pts[i]
                f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")


def main():
    parser = argparse.ArgumentParser(description="Test Server for iPhone LiDAR LDP2 Stream.")
    parser.add_argument("--port", type=int, default=9000, help="TCP port to listen on (default: 9000)")
    parser.add_argument("--bind", type=str, default="0.0.0.0", help="Bind IP address (default: 0.0.0.0)")
    parser.add_argument("--save-sample-pcd", action="store_true", help="Save the 5th received frame to output/test_iphone_ldp2.pcd")
    args = parser.parse_args()

    local_ips = get_local_ips()

    print(f"\n{C_BOLD}{C_CYAN}======================================================================{C_RESET}")
    print(f"{C_BOLD}{C_CYAN}           RVPoint iPhone LiDAR Test Server (dummy_main.py)           {C_RESET}")
    print(f"{C_BOLD}{C_CYAN}======================================================================{C_RESET}")
    print(f"Listening on: {C_GREEN}{args.bind}:{args.port}{C_RESET}")
    print(f"\n{C_BOLD}👉 ENTER ONE OF THESE IPs INTO YOUR IPHONE APP:{C_RESET}")
    for ip in local_ips:
        print(f"   ► {C_BOLD}{C_YELLOW}{ip}{C_RESET}")
    print(f"\nWaiting for iPhone LiDARStreamer connection on port {args.port}...\n")

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((args.bind, args.port))
    srv.listen(1)

    try:
        conn, addr = srv.accept()
        print(f"{C_BOLD}{C_GREEN}✔ iPhone Connected from {addr[0]}:{addr[1]}!{C_RESET}")
        print(f"{C_DIM}Streaming active... Press Ctrl+C to stop.{C_RESET}\n")

        conn.settimeout(3.0)
        buf = bytearray()

        frame_count = 0
        start_time = time.time()
        fps_timer = time.time()
        fps_count = 0
        current_fps = 0.0

        while True:
            frame = parse_next_frame(conn, buf)
            if frame is None:
                continue

            frame_count += 1
            fps_count += 1

            now = time.time()
            if now - fps_timer >= 1.0:
                current_fps = fps_count / (now - fps_timer)
                fps_count = 0
                fps_timer = now

            # Compute depth statistics
            depth = frame["depth"]
            conf = frame["conf"]
            if HAS_NUMPY:
                valid_mask = (depth > 0.05) & (depth < 10.0) & np.isfinite(depth)
                valid_pts_cnt = np.count_nonzero(valid_mask)
                min_d = float(np.min(depth[valid_mask])) if valid_pts_cnt > 0 else 0.0
                max_d = float(np.max(depth[valid_mask])) if valid_pts_cnt > 0 else 0.0
            else:
                valid_depths = [d for d in depth if 0.05 < d < 10.0 and math.isfinite(d)]
                valid_pts_cnt = len(valid_depths)
                min_d = min(valid_depths) if valid_pts_cnt > 0 else 0.0
                max_d = max(valid_depths) if valid_pts_cnt > 0 else 0.0

            # Unproject a few 3D points
            pts_3d = unproject_sample_points(frame)
            pts_cnt = pts_3d.shape[0] if (HAS_NUMPY and isinstance(pts_3d, np.ndarray)) else len(pts_3d)

            # Save sample PCD if requested
            if args.save_sample_pcd and frame_count == 5 and pts_cnt > 0:
                out_dir = Path("output")
                out_dir.mkdir(parents=True, exist_ok=True)

                # 1. Raw optical frame (X right, Y down, Z forward)
                raw_path = out_dir / "test_iphone_ldp2_raw.pcd"
                save_pcd_ascii(raw_path, pts_3d)

                # 2. Gravity-aligned ISO 8855 body frame (CameraAlignment algorithm)
                # +X forward, +Y left, +Z up (strictly anti-parallel to gravity)
                pts_oriented = pts_3d
                if proto == "LDP2" and "telemetry" in frame:
                    pts_oriented = align_optical_to_body(pts_3d, frame["telemetry"]["gravity"])
                elif "pose" in frame:
                    pts_oriented = align_with_pose(pts_3d, frame["pose"])

                oriented_path = out_dir / "test_iphone_ldp2_oriented.pcd"
                save_pcd_ascii(oriented_path, pts_oriented)

                # Main output path gets the oriented PCD
                main_path = out_dir / "test_iphone_ldp2.pcd"
                save_pcd_ascii(main_path, pts_oriented)

                # Compute elevation and bounds
                if HAS_NUMPY and isinstance(pts_oriented, np.ndarray):
                    mins = pts_oriented.min(axis=0)
                    maxs = pts_oriented.max(axis=0)
                else:
                    xs = [p[0] for p in pts_oriented]
                    ys = [p[1] for p in pts_oriented]
                    zs = [p[2] for p in pts_oriented]
                    mins = [min(xs), min(ys), min(zs)]
                    maxs = [max(xs), max(ys), max(zs)]

                print(f"\n{C_BOLD}{C_GREEN}💾 [CameraAlignment] Saved properly oriented PCD frame #5 ({pts_cnt} points):{C_RESET}")
                print(f"   ► Leveled / Body Frame: {main_path} (ISO 8855: +X fwd, +Y left, +Z up)")
                print(f"   ► Raw Optical Frame   : {raw_path}")
                print(f"   • Extents: X=[{mins[0]:+.2f}m, {maxs[0]:+.2f}m], Y=[{mins[1]:+.2f}m, {maxs[1]:+.2f}m], Z(Up)=[{mins[2]:+.2f}m, {maxs[2]:+.2f}m]\n")

            # Print frame report
            proto = frame["protocol"]
            proto_badge = f"{C_BOLD}{C_GREEN}[{proto}]{C_RESET}" if proto == "LDP2" else f"{C_YELLOW}[{proto}]{C_RESET}"
            
            report = (
                f"{proto_badge} Frame #{frame['seq']:<5d} | "
                f"Rate: {C_BOLD}{current_fps:4.1f} FPS{C_RESET} | "
                f"Res: {frame['w']}x{frame['h']} | "
                f"Valid Pts: {pts_cnt:<5d} | "
                f"Depth: {min_d:.2f}m - {max_d:.2f}m"
            )

            if proto == "LDP2":
                telem = frame["telemetry"]
                t_state = tracking_state_to_str(telem["track_state"])
                gyro = telem["gyro"]
                accel = telem["accel"]
                grav = telem["gravity"]
                
                # Detailed print every 10 frames
                if frame_count % 10 == 1:
                    print(f"\n{C_BOLD}--- LDP2 FRAME #{frame['seq']} TELEMETRY ---{C_RESET}")
                    print(f"  VIO Tracking: {t_state}")
                    print(f"  Gyro (rad/s): [wx={gyro[0]:+6.2f}, wy={gyro[1]:+6.2f}, wz={gyro[2]:+6.2f}]")
                    print(f"  Accel (m/s²): [ax={accel[0]:+6.2f}, ay={accel[1]:+6.2f}, az={accel[2]:+6.2f}]")
                    print(f"  Gravity     : [gx={grav[0]:+6.2f}, gy={grav[1]:+6.2f}, gz={grav[2]:+6.2f}]")
                    if pts_cnt > 0:
                        mid_pt = pts_3d[pts_cnt // 2]
                        print(f"  Sample 3D Pt: (X={mid_pt[0]:+5.2f}m, Y={mid_pt[1]:+5.2f}m, Z={mid_pt[2]:+5.2f}m)")
                    print(f"  Intrinsics  : fx={frame['fx']:.1f}, fy={frame['fy']:.1f}, cx={frame['cx']:.1f}, cy={frame['cy']:.1f}")
                    print(f"----------------------------------------")
                else:
                    print(report)
            else:
                print(report)

    except KeyboardInterrupt:
        print(f"\n{C_YELLOW}Test server stopped by user.{C_RESET}")
    except ConnectionError as e:
        print(f"\n{C_RED}Client connection closed: {e}{C_RESET}")
    finally:
        srv.close()
        print(f"\n{C_BOLD}Total frames received: {frame_count}{C_RESET}\n")


if __name__ == "__main__":
    main()
