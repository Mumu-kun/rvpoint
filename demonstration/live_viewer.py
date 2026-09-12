#!/usr/bin/env python3
"""
live_viewer.py — Real-Time 3D Point Cloud Visualizer for RVPoint Perception Stream.

Connects directly to the Orange Pi RV2 server (port 9001 by default), receives
real-time segmented & clustered point clouds over TCP, and renders them in 3D.

Features:
  - Sub-millisecond binary RVPT network protocol parser.
  - Dynamic 3D visualization with interactive orbit, pan, and zoom.
  - Live HUD displaying frame index, point counts, cluster counts, and RVV 1.0 acceleration timings.
  - Interactive key bindings:
      [G] Toggle ground plane visibility
      [+] Increase point size
      [-] Decrease point size
      [R] Reset camera viewpoint
      [S] Save current frame snapshot as PCD
      [Q] / [ESC] Exit visualizer

Usage:
  # Connect to Orange Pi over Tailscale:
  python3 demonstration/live_viewer.py --host 100.94.165.126

  # Connect to Orange Pi over local Wi-Fi / Hotspot:
  python3 demonstration/live_viewer.py --host 172.20.10.2

  # Connect locally (for testing with server_main.py --fake):
  python3 demonstration/live_viewer.py --host 127.0.0.1
"""

import argparse
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

POINTS_STREAM_MAGIC = b"RVPT"
SYNC_MAGIC = b"RVP\x01"
HEADER_FORMAT = "!4sIIIIff"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 28 bytes


def unpack_rgb_float(frgb: np.ndarray) -> np.ndarray:
    """Convert packed float32 RGB into normalized [N, 3] float32 array in range [0, 1]."""
    u32 = frgb.view(np.uint32)
    r = ((u32 >> 16) & 0xFF).astype(np.float32) / 255.0
    g = ((u32 >> 8) & 0xFF).astype(np.float32) / 255.0
    b = (u32 & 0xFF).astype(np.float32) / 255.0
    return np.column_stack([r, g, b])


class StreamClient:
    """Handles network connection and asynchronous reception of point clouds."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True
        self.connected = False
        self.frames_received = 0
        self._thread = threading.Thread(target=self._recv_worker, daemon=True)
        self._thread.start()

    def get_latest_frame(self) -> Optional[dict]:
        with self.lock:
            frame = self.latest_frame
            self.latest_frame = None
            return frame

    def _recv_worker(self):
        while self.running:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(4.0)
            print(f"Connecting to RVPoint stream at {self.host}:{self.port}...")

            try:
                sock.connect((self.host, self.port))
                self.connected = True
                print(f"[Connected] Streaming live RVPoint perception data from {self.host}:{self.port}")
            except Exception as e:
                self.connected = False
                time.sleep(2.0)
                continue

            buf = bytearray()
            try:
                while self.running:
                    # Search for packet magic markers
                    idx_pt = buf.find(POINTS_STREAM_MAGIC)
                    idx_file = buf.find(SYNC_MAGIC)

                    # Drain bytes preceding the earliest magic header
                    earliest = -1
                    if idx_pt != -1 and idx_file != -1:
                        earliest = min(idx_pt, idx_file)
                    elif idx_pt != -1:
                        earliest = idx_pt
                    elif idx_file != -1:
                        earliest = idx_file

                    if earliest > 0:
                        del buf[:earliest]
                    elif earliest == -1:
                        if len(buf) > 8:
                            del buf[: len(buf) - 4]
                        chunk = sock.recv(65536)
                        if not chunk:
                            break
                        buf += chunk
                        continue

                    # 1. Process LIVE POINTS PACKET (RVPT)
                    if buf.startswith(POINTS_STREAM_MAGIC):
                        if len(buf) < HEADER_SIZE:
                            chunk = sock.recv(65536)
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        magic, frame_idx, pt_cnt, ground_cnt, cl_cnt, comp_ms, wall_ms = struct.unpack_from(
                            HEADER_FORMAT, buf, 0
                        )
                        payload_size = pt_cnt * 16  # 4 x float32 per point
                        total_size = HEADER_SIZE + payload_size

                        if len(buf) < total_size:
                            chunk = sock.recv(max(65536, total_size - len(buf)))
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        # Extract payload
                        payload = bytes(buf[HEADER_SIZE:total_size])
                        del buf[:total_size]

                        pts_raw = np.frombuffer(payload, dtype=np.float32).reshape(pt_cnt, 4)
                        xyz = pts_raw[:, :3].copy()
                        rgb = unpack_rgb_float(pts_raw[:, 3])

                        with self.lock:
                            self.latest_frame = {
                                "frame_idx": frame_idx,
                                "xyz": xyz,
                                "rgb": rgb,
                                "point_count": pt_cnt,
                                "ground_count": ground_cnt,
                                "clusters_count": cl_cnt,
                                "compute_ms": comp_ms,
                                "wall_ms": wall_ms,
                                "recv_time": time.time(),
                            }
                        self.frames_received += 1

                    # 2. Skip/Drain FILE SYNC PACKET (RVP\x01) if present
                    elif buf.startswith(SYNC_MAGIC):
                        if len(buf) < 7:
                            chunk = sock.recv(65536)
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        _, name_len = struct.unpack_from("!BH", buf, 4)
                        file_header_size = 7 + name_len + 4
                        if len(buf) < file_header_size:
                            chunk = sock.recv(65536)
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        file_data_len = struct.unpack_from("!I", buf, 7 + name_len)[0]
                        total_file_size = file_header_size + file_data_len

                        if len(buf) < total_file_size:
                            chunk = sock.recv(max(65536, total_file_size - len(buf)))
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        del buf[:total_file_size]

            except socket.timeout:
                continue
            except Exception as e:
                print(f"[Disconnected] Stream connection lost ({e}). Reconnecting...")
            finally:
                self.connected = False
                sock.close()
                time.sleep(1.0)


def run_open3d_viewer(client: StreamClient, args: argparse.Namespace):
    """Launch interactive 3D visualizer using Open3D."""
    try:
        import open3d as o3d
    except ImportError:
        print("\n[Notice] Open3D is not installed in the active environment.")
        print("To run the GUI visualizer:")
        print("  pip install open3d\n")
        print("Falling back to terminal HUD mode...\n")
        run_terminal_hud(client, args)
        return

    vis = o3d.visualization.VisualizerWithKeyCallback()
    window_name = f"RVPoint 3D Live Stream — Orange Pi RV2 (RVV 1.0) [{client.host}]"
    vis.create_window(window_name=window_name, width=1280, height=800)

    opt = vis.get_render_option()
    opt.background_color = np.asarray([0.08, 0.09, 0.11])  # Premium dark charcoal
    opt.point_size = args.point_size
    opt.show_coordinate_frame = True

    pcd = o3d.geometry.PointCloud()
    coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.4, origin=[0, 0, 0])
    vis.add_geometry(coord_frame)

    state = {
        "show_ground": not args.no_ground,
        "point_size": args.point_size,
        "first_frame": True,
        "current_frame": None,
        "running": True,
    }

    def toggle_ground(v):
        state["show_ground"] = not state["show_ground"]
        print(f"[Controls] Ground plane display: {'ON' if state['show_ground'] else 'OFF'}")
        return False

    def increase_size(v):
        state["point_size"] = min(20.0, state["point_size"] + 1.0)
        opt.point_size = state["point_size"]
        return False

    def decrease_size(v):
        state["point_size"] = max(1.0, state["point_size"] - 1.0)
        opt.point_size = state["point_size"]
        return False

    def save_snapshot(v):
        if state["current_frame"] is not None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            pcd_name = f"rvpoint_snapshot_{ts}.pcd"
            o3d.io.write_point_cloud(pcd_name, pcd)
            img_name = f"rvpoint_snapshot_{ts}.png"
            vis.capture_screen_image(img_name)
            print(f"[Snapshot] Saved {pcd_name} and {img_name}")
        return False

    def quit_vis(v):
        state["running"] = False
        vis.close()
        return False

    # Register keyboard shortcuts
    vis.register_key_callback(ord("G"), toggle_ground)
    vis.register_key_callback(ord("g"), toggle_ground)
    vis.register_key_callback(ord("="), increase_size)
    vis.register_key_callback(ord("+"), increase_size)
    vis.register_key_callback(ord("-"), decrease_size)
    vis.register_key_callback(ord("_"), decrease_size)
    vis.register_key_callback(ord("S"), save_snapshot)
    vis.register_key_callback(ord("s"), save_snapshot)
    vis.register_key_callback(ord("Q"), quit_vis)
    vis.register_key_callback(ord("q"), quit_vis)
    vis.register_key_callback(256, quit_vis)  # ESC

    print("\n" + "=" * 70)
    print("  RVPoint 3D Real-Time Perception Visualizer")
    print("=" * 70)
    print("  Controls:")
    print("    [Left Click + Drag]   Rotate camera view")
    print("    [Right Click + Drag]  Pan camera")
    print("    [Scroll Wheel]        Zoom in / out")
    print("    [G]                   Toggle ground plane points")
    print("    [+] / [-]             Increase / decrease point size")
    print("    [R]                   Reset camera view")
    print("    [S]                   Save point cloud snapshot (PCD + PNG)")
    print("    [Q] / [ESC]           Exit")
    print("=" * 70 + "\n")

    pcd_added = False
    last_print = 0.0

    try:
        while state["running"]:
            frame = client.get_latest_frame()
            if frame is not None:
                state["current_frame"] = frame
                xyz = frame["xyz"]
                rgb = frame["rgb"]
                ground_cnt = frame["ground_count"]

                if not state["show_ground"] and ground_cnt > 0:
                    # Ground points are stored at the beginning of the array
                    xyz = xyz[ground_cnt:]
                    rgb = rgb[ground_cnt:]

                pcd.points = o3d.utility.Vector3dVector(xyz)
                pcd.colors = o3d.utility.Vector3dVector(rgb)

                if not pcd_added:
                    vis.add_geometry(pcd)
                    vis.reset_view_point(True)
                    pcd_added = True
                else:
                    vis.update_geometry(pcd)

                now = time.time()
                if now - last_print > 0.4:
                    last_print = now
                    print(
                        f"[{time.strftime('%H:%M:%S')}] Frame #{frame['frame_idx']:04d} | "
                        f"Points: {frame['point_count']:,} | "
                        f"Clusters: {frame['clusters_count']} | "
                        f"Ground: {frame['ground_count']:,} | "
                        f"RVV Compute: {frame['compute_ms']:.2f} ms | "
                        f"Wall: {frame['wall_ms']:.1f} ms"
                    )

            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.01)

    except KeyboardInterrupt:
        pass
    finally:
        vis.destroy_window()


def run_terminal_hud(client: StreamClient, args: argparse.Namespace):
    """Fallback text HUD displaying live frame stream statistics when no GUI is available."""
    print("=" * 75)
    print("  RVPoint Perception Stream Receiver — Terminal HUD Mode")
    print(f"  Target: {client.host}:{client.port}")
    print("=" * 75)
    print(f"{'Time':<10} {'Frame':<8} {'Points':<10} {'Clusters':<10} {'Ground':<10} {'Compute (ms)':<14} {'Status'}")
    print("-" * 75)

    last_time = time.time()
    frames_in_sec = 0

    try:
        while True:
            frame = client.get_latest_frame()
            if frame is not None:
                frames_in_sec += 1
                now = time.time()
                fps_str = ""
                if now - last_time >= 1.0:
                    fps = frames_in_sec / (now - last_time)
                    fps_str = f"({fps:.1f} FPS)"
                    last_time = now
                    frames_in_sec = 0

                print(
                    f"[{time.strftime('%H:%M:%S')}] #{frame['frame_idx']:<6d} "
                    f"{frame['point_count']:<10,d} "
                    f"{frame['clusters_count']:<10d} "
                    f"{frame['ground_count']:<10,d} "
                    f"{frame['compute_ms']:<14.2f} "
                    f"OK {fps_str}"
                )
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopping HUD...")


def main():
    parser = argparse.ArgumentParser(
        description="RVPoint Real-Time 3D Point Cloud Visualizer (Orange Pi RV2 Hardware RVV 1.0)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--host",
        default="100.94.165.126",
        help="IP address of Orange Pi RV2 (Tailscale: 100.94.165.126, Local: 172.20.10.2, Loopback: 127.0.0.1)",
    )
    parser.add_argument("--port", type=int, default=9001, help="TCP port of RVPoint stream broadcaster")
    parser.add_argument("--point-size", type=float, default=3.5, help="Initial point cloud rendering size")
    parser.add_argument("--no-ground", action="store_true", help="Hide ground plane points upon initial startup")
    parser.add_argument("--hud-only", action="store_true", help="Force text HUD mode without GUI window")
    args = parser.parse_args()

    client = StreamClient(host=args.host, port=args.port)

    if args.hud_only:
        run_terminal_hud(client, args)
    else:
        run_open3d_viewer(client, args)


if __name__ == "__main__":
    main()
