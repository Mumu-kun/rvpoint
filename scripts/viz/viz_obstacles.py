#!/usr/bin/env python3
"""
viz_obstacles.py — Real-Time LiDAR Bounding Box Extractor & Foxglove MCAP Visualizer
====================================================================================

Takes a PCD point cloud file, runs the RVPoint perception pipeline (CameraAlignment,
PassThroughFilter, ForwardCellClustering, and ObstacleGeometryExtractor for 3D OBBs
and Bounding Discs), exports a Foxglove-compliant MCAP timeline, and serves it for
instant 3D visualization.

Usage:
------
1. Process PCD, export Foxglove MCAP & render figure:
   uv run python scripts/viz/viz_obstacles.py data/pcd_compressed/0000000000.pcd

2. Process PCD and immediately serve for Foxglove Studio Web:
   uv run python scripts/viz/viz_obstacles.py data/pcd_compressed/0000000000.pcd --serve

3. Custom MCAP output destination:
   uv run python scripts/viz/viz_obstacles.py data/pcd_compressed/0000000000.pcd --mcap output/my_obstacles.mcap

4. Open interactive 2D BEV Matplotlib or 3D Open3D GUI:
   uv run python scripts/viz/viz_obstacles.py data/pcd_compressed/0000000000.pcd --show
   uv run python scripts/viz/viz_obstacles.py data/pcd_compressed/0000000000.pcd --open3d
"""

import argparse
import http.server
import io
import json
import math
import os
import socketserver
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

# Force UTF-8 on Windows
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import numpy as np

# Foxglove Protobuf & MCAP
try:
    from mcap_protobuf.writer import Writer as McapWriter
    from foxglove_schemas_protobuf import (
        ArrowPrimitive_pb2,
        Color_pb2,
        CubePrimitive_pb2,
        CylinderPrimitive_pb2,
        LinePrimitive_pb2,
        PackedElementField_pb2,
        Point3_pb2,
        PointCloud_pb2,
        Pose_pb2,
        SceneEntity_pb2,
        SceneUpdate_pb2,
        TextPrimitive_pb2,
        Vector3_pb2,
    )
    from google.protobuf.timestamp_pb2 import Timestamp
    HAS_FOXGLOVE = True
except ImportError:
    HAS_FOXGLOVE = False


# ──────────────────────────────────────────────────────────────────────────────
# 1. Pipeline Execution & PCD Loading
# ──────────────────────────────────────────────────────────────────────────────

def to_wsl_path(path: Path) -> str:
    """Convert Windows path to WSL /mnt/ path if running on Windows."""
    resolved = path.resolve()
    s = str(resolved).replace("\\", "/")
    if len(s) > 1 and s[1] == ":":
        drive = s[0].lower()
        return f"/mnt/{drive}{s[2:]}"
    return s


def run_cpp_bounding_boxing(pcd_path: Path, output_json: Path, output_pcd: Path, is_optical: bool) -> bool:
    """Run the C++ pipeline_obstacles binary via WSL or native linux."""
    print(f"\n[1/4] Executing RVPoint C++ Perception Pipeline on '{pcd_path.name}'...")
    wsl_pcd = to_wsl_path(pcd_path)
    wsl_json = to_wsl_path(output_json)
    wsl_out_pcd = to_wsl_path(output_pcd)
    opt_flag = "--optical" if is_optical else "--body"

    output_json.parent.mkdir(parents=True, exist_ok=True)

    # Check if inside native Linux or on Windows host
    if sys.platform.startswith("linux"):
        cmd = f"./scripts/run.sh pipeline_obstacles \"{wsl_pcd}\" \"{wsl_json}\" \"{wsl_out_pcd}\" {opt_flag}"
    else:
        cmd = f'wsl -d rvpoint bash -c "source env/activate.sh && ./scripts/run.sh pipeline_obstacles \\"{wsl_pcd}\\" \\"{wsl_json}\\" \\"{wsl_out_pcd}\\" {opt_flag}"'

    ret = os.system(cmd)
    if ret == 0 and output_json.exists():
        print(f"[OK] C++ Bounding Box Extractor completed successfully.")
        return True
    print("[WARN] C++ pipeline execution failed or not configured; falling back to Python geometry engine.")
    return False


def load_pcd_points(pcd_path: Path) -> np.ndarray:
    """Zero-dependency PCD loader supporting Binary Compressed, Binary, and ASCII."""
    try:
        with open(pcd_path, "rb") as f:
            header_lines = []
            data_mode = "ascii"
            num_points = 0
            fields = []

            while True:
                line = f.readline()
                if not line:
                    break
                line_str = line.decode("ascii", errors="ignore").strip()
                header_lines.append(line_str)
                if line_str.startswith("DATA"):
                    data_mode = line_str.split()[1].lower()
                    break

            for l in header_lines:
                if l.startswith("FIELDS"):
                    fields = l.split()[1:]
                elif l.startswith("POINTS"):
                    num_points = int(l.split()[1])

            if num_points == 0:
                return np.zeros((0, 3), dtype=np.float32)

            if data_mode == "ascii":
                lines = f.readlines()
                pts = []
                for line in lines:
                    parts = line.decode("ascii", errors="ignore").strip().split()
                    if len(parts) >= 3:
                        pts.append([float(parts[0]), float(parts[1]), float(parts[2])])
                return np.array(pts, dtype=np.float32)
            elif data_mode == "binary":
                # Standard float32 binary points
                stride = len(fields) * 4
                raw = f.read(num_points * stride)
                arr = np.frombuffer(raw, dtype=np.float32).reshape(-1, len(fields))
                return arr[:, :3].copy()
    except Exception as e:
        print(f"[WARN] Error reading PCD with native loader: {e}")

    # Fallback to open3d if available
    try:
        import open3d as o3d
        pcd = o3d.io.read_point_cloud(str(pcd_path))
        return np.asarray(pcd.points, dtype=np.float32)
    except Exception:
        return np.zeros((0, 3), dtype=np.float32)


# ──────────────────────────────────────────────────────────────────────────────
# 2. Python Fallback Bounding Box Engine (If C++ not invoked)
# ──────────────────────────────────────────────────────────────────────────────

def compute_convex_hull_2d(pts: list[dict]) -> list[tuple[float, float]]:
    """Andrew's Monotone Chain 2D convex hull."""
    if len(pts) < 3:
        return [(p["x"], p["y"]) for p in pts]
    coords = sorted([(p["x"], p["y"]) for p in pts], key=lambda c: (c[0], c[1]))

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower = []
    for p in coords:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 1e-6:
            lower.pop()
        lower.append(p)

    upper = []
    for p in reversed(coords):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 1e-6:
            upper.pop()
        upper.append(p)

    return lower[:-1] + upper[:-1]


def infer_label(obb: dict, disc: dict, pts_count: int) -> str:
    """Classify obstacle from geometry for semantic labeling."""
    lx = obb.get("extent_x", 0.0)
    ly = obb.get("extent_y", 0.0)
    lz = obb.get("extent_z", 0.0)
    max_dim = max(lx, ly)
    min_dim = min(lx, ly)

    if max_dim < 0.35 and lz > 0.6:
        return "Signpost / Pole"
    elif max_dim > 2.5 and min_dim < 0.6:
        return "Curb / Barrier"
    elif max_dim >= 2.5 and min_dim >= 1.0:
        return "Vehicle / Car"
    elif max_dim >= 1.2:
        return "Compact Object"
    else:
        return "Small Obstacle"


# ──────────────────────────────────────────────────────────────────────────────
# 3. Foxglove MCAP Timeline Writer
# ──────────────────────────────────────────────────────────────────────────────

def make_timestamp(ns: int) -> Timestamp:
    ts = Timestamp()
    ts.seconds = ns // 1_000_000_000
    ts.nanos = ns % 1_000_000_000
    return ts


def create_quaternion_from_yaw(yaw_rad: float):
    """Create a planar (Z-axis) rotation quaternion."""
    q = Pose_pb2.Pose().orientation
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def export_foxglove_mcap(
    data: dict,
    obstacle_points: np.ndarray,
    output_mcap: Path,
    frame_id: str = "lidar"
):
    """
    Packs point cloud and 3D bounding geometries into a Foxglove MCAP file.
    Topics:
      /lidar/obstacles              -> foxglove.PointCloud
      /perception/bounding_boxes    -> foxglove.SceneUpdate (Cube + Arrow + Text)
      /perception/bounding_discs    -> foxglove.SceneUpdate (Cylinders)
      /vehicle/safety_corridor      -> foxglove.SceneUpdate (Lines)
    """
    if not HAS_FOXGLOVE:
        print("[ERROR] foxglove-schemas-protobuf and mcap-protobuf-support are required.", file=sys.stderr)
        return

    output_mcap.parent.mkdir(parents=True, exist_ok=True)
    temp_path = output_mcap.with_suffix(".mcap.tmp")
    cur_time_ns = int(time.time() * 1e9)

    clusters = data.get("clusters", [])

    # Palette
    color_palette = [
        (0.23, 0.51, 0.96),  # blue
        (0.96, 0.62, 0.04),  # amber
        (0.93, 0.28, 0.60),  # pink
        (0.55, 0.36, 0.96),  # purple
        (0.02, 0.71, 0.83),  # cyan
        (0.06, 0.73, 0.51),  # emerald
        (0.39, 0.45, 0.55),  # slate
        (0.94, 0.27, 0.27),  # red
    ]

    with open(temp_path, "wb") as f_out:
        with McapWriter(f_out) as writer:
            # ──────────────────────────────────────────────────────────────────
            # 1. Point Cloud (/lidar/obstacles)
            # ──────────────────────────────────────────────────────────────────
            # Gather all points from clusters or array
            all_pts = []
            if len(obstacle_points) > 0:
                for p in obstacle_points:
                    all_pts.append((p[0], p[1], p[2], 1.0))
            else:
                for cl in clusters:
                    cid = cl.get("id", 0)
                    for p in cl.get("sample_points", []):
                        all_pts.append((p["x"], p["y"], p.get("z", 0.0), float(cid + 1)))

            if all_pts:
                raw_bytes = bytearray()
                for (x, y, z, intensity) in all_pts:
                    raw_bytes.extend(struct.pack("<ffff", x, y, z, intensity))

                pcd_msg = PointCloud_pb2.PointCloud()
                pcd_msg.timestamp.CopyFrom(make_timestamp(cur_time_ns))
                pcd_msg.frame_id = frame_id
                pcd_msg.pose.position.x = 0.0
                pcd_msg.pose.orientation.w = 1.0
                pcd_msg.point_stride = 16

                for idx, name in enumerate(["x", "y", "z", "intensity"]):
                    f_elem = PackedElementField_pb2.PackedElementField()
                    f_elem.name = name
                    f_elem.offset = idx * 4
                    f_elem.type = PackedElementField_pb2.PackedElementField.FLOAT32
                    pcd_msg.fields.append(f_elem)

                pcd_msg.data = bytes(raw_bytes)

                writer.write_message(
                    topic="/lidar/obstacles",
                    message=pcd_msg,
                    log_time=cur_time_ns,
                    publish_time=cur_time_ns,
                )

            # ──────────────────────────────────────────────────────────────────
            # 2. 3D Oriented Bounding Boxes (/perception/bounding_boxes)
            # ──────────────────────────────────────────────────────────────────
            scene_obb = SceneUpdate_pb2.SceneUpdate()
            obb_entity = scene_obb.entities.add()
            obb_entity.id = "obstacles_3d"
            obb_entity.frame_id = frame_id
            obb_entity.timestamp.CopyFrom(make_timestamp(cur_time_ns))

            for cl in clusters:
                cid = cl.get("id", 0)
                rgb = color_palette[cid % len(color_palette)]
                obb = cl.get("obb", {})
                disc = cl.get("disc", {})
                pts_cnt = cl.get("point_count", len(cl.get("sample_points", [])))
                label = infer_label(obb, disc, pts_cnt)

                cx, cy, cz = obb.get("cx", 0.0), obb.get("cy", 0.0), obb.get("cz", 0.0)
                ex, ey, ez = obb.get("extent_x", 1.0), obb.get("extent_y", 1.0), obb.get("extent_z", 1.0)
                yaw_deg = obb.get("yaw_deg", 0.0)
                yaw_rad = math.radians(yaw_deg)

                # A. 3D Minimal OBB Cube
                cube = obb_entity.cubes.add()
                cube.pose.position.x = cx
                cube.pose.position.y = cy
                cube.pose.position.z = cz
                cube.pose.orientation.CopyFrom(create_quaternion_from_yaw(yaw_rad))
                cube.size.x = ex
                cube.size.y = ey
                cube.size.z = ez
                cube.color.r = rgb[0]
                cube.color.g = rgb[1]
                cube.color.b = rgb[2]
                cube.color.a = 0.38

                # B. Heading Yaw Arrow
                arrow = obb_entity.arrows.add()
                arrow.pose.position.x = cx
                arrow.pose.position.y = cy
                arrow.pose.position.z = cz
                arrow.pose.orientation.CopyFrom(create_quaternion_from_yaw(yaw_rad))
                arrow.shaft_length = max(0.6, ex * 0.45)
                arrow.shaft_diameter = 0.08
                arrow.head_length = 0.25
                arrow.head_diameter = 0.18
                arrow.color.r = rgb[0]
                arrow.color.g = rgb[1]
                arrow.color.b = rgb[2]
                arrow.color.a = 1.0

                # C. 3D Billboard Text Label
                txt = obb_entity.texts.add()
                txt.pose.position.x = cx
                txt.pose.position.y = cy
                txt.pose.position.z = cz + ez * 0.5 + 0.35
                txt.text = f"#{cid} {label}\n{ex:.2f}m × {ey:.2f}m (θ={yaw_deg:+.1f}°)"
                txt.font_size = 14
                txt.billboard = True
                txt.color.r = 1.0
                txt.color.g = 1.0
                txt.color.b = 1.0
                txt.color.a = 0.95

            writer.write_message(
                topic="/perception/bounding_boxes",
                message=scene_obb,
                log_time=cur_time_ns,
                publish_time=cur_time_ns,
            )

            # ──────────────────────────────────────────────────────────────────
            # 3. Fast Reactive Bounding Discs (/perception/bounding_discs)
            # ──────────────────────────────────────────────────────────────────
            scene_discs = SceneUpdate_pb2.SceneUpdate()
            disc_entity = scene_discs.entities.add()
            disc_entity.id = "bounding_discs"
            disc_entity.frame_id = frame_id
            disc_entity.timestamp.CopyFrom(make_timestamp(cur_time_ns))

            for cl in clusters:
                disc = cl.get("disc", {})
                dcx = disc.get("cx", 0.0)
                dcy = disc.get("cy", 0.0)
                dr = disc.get("radius", 0.5)
                z_min = disc.get("z_min", 0.0)

                cyl = disc_entity.cylinders.add()
                cyl.pose.position.x = dcx
                cyl.pose.position.y = dcy
                cyl.pose.position.z = z_min + 0.02
                cyl.pose.orientation.w = 1.0
                cyl.size.x = dr * 2.0
                cyl.size.y = dr * 2.0
                cyl.size.z = 0.04
                cyl.color.r = 0.02
                cyl.color.g = 0.71
                cyl.color.b = 0.83
                cyl.color.a = 0.22

            writer.write_message(
                topic="/perception/bounding_discs",
                message=scene_discs,
                log_time=cur_time_ns,
                publish_time=cur_time_ns,
            )

            # ──────────────────────────────────────────────────────────────────
            # 4. Driving Safety Corridor Lines (/vehicle/safety_corridor)
            # ──────────────────────────────────────────────────────────────────
            scene_corr = SceneUpdate_pb2.SceneUpdate()
            corr_entity = scene_corr.entities.add()
            corr_entity.id = "safety_corridor"
            corr_entity.frame_id = frame_id
            corr_entity.timestamp.CopyFrom(make_timestamp(cur_time_ns))

            # Left boundary line (+0.40m)
            line_l = corr_entity.lines.add()
            line_l.type = LinePrimitive_pb2.LinePrimitive.LINE_STRIP
            line_l.thickness = 0.04
            line_l.color.r = 0.94
            line_l.color.g = 0.27
            line_l.color.b = 0.27
            line_l.color.a = 0.75
            p1 = line_l.points.add()
            p1.x, p1.y, p1.z = 0.0, 0.40, 0.05
            p2 = line_l.points.add()
            p2.x, p2.y, p2.z = 20.0, 0.40, 0.05

            # Right boundary line (-0.40m)
            line_r = corr_entity.lines.add()
            line_r.type = LinePrimitive_pb2.LinePrimitive.LINE_STRIP
            line_r.thickness = 0.04
            line_r.color.r = 0.94
            line_r.color.g = 0.27
            line_r.color.b = 0.27
            line_r.color.a = 0.75
            p3 = line_r.points.add()
            p3.x, p3.y, p3.z = 0.0, -0.40, 0.05
            p4 = line_r.points.add()
            p4.x, p4.y, p4.z = 20.0, -0.40, 0.05

            writer.write_message(
                topic="/vehicle/safety_corridor",
                message=scene_corr,
                log_time=cur_time_ns,
                publish_time=cur_time_ns,
            )

    temp_path.replace(output_mcap)
    size_kb = output_mcap.stat().st_size / 1024.0
    print(f"\n[OK] Exported Foxglove MCAP timeline -> {output_mcap} ({size_kb:.1f} KB)")
    print(f"     Topics: /lidar/obstacles, /perception/bounding_boxes, /perception/bounding_discs, /vehicle/safety_corridor")


# ──────────────────────────────────────────────────────────────────────────────
# 4. HTTP Range & CORS Streaming Server for Foxglove Studio
# ──────────────────────────────────────────────────────────────────────────────

class FoxgloveRangeServer(http.server.SimpleHTTPRequestHandler):
    """Compliant HTTP 206 Partial Content server with open CORS."""
    target_dir: Path = None

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, HEAD, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Range, Content-Type, Authorization")
        self.send_header("Access-Control-Expose-Headers", "Content-Length, Content-Range, Accept-Ranges")
        self.send_header("Accept-Ranges", "bytes")
        super().end_headers()

    def do_OPTIONS(self):
        self.send_response(200)
        self.end_headers()


def serve_mcap_server(target_file: Path, port: int = 8080):
    """Serve the generated MCAP directory over local HTTP."""
    target_dir = target_file.parent.resolve()
    filename = target_file.name

    class BoundHandler(FoxgloveRangeServer):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(target_dir), **kwargs)

    url = f"http://localhost:{port}/{filename}"
    foxglove_web_url = f"https://app.foxglove.dev/open?ds=file&ds.url={url}"

    print("\n" + "═" * 86)
    print(" 🦊 Foxglove Studio Streaming Server Active")
    print("═" * 86)
    print(f" Local URL:       {url}")
    print(f" Foxglove Web:    {foxglove_web_url}")
    print(f" Foxglove Native: foxglove://open?url={url}")
    print("═" * 86)
    print(" Press Ctrl+C to terminate server.\n")

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", port), BoundHandler) as httpd:
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\n[INFO] Foxglove server stopped.")


# ──────────────────────────────────────────────────────────────────────────────
# 5. Terminal Roster & Matplotlib Renderer
# ──────────────────────────────────────────────────────────────────────────────

def print_ansi_roster(data: dict):
    clusters = data.get("clusters", [])
    print("\n" + "═" * 88)
    print(" RVPoint ADR-0009 Dual Obstacle Telemetry Summary")
    print("═" * 88)
    print(f"{'ID':^4} | {'Pts':^6} | {'Centroid (X, Y, Z)':^22} | {'OBB (L × W × H)':^18} | {'Yaw':^8} | {'Disc R':^7} | {'Class':<14}")
    print("─" * 88)
    for idx, cl in enumerate(clusters):
        cid = cl.get("id", idx)
        pts_count = cl.get("point_count", len(cl.get("sample_points", [])))
        obb = cl.get("obb", {})
        disc = cl.get("disc", {})
        ocx, ocy, ocz = obb.get("cx", 0), obb.get("cy", 0), obb.get("cz", 0)
        ex, ey, ez = obb.get("extent_x", 0), obb.get("extent_y", 0), obb.get("extent_z", 0)
        yaw = obb.get("yaw_deg", 0)
        dr = disc.get("radius", 0)
        label = infer_label(obb, disc, pts_count)
        print(f"#{cid:^3} | {pts_count:^6} | ({ocx:5.1f}, {ocy:5.1f}, {ocz:4.1f})m | {ex:4.2f} × {ey:4.2f} × {ez:4.2f}m | {yaw:+6.1f}° | {dr:5.2f}m | {label:<14}")
    print("═" * 88 + "\n")


def render_matplotlib(data: dict, output_png: Path, show_window: bool):
    try:
        import matplotlib.pyplot as plt
        from matplotlib.patches import Circle, Polygon
    except ImportError:
        return

    clusters = data.get("clusters", [])
    if not clusters:
        return

    cluster_colors = [
        "#3b82f6", "#f59e0b", "#ec4899", "#8b5cf6",
        "#06b6d4", "#10b981", "#64748b", "#ef4444"
    ]

    fig = plt.figure(figsize=(18, 9), facecolor="#0f172a")
    gs = fig.add_gridspec(2, 3, width_ratios=[1.3, 1.2, 1.1], height_ratios=[1, 1], hspace=0.28, wspace=0.25)

    ax_bev = fig.add_subplot(gs[:, 0], facecolor="#1e293b")
    ax_3d = fig.add_subplot(gs[0, 1], projection="3d", facecolor="#1e293b")
    ax_close = fig.add_subplot(gs[1, 1], facecolor="#1e293b")
    ax_tbl = fig.add_subplot(gs[:, 2], facecolor="#1e293b")

    # 1. BEV
    ax_bev.set_title("ISO 8855 Bird's Eye View (BEV Top-Down)\nDual Geometric Bounds: Discs (Cyan) + 3D OBBs (Color)",
                     color="#f8fafc", fontsize=11, fontweight="bold", pad=12)
    ax_bev.grid(True, linestyle="--", alpha=0.2, color="#94a3b8")
    ax_bev.fill([0, 20, 20, 0], [-0.4, -0.4, 0.4, 0.4], color="#ef4444", alpha=0.10, label="Safety Corridor (|Y| ≤ 0.4m)")
    ax_bev.plot([0, 20], [0.4, 0.4], color="#ef4444", linestyle="--", linewidth=1.2, alpha=0.6)
    ax_bev.plot([0, 20], [-0.4, -0.4], color="#ef4444", linestyle="--", linewidth=1.2, alpha=0.6)
    ax_bev.scatter(0, 0, color="#10b981", s=140, marker="o", edgecolors="#f8fafc", linewidth=1.8, zorder=10)
    ax_bev.annotate("", xy=(1.2, 0), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#10b981", lw=2.5, mutation_scale=15))

    table_rows = []

    for idx, cl in enumerate(clusters):
        cid = cl.get("id", idx)
        color = cluster_colors[cid % len(cluster_colors)]
        disc = cl.get("disc", {})
        obb = cl.get("obb", {})
        pts = cl.get("sample_points", [])
        pt_count = cl.get("point_count", len(pts))
        label = infer_label(obb, disc, pt_count)

        if pts:
            px = [p["x"] for p in pts]
            py = [p["y"] for p in pts]
            ax_bev.scatter(px, py, color=color, s=12, alpha=0.75, zorder=4)

        dcx, dcy, dr = disc.get("cx", 0.0), disc.get("cy", 0.0), disc.get("radius", 0.0)
        ax_bev.add_patch(Circle((dcx, dcy), dr, edgecolor="#06b6d4", facecolor="#06b6d4", alpha=0.08, linestyle="--", linewidth=1.4, zorder=3))

        corners = obb.get("corners", [])
        if len(corners) == 4:
            poly_pts = [(c["x"], c["y"]) for c in corners]
            ax_bev.add_patch(Polygon(poly_pts, closed=True, edgecolor=color, facecolor=color, alpha=0.22, linestyle="-", linewidth=2.0, zorder=5))

        ocx, ocy = obb.get("cx", dcx), obb.get("cy", dcy)
        yaw_rad = math.radians(obb.get("yaw_deg", 0.0))
        arrow_len = max(0.6, obb.get("extent_x", 1.0) * 0.45)
        ax_bev.annotate("", xy=(ocx + arrow_len * math.cos(yaw_rad), ocy + arrow_len * math.sin(yaw_rad)), xytext=(ocx, ocy),
                        arrowprops=dict(arrowstyle="->", color=color, lw=2.0, mutation_scale=12), zorder=6)

        ax_bev.text(ocx + 0.2, ocy + 0.2, f"#{cid}\n{label.split()[0]}", color="#f8fafc", fontsize=8, fontweight="bold", zorder=7,
                    bbox=dict(boxstyle="round,pad=0.2", facecolor=color, alpha=0.65, edgecolor="none"))

        table_rows.append([
            f"#{cid}", f"{pt_count}", f"({ocx:.1f}, {ocy:.1f})",
            f"{obb.get('extent_x', 0):.2f} × {obb.get('extent_y', 0):.2f}",
            f"{obb.get('yaw_deg', 0):+.1f}°", f"{dr:.2f}m", label
        ])

    ax_bev.set_xlabel("Forward Axis +X (m) [Vehicle Heading →]", color="#cbd5e1", fontsize=10)
    ax_bev.set_ylabel("Lateral Axis +Y (m) [Left ← / Right →]", color="#cbd5e1", fontsize=10)
    ax_bev.tick_params(colors="#94a3b8")
    ax_bev.set_xlim(-1.0, 20.0)
    ax_bev.set_ylim(-6.0, 5.0)

    # 2. 3D Wireframe
    ax_3d.set_title("3D Isometric Multi-Cluster Wireframe Bounds", color="#f8fafc", fontsize=11, fontweight="bold", pad=8)
    ax_3d.tick_params(colors="#94a3b8", labelsize=8)
    ax_3d.grid(True, linestyle=":", alpha=0.2)

    for idx, cl in enumerate(clusters):
        cid = cl.get("id", idx)
        color = cluster_colors[cid % len(cluster_colors)]
        obb = cl.get("obb", {})
        pts = cl.get("sample_points", [])

        if pts:
            ax_3d.scatter([p["x"] for p in pts], [p["y"] for p in pts], [p.get("z", 0.0) for p in pts], color=color, s=6, alpha=0.6)

        corners = obb.get("corners", [])
        cz = obb.get("cz", 0.0)
        ez = obb.get("extent_z", 1.0)
        if len(corners) == 4:
            b = [(c["x"], c["y"], cz - ez / 2.0) for c in corners]
            t = [(c["x"], c["y"], cz + ez / 2.0) for c in corners]
            for i in range(4):
                ax_3d.plot([b[i][0], b[(i+1)%4][0]], [b[i][1], b[(i+1)%4][1]], [b[i][2], b[(i+1)%4][2]], color=color, lw=1.2, alpha=0.85)
                ax_3d.plot([t[i][0], t[(i+1)%4][0]], [t[i][1], t[(i+1)%4][1]], [t[i][2], t[(i+1)%4][2]], color=color, lw=1.2, alpha=0.85)
                ax_3d.plot([b[i][0], t[i][0]], [b[i][1], t[i][1]], [b[i][2], t[i][2]], color=color, lw=1.2, alpha=0.85)

    ax_3d.view_init(elev=26, azim=-125)
    ax_3d.set_xlim(2, 19)
    ax_3d.set_ylim(-6, 5)
    ax_3d.set_zlim(0, 3)

    # 3. Primary Obstacle Zoom
    ax_close.set_title("Close-up Detail: Primary Hazard Obstacle", color="#f8fafc", fontsize=11, fontweight="bold", pad=8)
    ax_close.grid(True, linestyle="--", alpha=0.2, color="#94a3b8")
    ax_close.tick_params(colors="#94a3b8")

    primary = next((c for c in clusters if c.get("id") == 7), clusters[0] if clusters else None)
    if primary:
        col = "#ef4444"
        pts = primary.get("sample_points", [])
        disc = primary.get("disc", {})
        obb = primary.get("obb", {})

        if pts:
            ax_close.scatter([p["x"] for p in pts], [p["y"] for p in pts], color="#f8fafc", s=18, alpha=0.9, zorder=5, label=f"LiDAR Pts ({primary.get('point_count')} pts)")
        dcx, dcy, dr = disc.get("cx", 0), disc.get("cy", 0), disc.get("radius", 0)
        ax_close.add_patch(Circle((dcx, dcy), dr, edgecolor="#06b6d4", facecolor="#06b6d4", alpha=0.12, linestyle="--", linewidth=2.0, zorder=3, label=f"Bounding Disc (R={dr:.2f}m)"))

        corners = obb.get("corners", [])
        if len(corners) == 4:
            ax_close.add_patch(Polygon([(c["x"], c["y"]) for c in corners], closed=True, edgecolor=col, facecolor=col, alpha=0.25, linewidth=2.5, zorder=4,
                               label=f"3D OBB ({obb.get('extent_x'):.2f}m × {obb.get('extent_y'):.2f}m)"))

        yaw_rad = math.radians(obb.get("yaw_deg", 0))
        arrow_len = obb.get("extent_x", 2.0) * 0.5
        ax_close.annotate("", xy=(obb.get("cx") + arrow_len * math.cos(yaw_rad), obb.get("cy") + arrow_len * math.sin(yaw_rad)),
                          xytext=(obb.get("cx"), obb.get("cy")),
                          arrowprops=dict(arrowstyle="->", color=col, lw=2.5, mutation_scale=15), zorder=6)
        ax_close.legend(loc="upper left", fontsize=8, facecolor="#0f172a", edgecolor="#475569", labelcolor="#e2e8f0")
        ax_close.set_xlim(3.0, 9.0)
        ax_close.set_ylim(-4.5, -0.5)

    # 4. Table
    ax_tbl.axis("off")
    ax_tbl.set_title("Real Obstacle Telemetry Roster (ADR-0009)", color="#f8fafc", fontsize=11, fontweight="bold", pad=8)
    col_labels = ["ID", "Pts", "Centroid (X,Y)", "OBB (L×W)", "Yaw", "Disc R", "Class"]
    col_widths = [0.10, 0.12, 0.22, 0.22, 0.14, 0.14, 0.24]
    table = ax_tbl.table(cellText=table_rows, colLabels=col_labels, colWidths=col_widths, loc="center", cellLoc="center")
    table.auto_set_font_size(False)
    table.set_fontsize(7.5)
    table.scale(1.0, 1.45)

    for (r, c), cell in table.get_celld().items():
        cell.set_edgecolor("#334155")
        if r == 0:
            cell.set_facecolor("#0284c7")
            cell.set_text_props(color="#ffffff", fontweight="bold")
        else:
            cid = int(table_rows[r - 1][0].replace("#", ""))
            row_col = cluster_colors[cid % len(cluster_colors)]
            cell.set_facecolor("#1e293b" if r % 2 == 0 else "#0f172a")
            if c == 0:
                cell.set_text_props(color=row_col, fontweight="bold")
            elif c == 5:
                cell.set_text_props(color="#38bdf8", fontweight="bold")
            elif c in (3, 4):
                cell.set_text_props(color="#4ade80")
            else:
                cell.set_text_props(color="#e2e8f0")

    output_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_png, dpi=180, bbox_inches="tight", facecolor=fig.get_facecolor())
    print(f"[OK] Rendered BEV figure saved to: {output_png}")

    if show_window:
        plt.show()
    else:
        plt.close(fig)


# ──────────────────────────────────────────────────────────────────────────────
# Main Entry Point
# ──────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="RVPoint PCD Obstacle Bounding Box & Foxglove MCAP Visualizer")
    parser.add_argument("pcd", nargs="?", default="data/pcd_compressed/0000000000.pcd",
                        help="Input PCD file path (default: data/pcd_compressed/0000000000.pcd)")
    parser.add_argument("--mcap", type=Path, default=Path("output/obstacles.mcap"),
                        help="Output MCAP timeline file path (default: output/obstacles.mcap)")
    parser.add_argument("--json", type=Path, default=Path("output/ticket_06_real_data.json"),
                        help="Path to intermediate or output obstacle JSON (default: output/ticket_06_real_data.json)")
    parser.add_argument("--png", type=Path, default=Path("output/ticket_06_obstacles.png"),
                        help="Output image path for rendered figure (default: output/ticket_06_obstacles.png)")
    parser.add_argument("--optical", action="store_true",
                        help="Specify if input PCD is in camera optical frame (X right, Y down, Z forward)")
    parser.add_argument("--serve", action="store_true",
                        help="Launch HTTP 206 CORS server to stream directly into Foxglove Studio")
    parser.add_argument("--port", type=int, default=8080, help="Port for Foxglove server (default: 8080)")
    parser.add_argument("--show", action="store_true", help="Display interactive matplotlib BEV window")
    parser.add_argument("--open3d", action="store_true", help="Display interactive 3D Open3D point cloud & box viewer")
    parser.add_argument("--skip-cpp", action="store_true", help="Skip C++ pipeline execution and use existing JSON")
    args = parser.parse_args()

    pcd_path = Path(args.pcd)
    if not pcd_path.exists():
        alt = Path("data") / pcd_path.name
        if alt.exists():
            pcd_path = alt

    output_corridor_pcd = Path("output/corridor_obstacles.pcd")

    # Step 1: Run C++ bounding box extraction on PCD
    if not args.skip_cpp and pcd_path.suffix.lower() == ".pcd":
        run_cpp_bounding_boxing(pcd_path, args.json, output_corridor_pcd, args.optical)

    # Step 2: Load extracted geometries from JSON
    if not args.json.exists():
        print(f"[ERROR] JSON telemetry not found at {args.json}", file=sys.stderr)
        sys.exit(1)

    with open(args.json, "r", encoding="utf-8") as f:
        data = json.load(f)

    # Step 3: Load segmented obstacle points
    obstacle_points = np.zeros((0, 3), dtype=np.float32)
    if output_corridor_pcd.exists():
        obstacle_points = load_pcd_points(output_corridor_pcd)

    # Step 4: Display ANSI telemetry roster
    print_ansi_roster(data)

    # Step 5: Export Foxglove MCAP timeline
    print(f"[2/4] Generating Foxglove Studio MCAP timeline...")
    export_foxglove_mcap(data, obstacle_points, args.mcap)

    # Step 6: Render Matplotlib Overview
    print(f"[3/4] Rendering high-resolution multi-panel overview...")
    render_matplotlib(data, args.png, args.show)

    # Step 7: Serve in Foxglove or print instructions
    print(f"\n[4/4] Foxglove Visualization Ready:")
    url = f"http://localhost:{args.port}/{args.mcap.name}"
    print(f"      MCAP File:   {args.mcap.resolve()}")
    print(f"      Foxglove URL: https://app.foxglove.dev/open?ds=file&ds.url={url}")
    print(f"      Direct Open:  foxglove://open?url={url}\n")

    if args.serve:
        serve_mcap_server(args.mcap, args.port)
    else:
        print(f"[TIP] Run with '--serve' to launch streaming server: uv run python scripts/viz/viz_obstacles.py {args.pcd} --serve")


if __name__ == "__main__":
    main()
