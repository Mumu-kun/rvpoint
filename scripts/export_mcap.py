#!/usr/bin/env python3
"""
export_mcap.py  —  RVPoint pipeline → Foxglove MCAP exporter
=============================================================

Reads a pipeline result directory (per-stage .pcd files) and packs everything
into a single .mcap file that opens directly in Foxglove Studio.

Layout inside the .mcap
────────────────────────
All messages share the SAME timestamp (one scan, synchronised stages):

  /stage/00_input              foxglove.PointCloud   — raw input
  /stage/01_downsampled        foxglove.PointCloud   — after voxel grid
  /stage/02_sor_filtered       foxglove.PointCloud   — after SOR
  /stage/04_ransac_inliers     foxglove.PointCloud   — RANSAC ground inliers
  /stage/05_ground_removed     foxglove.PointCloud   — objects only
  /stage/06_clusters           foxglove.PointCloud   — all objects, coloured by cluster
  /clusters/<NN>               foxglove.PointCloud   — one topic per cluster (toggle-able)
  /clusters/noise              foxglove.PointCloud   — unassigned points
"""

import argparse
import os
import struct
import sys
import time
from pathlib import Path

# Force UTF-8 output on Windows
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")

import numpy as np
import open3d as o3d
from mcap_protobuf.writer import Writer as McapWriter

from foxglove_schemas_protobuf import PackedElementField_pb2
from foxglove_schemas_protobuf import PointCloud_pb2
from foxglove_schemas_protobuf import Pose_pb2
from google.protobuf.timestamp_pb2 import Timestamp


# ─── constants ────────────────────────────────────────────────────────────────

PIPELINE_STAGES = [
    ("00_input", "00_input", "Raw Input"),
    ("01_downsampled", "01_downsampled", "Voxel Downsampled"),
    ("02_sor_filtered", "02_sor_filtered", "SOR Filtered"),
    ("04_ransac_inliers", "04_ransac_inliers", "RANSAC Ground Inliers"),
    ("05_ground_plane_removed", "05_ground_removed", "Ground Removed"),
]

FLOAT32 = PackedElementField_pb2.PackedElementField.FLOAT32  # 7
UINT8 = PackedElementField_pb2.PackedElementField.UINT8  # 1


# ─── helpers ──────────────────────────────────────────────────────────────────


def now_ns() -> int:
    return int(time.time() * 1e9)


def identity_pose() -> Pose_pb2.Pose:
    pose = Pose_pb2.Pose()
    pose.position.x = 0.0
    pose.position.y = 0.0
    pose.position.z = 0.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0
    pose.orientation.w = 1.0
    return pose


def make_timestamp(ns: int) -> Timestamp:
    ts = Timestamp()
    ts.seconds = ns // 1_000_000_000
    ts.nanos = ns % 1_000_000_000
    return ts


def xyz_fields() -> list:
    fields = []
    for name, offset in [("x", 0), ("y", 4), ("z", 8)]:
        f = PackedElementField_pb2.PackedElementField()
        f.name = name
        f.offset = offset
        f.type = FLOAT32
        fields.append(f)
    return fields


def xyzrgb_fields() -> list:
    fields = xyz_fields()
    for name, offset in [("red", 12), ("green", 13), ("blue", 14)]:
        f = PackedElementField_pb2.PackedElementField()
        f.name = name
        f.offset = offset
        f.type = UINT8
        fields.append(f)
    return fields


def pack_xyz(points: np.ndarray) -> bytes:
    return points.astype(np.float32).tobytes()


def pack_xyzrgb(points: np.ndarray, colors_u8: np.ndarray) -> bytes:
    n = len(points)
    buf = bytearray(n * 15)
    xyz_f32 = points.astype(np.float32)
    for i in range(n):
        base = i * 15
        buf[base : base + 12] = struct.pack(
            "3f", float(xyz_f32[i, 0]), float(xyz_f32[i, 1]), float(xyz_f32[i, 2])
        )
        buf[base + 12] = int(colors_u8[i, 0])
        buf[base + 13] = int(colors_u8[i, 1])
        buf[base + 14] = int(colors_u8[i, 2])
    return bytes(buf)


def build_point_cloud_msg(
    points: np.ndarray,
    timestamp_ns: int,
    frame_id: str = "lidar",
    colors_u8: np.ndarray = None,
) -> PointCloud_pb2.PointCloud:
    msg = PointCloud_pb2.PointCloud()
    msg.timestamp.CopyFrom(make_timestamp(timestamp_ns))
    msg.frame_id = frame_id
    msg.pose.CopyFrom(identity_pose())

    if colors_u8 is not None:
        msg.point_stride = 15
        msg.fields.extend(xyzrgb_fields())
        msg.data = pack_xyzrgb(points, colors_u8)
    else:
        msg.point_stride = 12
        msg.fields.extend(xyz_fields())
        msg.data = pack_xyz(points)

    return msg


def pcd_to_numpy(pcd_path: str) -> np.ndarray:
    pcd = o3d.io.read_point_cloud(pcd_path)
    if pcd.is_empty():
        raise RuntimeError(f"Failed to load or empty cloud: {pcd_path}")
    return np.asarray(pcd.points, dtype=np.float32)


def cluster_cloud(
    points: np.ndarray,
    tolerance: float,
    min_size: int,
    max_size: int,
):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    raw_labels = np.array(
        pcd.cluster_dbscan(eps=tolerance, min_points=min_size, print_progress=False)
    )

    n_clusters = int(raw_labels.max()) + 1 if raw_labels.max() >= 0 else 0
    print(
        f"   Clustering: {n_clusters} clusters found "
        f"({(raw_labels == -1).sum()} noise points)"
    )

    palette = np.zeros((max(n_clusters, 1), 3), dtype=np.uint8)
    for i in range(n_clusters):
        hue = (i / max(n_clusters, 1)) * 360.0
        h60 = hue / 60.0
        sector = int(h60) % 6
        f_val = h60 - int(h60)
        v, s = 0.95, 0.85
        p = v * (1 - s)
        q = v * (1 - s * f_val)
        t = v * (1 - s * (1 - f_val))
        rgb_f = [(v, t, p), (q, v, p), (p, v, t), (p, q, v), (t, p, v), (v, p, q)][
            sector
        ]
        palette[i] = [int(c * 255) for c in rgb_f]

    NOISE_COLOR = np.array([40, 40, 40], dtype=np.uint8)
    colored_all = np.full((len(points), 3), NOISE_COLOR, dtype=np.uint8)
    clusters = {}

    for lbl in range(n_clusters):
        mask = raw_labels == lbl
        cluster_pts = points[mask]
        sz = len(cluster_pts)
        if sz < min_size or sz > max_size:
            continue
        colored_all[mask] = palette[lbl]
        clusters[lbl] = (cluster_pts, palette[lbl], mask)

    return colored_all, clusters, raw_labels


# ─── main export logic ────────────────────────────────────────────────────────


def export(
    results_dir: str,
    output_path: str,
    cluster_tolerance: float,
    min_cluster: int,
    max_cluster: int,
    frame_id: str,
    timestamp_ns: int,
    num_frames: int = 1,
    fps: float = 1.0,
) -> None:
    results_dir = Path(results_dir).resolve()
    output_path = Path(output_path).resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"\n📦 RVPoint → MCAP Exporter")
    print(f"   Source : {results_dir}")
    print(f"   Output : {output_path}")
    print(f"   Frames : {num_frames} @ {fps} FPS\n")

    temp_path = output_path.with_suffix(".mcap.tmp")
    frame_dt_ns = int(1_000_000_000 / max(fps, 0.001))

    with open(temp_path, "wb") as f_out:
        with McapWriter(f_out) as writer:
            # Pre-load PCD datasets
            stage_data = []
            for stem, topic_suffix, label in PIPELINE_STAGES:
                pcd_path = results_dir / f"{stem}.pcd"
                if pcd_path.exists():
                    stage_data.append(
                        (stem, topic_suffix, label, pcd_to_numpy(str(pcd_path)))
                    )
                else:
                    print(f"   ⚠️  Skipping {stem}.pcd (not found)")

            cluster_source = results_dir / "05_ground_plane_removed.pcd"
            has_clusters = cluster_source.exists()
            clusters = {}
            colored_all = None
            ground_pts = None

            if has_clusters:
                print(
                    f"   🔬 Running Euclidean clustering "
                    f"(tol={cluster_tolerance}m, min={min_cluster}, max={max_cluster})"
                )
                ground_pts = pcd_to_numpy(str(cluster_source))
                colored_all, clusters, raw_labels = cluster_cloud(
                    ground_pts, cluster_tolerance, min_cluster, max_cluster
                )

            base_time = timestamp_ns

            for frame_idx in range(num_frames):
                cur_time = base_time + frame_idx * frame_dt_ns

                # 1. Pipeline stage topics
                for stem, topic_suffix, label, points in stage_data:
                    msg = build_point_cloud_msg(points, cur_time, frame_id)
                    writer.write_message(
                        topic=f"/stage/{topic_suffix}",
                        message=msg,
                        log_time=cur_time,
                        publish_time=cur_time,
                    )

                if has_clusters and ground_pts is not None:
                    # /stage/06_clusters
                    msg_all = build_point_cloud_msg(
                        ground_pts, cur_time, frame_id, colors_u8=colored_all
                    )
                    writer.write_message(
                        topic="/stage/06_clusters",
                        message=msg_all,
                        log_time=cur_time,
                        publish_time=cur_time,
                    )

                    # /clusters/<NN>
                    noise_mask = np.ones(len(ground_pts), dtype=bool)
                    for lbl, (pts, color, mask) in sorted(clusters.items()):
                        topic = f"/clusters/{lbl:02d}"
                        clr = np.tile(color, (len(pts), 1))
                        msg_c = build_point_cloud_msg(
                            pts, cur_time, frame_id, colors_u8=clr
                        )
                        writer.write_message(
                            topic=topic,
                            message=msg_c,
                            log_time=cur_time,
                            publish_time=cur_time,
                        )
                        noise_mask[mask] = False

                    # /clusters/noise
                    noise_pts = ground_pts[noise_mask]
                    if len(noise_pts) > 0:
                        noise_clr = colored_all[noise_mask]
                        msg_noise = build_point_cloud_msg(
                            noise_pts, cur_time, frame_id, colors_u8=noise_clr
                        )
                        writer.write_message(
                            topic="/clusters/noise",
                            message=msg_noise,
                            log_time=cur_time,
                            publish_time=cur_time,
                        )

            print(
                f"   📥 Wrote {len(stage_data)} stage topics & {len(clusters)} cluster topics across {num_frames} frame(s)."
            )

    # Atomically replace output_path after McapWriter finishes writing trailing magic bytes
    temp_path.replace(output_path)

    size_mb = output_path.stat().st_size / 1_048_576
    print(f"\n✅ Done — {output_path.name}  ({size_mb:.2f} MB)")
    print(f"\n🦊 Open in Foxglove Studio:")
    print(f"   https://studio.foxglove.dev  →  Open local file  →  {output_path}\n")


# ─── CLI ─────────────────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export RVPoint pipeline PCD stages to a Foxglove-compatible .mcap file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "results_dir",
        help="Path to a pipeline result directory (contains 00_input.pcd, etc.)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output .mcap file path. Default: <results_dir>/pipeline.mcap",
    )
    parser.add_argument(
        "--cluster-tolerance",
        type=float,
        default=0.15,
        metavar="METRES",
        help="Euclidean cluster tolerance in metres (default: 0.15)",
    )
    parser.add_argument(
        "--min-cluster",
        type=int,
        default=50,
        metavar="N",
        help="Minimum points per cluster (default: 50)",
    )
    parser.add_argument(
        "--max-cluster",
        type=int,
        default=100_000,
        metavar="N",
        help="Maximum points per cluster (default: 100000)",
    )
    parser.add_argument(
        "--frame-id",
        default="lidar",
        help="Coordinate frame ID for all point cloud messages (default: lidar)",
    )
    parser.add_argument(
        "--frames",
        "-n",
        type=int,
        default=1,
        help="Number of timeline frames to write (default: 1 for single snapshot)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=1.0,
        help="Frame rate (FPS) for multi-frame playback (default: 1.0)",
    )

    args = parser.parse_args()

    if not os.path.isdir(args.results_dir):
        print(f"❌ Error: '{args.results_dir}' is not a directory.")
        sys.exit(1)

    output_path = args.output or os.path.join(args.results_dir, "pipeline.mcap")

    export(
        results_dir=args.results_dir,
        output_path=output_path,
        cluster_tolerance=args.cluster_tolerance,
        min_cluster=args.min_cluster,
        max_cluster=args.max_cluster,
        frame_id=args.frame_id,
        timestamp_ns=now_ns(),
        num_frames=args.frames,
        fps=args.fps,
    )


if __name__ == "__main__":
    main()
