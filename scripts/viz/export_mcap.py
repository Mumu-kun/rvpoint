#!/usr/bin/env python3
"""
export_mcap.py  —  RVPoint Pipeline → Foxglove MCAP Timeline Exporter
====================================================================
export_mcap.py  —  RVPoint Pipeline → Foxglove MCAP Timeline Exporter
====================================================================

Reads pipeline run outputs (per-stage .pcd files across single or multi-frame
runs from `by_frame/`) and packs everything into a single .mcap timeline file
for Foxglove Studio visualization.

Usage Examples:
───────────────
1. Interactive Wizard (scans directory, lets you pick stages, outputs 1-line command):
   python scripts/export_mcap.py output/pcd_compressed_pipeline -i

2. Multi-Frame Export (all stages, 10 FPS timeline playback):
   python scripts/export_mcap.py output/pcd_compressed_pipeline --fps 10

3. Specific Stages Only:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --stages 00_input,05_ground_plane_removed,06_clusters

4. Exclude Specific Stages:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --exclude-stages 01_downsampled,02_sor_filtered

5. Frame Slicing:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --max-frames 5 --fps 5.0
Reads pipeline run outputs (per-stage .pcd files across single or multi-frame
runs from `by_frame/`) and packs everything into a single .mcap timeline file
for Foxglove Studio visualization.

Usage Examples:
───────────────
1. Interactive Wizard (scans directory, lets you pick stages, outputs 1-line command):
   python scripts/export_mcap.py output/pcd_compressed_pipeline -i

2. Multi-Frame Export (all stages, 10 FPS timeline playback):
   python scripts/export_mcap.py output/pcd_compressed_pipeline --fps 10

3. Specific Stages Only:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --stages 00_input,05_ground_plane_removed,06_clusters

4. Exclude Specific Stages:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --exclude-stages 01_downsampled,02_sor_filtered

5. Frame Slicing:
   python scripts/export_mcap.py output/pcd_compressed_pipeline --max-frames 5 --fps 5.0
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

FLOAT32 = PackedElementField_pb2.PackedElementField.FLOAT32  # 7
UINT8 = PackedElementField_pb2.PackedElementField.UINT8  # 1


# ─── Helpers ──────────────────────────────────────────────────────────────────
# ─── Helpers ──────────────────────────────────────────────────────────────────


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


def build_point_cloud_msg(
    fields: list,
    raw_payload: bytes,
    num_points: int,
    timestamp_ns: int,
    frame_id: str = "lidar",
) -> PointCloud_pb2.PointCloud:
    msg = PointCloud_pb2.PointCloud()
    msg.timestamp.CopyFrom(make_timestamp(timestamp_ns))
    msg.frame_id = frame_id
    msg.pose.CopyFrom(identity_pose())

    if num_points == 0 or not raw_payload:
        return msg

    stride = len(raw_payload) // num_points
    msg.point_stride = stride

    offset = 0
    for name in fields:
        f = PackedElementField_pb2.PackedElementField()
        f.name = name
        f.offset = offset
        f.type = FLOAT32
        msg.fields.append(f)
        offset += 4

    msg.data = raw_payload
    return msg


def load_pcd_data(pcd_path: Path):
    """
    Parses PCD header and raw binary payload.
    Returns: (fields, raw_payload, num_points)
    """
    try:
        with open(pcd_path, "rb") as f:
            header_lines = []
            data_mode = None
            while True:
                line = f.readline()
                if not line:
                    break
                line_str = line.decode("ascii", errors="ignore").strip()
                header_lines.append(line_str)
                if line_str.startswith("DATA"):
                    data_mode = line_str.split()[1].lower()
                    break

            fields = []
            num_points = 0
            for l in header_lines:
                if l.startswith("FIELDS"):
                    fields = l.split()[1:]
                elif l.startswith("POINTS"):
                    num_points = int(l.split()[1])

            if num_points == 0 or not fields:
                return None, None, 0

            if data_mode == "ascii":
                lines = f.readlines()
                vals = []
                for l in lines:
                    parts = l.decode("ascii", errors="ignore").strip().split()
                    if len(parts) >= len(fields):
                        vals.extend([float(p) for p in parts[: len(fields)]])
                arr = np.array(vals, dtype=np.float32)
                raw_payload = arr.tobytes()
            else:
                raw_payload = f.read()

            return fields, raw_payload, num_points
    except Exception as e:
        print(f"⚠️ Error reading PCD {pcd_path}: {e}")
        return None, None, 0


# ─── Frame & Stage Resolution ──────────────────────────────────────────────────


def discover_frames(input_dir: Path) -> list:
    input_dir = input_dir.resolve()
    by_frame_dir = input_dir / "by_frame"

    target_dir = by_frame_dir if by_frame_dir.is_dir() else input_dir

    # Collect subdirectories
    subdirs = sorted(
        [d for d in target_dir.iterdir() if d.is_dir()], key=lambda p: p.name
    )

    if subdirs:
        return subdirs

    # If no subdirectories, check if input_dir contains .pcd files directly
    pcd_files = list(input_dir.glob("*.pcd"))
    if pcd_files:
        return [input_dir]

    return []


def discover_available_stages(frame_dir: Path) -> list:
    pcd_files = sorted(list(frame_dir.glob("*.pcd")), key=lambda p: p.name)
    return [p.stem for p in pcd_files]


def filter_stages(
    available_stages: list, stages_opt: str = None, exclude_stages_opt: str = None
) -> list:
    selected = list(available_stages)

    if stages_opt:
        wanted = [s.strip() for s in stages_opt.split(",") if s.strip()]
        selected = [
            s for s in selected if any(w == s or w in s for w in wanted)
        ]

    if exclude_stages_opt:
        excluded = [s.strip() for s in exclude_stages_opt.split(",") if s.strip()]
        selected = [
            s for s in selected if not any(e == s or e in s for e in excluded)
        ]

    return selected


# ─── Interactive Mode Wizard ───────────────────────────────────────────────────


def run_interactive_wizard(input_dir: Path, frames: list) -> dict:
    print(f"\n🧙 RVPoint MCAP Exporter — Interactive Setup")
    print(f"   Target Dir : {input_dir}")
    print(f"   Found      : {len(frames)} frame(s)\n")

    sample_frame = frames[0]
    available_stages = discover_available_stages(sample_frame)

    if not available_stages:
        print(f"❌ Error: No .pcd files found in sample frame {sample_frame}")
        sys.exit(1)

    print("Available PCD stages:")
    for idx, stage in enumerate(available_stages, 1):
        print(f"  [{idx}] {stage}.pcd")

    print("\nEnter stage numbers to export (e.g., '1,5,6' or 'all') [all]: ", end="")
    try:
        user_choice = input().strip()
    except (EOFError, KeyboardInterrupt):
        user_choice = "all"

    if user_choice and user_choice.lower() != "all":
        selected_stems = []
        for part in user_choice.split(","):
            part = part.strip()
            if part.isdigit():
                idx = int(part) - 1
                if 0 <= idx < len(available_stages):
                    selected_stems.append(available_stages[idx])
            else:
                if part in available_stages:
                    selected_stems.append(part)
        stages_str = ",".join(selected_stems)
    else:
        stages_str = None
        selected_stems = available_stages

    print("Enter playback FPS [10.0]: ", end="")
    try:
        fps_choice = input().strip()
        fps_val = float(fps_choice) if fps_choice else 10.0
    except (ValueError, EOFError, KeyboardInterrupt):
        fps_val = 10.0

    # Build 1-line command
    cmd_parts = ["python", "scripts/export_mcap.py", str(input_dir)]
    if fps_val != 10.0:
        cmd_parts.extend(["--fps", str(fps_val)])
    if stages_str:
        cmd_parts.extend(["--stages", stages_str])

    one_liner = " ".join(cmd_parts)

    print("\n" + "─" * 65)
    print("💡 Equivalent 1-Line Command for Automation:")
    print(f"   {one_liner}")
    print("─" * 65 + "\n")

    return {
        "fps": fps_val,
        "stages": stages_str,
        "selected_stems": selected_stems,
    }


# ─── Main Export Logic ────────────────────────────────────────────────────────


def export(
    input_dir: str,
    output_path: str = None,
    fps: float = 10.0,
    frame_id: str = "lidar",
    stages: str = None,
    exclude_stages: str = None,
    start_frame: int = 0,
    max_frames: int = None,
    interactive: bool = False,
    input_dir: str,
    output_path: str = None,
    fps: float = 10.0,
    frame_id: str = "lidar",
    stages: str = None,
    exclude_stages: str = None,
    start_frame: int = 0,
    max_frames: int = None,
    interactive: bool = False,
) -> None:
    input_dir = Path(input_dir).resolve()
    frames = discover_frames(input_dir)

    if not frames:
        print(f"❌ Error: No frame directories or PCD files found in '{input_dir}'.")
        sys.exit(1)

    if interactive or (not stages and sys.stdin.isatty() and "-i" in sys.argv):
        wizard_res = run_interactive_wizard(input_dir, frames)
        fps = wizard_res["fps"]
        stages = wizard_res["stages"]

    # Slice frames
    sliced_frames = frames[start_frame:]
    if max_frames and max_frames > 0:
        sliced_frames = sliced_frames[:max_frames]

    # Resolve output path
    if not output_path:
        output_path = input_dir / "pipeline.mcap"
    else:
        output_path = Path(output_path).resolve()

    input_dir = Path(input_dir).resolve()
    frames = discover_frames(input_dir)

    if not frames:
        print(f"❌ Error: No frame directories or PCD files found in '{input_dir}'.")
        sys.exit(1)

    if interactive or (not stages and sys.stdin.isatty() and "-i" in sys.argv):
        wizard_res = run_interactive_wizard(input_dir, frames)
        fps = wizard_res["fps"]
        stages = wizard_res["stages"]

    # Slice frames
    sliced_frames = frames[start_frame:]
    if max_frames and max_frames > 0:
        sliced_frames = sliced_frames[:max_frames]

    # Resolve output path
    if not output_path:
        output_path = input_dir / "pipeline.mcap"
    else:
        output_path = Path(output_path).resolve()

    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Determine stages to export
    available_stages = discover_available_stages(sliced_frames[0])
    selected_stages = filter_stages(available_stages, stages, exclude_stages)

    if not selected_stages:
        print("❌ Error: No stages matched your selection filters.")
        sys.exit(1)

    print(f"📦 RVPoint → Foxglove MCAP Timeline Exporter")
    print(f"   Input      : {input_dir}")
    print(f"   Output     : {output_path}")
    print(f"   Frames     : {len(sliced_frames)} frame(s) @ {fps:.1f} FPS")
    print(f"   Stages ({len(selected_stages)}) : {', '.join(selected_stages)}\n")
    # Determine stages to export
    available_stages = discover_available_stages(sliced_frames[0])
    selected_stages = filter_stages(available_stages, stages, exclude_stages)

    if not selected_stages:
        print("❌ Error: No stages matched your selection filters.")
        sys.exit(1)

    print(f"📦 RVPoint → Foxglove MCAP Timeline Exporter")
    print(f"   Input      : {input_dir}")
    print(f"   Output     : {output_path}")
    print(f"   Frames     : {len(sliced_frames)} frame(s) @ {fps:.1f} FPS")
    print(f"   Stages ({len(selected_stages)}) : {', '.join(selected_stages)}\n")

    temp_path = output_path.with_suffix(".mcap.tmp")
    frame_dt_ns = int(1_000_000_000 / max(fps, 0.001))
    base_time = now_ns()

    total_msgs_written = 0
    base_time = now_ns()

    total_msgs_written = 0

    with open(temp_path, "wb") as f_out:
        with McapWriter(f_out) as writer:
            for frame_idx, frame_dir in enumerate(sliced_frames):
            for frame_idx, frame_dir in enumerate(sliced_frames):
                cur_time = base_time + frame_idx * frame_dt_ns

                for stage_stem in selected_stages:
                    pcd_path = frame_dir / f"{stage_stem}.pcd"
                    if not pcd_path.exists():
                        continue

                    fields, raw_payload, num_points = load_pcd_data(pcd_path)
                    if num_points == 0 or not raw_payload:
                        continue

                    topic = f"/3d_points/{stage_stem}"
                    msg = build_point_cloud_msg(
                        fields=fields,
                        raw_payload=raw_payload,
                        num_points=num_points,
                        timestamp_ns=cur_time,
                        frame_id=frame_id,
                    )

                    writer.write_message(
                        topic=topic,
                        topic=topic,
                        message=msg,
                        log_time=cur_time,
                        publish_time=cur_time,
                    )
                    total_msgs_written += 1

                    total_msgs_written += 1

    temp_path.replace(output_path)

    size_mb = output_path.stat().st_size / 1_048_576
    duration_s = (len(sliced_frames) - 1) / max(fps, 0.001) if len(sliced_frames) > 1 else 0.0

    print(f"✅ Success — Wrote {total_msgs_written} messages across {len(sliced_frames)} frame(s)")
    print(f"   File       : {output_path.name} ({size_mb:.2f} MB)")
    print(f"   Timeline   : {duration_s:.1f} seconds playback\n")
    print(f"🦊 Open in Foxglove Studio:")
    duration_s = (len(sliced_frames) - 1) / max(fps, 0.001) if len(sliced_frames) > 1 else 0.0

    print(f"✅ Success — Wrote {total_msgs_written} messages across {len(sliced_frames)} frame(s)")
    print(f"   File       : {output_path.name} ({size_mb:.2f} MB)")
    print(f"   Timeline   : {duration_s:.1f} seconds playback\n")
    print(f"🦊 Open in Foxglove Studio:")
    print(f"   https://studio.foxglove.dev  →  Open local file  →  {output_path}\n")


# ─── CLI Entrypoint ───────────────────────────────────────────────────────────
# ─── CLI Entrypoint ───────────────────────────────────────────────────────────


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Export RVPoint PCD pipeline stages to a Foxglove MCAP timeline file.",
        description="Export RVPoint PCD pipeline stages to a Foxglove MCAP timeline file.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Interactive mode wizard:
  python scripts/export_mcap.py output/pcd_compressed_pipeline -i

  # Export 20-frame sequence at 10 FPS:
  python scripts/export_mcap.py output/pcd_compressed_pipeline --fps 10

  # Export specific stages only:
  python scripts/export_mcap.py output/pcd_compressed_pipeline --stages 00_input,05_ground_plane_removed,06_clusters
""",
        epilog="""
Examples:
  # Interactive mode wizard:
  python scripts/export_mcap.py output/pcd_compressed_pipeline -i

  # Export 20-frame sequence at 10 FPS:
  python scripts/export_mcap.py output/pcd_compressed_pipeline --fps 10

  # Export specific stages only:
  python scripts/export_mcap.py output/pcd_compressed_pipeline --stages 00_input,05_ground_plane_removed,06_clusters
""",
    )
    parser.add_argument(
        "input_dir",
        help="Pipeline output directory (contains by_frame/ or PCD files)",
        "input_dir",
        help="Pipeline output directory (contains by_frame/ or PCD files)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output .mcap file path. Default: <input_dir>/pipeline.mcap",
        help="Output .mcap file path. Default: <input_dir>/pipeline.mcap",
    )
    parser.add_argument(
        "--fps",
        "--fps",
        type=float,
        default=10.0,
        help="Playback frame rate (FPS) for multi-frame timeline (default: 10.0)",
    )
    parser.add_argument(
        "--frame-id",
        default="lidar",
        help="Coordinate frame ID for point cloud messages (default: lidar)",
        default=10.0,
        help="Playback frame rate (FPS) for multi-frame timeline (default: 10.0)",
    )
    parser.add_argument(
        "--frame-id",
        default="lidar",
        help="Coordinate frame ID for point cloud messages (default: lidar)",
    )
    parser.add_argument(
        "--stages",
        help="Comma-separated stage stems to export (e.g., '00_input,05_ground_plane_removed,06_clusters')",
        "--stages",
        help="Comma-separated stage stems to export (e.g., '00_input,05_ground_plane_removed,06_clusters')",
    )
    parser.add_argument(
        "--exclude-stages",
        help="Comma-separated stage stems to exclude",
        "--exclude-stages",
        help="Comma-separated stage stems to exclude",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=0,
        help="Index of first frame to export (default: 0)",
        "--start-frame",
        type=int,
        default=0,
        help="Index of first frame to export (default: 0)",
    )
    parser.add_argument(
        "--max-frames",
        "--max-frames",
        type=int,
        default=None,
        help="Maximum number of frames to export",
        default=None,
        help="Maximum number of frames to export",
    )
    parser.add_argument(
        "--interactive",
        "-i",
        action="store_true",
        help="Run interactive setup wizard",
        "--interactive",
        "-i",
        action="store_true",
        help="Run interactive setup wizard",
    )

    args = parser.parse_args()

    export(
        input_dir=args.input_dir,
        output_path=args.output,
        fps=args.fps,
        input_dir=args.input_dir,
        output_path=args.output,
        fps=args.fps,
        frame_id=args.frame_id,
        stages=args.stages,
        exclude_stages=args.exclude_stages,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
        interactive=args.interactive,
        stages=args.stages,
        exclude_stages=args.exclude_stages,
        start_frame=args.start_frame,
        max_frames=args.max_frames,
        interactive=args.interactive,
    )


if __name__ == "__main__":
    main()
