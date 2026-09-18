#!/usr/bin/env python3
"""
scripts/lib/bin_to_pcd.py

Convert KITTI binary point cloud files (.bin) to Point Cloud Data format (.pcd).
Each KITTI point consists of 4 single-precision floats: x, y, z, and reflectance/intensity.
Outputs standard PCD v0.7 format compatible with RVPoint and PCL.

Usage:
    python3 scripts/lib/bin_to_pcd.py
    python3 scripts/lib/bin_to_pcd.py --input data/data_2
    python3 scripts/lib/bin_to_pcd.py --input data/data_2/0000000000.bin --output data/data_2/0000000000.pcd
"""

import argparse
import glob
import os
import sys
import time


def convert_bin_to_pcd(bin_path: str, pcd_path: str, include_intensity: bool = False, ascii_format: bool = False) -> int:
    file_size = os.path.getsize(bin_path)
    if file_size % 16 != 0:
        raise ValueError(f"File size {file_size} is not a multiple of 16 bytes: {bin_path}")
    
    n_points = file_size // 16

    with open(bin_path, "rb") as f_in:
        raw_data = f_in.read()

    os.makedirs(os.path.dirname(os.path.abspath(pcd_path)), exist_ok=True)

    if include_intensity:
        fields = "x y z intensity"
        sizes = "4 4 4 4"
        types = "F F F F"
        counts = "1 1 1 1"
    else:
        fields = "x y z"
        sizes = "4 4 4"
        types = "F F F"
        counts = "1 1 1"

    data_type = "ascii" if ascii_format else "binary"

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {fields}\n"
        f"SIZE {sizes}\n"
        f"TYPE {types}\n"
        f"COUNT {counts}\n"
        f"WIDTH {n_points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {n_points}\n"
        f"DATA {data_type}\n"
    )

    if not ascii_format:
        with open(pcd_path, "wb") as f_out:
            f_out.write(header.encode("ascii"))
            if include_intensity:
                # Direct byte copy since raw_data is already [x, y, z, intensity] in float32
                f_out.write(raw_data)
            else:
                # Extract 12 bytes (x, y, z) from every 16-byte record
                out_buf = bytearray(n_points * 12)
                mv_in = memoryview(raw_data)
                mv_out = memoryview(out_buf)
                for i in range(n_points):
                    mv_out[i * 12 : (i + 1) * 12] = mv_in[i * 16 : i * 16 + 12]
                f_out.write(out_buf)
    else:
        import struct
        with open(pcd_path, "w", encoding="ascii") as f_out:
            f_out.write(header)
            mv = memoryview(raw_data)
            for i in range(n_points):
                if include_intensity:
                    x, y, z, r = struct.unpack_from("<4f", mv, i * 16)
                    f_out.write(f"{x:.4f} {y:.4f} {z:.4f} {r:.4f}\n")
                else:
                    x, y, z = struct.unpack_from("<3f", mv, i * 16)
                    f_out.write(f"{x:.4f} {y:.4f} {z:.4f}\n")

    return n_points


def main():
    parser = argparse.ArgumentParser(description="Convert KITTI .bin point clouds to .pcd")
    parser.add_argument(
        "--input",
        "-i",
        default="data/data_2",
        help="Input .bin file or directory containing .bin files (default: data/data_2)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Output .pcd file or directory (default: same as input)",
    )
    parser.add_argument(
        "--intensity",
        action="store_true",
        help="Include intensity field in PCD (FIELDS x y z intensity)",
    )
    parser.add_argument(
        "--ascii",
        action="store_true",
        help="Write ASCII PCD instead of binary PCD",
    )
    args = parser.parse_args()

    input_path = args.input
    if os.path.isfile(input_path):
        bin_files = [input_path]
    elif os.path.isdir(input_path):
        bin_files = sorted(glob.glob(os.path.join(input_path, "*.bin")))
    else:
        print(f"Error: Input path does not exist: {input_path}", file=sys.stderr)
        sys.exit(1)

    if not bin_files:
        print(f"No .bin files found in {input_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Found {len(bin_files)} .bin file(s) to convert...")
    t0 = time.time()
    total_points = 0

    for idx, bin_file in enumerate(bin_files, 1):
        if args.output and os.path.isfile(args.output):
            out_pcd = args.output
        elif args.output and os.path.isdir(args.output):
            base_name = os.path.splitext(os.path.basename(bin_file))[0]
            out_pcd = os.path.join(args.output, f"{base_name}.pcd")
        else:
            base_name = os.path.splitext(bin_file)[0]
            out_pcd = f"{base_name}.pcd"

        n_pts = convert_bin_to_pcd(
            bin_file,
            out_pcd,
            include_intensity=args.intensity,
            ascii_format=args.ascii,
        )
        total_points += n_pts
        out_size_mb = os.path.getsize(out_pcd) / (1024 * 1024)
        print(f"  [{idx}/{len(bin_files)}] {os.path.basename(bin_file)} -> {os.path.basename(out_pcd)}: {n_pts:,} points ({out_size_mb:.2f} MB)")

    elapsed = time.time() - t0
    print(f"Successfully converted {len(bin_files)} files ({total_points:,} total points) in {elapsed:.2f}s.")


if __name__ == "__main__":
    main()
