#!/usr/bin/env python3
"""
==============================================================================
SCRIPT: scripts/lib/downsample_pcd.py
PURPOSE: Host-side zero-dependency PCD point cloud downsampler for gem5 simulation
         budgeting. Preserves true 3D spatial geometry, ground plane orientation,
         and cluster distributions across discrete simulation tiers.

TIERS:
  --sanity : ~100 – 250 points   (Instant smoke testing, ~5-15s gem5 simulation)
  --dev    : ~1,000 points       (Fast dev loop / L1D boundary, ~1-2m simulation)
  --eval   : ~5,000 – 10,000 pts (Official benchmarks / L2 stressed, ~15-25m simulation)
  --full   : 100% (Pass-through, no decimation)

USAGE:
  python3 scripts/lib/downsample_pcd.py --input data/bunny.pcd --tier dev --output output/.downsampled_dev.pcd
  python3 scripts/lib/downsample_pcd.py --input scene.pcd --target-points 2500 --output out.pcd
==============================================================================
"""

import sys
import os
import struct
import math
import argparse
from typing import List, Tuple, Dict, Any, Optional

TIER_BUDGETS = {
    "sanity": 150,    # Functional smoke verification (~5-15s simulation)
    "dev": 1000,      # L1D cache resident dev loop (~1-2m simulation)
    "eval": 5000,     # Official benchmark evaluation (~10-15m simulation)
    "stress": 10000,  # L2 cache pressure stress test (~20-30m simulation)
    "full": 0,        # 100% pass-through (no decimation)
}

class PCDData:
    def __init__(self):
        self.headers: List[str] = []
        self.fields: List[str] = []
        self.sizes: List[int] = []
        self.types: List[str] = []
        self.counts: List[int] = []
        self.width: int = 0
        self.height: int = 1
        self.points: int = 0
        self.data_type: str = "ascii"
        self.viewpoint: str = "0 0 0 1 0 0 0"
        self.point_records: List[List[float]] = [] # Each record is [x, y, z, ...]
        self.x_idx: int = 0
        self.y_idx: int = 1
        self.z_idx: int = 2
        self.version: str = "0.7"

def decompress_lzf(input_bytes: bytes, uncompressed_size: int) -> bytes:
    """Zero-dependency LZF decompressor for PCD binary_compressed format."""
    output = bytearray(uncompressed_size)
    ip = 0
    op = 0
    in_len = len(input_bytes)
    
    while ip < in_len:
        ctrl = input_bytes[ip]
        ip += 1
        if ctrl < 32:  # Literal run
            length = ctrl + 1
            if op + length > uncompressed_size or ip + length > in_len:
                raise ValueError("LZF decompression error: buffer overrun on literal run")
            output[op : op + length] = input_bytes[ip : ip + length]
            op += length
            ip += length
        else:  # Back reference
            length = ctrl >> 5
            ref = op - ((ctrl & 0x1F) << 8) - 1
            if length == 7:
                if ip >= in_len:
                    raise ValueError("LZF decompression error: unexpected EOF in length extension")
                length += input_bytes[ip]
                ip += 1
            if ip >= in_len:
                raise ValueError("LZF decompression error: unexpected EOF in ref offset")
            ref -= input_bytes[ip]
            ip += 1
            length += 2
            
            if op + length > uncompressed_size:
                raise ValueError("LZF decompression error: buffer overrun on back-reference")
            if ref < 0 or ref >= op:
                raise ValueError("LZF decompression error: invalid back-reference offset")
            
            for _ in range(length):
                output[op] = output[ref]
                op += 1
                ref += 1
                
    if op != uncompressed_size:
        raise ValueError(f"LZF decompression size mismatch: got {op}, expected {uncompressed_size}")
    return bytes(output)

def parse_pcd(filepath: str) -> PCDData:
    if not os.path.isfile(filepath):
        raise FileNotFoundError(f"Input PCD file not found: {filepath}")

    pcd = PCDData()
    with open(filepath, "rb") as f:
        header_lines = []
        while True:
            line_bytes = f.readline()
            if not line_bytes:
                break
            line_str = line_bytes.decode("ascii", errors="ignore").strip()
            header_lines.append(line_str)
            if line_str.startswith("DATA"):
                break
        
        for line in header_lines:
            parts = line.split()
            if not parts: continue
            key = parts[0].upper()
            if key == "VERSION":
                pcd.version = parts[1]
            elif key == "FIELDS":
                pcd.fields = [p.lower() for p in parts[1:]]
                for idx, fld in enumerate(pcd.fields):
                    if fld == "x": pcd.x_idx = idx
                    elif fld == "y": pcd.y_idx = idx
                    elif fld == "z": pcd.z_idx = idx
            elif key == "SIZE":
                pcd.sizes = [int(p) for p in parts[1:]]
            elif key == "TYPE":
                pcd.types = [p.upper() for p in parts[1:]]
            elif key == "COUNT":
                pcd.counts = [int(p) for p in parts[1:]]
            elif key == "WIDTH":
                pcd.width = int(parts[1])
            elif key == "HEIGHT":
                pcd.height = int(parts[1])
            elif key == "VIEWPOINT":
                pcd.viewpoint = " ".join(parts[1:])
            elif key == "POINTS":
                pcd.points = int(parts[1])
            elif key == "DATA":
                pcd.data_type = parts[1].lower()

        total_fields = len(pcd.fields) if pcd.fields else 3
        if not pcd.sizes: pcd.sizes = [4] * total_fields
        if not pcd.types: pcd.types = ["F"] * total_fields
        if not pcd.counts: pcd.counts = [1] * total_fields

        if pcd.data_type == "ascii":
            for line_bytes in f:
                line_str = line_bytes.decode("ascii", errors="ignore").strip()
                if not line_str: continue
                parts = line_str.split()
                if len(parts) >= 3:
                    try:
                        record = [float(p) for p in parts[:total_fields]]
                        if not any(math.isnan(p) for p in record[:3]):
                            pcd.point_records.append(record)
                    except ValueError: continue
        elif pcd.data_type == "binary":
            fmt_str = "<"
            for sz, tp in zip(pcd.sizes, pcd.types):
                if tp == "F": fmt_str += "f" if sz == 4 else "d"
                elif tp == "I":
                    if sz == 1: fmt_str += "b"
                    elif sz == 2: fmt_str += "h"
                    elif sz == 4: fmt_str += "i"
                    elif sz == 8: fmt_str += "q"
                elif tp == "U":
                    if sz == 1: fmt_str += "B"
                    elif sz == 2: fmt_str += "H"
                    elif sz == 4: fmt_str += "I"
                    elif sz == 8: fmt_str += "Q"
                else: fmt_str += f"{sz}s"
            record_size = struct.calcsize(fmt_str)
            raw_data = f.read()
            num_records = len(raw_data) // record_size
            for i in range(num_records):
                chunk = raw_data[i * record_size : (i + 1) * record_size]
                unpacked = list(struct.unpack(fmt_str, chunk))
                if not any(isinstance(p, float) and math.isnan(p) for p in unpacked[:3]):
                    pcd.point_records.append(unpacked)
        elif pcd.data_type == "binary_compressed":
            header_bytes = f.read(8)
            if len(header_bytes) < 8:
                raise ValueError("Unexpected EOF in binary_compressed PCD header")
            compressed_size, uncompressed_size = struct.unpack("<II", header_bytes)
            compressed_data = f.read(compressed_size)
            uncompressed_data = decompress_lzf(compressed_data, uncompressed_size)
            field_arrays = []
            offset = 0
            n_pts = pcd.points
            for sz, tp in zip(pcd.sizes, pcd.types):
                field_bytes = sz * n_pts
                chunk = uncompressed_data[offset : offset + field_bytes]
                offset += field_bytes
                if tp == "F": code = "f" if sz == 4 else "d"
                elif tp == "I": code = {1: "b", 2: "h", 4: "i", 8: "q"}.get(sz, "i")
                elif tp == "U": code = {1: "B", 2: "H", 4: "I", 8: "Q"}.get(sz, "I")
                else: code = f"{sz}s"
                field_vals = struct.unpack(f"<{n_pts}{code}", chunk)
                field_arrays.append(field_vals)
            for i in range(n_pts):
                rec = [field_arrays[col][i] for col in range(len(field_arrays))]
                if not any(isinstance(p, float) and math.isnan(p) for p in rec[:3]):
                    pcd.point_records.append(rec)
        else:
            raise ValueError(f"Unsupported PCD DATA format: {pcd.data_type}.")

    pcd.points = len(pcd.point_records)
    pcd.width = pcd.points
    pcd.height = 1
    return pcd

def downsample_voxel(pcd: PCDData, target_budget: int) -> PCDData:
    n_pts = len(pcd.point_records)
    if target_budget <= 0 or n_pts <= target_budget:
        return pcd

    # Filter out invalid / NaN points for bbox calculation
    valid_coords = []
    for r in pcd.point_records:
        x, y, z = r[pcd.x_idx], r[pcd.y_idx], r[pcd.z_idx]
        if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
            valid_coords.append((x, y, z, r))

    if len(valid_coords) <= target_budget:
        out = PCDData()
        out.__dict__.update(pcd.__dict__)
        out.point_records = [r for _, _, _, r in valid_coords]
        out.points = len(out.point_records)
        out.width = out.points
        return out

    # Compute bounding box
    min_x = min(c[0] for c in valid_coords)
    max_x = max(c[0] for c in valid_coords)
    min_y = min(c[1] for c in valid_coords)
    max_y = max(c[1] for c in valid_coords)
    min_z = min(c[2] for c in valid_coords)
    max_z = max(c[2] for c in valid_coords)

    dx = max(max_x - min_x, 1e-4)
    dy = max(max_y - min_y, 1e-4)
    dz = max(max_z - min_z, 1e-4)
    volume = dx * dy * dz

    # Bisection search on voxel leaf size
    s_min = (volume / (target_budget * 50.0)) ** (1.0 / 3.0)
    s_max = (volume / max(target_budget * 0.05, 1.0)) ** (1.0 / 3.0)
    best_records = []
    best_diff = float("inf")

    def test_leaf(leaf: float) -> Tuple[List[List[float]], int]:
        grid: Dict[Tuple[int, int, int], List[float]] = {}
        inv_s = 1.0 / leaf
        for x, y, z, rec in valid_coords:
            ix = int(math.floor((x - min_x) * inv_s))
            iy = int(math.floor((y - min_y) * inv_s))
            iz = int(math.floor((z - min_z) * inv_s))
            key = (ix, iy, iz)
            if key not in grid:
                grid[key] = rec
        return list(grid.values()), len(grid)

    for _ in range(8):
        s_mid = 0.5 * (s_min + s_max)
        sampled, count = test_leaf(s_mid)
        diff = abs(count - target_budget)
        if diff < best_diff:
            best_diff = diff
            best_records = sampled

        # If count is within 15% of target budget, exit early
        if abs(count - target_budget) / target_budget < 0.15:
            break

        if count > target_budget:
            # Need coarser voxels (larger leaf size)
            s_min = s_mid
        else:
            # Need finer voxels (smaller leaf size)
            s_max = s_mid

    out_pcd = PCDData()
    out_pcd.__dict__.update(pcd.__dict__)
    out_pcd.point_records = best_records
    out_pcd.points = len(best_records)
    out_pcd.width = out_pcd.points
    out_pcd.height = 1
    return out_pcd

def write_pcd_ascii(pcd: PCDData, out_filepath: str) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(out_filepath)), exist_ok=True)
    with open(out_filepath, "w", encoding="ascii") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write(f"FIELDS {' '.join(pcd.fields if pcd.fields else ['x', 'y', 'z'])}\n")
        f.write(f"SIZE {' '.join(str(s) for s in (pcd.sizes if pcd.sizes else [4, 4, 4]))}\n")
        f.write(f"TYPE {' '.join(pcd.types if pcd.types else ['F', 'F', 'F'])}\n")
        f.write(f"COUNT {' '.join(str(c) for c in (pcd.counts if pcd.counts else [1, 1, 1]))}\n")
        f.write(f"WIDTH {pcd.points}\n")
        f.write("HEIGHT 1\n")
        f.write(f"VIEWPOINT {pcd.viewpoint}\n")
        f.write(f"POINTS {pcd.points}\n")
        f.write("DATA ascii\n")
        for rec in pcd.point_records:
            f.write(" ".join(f"{val:.6g}" if isinstance(val, float) else str(val) for val in rec) + "\n")

def main():
    parser = argparse.ArgumentParser(description="Adaptive Geometric PCD Downsampler for gem5 Benchmarking")
    parser.add_argument("--input", "-i", required=True, help="Path to input .pcd file")
    parser.add_argument("--output", "-o", required=True, help="Path to output .pcd file")
    parser.add_argument("--tier", choices=list(TIER_BUDGETS.keys()), default=None,
                        help="Pre-configured budget tier: sanity (~150), dev (~1000), eval (~5000), stress (~10000), full (100%)")
    parser.add_argument("--target-points", "-n", type=int, default=None,
                        help="Explicit target point count")

    args = parser.parse_args()

    budget = 0
    if args.tier:
        budget = TIER_BUDGETS[args.tier]
    elif args.target_points is not None:
        budget = args.target_points
    else:
        budget = 1000 # Default to dev tier

    if not os.path.exists(args.input):
        print(f"Error: Input PCD file not found: {args.input}", file=sys.stderr)
        sys.exit(1)

    pcd = parse_pcd(args.input)
    orig_points = pcd.points

    if budget > 0 and orig_points > budget:
        downsampled = downsample_voxel(pcd, budget)
    else:
        downsampled = pcd

    write_pcd_ascii(downsampled, args.output)
    print(f"PCD Downsampled: {orig_points} -> {downsampled.points} points (Target: {budget if budget > 0 else 'Full'}) saved to: {args.output}")

if __name__ == "__main__":
    main()
