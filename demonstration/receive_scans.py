#!/usr/bin/env python3
"""
RVPoint Real-Time PCD Receiver Client (Mac Workstation)
======================================================
Connects to server_main.py running on the Orange Pi RV2 via high-speed TCP.
1. Automatically wipes demonstration/mains/* and demonstration/prcsd/* at launch.
2. Receives and saves both raw and processed PCD scans in real time.

Usage:
    python3 demonstration/receive_scans.py
    python3 demonstration/receive_scans.py --host 100.94.165.126 --port 9001
"""

import argparse
import os
import shutil
import socket
import struct
import sys
import time
from pathlib import Path

SYNC_MAGIC = b"RVP\x01"


def clean_local_directories(mains_dir: Path, prcsd_dir: Path):
    """Delete all files inside destination directories so the session starts fresh."""
    mains_dir.mkdir(parents=True, exist_ok=True)
    prcsd_dir.mkdir(parents=True, exist_ok=True)

    deleted_main = 0
    deleted_prcsd = 0

    for f in mains_dir.glob("*"):
        if f.is_file():
            try:
                f.unlink()
                deleted_main += 1
            except Exception as e:
                print(f"[Warning] Failed to remove {f}: {e}")

    for f in prcsd_dir.glob("*"):
        if f.is_file():
            try:
                f.unlink()
                deleted_prcsd += 1
            except Exception as e:
                print(f"[Warning] Failed to remove {f}: {e}")

    print(f"[{time.strftime('%H:%M:%S')}] Wiped old scans: {deleted_main} from {mains_dir}/ | {deleted_prcsd} from {prcsd_dir}/")


def read_exact(sock: socket.socket, n: int) -> bytes:
    """Read exactly n bytes from a TCP socket."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionResetError("Connection closed by Orange Pi server")
        buf.extend(chunk)
    return bytes(buf)


def main():
    repo_root = Path(__file__).resolve().parent.parent
    default_mains = repo_root / "demonstration" / "mains"
    default_prcsd = repo_root / "demonstration" / "prcsd"

    ap = argparse.ArgumentParser(
        description="RVPoint Live PCD Receiver (Mac Client for Orange Pi Stream)"
    )
    ap.add_argument(
        "--host",
        default="100.94.165.126",
        help="Orange Pi IP address (Tailscale or LAN, default: 100.94.165.126)",
    )
    ap.add_argument(
        "--port",
        type=int,
        default=9001,
        help="PCD sync port on Orange Pi (default: 9001)",
    )
    ap.add_argument(
        "--mains-dir",
        default=str(default_mains),
        help="Local directory for raw main PCD scans",
    )
    ap.add_argument(
        "--prcsd-dir",
        default=str(default_prcsd),
        help="Local directory for processed/clustered PCD scans",
    )
    ap.add_argument(
        "--no-clean",
        action="store_true",
        help="Do not wipe destination directories at launch",
    )
    args = ap.parse_args()

    mains_path = Path(args.mains_dir).resolve()
    prcsd_path = Path(args.prcsd_dir).resolve()

    print("=" * 76)
    print(" RVPoint Real-Time PCD Receiver Client (Mac Workstation)")
    print("=" * 76)
    print(f"  Target Orange Pi : {args.host}:{args.port}")
    print(f"  Raw destination  : {mains_path}")
    print(f"  Processed dest   : {prcsd_path}")
    print("=" * 76)

    # 1. DELETE OLD FILES IN mains/* AND prcsd/*
    if not args.no_clean:
        clean_local_directories(mains_path, prcsd_path)

    total_received = 0
    total_bytes = 0

    while True:
        sock = None
        try:
            print(f"\n[{time.strftime('%H:%M:%S')}] Connecting to Orange Pi sync server ({args.host}:{args.port})...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((args.host, args.port))
            sock.settimeout(None)  # Blocking for reliable stream reads
            print(f"[{time.strftime('%H:%M:%S')}] Connected! Streaming live PCD scans from Orange Pi...")

            while True:
                # Read Magic Header (4 bytes)
                magic = read_exact(sock, 4)
                if magic != SYNC_MAGIC:
                    print(f"[{time.strftime('%H:%M:%S')}] Protocol desync (bad magic {magic!r}), reconnecting...")
                    break

                # Read category (1 byte) & name_len (2 bytes uint16)
                cat_raw, name_len = struct.unpack("!BH", read_exact(sock, 3))
                filename = read_exact(sock, name_len).decode("utf-8")

                # Read data_len (4 bytes uint32) & data
                (data_len,) = struct.unpack("!I", read_exact(sock, 4))
                pcd_data = read_exact(sock, data_len)

                # Route to appropriate destination
                if cat_raw == 1:
                    dest_file = mains_path / filename
                    category_label = "MAIN"
                elif cat_raw == 2:
                    dest_file = prcsd_path / filename
                    category_label = "PROCESSED"
                elif cat_raw == 3:
                    dest_file = prcsd_path / filename
                    category_label = "CLUSTERS"
                else:
                    dest_file = prcsd_path / filename
                    category_label = f"CAT_{cat_raw}"

                # Atomically write file
                temp_file = dest_file.with_suffix(".tmp")
                with open(temp_file, "wb") as f:
                    f.write(pcd_data)
                temp_file.replace(dest_file)

                total_received += 1
                total_bytes += len(pcd_data)
                size_kb = len(pcd_data) / 1024.0

                print(
                    f"[{time.strftime('%H:%M:%S')}] Received #{total_received:04d} "
                    f"[{category_label:<9}] {filename:<32} ({size_kb:6.1f} KB) -> "
                    f"{dest_file.parent.name}/{filename}"
                )

        except (ConnectionRefusedError, TimeoutError, OSError) as e:
            print(f"[{time.strftime('%H:%M:%S')}] Connection error: {e}. Retrying in 2 seconds (Ctrl+C to stop)...")
            time.sleep(2)
        except ConnectionResetError as e:
            print(f"[{time.strftime('%H:%M:%S')}] Disconnected: {e}. Reconnecting in 2 seconds...")
            time.sleep(2)
        except KeyboardInterrupt:
            print(f"\n[{time.strftime('%H:%M:%S')}] Exiting. Total files received: {total_received} ({total_bytes/1024/1024:.2f} MB)")
            break
        finally:
            if sock:
                try:
                    sock.close()
                except Exception:
                    pass


if __name__ == "__main__":
    main()
