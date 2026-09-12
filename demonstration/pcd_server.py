#!/usr/bin/env python3
"""
pcd_server.py — receive iPhone LiDAR depth frames over TCP and save a PCD
file every N seconds.

  python3 pcd_server.py                    # PCD every 5 s into ./scans
  python3 pcd_server.py --interval 5 --binary
  python3 pcd_server.py --accumulate       # stack all frames per interval (denser)
  python3 pcd_server.py --fake             # test Mac side without the iPhone
"""

import argparse
import os
import socket
import struct
import time

import numpy as np

MAGIC = b"LDP1"
STATS = {"bytes": 0}
HEADER = struct.Struct("<III4f16f")  # frame_idx, w, h, fx, fy, cx, cy, pose(4x4)


def local_ips():
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


def recv_frame(conn, buf):
    """Parse next complete frame; returns dict or None on timeout."""
    while True:
        i = buf.find(MAGIC)
        if i == -1:
            del buf[:max(0, len(buf) - 3)]
        elif i > 0:
            del buf[:i]

        if len(buf) >= 4 + HEADER.size:
            frame_idx, w, h, fx, fy, cx, cy, *pose = HEADER.unpack_from(buf, 4)
            need = w * h * 5  # float32 depth + uint8 confidence
            if len(buf) >= 4 + HEADER.size + need:
                start = 4 + HEADER.size
                depth = np.frombuffer(buf, "<f4", w * h, start).reshape(h, w).copy()
                conf = np.frombuffer(buf, np.uint8, w * h, start + w * h * 4).reshape(h, w).copy()
                del buf[: start + need]
                return {"idx": frame_idx, "w": w, "h": h,
                        "fx": fx, "fy": fy, "cx": cx, "cy": cy,
                        "pose": np.asarray(pose, np.float64).reshape(4, 4),
                        "depth": depth, "conf": conf}

        try:
            chunk = conn.recv(262144)
        except socket.timeout:
            return None
        if not chunk:
            raise ConnectionError("iPhone disconnected")
        first = not buf and len(chunk) < 262144   # very first chunk of a connection
        buf += chunk
        STATS["bytes"] += len(chunk)


def unproject(frame, args):
    """Depth map -> Nx3 float32 points (camera frame, then world via pose)."""
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

    if args.rot90 == 1:   pts = np.stack([-pts[:, 1],  pts[:, 0], pts[:, 2]], axis=1)
    elif args.rot90 == 2: pts = np.stack([-pts[:, 0], -pts[:, 1], pts[:, 2]], axis=1)
    elif args.rot90 == 3: pts = np.stack([ pts[:, 1], -pts[:, 0], pts[:, 2]], axis=1)
    if args.flip_x: pts[:, 0] *= -1
    if args.flip_y: pts[:, 1] *= -1
    if args.flip_z: pts[:, 2] *= -1

    if not args.cam_frame:
        pose = frame["pose"]
        pts = pts @ pose[:3, :3].T + pose[:3, 3]
    return pts.astype(np.float32)


def write_pcd_ascii(path, pts):
    n = pts.shape[0]
    with open(path, "w") as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n"
                "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
                f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA ascii\n")
        np.savetxt(f, pts, fmt="%.3f")


def write_pcd_binary(path, pts):
    n = pts.shape[0]
    header = ("# .PCD v0.7 - Point Cloud Data file format\n"
              "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
              f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n").encode()
    with open(path, "wb") as f:
        f.write(header)
        f.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())


def fake_frame(t):
    """Synthetic 'wavy wall' at ~1.5 m, for testing without an iPhone."""
    w, h = 256, 192
    u = np.arange(w, dtype=np.float32)[None, :].repeat(h, 0)
    v = np.arange(h, dtype=np.float32)[:, None].repeat(w, 1)
    z = 1.5 + 0.2 * np.sin((u + v) / 30.0 + t)
    return {"idx": int(t * 10), "w": w, "h": h,
            "fx": 230.0, "fy": 230.0, "cx": w / 2, "cy": h / 2,
            "pose": np.eye(4), "depth": z.astype(np.float32),
            "conf": np.full((h, w), 2, np.uint8)}


class IntervalWriter:
    def __init__(self, args):
        self.args = args
        self.t0 = time.time()
        self.latest = None
        self.frames = 0
        self.pending = []

    def add(self, frame):
        self.latest = frame
        self.frames += 1
        if self.args.accumulate:
            p = unproject(frame, self.args)
            if len(p):
                self.pending.append(p)

    def reset(self):
        self.latest, self.frames, self.pending = None, 0, []
        self.t0 = time.time()

    def maybe_write(self):
        if self.frames == 0 or time.time() - self.t0 < self.args.interval:
            return
        if self.args.accumulate:
            pts = np.vstack(self.pending) if self.pending else np.zeros((0, 3), np.float32)
            src = f"{self.frames} frames"
        else:
            pts = unproject(self.latest, self.args)
            src = f"frame #{self.latest['idx']}"
        if len(pts) == 0:
            print(f"[{time.strftime('%H:%M:%S')}] no valid points in this interval")
        else:
            if len(pts) > self.args.max_pts:
                sel = np.linspace(0, len(pts) - 1, self.args.max_pts).astype(np.int64)
                pts = pts[sel]
            path = os.path.join(self.args.out, f"scan_{time.strftime('%H%M%S')}.pcd")
            (write_pcd_binary if self.args.binary else write_pcd_ascii)(path, pts)
            print(f"[{time.strftime('%H:%M:%S')}] wrote {path}  ({len(pts):,} pts, {src})")
        self.reset()


def main():
    ap = argparse.ArgumentParser(description="iPhone LiDAR -> PCD server")
    ap.add_argument("--port", type=int, default=9000)
    ap.add_argument("--interval", type=float, default=5.0, help="seconds between PCD files")
    ap.add_argument("--out", default="scans")
    ap.add_argument("--binary", action="store_true", help="binary PCD instead of ASCII")
    ap.add_argument("--accumulate", action="store_true", help="stack frames per interval")
    ap.add_argument("--max-pts", type=int, default=500_000)
    ap.add_argument("--min-conf", type=int, default=1, choices=(0, 1, 2), help="0 low, 1 medium, 2 high")
    ap.add_argument("--min-range", type=float, default=0.1)
    ap.add_argument("--max-range", type=float, default=6.0)
    ap.add_argument("--cam-frame", action="store_true", help="keep points in camera frame")
    ap.add_argument("--rot90", type=int, default=0, choices=(0, 1, 2, 3))
    ap.add_argument("--flip-x", action="store_true")
    ap.add_argument("--flip-y", action="store_true")
    ap.add_argument("--flip-z", action="store_true")
    ap.add_argument("--fake", action="store_true", help="synthetic data, no iPhone needed")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    print("LiDAR PCD server — enter one of these IPs in the iPhone app:")
    for ip in local_ips():
        print(f"    {ip}")
    print(f"Listening on 0.0.0.0:{args.port} | PCD every {args.interval:g}s -> {args.out}/")

    writer = IntervalWriter(args)

    if args.fake:
        print("FAKE mode: synthetic data (validates everything except the iPhone).")
        t = 0.0
        while True:
            writer.add(fake_frame(t))
            writer.maybe_write()
            t += 0.1
            time.sleep(0.1)
        return

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)

    while True:
        print("Waiting for the iPhone app to connect…")
        conn, addr = srv.accept()
        conn.settimeout(0.2)
        print(f"iPhone connected from {addr[0]}")
        STATS["bytes"] = 0
        buf = bytearray()
        try:
            while True:
                frame = recv_frame(conn, buf)
                if frame is not None:
                    writer.add(frame)
                writer.maybe_write()
        except OSError as e:
            print(f"connection lost ({e}) — waiting for reconnect…")
            writer.reset()
        finally:
            conn.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")