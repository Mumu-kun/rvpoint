#!/usr/bin/env python3
"""
sensorstream_rx.py — SensorStream (iOS) WiFi receiver, ROS-free.

Wire format reverse-engineered from sensorstream_driver/src/protocol_handler.cpp:

  Host = TCP server, phone = client.
  Every phone->host chunk:

      +----------+----------+----------+-----------+---------------------+
      | msg_id   | sequence | is_last  | data_size | payload             |
      | 4B BE    | 4B BE    | 1B       | 4B BE     | data_size bytes     |
      +----------+----------+----------+-----------+---------------------+
       <---------------- 13 byte header ---------->

  Chunks with the same msg_id are reassembled in `sequence` order; once the
  chunk carrying is_last=1 has arrived AND seq 0..max are all present, the
  concatenated payload is a serialized sensor.SensorMessage protobuf.

Why chunking exists: a 256x192 XYZRGB PointCloud2 is ~786 KB, far larger than
one comfortable write; the app splits it so a slow reader cannot stall the
sensor pipeline. Note the reassembly buffer is an unbounded dict keyed by
msg_id — a malicious/buggy peer that never sends is_last will grow it without
limit. MAX_PENDING below is the bound the C++ driver lacks.
"""

import argparse
import os
import socket
import struct
import sys
import time
from collections import OrderedDict

import numpy as np
import sensor_pb2

HEADER = struct.Struct(">IIBI")      # msg_id, sequence, is_last, data_size
HEADER_SIZE = HEADER.size            # 13
MAX_PAYLOAD = 8 << 20               # 8 MiB sanity cap per chunk
MAX_PENDING = 16                     # max concurrent in-flight messages
RECV_BUF = 1 << 16


class Reassembler:
    """LRU-bounded chunk reassembly. One instance per TCP connection."""

    def __init__(self):
        self.pending = OrderedDict()   # msg_id -> {"chunks": {seq: bytes}, "last": bool}

    def push(self, msg_id, seq, is_last, payload):
        entry = self.pending.get(msg_id)
        if entry is None or seq == 0:
            # seq == 0 always begins a new message: evict any stale partial.
            entry = {"chunks": {}, "last": False}
            self.pending[msg_id] = entry
        self.pending.move_to_end(msg_id)

        entry["chunks"][seq] = payload
        entry["last"] = entry["last"] or is_last

        # Bound memory: drop the oldest incomplete message.
        while len(self.pending) > MAX_PENDING:
            stale_id, _ = self.pending.popitem(last=False)
            print(f"[warn] dropped stale incomplete msg_id={stale_id}", file=sys.stderr)

        if not entry["last"]:
            return None
        n = max(entry["chunks"]) + 1
        if len(entry["chunks"]) != n:
            return None                       # hole in the sequence, wait
        del self.pending[msg_id]
        return b"".join(entry["chunks"][i] for i in range(n))


def pointcloud_to_xyz(pc):
    """
    Zero-copy-ish view of a sensor.PointCloud2 payload as an (N,3) float32 array.

    We do NOT assume x,y,z are the first three fields or that point_step == 12;
    we read the declared offsets. datatype 7 == FLOAT32 in the ROS PointField
    enum. Anything else we refuse rather than silently misinterpret.
    """
    off = {}
    for f in pc.fields:
        if f.name in ("x", "y", "z"):
            if f.datatype != 7:
                raise ValueError(f"field {f.name} is not FLOAT32 (datatype={f.datatype})")
            off[f.name] = f.offset
    if len(off) != 3:
        raise ValueError(f"missing xyz fields, got {[f.name for f in pc.fields]}")
    if pc.is_bigendian:
        raise ValueError("big-endian payload not handled")

    step = pc.point_step
    n = len(pc.data) // step
    raw = np.frombuffer(pc.data, dtype=np.uint8, count=n * step).reshape(n, step)
    # Gather the 12 bytes of x,y,z per point, then reinterpret as float32.
    idx = np.concatenate([np.arange(off[c], off[c] + 4) for c in ("x", "y", "z")])
    return raw[:, idx].copy().view(np.float32).reshape(n, 3)


def write_pcd(path, xyz, width, height, binary=True, dense=True):
    """
    Write a PCD.

    dense=True  (default, and what rvpoint needs)
        NaN points are dropped and HEIGHT is 1. This is the unorganized layout
        that rvpoint/simple_pcd_loader.h expects: it ignores HEIGHT entirely and
        flattens everything into std::vector<PointXYZ>, with no NaN guard.
        Feeding it NaN means (int)floor(NaN) in the voxel key path -- undefined
        behaviour, and it differs by target: x86 gives INT_MIN, RVV's vfcvt.rtz
        saturates to INT_MAX. That alone would break any x86-vs-RISC-V
        correctness comparison.

    dense=False (organized)
        Keeps every pixel in row-major order, NaN included, HEIGHT = sensor rows.
        Only useful once a loader actually reads HEIGHT and exploits the grid for
        O(1) image-space neighbour lookups. Do not feed this to rvpoint today.
    """
    n_in = width * height
    if xyz.shape[0] != n_in:
        raise ValueError(f"expected {n_in} points for {width}x{height}, got {xyz.shape[0]}")

    if dense:
        pts = xyz[np.isfinite(xyz).all(axis=1)]
        w, h = pts.shape[0], 1
    else:
        pts, w, h = xyz, width, height

    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z\n"
        "SIZE 4 4 4\n"
        "TYPE F F F\n"
        "COUNT 1 1 1\n"
        f"WIDTH {w}\n"
        f"HEIGHT {h}\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {w * h}\n"
        f"DATA {'binary' if binary else 'ascii'}\n"
    ).encode("ascii")

    with open(path, "wb") as fh:
        fh.write(header)
        if binary:
            # Raw little-endian dump == in-memory layout, so a C++ reader can
            # mmap() and cast to float* with zero parsing.
            fh.write(np.ascontiguousarray(pts, dtype="<f4").tobytes())
        else:
            np.savetxt(fh, pts, fmt="%.6f")
    return n_in, pts.shape[0]


def write_ply(path, xyz):
    finite = xyz[np.isfinite(xyz).all(axis=1)]
    with open(path, "wb") as fh:
        fh.write(
            f"ply\nformat binary_little_endian 1.0\n"
            f"element vertex {len(finite)}\n"
            f"property float x\nproperty float y\nproperty float z\n"
            f"end_header\n".encode()
        )
        fh.write(finite.astype("<f4").tobytes())
    return len(finite)


class Stats:
    def __init__(self):
        self.counts = {}
        self.bytes = 0
        self.t0 = time.monotonic()

    def tick(self, kind, nbytes):
        self.counts[kind] = self.counts.get(kind, 0) + 1
        self.bytes += nbytes
        dt = time.monotonic() - self.t0
        if dt >= 1.0:
            rates = " ".join(f"{k}={v/dt:5.1f}Hz" for k, v in sorted(self.counts.items()))
            print(f"\r{rates}  {self.bytes/dt/1e6:5.2f} MB/s   ", end="", flush=True)
            self.counts.clear()
            self.bytes = 0
            self.t0 = time.monotonic()


def handle_message(blob, stats, args, state):
    msg = sensor_pb2.SensorMessage()
    msg.ParseFromString(blob)                 # raises on malformed input
    kind = msg.WhichOneof("sensor_data")
    stats.tick(kind or "unknown", len(blob))

    if kind == "pointcloud" and state["saved"] < args.frames:
        pc = msg.pointcloud
        if pc.width * pc.height == 0:
            # ARKit emits empty clouds until the session warms up. Counting one
            # of these against --frames would end a capture before it began.
            return
        xyz = pointcloud_to_xyz(pc)
        idx = state["saved"]
        stem = os.path.join(args.outdir, f"cloud_{idx:04d}")

        if args.format in ("pcd", "both"):
            n, finite = write_pcd(f"{stem}.pcd", xyz, pc.width, pc.height,
                                  binary=not args.ascii, dense=not args.organized)
        if args.format in ("ply", "both"):
            finite = write_ply(f"{stem}.ply", xyz)
            n = xyz.shape[0]

        state["saved"] += 1
        if idx == 0:
            src_layout = "dense" if pc.is_dense else "organized"
            print(f"\n[ok] source: {pc.width}x{pc.height} {src_layout}, "
                  f"point_step={pc.point_step}, "
                  f"fields={[f.name for f in pc.fields]}")
        layout = "organized" if args.organized else "dense"
        print(f"[ok] {stem}  {finite} points written ({n - finite} NaN "
              f"{'kept' if args.organized else 'dropped'}, {layout})")
        if state["saved"] == args.frames:
            print(f"[done] captured {args.frames} frame(s) in {args.outdir}/")

    elif kind == "depth" and state["depth_logged"] is False:
        d = msg.depth
        arr = np.frombuffer(d.depth_data, dtype="<u2").reshape(d.height, d.width)
        valid = arr[arr > 0]
        print(f"\n[ok] depth {d.width}x{d.height} enc={d.encoding!r} "
              f"frame_id={d.frame_id!r} range={valid.min() if valid.size else 0}"
              f"..{valid.max() if valid.size else 0} mm")
        state["depth_logged"] = True


def serve(args):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.port))
    srv.listen(1)

    ip = local_ip()
    print(f"listening on 0.0.0.0:{args.port}   pair the app with:  {ip}:{args.port}")
    if args.qr:
        try:
            import qrcode
            q = qrcode.QRCode(border=1)
            q.add_data(f"{ip}:{args.port}")
            q.print_ascii(invert=True)
        except ImportError:
            print("(pip install qrcode for the pairing QR)")

    # Capture state lives across connections: the app reconnects (settings
    # changes, backgrounding, WiFi hiccups) and a per-connection counter would
    # restart at cloud_0000 and silently overwrite earlier frames.
    state = {"saved": 0, "depth_logged": False}

    while True:
        conn, peer = srv.accept()
        print(f"\n[conn] {peer[0]}:{peer[1]}")
        try:
            pump(conn, args, state)
        except Exception as e:                # noqa: BLE001 - keep server alive
            print(f"\n[err] {type(e).__name__}: {e}", file=sys.stderr)
        finally:
            conn.close()
            print(f"\n[conn] closed {peer[0]}")


def pump(conn, args, state):
    # Reassembly is per-connection (message ids restart), capture state is not.
    asm = Reassembler()
    stats = Stats()
    buf = bytearray()

    while True:
        data = conn.recv(RECV_BUF)
        if not data:
            return
        buf += data
        while len(buf) >= HEADER_SIZE:
            msg_id, seq, is_last, size = HEADER.unpack_from(buf, 0)
            if size > MAX_PAYLOAD:
                raise ValueError(f"implausible chunk size {size}; stream desynced")
            if len(buf) < HEADER_SIZE + size:
                break                          # partial chunk, wait for more
            payload = bytes(buf[HEADER_SIZE:HEADER_SIZE + size])
            del buf[:HEADER_SIZE + size]
            blob = asm.push(msg_id, seq, bool(is_last), payload)
            if blob is not None:
                handle_message(blob, stats, args, state)


def local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))             # no packet sent; just picks the route
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=5678)
    ap.add_argument("--outdir", default="capture", help="directory for dumped clouds")
    ap.add_argument("--format", choices=["pcd", "ply", "both"], default="pcd")
    ap.add_argument("--frames", type=int, default=1,
                    help="how many pointcloud frames to dump (0 = stream only)")
    ap.add_argument("--organized", action="store_true",
                    help="keep the 256x192 grid and NaN placeholders "
                         "(rvpoint's loader cannot consume this yet)")
    ap.add_argument("--ascii", action="store_true",
                    help="ASCII PCD instead of binary (debuggable, ~5x larger)")
    ap.add_argument("--qr", action="store_true", help="print pairing QR")
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    try:
        serve(args)
    except KeyboardInterrupt:
        print("\n[bye]")