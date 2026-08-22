#!/usr/bin/env python3
"""Synthetic SensorStream client — validates the receiver without an iPhone."""
import socket, struct, sys, time
import numpy as np
import sensor_pb2

HOST, PORT = "127.0.0.1", int(sys.argv[1]) if len(sys.argv) > 1 else 5678
W, H, CHUNK = 256, 192, 16384

def frames():
    # depth: synthetic ramp in millimetres
    d = sensor_pb2.SensorMessage()
    d.depth.timestamp = time.time_ns()
    d.depth.width, d.depth.height = W, H
    d.depth.encoding, d.depth.frame_id = "16UC1", "iphone_depth"
    d.depth.depth_data = (np.tile(np.linspace(300, 4800, W), (H, 1))
                          .astype("<u2").tobytes())
    yield d

    # pointcloud: XYZ float32, organized 256x192
    pc = sensor_pb2.SensorMessage()
    p = pc.pointcloud
    p.timestamp, p.frame_id = time.time_ns(), "iphone_depth"
    p.height, p.width = H, W
    p.point_step, p.row_step = 16, 16 * W
    p.is_dense = False
    for name, off in (("x", 0), ("y", 4), ("z", 8)):
        f = p.fields.add(); f.name, f.offset, f.datatype, f.count = name, off, 7, 1
    buf = np.zeros((H * W, 4), dtype="<f4")
    u, v = np.meshgrid(np.arange(W), np.arange(H))
    z = np.linspace(0.3, 4.8, W)[None, :].repeat(H, 0)
    buf[:, 0] = ((u - 128) * z / 210.0).ravel()
    buf[:, 1] = ((v - 96) * z / 210.0).ravel()
    buf[:, 2] = z.ravel()
    p.data = buf.tobytes()
    yield pc

s = socket.create_connection((HOST, PORT))
hdr = struct.Struct(">IIBI")
for mid, msg in enumerate(frames()):
    blob = msg.SerializeToString()
    parts = [blob[i:i + CHUNK] for i in range(0, len(blob), CHUNK)]
    for seq, part in enumerate(parts):
        s.sendall(hdr.pack(mid, seq, seq == len(parts) - 1, len(part)) + part)
    print(f"sent msg_id={mid} {len(blob)}B in {len(parts)} chunks")
time.sleep(0.5)
s.close()
