#!/usr/bin/env python3
"""
web_viewer.py — Real-Time 3D WebGL Point Cloud Visualizer for RVPoint Perception Stream.

Connects to the Orange Pi RV2 server (port 9001), receives live segmented & clustered
point clouds over TCP, and serves a high-performance 60 FPS Three.js 3D dashboard
accessible via browser at http://localhost:8080.

Zero external Python dependencies (uses Python standard library http.server and socket).

Features:
  - Instant WebGL hardware-accelerated 3D point cloud rendering.
  - Interactive OrbitControls (Mouse drag rotate, right-click pan, scroll zoom).
  - Ground plane toggle, point size control, and cluster isolation.
  - Real-time RVV 1.0 hardware acceleration metrics (compute ms, wall ms, cluster counts).
  - One-click PCD snapshot download directly to host browser.

Usage:
  # Connect to Orange Pi over Tailscale:
  python3 demonstration/web_viewer.py --host 100.94.165.126

  # Connect to Orange Pi over Local Hotspot:
  python3 demonstration/web_viewer.py --host 172.20.10.2

  # Custom web port:
  python3 demonstration/web_viewer.py --host 100.94.165.126 --web-port 8088
"""

import argparse
import http.server
import json
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

POINTS_STREAM_MAGIC = b"RVPT"
SYNC_MAGIC = b"RVP\x01"
HEADER_FORMAT = "!4sIIIIff"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)  # 28 bytes


class StreamBridge:
    """Connects to Orange Pi TCP broadcaster and holds the latest binary frame buffer."""

    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.latest_raw_packet: Optional[bytes] = None
        self.latest_frame_meta = {
            "connected": False,
            "frame_idx": 0,
            "point_count": 0,
            "ground_count": 0,
            "clusters_count": 0,
            "compute_ms": 0.0,
            "wall_ms": 0.0,
            "timestamp": 0.0,
        }
        self.lock = threading.Lock()
        self.running = True
        self.frames_received = 0
        self._thread = threading.Thread(target=self._network_loop, daemon=True)
        self._thread.start()

    def get_latest_data(self) -> Tuple[Optional[bytes], dict]:
        with self.lock:
            return self.latest_raw_packet, dict(self.latest_frame_meta)

    def _network_loop(self):
        while self.running:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(3.0)

            try:
                sock.connect((self.host, self.port))
                with self.lock:
                    self.latest_frame_meta["connected"] = True
                print(f"[Bridge Connected] Receiving RVPoint stream from {self.host}:{self.port}")
            except Exception:
                with self.lock:
                    self.latest_frame_meta["connected"] = False
                time.sleep(2.0)
                continue

            buf = bytearray()
            try:
                while self.running:
                    idx_pt = buf.find(POINTS_STREAM_MAGIC)
                    idx_file = buf.find(SYNC_MAGIC)

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
                        payload_size = pt_cnt * 16
                        total_size = HEADER_SIZE + payload_size

                        if len(buf) < total_size:
                            chunk = sock.recv(max(65536, total_size - len(buf)))
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        raw_packet = bytes(buf[:total_size])
                        del buf[:total_size]

                        with self.lock:
                            self.latest_raw_packet = raw_packet
                            self.latest_frame_meta.update({
                                "connected": True,
                                "frame_idx": frame_idx,
                                "point_count": pt_cnt,
                                "ground_count": ground_cnt,
                                "clusters_count": cl_cnt,
                                "compute_ms": round(comp_ms, 2),
                                "wall_ms": round(wall_ms, 2),
                                "timestamp": time.time(),
                            })
                        self.frames_received += 1

                    elif buf.startswith(SYNC_MAGIC):
                        if len(buf) < 7:
                            chunk = sock.recv(65536)
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        _, name_len = struct.unpack_from("!BH", buf, 4)
                        file_hdr = 7 + name_len + 4
                        if len(buf) < file_hdr:
                            chunk = sock.recv(65536)
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        data_len = struct.unpack_from("!I", buf, 7 + name_len)[0]
                        total_f_size = file_hdr + data_len
                        if len(buf) < total_f_size:
                            chunk = sock.recv(max(65536, total_f_size - len(buf)))
                            if not chunk:
                                break
                            buf += chunk
                            continue

                        del buf[:total_f_size]

            except socket.timeout:
                continue
            except Exception:
                pass
            finally:
                with self.lock:
                    self.latest_frame_meta["connected"] = False
                sock.close()
                time.sleep(1.0)


HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>RVV PCL LIVE — Orange Pi RV2 (RVV 1.0)</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Oswald:wght@400;500;600;700&display=swap" rel="stylesheet">
  <style>
    :root {
      --c-deep-blue: #003049;
      --c-dark-red: #780000;
      --c-vivid-red: #c1121f;
      --c-cream: #fdf0d5;
      --c-steel-blue: #669bbc;
      --bg-dark: #001726;
      --panel-bg: rgba(0, 48, 73, 0.92);
      --panel-border: rgba(102, 155, 188, 0.35);
      --font-family: 'Oswald', sans-serif;
    }

    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
      user-select: none;
    }

    body, html {
      width: 100%;
      height: 100%;
      overflow: hidden;
      background-color: var(--bg-dark);
      font-family: var(--font-family);
      font-weight: 400;
      color: var(--c-cream);
    }

    #viewport {
      width: 100%;
      height: 100%;
      position: absolute;
      top: 0;
      left: 0;
      z-index: 1;
    }

    /* Floating Panels */
    .glass-panel {
      position: absolute;
      z-index: 10;
      background: var(--panel-bg);
      backdrop-filter: blur(16px);
      -webkit-backdrop-filter: blur(16px);
      border: 1px solid var(--panel-border);
      border-radius: 8px;
      box-shadow: 0 10px 30px rgba(0, 0, 0, 0.6);
      padding: 14px 18px;
      pointer-events: auto;
    }

    /* Top Left Header & Status */
    #header-panel {
      top: 20px;
      left: 20px;
      min-width: 280px;
    }

    .brand-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 10px;
    }

    .brand-title {
      font-size: 20px;
      font-weight: 700;
      letter-spacing: 1.5px;
      color: var(--c-cream);
      text-transform: uppercase;
      display: flex;
      align-items: center;
      gap: 8px;
    }

    .status-badge {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 10px;
      border-radius: 4px;
      font-size: 11px;
      font-weight: 500;
      letter-spacing: 1px;
      text-transform: uppercase;
    }

    .status-badge.connected {
      background: rgba(102, 155, 188, 0.18);
      border: 1px solid var(--c-steel-blue);
      color: var(--c-cream);
    }

    .status-badge.disconnected {
      background: var(--c-dark-red);
      border: 1px solid var(--c-vivid-red);
      color: var(--c-cream);
    }

    .status-dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background-color: var(--c-steel-blue);
    }

    .connected .status-dot {
      background-color: var(--c-steel-blue);
      box-shadow: 0 0 8px var(--c-steel-blue);
    }

    .disconnected .status-dot {
      background-color: var(--c-vivid-red);
      box-shadow: 0 0 8px var(--c-vivid-red);
    }

    .meta-row {
      display: flex;
      flex-direction: column;
      gap: 6px;
      font-size: 13px;
      color: var(--c-steel-blue);
      border-top: 1px solid rgba(102, 155, 188, 0.2);
      padding-top: 10px;
      letter-spacing: 0.5px;
    }

    .meta-item b {
      color: var(--c-cream);
      font-weight: 600;
      margin-left: 4px;
    }

    /* Metric Cards Grid (Top Right) */
    #metrics-panel {
      top: 20px;
      right: 20px;
      display: flex;
      gap: 10px;
    }

    .metric-card {
      min-width: 96px;
      text-align: center;
      padding: 8px 12px;
      background: rgba(0, 30, 48, 0.7);
      border: 1px solid rgba(102, 155, 188, 0.25);
      border-radius: 6px;
    }

    .metric-label {
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 1px;
      color: var(--c-steel-blue);
      font-weight: 500;
      margin-bottom: 2px;
    }

    .metric-value {
      font-size: 20px;
      font-weight: 700;
      color: var(--c-cream);
      letter-spacing: 0.5px;
    }

    .metric-value.blue { color: var(--c-steel-blue); }
    .metric-value.red { color: var(--c-vivid-red); }

    /* Bottom Floating Controls */
    #controls-panel {
      bottom: 24px;
      left: 50%;
      transform: translateX(-50%);
      display: flex;
      align-items: center;
      gap: 16px;
      padding: 10px 20px;
      flex-wrap: wrap;
      justify-content: center;
    }

    .control-group {
      display: flex;
      align-items: center;
      gap: 10px;
      font-size: 14px;
      font-weight: 500;
      letter-spacing: 0.8px;
      text-transform: uppercase;
      color: var(--c-cream);
    }

    .toggle-switch {
      position: relative;
      width: 38px;
      height: 20px;
      background: #002235;
      border: 1px solid var(--c-steel-blue);
      border-radius: 20px;
      cursor: pointer;
      transition: background 0.2s;
    }

    .toggle-switch.active {
      background: var(--c-vivid-red);
      border-color: var(--c-dark-red);
    }

    .toggle-thumb {
      position: absolute;
      top: 2px;
      left: 2px;
      width: 14px;
      height: 14px;
      background: var(--c-cream);
      border-radius: 50%;
      transition: transform 0.2s;
    }

    .toggle-switch.active .toggle-thumb {
      transform: translateX(18px);
    }

    /* Buttons: STRICTLY SOLID, ZERO GRADIENTS */
    .btn {
      background-color: var(--c-deep-blue);
      border: 1px solid var(--c-steel-blue);
      color: var(--c-cream);
      padding: 8px 16px;
      border-radius: 6px;
      font-family: var(--font-family);
      font-size: 13px;
      font-weight: 500;
      letter-spacing: 1px;
      text-transform: uppercase;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: background-color 0.15s ease, border-color 0.15s ease, transform 0.1s ease;
      outline: none;
    }

    .btn:hover {
      background-color: var(--c-steel-blue);
      color: var(--c-deep-blue);
      border-color: var(--c-cream);
      transform: translateY(-1px);
    }

    .btn:active {
      transform: scale(0.97);
    }

    .btn-primary {
      background-color: var(--c-vivid-red);
      border: 1px solid var(--c-dark-red);
      color: var(--c-cream);
      font-weight: 700;
    }

    .btn-primary:hover {
      background-color: var(--c-dark-red);
      border-color: var(--c-vivid-red);
      color: var(--c-cream);
      transform: translateY(-1px);
    }

    input[type="range"] {
      -webkit-appearance: none;
      width: 80px;
      height: 4px;
      background: #002235;
      border: 1px solid rgba(102, 155, 188, 0.4);
      border-radius: 2px;
      outline: none;
    }

    input[type="range"]::-webkit-slider-thumb {
      -webkit-appearance: none;
      width: 14px;
      height: 14px;
      border-radius: 50%;
      background: var(--c-vivid-red);
      border: 2px solid var(--c-cream);
      cursor: pointer;
    }

    #size-val {
      font-size: 13px;
      font-weight: 600;
      color: var(--c-cream);
      min-width: 44px;
    }

    /* Hint text */
    #hint {
      position: absolute;
      bottom: 24px;
      right: 24px;
      z-index: 5;
      font-size: 12px;
      letter-spacing: 0.5px;
      color: rgba(102, 155, 188, 0.5);
      text-align: right;
      line-height: 1.5;
    }
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
</head>
<body>
  <div id="viewport"></div>

  <!-- Header Info Panel -->
  <div id="header-panel" class="glass-panel">
    <div class="brand-row">
      <div class="brand-title">
        <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><path d="M21 16V8a2 2 0 0 0-1-1.73l-7-4a2 2 0 0 0-2 0l-7 4A2 2 0 0 0 3 8v8a2 2 0 0 0 1 1.73l7 4a2 2 0 0 0 2 0l7-4A2 2 0 0 0 21 16z"></path><polyline points="3.27 6.96 12 12.01 20.73 6.96"></polyline><line x1="12" y1="22.08" x2="12" y2="12"></line></svg>
        RVV PCL LIVE
      </div>
      <div id="status-badge" class="status-badge disconnected">
        <span class="status-dot"></span>
        <span id="status-text">Connecting</span>
      </div>
    </div>
    <div class="meta-row">
      <div class="meta-item">Hardware: <b>Orange Pi RV2</b></div>
      <div class="meta-item">Architecture: <b>RVV 1.0 (Vector)</b></div>
    </div>
  </div>

  <!-- Metrics Grid -->
  <div id="metrics-panel" class="glass-panel">
    <div class="metric-card">
      <div class="metric-label">Frame</div>
      <div id="metric-frame" class="metric-value">#0</div>
    </div>
    <div class="metric-card">
      <div class="metric-label">Points</div>
      <div id="metric-points" class="metric-value blue">0</div>
    </div>
    <div class="metric-card">
      <div class="metric-label">Clusters</div>
      <div id="metric-clusters" class="metric-value red">0</div>
    </div>
    <div class="metric-card">
      <div class="metric-label">Ground Pts</div>
      <div id="metric-ground" class="metric-value">0</div>
    </div>
    <div class="metric-card">
      <div class="metric-label">RVV Compute</div>
      <div id="metric-compute" class="metric-value blue">0.0 ms</div>
    </div>
  </div>

  <!-- Bottom Floating Controls -->
  <div id="controls-panel" class="glass-panel">
    <div class="control-group">
      <span>Ground Plane:</span>
      <div id="toggle-ground" class="toggle-switch active" title="Toggle ground plane points on/off">
        <div class="toggle-thumb"></div>
      </div>
    </div>

    <div class="control-group">
      <span>Point Size:</span>
      <input type="range" id="size-slider" min="1" max="10" step="0.5" value="3.0">
      <span id="size-val">3.0 px</span>
    </div>

    <button id="btn-reset" class="btn" title="Center camera on point cloud">Reset Camera</button>
    <button id="btn-snapshot" class="btn btn-primary" title="Download current frame as .PCD file">Save PCD Snapshot</button>
  </div>

  <div id="hint">
    Left Mouse: Rotate View<br>
    Right Mouse: Pan View<br>
    Scroll: Zoom In/Out
  </div>

  <script>
    // State management
    const state = {
      showGround: true,
      pointSize: 3.0,
      lastFrameIdx: -1,
      currentPoints: null,
      currentColors: null,
      groundCount: 0,
      cameraInitialized: false,
    };

    // ── 1. THREE.JS INITIALIZATION ──────────────────────────────────────────
    const container = document.getElementById('viewport');
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x001726);

    const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.01, 100.0);
    // Standard 3D right-handed orientation: +Y is UP (matching ARKit and standard PCD viewers)
    camera.up.set(0, 1, 0);
    camera.position.set(0, 0.5, 3.0);

    const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.setSize(window.innerWidth, window.innerHeight);
    container.appendChild(renderer.domElement);

    const controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0, -2.5);
    controls.update();

    // Subtle horizontal coordinate ground grid in the X-Z plane (Y = floor)
    const grid = new THREE.GridHelper(10, 20, 0x669bbc, 0x003049);
    grid.position.set(0, -0.85, -2.5);
    scene.add(grid);

    // Coordinate axes helper (RGB = XYZ, size = 0.5m)
    const axesHelper = new THREE.AxesHelper(0.5);
    scene.add(axesHelper);

    // Generate circular antialiased point sprite texture
    function createCircleTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 64;
      canvas.height = 64;
      const ctx = canvas.getContext('2d');
      const grad = ctx.createRadialGradient(32, 32, 0, 32, 32, 32);
      grad.addColorStop(0, 'rgba(255, 255, 255, 1.0)');
      grad.addColorStop(0.75, 'rgba(255, 255, 255, 0.95)');
      grad.addColorStop(1, 'rgba(255, 255, 255, 0.0)');
      ctx.fillStyle = grad;
      ctx.beginPath();
      ctx.arc(32, 32, 30, 0, Math.PI * 2);
      ctx.fill();
      return new THREE.CanvasTexture(canvas);
    }

    const circleTexture = createCircleTexture();

    // Dynamic PointCloud Geometry
    // sizeAttenuation: false ensures points are rendered in SCREEN PIXELS (2-3px)
    // exactly like VS Code's 3D Point Cloud Visualizer, CloudCompare, and MeshLab!
    const geometry = new THREE.BufferGeometry();
    const material = new THREE.PointsMaterial({
      size: state.pointSize,
      vertexColors: true,
      sizeAttenuation: false,
      map: circleTexture,
      transparent: true,
      alphaTest: 0.05,
      depthWrite: true,
    });
    const pointCloud = new THREE.Points(geometry, material);
    scene.add(pointCloud);

    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight;
      camera.updateProjectionMatrix();
      renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // ── 2. CONTROLS WIRING ──────────────────────────────────────────────────
    const toggleGroundBtn = document.getElementById('toggle-ground');
    toggleGroundBtn.addEventListener('click', () => {
      state.showGround = !state.showGround;
      toggleGroundBtn.classList.toggle('active', state.showGround);
      updateRenderedPoints();
    });

    const sizeSlider = document.getElementById('size-slider');
    const sizeVal = document.getElementById('size-val');
    sizeSlider.addEventListener('input', (e) => {
      const v = parseFloat(e.target.value);
      state.pointSize = v;
      material.size = v;
      sizeVal.textContent = v.toFixed(1) + ' px';
    });

    function fitCameraToCloud() {
      if (!geometry.attributes.position || geometry.attributes.position.count === 0) return;
      geometry.computeBoundingBox();
      const box = geometry.boundingBox;
      if (!box) return;

      const center = new THREE.Vector3();
      box.getCenter(center);
      const size = new THREE.Vector3();
      box.getSize(size);
      const maxDim = Math.max(size.x, size.y, size.z, 0.5);

      controls.target.copy(center);

      // Position camera in front of the object looking towards it
      const dist = maxDim * 1.6;
      camera.position.set(center.x, center.y + maxDim * 0.15, center.z + dist);
      camera.lookAt(center);
      controls.update();

      // Position ground grid right below the lowest point
      grid.position.set(center.x, box.min.y - 0.005, center.z);
    }

    document.getElementById('btn-reset').addEventListener('click', () => {
      fitCameraToCloud();
    });

    document.getElementById('btn-snapshot').addEventListener('click', () => {
      if (!state.currentPoints || state.currentPoints.length === 0) return;
      downloadPCD();
    });

    function downloadPCD() {
      const n = state.currentPoints.length / 3;
      let header = "# .PCD v0.7 - Point Cloud Data file format\\n";
      header += "VERSION 0.7\\nFIELDS x y z rgb\\nSIZE 4 4 4 4\\nTYPE F F F U\\nCOUNT 1 1 1 1\\n";
      header += `WIDTH ${n}\\nHEIGHT 1\\nVIEWPOINT 0 0 0 1 0 0 0\\nPOINTS ${n}\\nDATA ascii\\n`;

      let body = "";
      for (let i = 0; i < n; i++) {
        const x = state.currentPoints[i * 3 + 0].toFixed(4);
        const y = state.currentPoints[i * 3 + 1].toFixed(4);
        const z = state.currentPoints[i * 3 + 2].toFixed(4);
        const r = Math.floor(state.currentColors[i * 3 + 0] * 255);
        const g = Math.floor(state.currentColors[i * 3 + 1] * 255);
        const b = Math.floor(state.currentColors[i * 3 + 2] * 255);
        const packed = ((r << 16) | (g << 8) | b) >>> 0;
        body += `${x} ${y} ${z} ${packed}\\n`;
      }

      const blob = new Blob([header + body], { type: 'text/plain' });
      const link = document.createElement('a');
      link.href = URL.createObjectURL(blob);
      link.download = `rvpoint_stream_frame_${state.lastFrameIdx}.pcd`;
      link.click();
    }

    // ── 3. REAL-TIME DATA STREAMING LOOP ────────────────────────────────────
    const statusBadge = document.getElementById('status-badge');
    const statusText = document.getElementById('status-text');

    const mFrame = document.getElementById('metric-frame');
    const mPoints = document.getElementById('metric-points');
    const mClusters = document.getElementById('metric-clusters');
    const mGround = document.getElementById('metric-ground');
    const mCompute = document.getElementById('metric-compute');

    function updateRenderedPoints() {
      if (!state.currentPoints) return;

      let pts = state.currentPoints;
      let cols = state.currentColors;

      if (!state.showGround && state.groundCount > 0) {
        // Ground points are packed first; slice to show ONLY obstacle clusters
        const offset = state.groundCount * 3;
        pts = pts.subarray(offset);
        cols = cols.subarray(offset);
      }

      geometry.setAttribute('position', new THREE.BufferAttribute(pts, 3));
      geometry.setAttribute('color', new THREE.BufferAttribute(cols, 3));
      geometry.computeBoundingSphere();
      geometry.computeBoundingBox();
    }

    async function streamLoop() {
      while (true) {
        try {
          const resp = await fetch('/latest_frame');
          if (resp.status === 200) {
            const buf = await resp.arrayBuffer();
            if (buf.byteLength >= 28) {
              parseFrameBuffer(buf);
            }
          }
        } catch (e) {
          statusBadge.className = 'status-badge disconnected';
          statusText.textContent = 'Reconnecting';
        }
        await new Promise(r => setTimeout(r, 40)); // Poll at ~25 Hz
      }
    }

    function parseFrameBuffer(buf) {
      const view = new DataView(buf);
      const frameIdx = view.getUint32(4, false);
      const ptCnt = view.getUint32(8, false);
      const groundCnt = view.getUint32(12, false);
      const clusterCnt = view.getUint32(16, false);
      const compMs = view.getFloat32(20, false);
      const wallMs = view.getFloat32(24, false);

      statusBadge.className = 'status-badge connected';
      statusText.textContent = 'Live RVV 1.0';

      mFrame.textContent = `#${frameIdx}`;
      mPoints.textContent = ptCnt.toLocaleString();
      mClusters.textContent = clusterCnt;
      mGround.textContent = groundCnt.toLocaleString();
      mCompute.textContent = `${compMs.toFixed(1)} ms`;

      if (frameIdx === state.lastFrameIdx) return;
      state.lastFrameIdx = frameIdx;
      state.groundCount = groundCnt;

      // Extract coordinates and packed colors directly
      // Exact XYZ matching standard PCD format without distortion
      const f32 = new Float32Array(buf, 28, ptCnt * 4);
      const u32 = new Uint32Array(buf, 28, ptCnt * 4);

      const positions = new Float32Array(ptCnt * 3);
      const colors = new Float32Array(ptCnt * 3);

      for (let i = 0; i < ptCnt; i++) {
        // Point layout: [x, y, z, packed_rgb]
        positions[i * 3 + 0] = f32[i * 4 + 0];
        positions[i * 3 + 1] = f32[i * 4 + 1];
        positions[i * 3 + 2] = f32[i * 4 + 2];

        const packed = u32[i * 4 + 3];
        colors[i * 3 + 0] = ((packed >> 16) & 0xFF) / 255.0;
        colors[i * 3 + 1] = ((packed >> 8) & 0xFF) / 255.0;
        colors[i * 3 + 2] = (packed & 0xFF) / 255.0;
      }

      state.currentPoints = positions;
      state.currentColors = colors;

      updateRenderedPoints();

      if (!state.cameraInitialized) {
        state.cameraInitialized = true;
        fitCameraToCloud();
      }
    }

    // Render loop
    function animate() {
      requestAnimationFrame(animate);
      controls.update();
      renderer.render(scene, camera);
    }

    animate();
    streamLoop();
  </script>
</body>
</html>
"""


class WebStreamHandler(http.server.BaseHTTPRequestHandler):
    bridge: StreamBridge = None

    def log_message(self, format, *args):
        pass  # Suppress noisy HTTP request logs on console

    def end_headers(self):
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
        super().end_headers()

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index.html"):
            content = HTML_PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif self.path.startswith("/latest_frame"):
            raw_pkt, meta = self.bridge.get_latest_data()
            if raw_pkt is not None:
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(raw_pkt)))
                self.end_headers()
                self.wfile.write(raw_pkt)
            else:
                self.send_response(204)  # No content yet
                self.end_headers()

        elif self.path.startswith("/sample_frame"):
            repo_root = Path(__file__).resolve().parent.parent
            sample_candidates = [
                repo_root / "demonstration" / "prcsd" / "processed_frame_000003_070422.pcd",
                repo_root / "demonstration" / "prcsd" / "clusters_only_frame_000003_070422.pcd",
            ]
            sample_path = None
            for cand in sample_candidates:
                if cand.is_file():
                    sample_path = cand
                    break

            if sample_path:
                with open(sample_path, "rb") as f:
                    while True:
                        line = f.readline().decode("latin1", errors="ignore")
                        if line.startswith("DATA"):
                            break
                    raw_pcd = f.read()

                pt_cnt = len(raw_pcd) // 16
                is_processed = "processed_" in sample_path.name
                ground_cnt = 2555 if is_processed else 0
                cl_cnt = 4
                hdr = struct.pack(
                    HEADER_FORMAT,
                    POINTS_STREAM_MAGIC,
                    3,
                    pt_cnt,
                    ground_cnt,
                    cl_cnt,
                    14.7,
                    18.2,
                )
                pkt = hdr + raw_pcd
                self.send_response(200)
                self.send_header("Content-Type", "application/octet-stream")
                self.send_header("Content-Length", str(len(pkt)))
                self.end_headers()
                self.wfile.write(pkt)
            else:
                self.send_error(404, "Sample PCD not found")

        elif self.path.startswith("/status"):
            _, meta = self.bridge.get_latest_data()
            payload = json.dumps({
                "host": self.bridge.host,
                "port": self.bridge.port,
                **meta,
            }).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        else:
            self.send_error(404, "Not Found")


def main():
    parser = argparse.ArgumentParser(
        description="RVPoint Real-Time WebGL 3D Point Cloud Visualizer (Hardware RVV 1.0)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--host",
        default="100.94.165.126",
        help="IP address of Orange Pi RV2 (Tailscale: 100.94.165.126, Local: 172.20.10.2, Loopback: 127.0.0.1)",
    )
    parser.add_argument("--port", type=int, default=9001, help="TCP port of RVPoint stream broadcaster")
    parser.add_argument("--web-port", type=int, default=8080, help="Local HTTP port to serve WebGL dashboard")
    args = parser.parse_args()

    bridge = StreamBridge(host=args.host, port=args.port)
    WebStreamHandler.bridge = bridge

    server_address = ("0.0.0.0", args.web_port)
    try:
        httpd = http.server.ThreadingHTTPServer(server_address, WebStreamHandler)
    except Exception as e:
        print(f"[Error] Failed to bind WebGL server on port {args.web_port}: {e}")
        sys.exit(1)

    print("=" * 76)
    print("  RVPoint Real-Time WebGL 3D Perception Visualizer")
    print("=" * 76)
    print(f"Orange Pi Target    : {args.host}:{args.port}")
    print(f"WebGL 3D Dashboard  : http://localhost:{args.web_port}")
    print(f"LAN Access          : http://0.0.0.0:{args.web_port}")
    print("-" * 76)
    print("Open the link above in Chrome, Safari, or Edge for live 60 FPS 3D viewing!")
    print("=" * 76 + "\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping WebGL Visualizer...")
    finally:
        bridge.running = False
        httpd.server_close()


if __name__ == "__main__":
    main()
