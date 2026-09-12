# RVPoint Demonstration Run Commands Reference

Quick reference commands for running the real-time LiDAR perception server and live visualizers.

---

## 1. On Orange Pi RV2 (Perception & Processing Engine)

> Connect your iPhone LiDAR Streamer app to `100.94.165.126:9000` (Tailscale) or `172.20.10.2:9000` (Local Wi-Fi/Hotspot).

### Mode A: Stream-Only (Recommended for Continuous Live Viewing — 0 Disk Writes)
Bypasses saving files to flash/SD card, keeping scratch buffers in RAM (`/dev/shm`) and broadcasting continuous 3D points directly to your PC:
```bash
python3 demonstration/server_main.py \
  --pipeline ultra \
  --leaf-size 0.03 \
  --ransac-dist 0.06 \
  --ground-angle-thresh 6.7 \
  --cluster-tolerance 0.10 \
  --min-cluster 15 \
  --stream-only \
  --continuous
```
*(Passing `--continuous` or `--interval 0` disables the 500ms throttle so every frame from iPhone 15 Pro LiDAR is processed immediately in real time).*

### Mode B: Save to Disk + Live Stream
Saves raw scans to `main_scans/` and clustered scans to `processed_scans/`, while simultaneously broadcasting to the live visualizer:
```bash
python3 demonstration/server_main.py \
  --pipeline ultra \
  --leaf-size 0.03 \
  --ransac-dist 0.06 \
  --ground-angle-thresh 6.7 \
  --cluster-tolerance 0.10 \
  --min-cluster 15 \
  --save-scans
```

---

## 2. On Host PC / Mac (Real-Time 3D Visualizers)

### Option A: WebGL Three.js Browser Visualizer (Recommended — 60 FPS GPU Viewport)
Runs a zero-dependency local bridge and serves a high-performance 3D dashboard:
```bash
# Over Tailscale:
python3 demonstration/web_viewer.py --host 100.94.165.126

# Over Local Wi-Fi / Hotspot:
python3 demonstration/web_viewer.py --host 172.20.10.2
```
👉 Open **`http://localhost:8080`** in Chrome, Safari, or Edge.
- **Controls**: Left-drag to orbit, right-drag to pan, scroll to zoom.
- **Features**: Toggle ground plane, adjust point size, one-click PCD snapshot download, live RVV 1.0 telemetry.

---

### Option B: Native 3D Window (Open3D Desktop Client)
Renders inside a native interactive GUI window with keyboard shortcuts:
```bash
# Using Python with Open3D:
./demonstration/lidar-env/bin/python3 demonstration/live_viewer.py --host 100.94.165.126
```
- **Controls**:
  - `G`: Toggle ground plane visibility
  - `+` / `-`: Increase / decrease point size
  - `R`: Reset camera viewpoint
  - `S`: Save current frame snapshot as PCD and PNG
  - `Q` / `ESC`: Exit

---

### Option C: File Synchronizer (PCD File Downloader)
Automatically downloads raw and processed PCD files to your local `demonstration/mains/` and `demonstration/prcsd/` folders:
```bash
python3 demonstration/receive_scans.py --host 100.94.165.126
```
