#!/usr/bin/env python3
"""
teleop_server.py — Zero-Dependency Web-Based Teleoperation & Live Calibration Server.

Serves an interactive dark-mode dashboard accessible from any phone, tablet, or PC
browser over Wi-Fi / Hotspot or Tailscale (default: http://<ip>:8085).

Features:
  - Mobile-friendly touch virtual joystick & D-Pad for intuitive 2-DoF driving.
  - Desktop keyboard controls (WASD, Space for active brake, +/- for speed limit).
  - Prominent emergency stop (E-STOP) button with active dynamic brake.
  - Live adjustable calibration sliders (per-wheel forward/reverse trims & deadbands).
  - Live telemetry meters for all 4 wheels (FL, FR, RL, RR).
  - One-click automated testing (single-wheel pulses, 4-way directional check, stiction sweep).
  - One-click configuration persistence directly to eval/actuators/l298n_pins.json.
  - 100% self-contained: Pure Python standard library (no pip dependencies required).
"""

import argparse
import json
import os
import signal
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import parse_qs, urlparse

# Ensure project root is in sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from demonstration.actuators.l298n_actuator import DualL298NActuator, clamp


# ---------------------------------------------------------------------------
# Global State & Automated Routine Manager
# ---------------------------------------------------------------------------
class TeleopState:
    def __init__(self, config_path: Path):
        self.config_path = config_path
        self.actuator = DualL298NActuator(config_path=config_path, auto_init=True)
        self.lock = threading.Lock()

        # Teleop driving command cache
        self.linear = 0.0
        self.angular = 0.0
        self.strafe = 0.0
        self.speed_limit = 0.35
        self.last_client_ping = time.time()

        # Async automated test routine status
        self.active_test_name: Optional[str] = None
        self.test_progress: str = "Idle"
        self.test_thread: Optional[threading.Thread] = None

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            duties = list(self.actuator._target_duties)
            estop = self.actuator.is_emergency_stopped()
            watchdog_tripped = getattr(self.actuator, "_watchdog_tripped", False)
            is_sim = getattr(self.actuator, "is_simulated", False)
            wheel_cfg = self.actuator.config.get("wheels", {})

            # Map raw channels back to logical FL, FR, RL, RR duties
            wheel_duties = {}
            for name in ["FL", "FR", "RL", "RR"]:
                cfg = wheel_cfg.get(name, {})
                ch = cfg.get("channel", -1)
                raw_d = duties[ch] if (0 <= ch < len(duties)) else 0.0
                # If electrically inverted, flip sign so gauge shows true motion direction (fwd/rev)
                wheel_duties[name] = -raw_d if cfg.get("invert", False) else raw_d

            return {
                "emergency_stop": estop,
                "watchdog_tripped": watchdog_tripped,
                "is_simulated": is_sim,
                "speed_limit": self.speed_limit,
                "linear": self.linear,
                "angular": self.angular,
                "wheel_duties": wheel_duties,
                "raw_duties": duties,
                "active_test": self.active_test_name,
                "test_progress": self.test_progress,
                "uptime": round(time.time() - self.last_client_ping, 1),
            }


TELEOP_STATE: Optional[TeleopState] = None


# ---------------------------------------------------------------------------
# HTML / CSS / JS Single-Page Application (Zero External Dependencies)
# ---------------------------------------------------------------------------
HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>RVPoint Vehicle Teleop & Calibration</title>
  <style>
    :root {
      --bg-color: #0b0f19;
      --card-bg: #151d2f;
      --card-border: #23304b;
      --accent: #3b82f6;
      --accent-hover: #2563eb;
      --success: #10b981;
      --warning: #f59e0b;
      --danger: #ef4444;
      --text: #f3f4f6;
      --text-muted: #9ca3af;
      --gauge-bg: #1f293d;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; user-select: none; -webkit-user-select: none; }
    body {
      background-color: var(--bg-color);
      color: var(--text);
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      padding-bottom: 24px;
    }
    header {
      background: #111827;
      border-bottom: 1px solid var(--card-border);
      padding: 12px 16px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      flex-wrap: wrap;
      gap: 8px;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 8px;
      font-weight: 700;
      font-size: 1.15rem;
      letter-spacing: 0.5px;
    }
    .badge {
      font-size: 0.75rem;
      padding: 3px 8px;
      border-radius: 9999px;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 0.5px;
    }
    .badge-sim { background: #374151; color: #d1d5db; }
    .badge-hw { background: #065f46; color: #a7f3d0; }
    .badge-ok { background: #065f46; color: #a7f3d0; }
    .badge-err { background: #7f1d1d; color: #fecaca; animation: pulse 1s infinite; }

    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }

    .main-nav {
      display: flex;
      background: var(--card-bg);
      border-bottom: 1px solid var(--card-border);
    }
    .nav-btn {
      flex: 1;
      padding: 12px;
      text-align: center;
      color: var(--text-muted);
      font-weight: 600;
      font-size: 0.95rem;
      border: none;
      background: none;
      cursor: pointer;
      border-bottom: 3px solid transparent;
      transition: all 0.2s;
    }
    .nav-btn.active {
      color: var(--accent);
      border-bottom-color: var(--accent);
      background: rgba(59, 130, 246, 0.08);
    }

    .container {
      max-width: 900px;
      width: 100%;
      margin: 0 auto;
      padding: 16px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }

    .tab-pane { display: none; }
    .tab-pane.active { display: flex; flex-direction: column; gap: 16px; }

    /* E-Stop Bar */
    .estop-bar {
      display: grid;
      grid-template-columns: 2fr 1fr;
      gap: 12px;
    }
    .btn-estop {
      background: var(--danger);
      color: white;
      font-size: 1.25rem;
      font-weight: 800;
      padding: 16px;
      border-radius: 8px;
      border: none;
      cursor: pointer;
      box-shadow: 0 4px 14px rgba(239, 68, 68, 0.4);
      transition: transform 0.1s;
    }
    .btn-estop:active { transform: scale(0.97); }
    .btn-reset {
      background: #374151;
      color: #e5e7eb;
      font-size: 0.95rem;
      font-weight: 600;
      padding: 16px;
      border-radius: 8px;
      border: 1px solid #4b5563;
      cursor: pointer;
    }
    .btn-reset:active { background: #4b5563; }

    /* Teleop Cards */
    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 10px;
      padding: 16px;
    }
    .card-title {
      font-size: 1rem;
      font-weight: 700;
      margin-bottom: 12px;
      color: #e5e7eb;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }

    /* Driving Layout */
    .driving-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 16px;
    }
    @media (max-width: 680px) {
      .driving-grid { grid-template-columns: 1fr; }
    }

    /* Virtual Joystick */
    .joystick-container {
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      padding: 12px;
    }
    .joystick-base {
      width: 200px;
      height: 200px;
      background: radial-gradient(circle, #1e293b 0%, #0f172a 100%);
      border: 2px solid #334155;
      border-radius: 50%;
      position: relative;
      touch-action: none;
      box-shadow: inset 0 2px 8px rgba(0,0,0,0.5);
    }
    .joystick-thumb {
      width: 64px;
      height: 64px;
      background: linear-gradient(135deg, #3b82f6 0%, #1d4ed8 100%);
      border: 2px solid #60a5fa;
      border-radius: 50%;
      position: absolute;
      top: calc(50% - 32px);
      left: calc(50% - 32px);
      box-shadow: 0 4px 12px rgba(59, 130, 246, 0.5);
      cursor: grab;
      pointer-events: none;
    }

    /* D-Pad Buttons */
    .dpad {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-template-rows: repeat(3, 1fr);
      gap: 8px;
      max-width: 220px;
      margin: 0 auto;
    }
    .dpad-btn {
      aspect-ratio: 1;
      background: #1f293d;
      border: 1px solid #374151;
      color: var(--text);
      font-size: 1.3rem;
      border-radius: 8px;
      cursor: pointer;
      display: flex;
      align-items: center;
      justify-content: center;
      transition: background 0.1s;
    }
    .dpad-btn:active, .dpad-btn.pressed {
      background: var(--accent);
      color: white;
    }
    .dpad-empty { visibility: hidden; }

    /* Telemetry Meters */
    .wheel-meters {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-top: 12px;
    }
    .meter-box {
      background: var(--gauge-bg);
      border-radius: 6px;
      padding: 8px 12px;
      border: 1px solid #26334d;
    }
    .meter-header {
      display: flex;
      justify-content: space-between;
      font-size: 0.8rem;
      font-weight: 600;
      color: var(--text-muted);
      margin-bottom: 4px;
    }
    .meter-track {
      height: 12px;
      background: #111827;
      border-radius: 6px;
      overflow: hidden;
      position: relative;
    }
    .meter-center {
      position: absolute;
      left: 50%;
      width: 2px;
      height: 100%;
      background: #4b5563;
      z-index: 2;
    }
    .meter-bar {
      position: absolute;
      height: 100%;
      background: var(--accent);
      transition: width 0.05s, left 0.05s;
    }

    /* Calibration Form */
    .wheel-cal-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 12px;
    }
    .wheel-cal-card {
      background: #182238;
      border: 1px solid #2a3b5c;
      border-radius: 8px;
      padding: 12px;
    }
    .row-ctrl {
      display: flex;
      align-items: center;
      justify-content: space-between;
      margin-bottom: 8px;
      font-size: 0.85rem;
    }
    .row-ctrl label { color: var(--text-muted); flex: 1; }
    .row-ctrl .val-disp { width: 48px; text-align: right; font-weight: 700; color: #60a5fa; }
    input[type="range"] {
      flex: 2;
      margin: 0 8px;
      accent-color: var(--accent);
    }
    .toggle-row {
      display: flex;
      align-items: center;
      justify-content: space-between;
      padding-top: 6px;
      border-top: 1px solid #26334d;
      margin-top: 8px;
    }

    /* Action Buttons */
    .btn-action {
      background: #1f293d;
      color: var(--text);
      border: 1px solid #374151;
      padding: 10px 14px;
      border-radius: 6px;
      font-weight: 600;
      font-size: 0.9rem;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      gap: 6px;
      transition: all 0.15s;
    }
    .btn-action:hover { background: #2b3952; }
    .btn-action-primary {
      background: var(--accent);
      border-color: var(--accent-hover);
      color: white;
    }
    .btn-action-primary:hover { background: var(--accent-hover); }

    .btn-group {
      display: flex;
      gap: 8px;
      flex-wrap: wrap;
    }

    .status-toast {
      padding: 10px 14px;
      background: #1e293b;
      border-left: 4px solid var(--accent);
      border-radius: 4px;
      font-size: 0.85rem;
      color: #cbd5e1;
    }
  </style>
</head>
<body>

  <header>
    <div class="brand">
      <span>🚗</span>
      <span>RVPoint Autonomous Teleop</span>
    </div>
    <div style="display:flex; gap:6px;">
      <span id="badge-hw" class="badge badge-sim">Simulation</span>
      <span id="badge-status" class="badge badge-ok">Normal</span>
    </div>
  </header>

  <nav class="main-nav">
    <button class="nav-btn active" onclick="switchTab('tab-drive')">🕹️ Drive</button>
    <button class="nav-btn" onclick="switchTab('tab-cal')">⚙️ Calibration</button>
    <button class="nav-btn" onclick="switchTab('tab-tests')">⚡ Automated Tests</button>
  </nav>

  <div class="container">

    <!-- Top E-Stop Bar (Always Available) -->
    <div class="estop-bar">
      <button class="btn-estop" onclick="triggerEStop()">🛑 EMERGENCY BRAKE</button>
      <button class="btn-reset" onclick="resetEStop()">🔄 Reset E-Stop</button>
    </div>

    <!-- TAB 1: DRIVE -->
    <div id="tab-drive" class="tab-pane active">
      <div class="card">
        <div class="card-title">
          <span>Speed Limit</span>
          <span id="speed-disp" style="color:var(--accent);">35%</span>
        </div>
        <input type="range" id="speed-slider" min="15" max="100" value="35" style="width:100%;" oninput="updateSpeedLimit(this.value)">
      </div>

      <div class="driving-grid">
        <!-- Touch Joystick -->
        <div class="card">
          <div class="card-title">Analog Joystick</div>
          <div class="joystick-container">
            <div class="joystick-base" id="joy-base">
              <div class="joystick-thumb" id="joy-thumb"></div>
            </div>
          </div>
          <p style="text-align:center; font-size:0.75rem; color:var(--text-muted); margin-top:8px;">
            Touch & Drag inside circle to steer
          </p>
        </div>

        <!-- D-Pad -->
        <div class="card">
          <div class="card-title">Directional Pad (WASD / QE)</div>
          <div class="dpad">
            <button class="dpad-btn" id="btn-strafe-left" title="Strafe Left [Q]">⇇</button>
            <button class="dpad-btn" id="btn-up" title="Forward [W]">▲</button>
            <button class="dpad-btn" id="btn-strafe-right" title="Strafe Right [E]">⇉</button>

            <button class="dpad-btn" id="btn-left" title="Pivot Left [A]">◀</button>
            <button class="dpad-btn" id="btn-brake" style="font-size:0.8rem; font-weight:800; color:var(--danger);">STOP</button>
            <button class="dpad-btn" id="btn-right" title="Pivot Right [D]">▶</button>

            <div class="dpad-empty"></div>
            <button class="dpad-btn" id="btn-down" title="Reverse [S]">▼</button>
            <div class="dpad-empty"></div>
          </div>
          <p style="text-align:center; font-size:0.75rem; color:var(--text-muted); margin-top:8px;">
            Keyboard: [W] [A] [S] [D] | [Q][E] Strafe | [Space] Brake | [+][-] Speed
          </p>
        </div>
      </div>

      <!-- Real-Time Wheel Meters -->
      <div class="card">
        <div class="card-title">Active Wheel Duty Cycles</div>
        <div class="wheel-meters">
          <div class="meter-box">
            <div class="meter-header"><span>Front-Left (FL)</span><span id="val-fl">0%</span></div>
            <div class="meter-track"><div class="meter-center"></div><div class="meter-bar" id="bar-fl"></div></div>
          </div>
          <div class="meter-box">
            <div class="meter-header"><span>Front-Right (FR)</span><span id="val-fr">0%</span></div>
            <div class="meter-track"><div class="meter-center"></div><div class="meter-bar" id="bar-fr"></div></div>
          </div>
          <div class="meter-box">
            <div class="meter-header"><span>Rear-Left (RL)</span><span id="val-rl">0%</span></div>
            <div class="meter-track"><div class="meter-center"></div><div class="meter-bar" id="bar-rl"></div></div>
          </div>
          <div class="meter-box">
            <div class="meter-header"><span>Rear-Right (RR)</span><span id="val-rr">0%</span></div>
            <div class="meter-track"><div class="meter-center"></div><div class="meter-bar" id="bar-rr"></div></div>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 2: CALIBRATION -->
    <div id="tab-cal" class="tab-pane">
      <div class="card">
        <div class="card-title">
          <span>Live Per-Wheel Trim & Stiction Tuning</span>
          <div class="btn-group">
            <button class="btn-action btn-action-primary" onclick="saveConfig()">💾 Save to Disk</button>
            <button class="btn-action" onclick="loadConfig()">🔄 Revert</button>
          </div>
        </div>
        <div class="wheel-cal-grid" id="wheel-cards-container">
          <!-- Dynamically populated -->
        </div>
      </div>

      <div class="card">
        <div class="card-title">Global Actuation Settings</div>
        <div style="display:grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap:12px;">
          <div class="row-ctrl">
            <label>Global Deadband:</label>
            <input type="range" id="glob-db" min="5" max="35" value="12" oninput="updateGlobalParam('deadband', this.value / 100)">
            <span class="val-disp" id="glob-db-disp">0.12</span>
          </div>
          <div class="row-ctrl">
            <label>Watchdog (ms):</label>
            <input type="range" id="glob-wd" min="100" max="1000" step="50" value="200" oninput="updateGlobalParam('watchdog_timeout_ms', parseInt(this.value))">
            <span class="val-disp" id="glob-wd-disp">200ms</span>
          </div>
        </div>
      </div>
    </div>

    <!-- TAB 3: AUTOMATED TESTS -->
    <div id="tab-tests" class="tab-pane">
      <div class="card">
        <div class="card-title">Automated Verification Routines</div>
        <div style="display:flex; flex-direction:column; gap:12px;">
          <div style="display:flex; justify-content:space-between; align-items:center; background:#182238; padding:12px; border-radius:6px;">
            <div>
              <strong style="display:block;">4-Way Directional Check</strong>
              <span style="font-size:0.8rem; color:var(--text-muted);">Sequentially runs Forward -> Reverse -> Pivot Left -> Pivot Right (1.5s each)</span>
            </div>
            <button class="btn-action btn-action-primary" onclick="runAutomatedTest('directional')">Start Test</button>
          </div>

          <div style="display:flex; justify-content:space-between; align-items:center; background:#182238; padding:12px; border-radius:6px;">
            <div>
              <strong style="display:block;">Stiction Breakaway Sweep</strong>
              <span style="font-size:0.8rem; color:var(--text-muted);">Ramps duty cycle from 5% to 50% in steps of 5% to locate motor breakaway point</span>
            </div>
            <button class="btn-action btn-action-primary" onclick="runAutomatedTest('sweep')">Start Sweep</button>
          </div>
        </div>

        <div style="margin-top:16px;">
          <div class="card-title">Test Progress & Status</div>
          <div class="status-toast" id="test-status-box">System idle. Ready for tests.</div>
        </div>
      </div>
    </div>

  </div>

  <script>
    let activeTab = 'tab-drive';
    let currentConfig = null;
    let teleopInterval = null;
    let currentLinear = 0.0;
    let currentAngular = 0.0;
    let speedLimit = 0.35;
    const pressedKeys = {};

    function switchTab(tabId) {
      document.querySelectorAll('.tab-pane').forEach(el => el.classList.remove('active'));
      document.querySelectorAll('.nav-btn').forEach(el => el.classList.remove('active'));
      document.getElementById(tabId).classList.add('active');
      event.target.classList.add('active');
      activeTab = tabId;
      if (tabId === 'tab-cal') loadConfig();
    }

    function updateSpeedLimit(val) {
      speedLimit = val / 100.0;
      document.getElementById('speed-disp').innerText = val + '%';
    }

    // --- API Calls ---
    async function triggerEStop() {
      await fetch('/api/estop', { method: 'POST' });
      currentLinear = 0.0;
      currentAngular = 0.0;
      resetJoystick();
    }

    async function resetEStop() {
      await fetch('/api/reset_estop', { method: 'POST' });
    }

    let currentStrafe = 0.0;

    async function sendTeleop(lin, ang, strf = 0.0) {
      try {
        await fetch('/api/teleop', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ linear: lin, angular: ang, strafe: strf })
        });
      } catch (err) {
        console.error('Teleop error:', err);
      }
    }

    async function loadConfig() {
      try {
        const res = await fetch('/api/config');
        currentConfig = await res.json();
        renderCalibrationCards();
      } catch (err) {
        console.error('Failed to load config:', err);
      }
    }

    async function saveConfig() {
      try {
        const res = await fetch('/api/config/save', { method: 'POST' });
        const data = await res.json();
        alert(data.success ? 'Configuration saved to l298n_pins.json!' : 'Failed to save config.');
      } catch (err) {
        alert('Error saving config: ' + err);
      }
    }

    async function runAutomatedTest(testType) {
      try {
        const res = await fetch('/api/test/' + testType, { method: 'POST' });
        const data = await res.json();
        document.getElementById('test-status-box').innerText = data.message;
      } catch (err) {
        alert('Failed to start test: ' + err);
      }
    }

    async function pulseWheel(wheelKey, dir) {
      try {
        await fetch('/api/test/pulse', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ wheel: wheelKey, direction: dir, duty: speedLimit, duration: 1.0 })
        });
      } catch (err) {
        console.error('Pulse error:', err);
      }
    }

    // --- Dynamic Calibration UI ---
    function renderCalibrationCards() {
      if (!currentConfig || !currentConfig.wheels) return;
      const container = document.getElementById('wheel-cards-container');
      container.innerHTML = '';

      const labels = { FL: 'Front-Left (FL)', FR: 'Front-Right (FR)', RL: 'Rear-Left (RL)', RR: 'Rear-Right (RR)' };

      for (const [key, cfg] of Object.entries(currentConfig.wheels)) {
        const tf = cfg.trim_forward !== undefined ? cfg.trim_forward : 1.0;
        const tr = cfg.trim_reverse !== undefined ? cfg.trim_reverse : 1.0;
        const df = cfg.deadband_forward !== undefined ? cfg.deadband_forward : (currentConfig.deadband || 0.12);
        const dr = cfg.deadband_reverse !== undefined ? cfg.deadband_reverse : (currentConfig.deadband || 0.12);
        const inv = cfg.invert || false;

        const card = document.createElement('div');
        card.className = 'wheel-cal-card';
        card.innerHTML = `
          <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:8px;">
            <strong>${labels[key] || key}</strong>
            <span style="font-size:0.75rem; color:var(--text-muted);">Channel ${cfg.channel}</span>
          </div>

          <div class="row-ctrl">
            <label>Trim Forward:</label>
            <input type="range" min="50" max="150" value="${Math.round(tf * 100)}" oninput="updateWheelVal('${key}', 'trim_forward', this.value / 100, '${key}-tf-disp')">
            <span class="val-disp" id="${key}-tf-disp">${tf.toFixed(2)}</span>
          </div>

          <div class="row-ctrl">
            <label>Trim Reverse:</label>
            <input type="range" min="50" max="150" value="${Math.round(tr * 100)}" oninput="updateWheelVal('${key}', 'trim_reverse', this.value / 100, '${key}-tr-disp')">
            <span class="val-disp" id="${key}-tr-disp">${tr.toFixed(2)}</span>
          </div>

          <div class="row-ctrl">
            <label>Deadband Fwd:</label>
            <input type="range" min="5" max="35" value="${Math.round(df * 100)}" oninput="updateWheelVal('${key}', 'deadband_forward', this.value / 100, '${key}-df-disp')">
            <span class="val-disp" id="${key}-df-disp">${df.toFixed(2)}</span>
          </div>

          <div class="row-ctrl">
            <label>Deadband Rev:</label>
            <input type="range" min="5" max="35" value="${Math.round(dr * 100)}" oninput="updateWheelVal('${key}', 'deadband_reverse', this.value / 100, '${key}-dr-disp')">
            <span class="val-disp" id="${key}-dr-disp">${dr.toFixed(2)}</span>
          </div>

          <div class="toggle-row">
            <label style="font-size:0.85rem; color:var(--text-muted);">Invert Polarity:</label>
            <input type="checkbox" ${inv ? 'checked' : ''} onchange="updateWheelVal('${key}', 'invert', this.checked)">
          </div>

          <div style="display:flex; gap:6px; margin-top:8px;">
            <button class="btn-action" style="flex:1; padding:6px; font-size:0.75rem;" onclick="pulseWheel('${key}', 1)">▲ Test Fwd</button>
            <button class="btn-action" style="flex:1; padding:6px; font-size:0.75rem;" onclick="pulseWheel('${key}', -1)">▼ Test Rev</button>
          </div>
        `;
        container.appendChild(card);
      }

      // Globals
      document.getElementById('glob-db').value = Math.round((currentConfig.deadband || 0.12) * 100);
      document.getElementById('glob-db-disp').innerText = (currentConfig.deadband || 0.12).toFixed(2);
      document.getElementById('glob-wd').value = currentConfig.watchdog_timeout_ms || 200;
      document.getElementById('glob-wd-disp').innerText = (currentConfig.watchdog_timeout_ms || 200) + 'ms';
    }

    function updateWheelVal(wheelKey, prop, val, dispId) {
      if (dispId) document.getElementById(dispId).innerText = typeof val === 'number' ? val.toFixed(2) : val;
      if (!currentConfig.wheels[wheelKey]) currentConfig.wheels[wheelKey] = {};
      currentConfig.wheels[wheelKey][prop] = val;
      syncConfig();
    }

    function updateGlobalParam(prop, val) {
      if (prop === 'deadband') document.getElementById('glob-db-disp').innerText = val.toFixed(2);
      if (prop === 'watchdog_timeout_ms') document.getElementById('glob-wd-disp').innerText = val + 'ms';
      currentConfig[prop] = val;
      syncConfig();
    }

    async function syncConfig() {
      await fetch('/api/config', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(currentConfig)
      });
    }

    // --- Virtual Joystick ---
    const joyBase = document.getElementById('joy-base');
    const joyThumb = document.getElementById('joy-thumb');
    let joyActive = false;
    let joyCenter = { x: 0, y: 0 };
    const maxRadius = 70;

    function resetJoystick() {
      joyThumb.style.left = 'calc(50% - 32px)';
      joyThumb.style.top = 'calc(50% - 32px)';
      currentLinear = 0.0;
      currentAngular = 0.0;
      sendTeleop(0, 0);
    }

    function handleJoyMove(clientX, clientY) {
      const dx = clientX - joyCenter.x;
      const dy = clientY - joyCenter.y;
      const dist = Math.sqrt(dx * dx + dy * dy);
      const clampedDist = Math.min(dist, maxRadius);
      const angle = Math.atan2(dy, dx);

      const targetX = Math.cos(angle) * clampedDist;
      const targetY = Math.sin(angle) * clampedDist;

      joyThumb.style.transform = `translate(${targetX}px, ${targetY}px)`;

      // Map Y to linear (-1 to +1, up is positive forward)
      // Map X to angular (-1 to +1, right is positive right turn)
      currentLinear = (-targetY / maxRadius) * speedLimit;
      currentAngular = (targetX / maxRadius) * speedLimit;
      sendTeleop(currentLinear, currentAngular);
    }

    joyBase.addEventListener('pointerdown', (e) => {
      joyActive = true;
      const rect = joyBase.getBoundingClientRect();
      joyCenter = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 };
      joyBase.setPointerCapture(e.pointerId);
      handleJoyMove(e.clientX, e.clientY);
    });

    joyBase.addEventListener('pointermove', (e) => {
      if (joyActive) handleJoyMove(e.clientX, e.clientY);
    });

    joyBase.addEventListener('pointerup', (e) => {
      joyActive = false;
      joyThumb.style.transform = `translate(0px, 0px)`;
      resetJoystick();
    });

    // --- D-Pad Handlers ---
    function setupDPadButton(btnId, lin, ang, strf = 0.0) {
      const btn = document.getElementById(btnId);
      if (!btn) return;
      const startMove = (e) => {
        e.preventDefault();
        btn.classList.add('pressed');
        currentLinear = lin * speedLimit;
        currentAngular = ang * speedLimit;
        currentStrafe = strf * speedLimit;
        sendTeleop(currentLinear, currentAngular, currentStrafe);
      };
      const stopMove = (e) => {
        e.preventDefault();
        btn.classList.remove('pressed');
        resetJoystick();
      };
      btn.addEventListener('pointerdown', startMove);
      btn.addEventListener('pointerup', stopMove);
      btn.addEventListener('pointercancel', stopMove);
    }

    setupDPadButton('btn-up', 1.0, 0.0, 0.0);
    setupDPadButton('btn-down', -1.0, 0.0, 0.0);
    setupDPadButton('btn-left', 0.0, -1.0, 0.0);
    setupDPadButton('btn-right', 0.0, 1.0, 0.0);
    setupDPadButton('btn-strafe-left', 0.0, 0.0, -1.0);
    setupDPadButton('btn-strafe-right', 0.0, 0.0, 1.0);
    document.getElementById('btn-brake').addEventListener('click', triggerEStop);

    // --- Keyboard Event Handlers ---
    window.addEventListener('keydown', (e) => {
      if (['input', 'textarea'].includes(document.activeElement.tagName.toLowerCase())) return;
      pressedKeys[e.key.toLowerCase()] = true;

      if (e.key === ' ') {
        triggerEStop();
        return;
      }

      if (e.key === '+' || e.key === '=') {
        const slider = document.getElementById('speed-slider');
        slider.value = Math.min(100, parseInt(slider.value) + 5);
        updateSpeedLimit(slider.value);
        return;
      }
      if (e.key === '-' || e.key === '_') {
        const slider = document.getElementById('speed-slider');
        slider.value = Math.max(15, parseInt(slider.value) - 5);
        updateSpeedLimit(slider.value);
        return;
      }

      let lin = 0.0;
      let ang = 0.0;
      let strf = 0.0;
      if (pressedKeys['w']) lin += 1.0;
      if (pressedKeys['s']) lin -= 1.0;
      if (pressedKeys['a']) ang -= 1.0;
      if (pressedKeys['d']) ang += 1.0;
      if (pressedKeys['q']) strf -= 1.0;
      if (pressedKeys['e']) strf += 1.0;

      currentLinear = lin * speedLimit;
      currentAngular = ang * speedLimit;
      currentStrafe = strf * speedLimit;
      sendTeleop(currentLinear, currentAngular, currentStrafe);
    });

    window.addEventListener('keyup', (e) => {
      delete pressedKeys[e.key.toLowerCase()];
      let lin = 0.0;
      let ang = 0.0;
      let strf = 0.0;
      if (pressedKeys['w']) lin += 1.0;
      if (pressedKeys['s']) lin -= 1.0;
      if (pressedKeys['a']) ang -= 1.0;
      if (pressedKeys['d']) ang += 1.0;
      if (pressedKeys['q']) strf -= 1.0;
      if (pressedKeys['e']) strf += 1.0;

      currentLinear = lin * speedLimit;
      currentAngular = ang * speedLimit;
      currentStrafe = strf * speedLimit;
      sendTeleop(currentLinear, currentAngular, currentStrafe);
    });

    // --- Telemetry Polling Loop (10 Hz) ---
    async function pollStatus() {
      try {
        const res = await fetch('/api/status');
        const data = await res.json();

        // Update Badges
        const hwBadge = document.getElementById('badge-hw');
        hwBadge.innerText = data.is_simulated ? 'Simulation' : 'Hardware Sysfs';
        hwBadge.className = 'badge ' + (data.is_simulated ? 'badge-sim' : 'badge-hw');

        const statusBadge = document.getElementById('badge-status');
        if (data.emergency_stop) {
          statusBadge.innerText = 'E-STOPPED';
          statusBadge.className = 'badge badge-err';
        } else if (data.watchdog_tripped) {
          statusBadge.innerText = 'Watchdog Halt';
          statusBadge.className = 'badge badge-err';
        } else {
          statusBadge.innerText = 'Active';
          statusBadge.className = 'badge badge-ok';
        }

        // Update Wheel Meters
        const meters = data.wheel_duties || {};
        for (const [w, val] of Object.entries(meters)) {
          const valEl = document.getElementById('val-' + w.toLowerCase());
          const barEl = document.getElementById('bar-' + w.toLowerCase());
          if (valEl && barEl) {
            valEl.innerText = Math.round(val * 100) + '%';
            const widthPct = Math.abs(val) * 50;
            barEl.style.width = widthPct + '%';
            if (val >= 0) {
              barEl.style.left = '50%';
              barEl.style.background = 'var(--accent)';
            } else {
              barEl.style.left = (50 - widthPct) + '%';
              barEl.style.background = 'var(--danger)';
            }
          }
        }

        // Update Test Progress
        if (data.test_progress) {
          document.getElementById('test-status-box').innerText = data.test_progress;
        }

      } catch (err) {
        console.warn('Status poll error:', err);
      }
    }

    setInterval(pollStatus, 100);
  </script>
</body>
</html>
"""


# ---------------------------------------------------------------------------
# HTTP & REST API Request Handler
# ---------------------------------------------------------------------------
class TeleopHTTPHandler(BaseHTTPRequestHandler):
    server_version = "RVPointTeleop/1.0"

    def _send_json(self, data: Any, status: int = 200) -> None:
        payload = json.dumps(data).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(payload)

    def _send_html(self, html_str: str) -> None:
        payload = html_str.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path in ("", "/"):
            self._send_html(HTML_PAGE)
        elif path == "/api/status":
            if TELEOP_STATE:
                self._send_json(TELEOP_STATE.get_status())
            else:
                self._send_json({"error": "Uninitialized"}, 500)
        elif path == "/api/config":
            if TELEOP_STATE:
                self._send_json(TELEOP_STATE.actuator.config)
            else:
                self._send_json({"error": "Uninitialized"}, 500)
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path

        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length) if content_length > 0 else b""
        data = {}
        if body:
            try:
                data = json.loads(body.decode("utf-8"))
            except Exception:
                pass

        if not TELEOP_STATE:
            self._send_json({"error": "Uninitialized"}, 500)
            return

        state = TELEOP_STATE

        if path == "/api/teleop":
            lin = float(data.get("linear", 0.0))
            ang = float(data.get("angular", 0.0))
            strf = float(data.get("strafe", 0.0))
            with state.lock:
                state.linear = lin
                state.angular = ang
                state.last_client_ping = time.time()
                # Auto-clear active emergency brake when an intentional drive command is issued
                if abs(lin) > 0.001 or abs(ang) > 0.001 or abs(strf) > 0.001:
                    if state.actuator.is_emergency_stopped():
                        state.actuator.reset_emergency_stop()

                # Differential & Mecanum mixer:
                # lin:  forward (+), reverse (-)
                # ang:  pivot right (+), pivot left (-)
                # strf: strafe right (+), strafe left (-)
                if abs(strf) > 0.001:
                    # 4-Wheel Mecanum Holonomic Drive
                    fl = clamp(lin + strf + ang, -1.0, 1.0)
                    fr = clamp(lin - strf - ang, -1.0, 1.0)
                    rl = clamp(lin - strf + ang, -1.0, 1.0)
                    rr = clamp(lin + strf - ang, -1.0, 1.0)
                    state.actuator.set_wheel_duties(fl, fr, rl, rr)
                else:
                    # Standard Omni-Tank Differential Drive:
                    # Left Bank  = lin + ang  (W: +lin, S: -lin, A: -ang, D: +ang)
                    # Right Bank = lin - ang  (W: +lin, S: -lin, A: +ang, D: -ang)
                    duty_left = clamp(lin + ang, -1.0, 1.0)
                    duty_right = clamp(lin - ang, -1.0, 1.0)
                    state.actuator.set_duty_cycles(duty_left, duty_right)

            self._send_json({"success": True})

        elif path == "/api/estop":
            with state.lock:
                state.actuator.emergency_brake()
                state.linear = 0.0
                state.angular = 0.0
            self._send_json({"success": True, "emergency_stop": True})

        elif path == "/api/reset_estop":
            with state.lock:
                state.actuator.reset_emergency_stop()
            self._send_json({"success": True, "emergency_stop": False})

        elif path == "/api/config":
            # Live in-memory configuration update
            with state.lock:
                state.actuator.config.update(data)
            self._send_json({"success": True})

        elif path == "/api/config/save":
            with state.lock:
                ok = state.actuator.save_config()
            self._send_json({"success": ok, "path": str(state.config_path)})

        elif path == "/api/test/pulse":
            wheel = str(data.get("wheel", "FL"))
            dir_mult = int(data.get("direction", 1))
            duty = float(data.get("duty", 0.30)) * dir_mult
            duration = float(data.get("duration", 1.0))

            def _pulse_worker():
                with state.lock:
                    state.test_progress = f"Pulsing {wheel} for {duration:.1f}s..."
                    wheels_dict = {"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0}
                    wheels_dict[wheel] = duty
                    state.actuator.set_wheel_duties(
                        wheels_dict["FL"],
                        wheels_dict["FR"],
                        wheels_dict["RL"],
                        wheels_dict["RR"],
                    )
                time.sleep(duration)
                with state.lock:
                    state.actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)
                    state.actuator.emergency_brake()
                    time.sleep(0.2)
                    state.actuator.reset_emergency_stop()
                    state.test_progress = f"Pulse on {wheel} completed."

            t = threading.Thread(target=_pulse_worker, daemon=True)
            t.start()
            self._send_json({"success": True, "message": f"Pulsing {wheel}..."})

        elif path == "/api/test/directional":
            if state.test_thread and state.test_thread.is_alive():
                self._send_json({"error": "Test already running"}, 400)
                return

            def _dir_worker():
                state.active_test_name = "directional"
                duty = state.speed_limit
                moves = [
                    ("FORWARD", duty, duty),
                    ("REVERSE", -duty, -duty),
                    ("PIVOT LEFT", -duty, duty),
                    ("PIVOT RIGHT", duty, -duty),
                ]
                try:
                    for label, dl, dr in moves:
                        with state.lock:
                            state.test_progress = f"Running: {label} (1.5s)..."
                            state.actuator.set_duty_cycles(dl, dr)
                        time.sleep(1.5)
                        with state.lock:
                            state.actuator.set_duty_cycles(0.0, 0.0)
                            state.actuator.emergency_brake()
                            time.sleep(0.3)
                            state.actuator.reset_emergency_stop()
                            state.test_progress = f"{label} stopped. Settling (1.0s)..."
                        time.sleep(1.0)
                    state.test_progress = "Directional test completed successfully."
                finally:
                    state.active_test_name = None
                    state.actuator.set_duty_cycles(0.0, 0.0)

            state.test_thread = threading.Thread(target=_dir_worker, daemon=True)
            state.test_thread.start()
            self._send_json({"success": True, "message": "Directional test started."})

        elif path == "/api/test/sweep":
            if state.test_thread and state.test_thread.is_alive():
                self._send_json({"error": "Test already running"}, 400)
                return

            def _sweep_worker():
                state.active_test_name = "sweep"
                try:
                    for pct in range(5, 55, 5):
                        d = pct / 100.0
                        with state.lock:
                            state.test_progress = f"Stiction Sweep: Testing {pct}% duty..."
                            state.actuator.set_wheel_duties(d, d, d, d)
                        time.sleep(1.5)
                        with state.lock:
                            state.actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)
                        time.sleep(0.8)
                    state.test_progress = "Stiction sweep completed."
                finally:
                    state.active_test_name = None
                    state.actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)

            state.test_thread = threading.Thread(target=_sweep_worker, daemon=True)
            state.test_thread.start()
            self._send_json({"success": True, "message": "Stiction sweep started."})

        else:
            self.send_error(404, "Not Found")

    def log_message(self, format, *args):
        # Silence routine HTTP polling logs to keep terminal readable
        if "/api/status" in str(args) or "/api/teleop" in str(args):
            return
        super().log_message(format, *args)


# ---------------------------------------------------------------------------
# Server Startup Entry Point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="RVPoint Web-Based Vehicle Teleoperation & Live Calibration Server"
    )
    parser.add_argument(
        "--host",
        default="0.0.0.0",
        help="Host address to bind HTTP server (default: 0.0.0.0 for all interfaces)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8085,
        help="HTTP port for the web dashboard (default: 8085)",
    )
    parser.add_argument(
        "--config",
        default="eval/actuators/l298n_pins.json",
        help="Path to motor configuration JSON",
    )
    args = parser.parse_args()

    config_path = PROJECT_ROOT / args.config if not Path(args.config).is_absolute() else Path(args.config)
    global TELEOP_STATE
    TELEOP_STATE = TeleopState(config_path)

    # Clean signal handling
    def sig_handler(sig, frame):
        print("\n[TeleopServer] Stopping server and shutting down motor hardware...")
        if TELEOP_STATE:
            TELEOP_STATE.actuator.emergency_brake()
            TELEOP_STATE.actuator.shutdown_hardware()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    server = ThreadingHTTPServer((args.host, args.port), TeleopHTTPHandler)

    # Discover local IP for convenient user instructions
    local_ips = []
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        local_ips.append(s.getsockname()[0])
        s.close()
    except Exception:
        pass

    print("\n" + "=" * 65)
    print("   RVPoint Web Teleop & Live Calibration Server Running!   ")
    print("=" * 65)
    print(f"--> Local Browser:    http://localhost:{args.port}")
    for ip in local_ips:
        print(f"--> Mobile / Network: http://{ip}:{args.port}")
    print("=" * 65)
    print(f"Config File: {config_path}")
    print(f"Hardware:    {'SIMULATION / MOCK' if TELEOP_STATE.actuator.is_simulated else 'LINUX SYSFS GPIO'}")
    print("Press Ctrl+C to stop.\n")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if TELEOP_STATE:
            TELEOP_STATE.actuator.emergency_brake()
            TELEOP_STATE.actuator.shutdown_hardware()


if __name__ == "__main__":
    main()
