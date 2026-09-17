#!/usr/bin/env python3
"""
l298n_actuator.py — Axle-Partitioned Quad-Channel Omni-Tank Dual L298N Actuator Driver in Python.

Implements the low-level hardware bridge for two discrete L298N dual H-bridge modules
partitioned by axle:
  - Front Module: Channel A (FL), Channel B (FR)
  - Rear Module:  Channel A (RL), Channel B (RR)

Features:
  - Zero external dependencies: Uses standard library only (os, sys, time, json, threading).
  - Dual PWM backend: Linux sysfs hardware PWM or high-precision software timer PWM.
  - Active braking (LOW/LOW) on zero command or e-stop.
  - 200 ms deadman safety watchdog.
  - Per-wheel polarity inversion and trim calibration.
  - Graceful simulation fallback when running off-target (CI/Windows/macOS).
"""

import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, Optional


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


class DualL298NActuator:
    """Axle-Partitioned Dual L298N Motor Driver for Omni-Tank Kinematics."""

    DEFAULT_CONFIG_PATH = Path("eval/actuators/l298n_pins.json")

    def __init__(
        self, config_path: Optional[str | Path] = None, auto_init: bool = True
    ):
        self.config_path = (
            Path(config_path) if config_path else self.DEFAULT_CONFIG_PATH
        )
        self.config: Dict[str, Any] = self._load_default_config()

        if self.config_path.exists():
            self.load_config(self.config_path)

        self._emergency_stop = False
        self._watchdog_enabled = self.config.get("enable_watchdog", True)
        self._watchdog_timeout_ms = self.config.get("watchdog_timeout_ms", 200)
        self._watchdog_tripped = False
        self._last_command_time = time.perf_counter()

        # Commanded duty cycles for the 4 raw channels [-1.0, 1.0]
        # Channels: 0: Front ChA (FL), 1: Front ChB (FR), 2: Rear ChA (RL), 3: Rear ChB (RR)
        self._target_duties = [0.0, 0.0, 0.0, 0.0]
        self._lock = threading.Lock()

        # Hardware state tracking
        self.is_simulated = False
        self._pin_states: Dict[int, int] = {}
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

        # Register signal handlers for clean hardware stop on Ctrl+C / SIGINT
        try:
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)
        except Exception:
            pass

        if auto_init:
            self.initialize_hardware()

    def _signal_handler(self, signum, frame):
        self.emergency_brake()
        self.shutdown_hardware()
        sys.exit(0)

    def _load_default_config(self) -> Dict[str, Any]:
        return {
            "use_hardware_pwm": False,
            "pwm_frequency_hz": 25,
            "deadband": 0.15,
            "enable_watchdog": True,
            "watchdog_timeout_ms": 200,
            "wheels": {
                "FL": {
                    "channel": 1,
                    "pwm_pin": -1,
                    "sysfs_pwm_path": "",
                    "in1_pin": 72,
                    "in2_pin": 73,
                    "invert": False,
                    "trim": 1.0,
                },
                "FR": {
                    "channel": 0,
                    "pwm_pin": -1,
                    "sysfs_pwm_path": "",
                    "in1_pin": 74,
                    "in2_pin": 71,
                    "invert": True,
                    "trim": 1.0,
                },
                "RL": {
                    "channel": 3,
                    "pwm_pin": -1,
                    "sysfs_pwm_path": "",
                    "in1_pin": 70,
                    "in2_pin": 91,
                    "invert": True,
                    "trim": 1.0,
                },
                "RR": {
                    "channel": 2,
                    "pwm_pin": -1,
                    "sysfs_pwm_path": "",
                    "in1_pin": 47,
                    "in2_pin": 48,
                    "invert": False,
                    "trim": 1.0,
                },
            },
        }

    def load_config(self, filepath: str | Path) -> bool:
        path = Path(filepath)
        if not path.exists():
            return False
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.config.update(data)
            self.config_path = path
            return True
        except Exception as e:
            print(f"[DualL298NActuator] Error loading {path}: {e}", file=sys.stderr)
            return False

    def save_config(self, filepath: Optional[str | Path] = None) -> bool:
        path = Path(filepath) if filepath else self.config_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.config, f, indent=2)
            return True
        except Exception as e:
            print(f"[DualL298NActuator] Error saving {path}: {e}", file=sys.stderr)
            return False

    # -----------------------------------------------------------------------
    # Hardware Initialization & GPIO Sysfs Handling
    # -----------------------------------------------------------------------
    def _export_gpio(self, pin: int) -> bool:
        if pin < 0:
            return False
        gpio_path = f"/sys/class/gpio/gpio{pin}"
        if os.path.exists(gpio_path):
            return True
        try:
            with open("/sys/class/gpio/export", "w") as f:
                f.write(str(pin))
            return True
        except Exception:
            return False

    def _set_gpio_direction(self, pin: int, direction: str = "low") -> bool:
        if pin < 0:
            return False
        dir_path = f"/sys/class/gpio/gpio{pin}/direction"
        try:
            with open(dir_path, "w") as f:
                f.write(direction)
            return True
        except Exception:
            return False

    def _write_gpio(self, pin: int, val: int) -> None:
        if pin < 0:
            return
        if self._pin_states.get(pin) == val:
            return
        val_bytes = b"1" if val else b"0"
        try:
            fd = os.open(f"/sys/class/gpio/gpio{pin}/value", os.O_WRONLY)
            os.write(fd, val_bytes)
            os.close(fd)
            self._pin_states[pin] = val
        except Exception:
            pass

    def _force_pin_low(self, pin: int) -> None:
        if pin < 0:
            return
        try:
            fd = os.open(f"/sys/class/gpio/gpio{pin}/value", os.O_WRONLY)
            os.write(fd, b"0")
            os.close(fd)
        except Exception:
            pass
        self._pin_states[pin] = 0

    def initialize_hardware(self) -> bool:
        self.shutdown_hardware()

        # Check if sysfs gpio is writable
        sysfs_gpio_available = (
            os.access("/sys/class/gpio", os.W_OK)
            if os.path.exists("/sys/class/gpio")
            else False
        )

        if not sysfs_gpio_available:
            self.is_simulated = True
            print(
                "[DualL298NActuator:Python] Hardware sysfs GPIO inaccessible. Running in SIMULATION / MOCK mode."
            )
        else:
            self.is_simulated = False
            print(
                "[DualL298NActuator:Python] Initializing Linux sysfs GPIO / PWM pins..."
            )

            wheels = self.config.get("wheels", {})
            for name, w in wheels.items():
                in1 = w.get("in1_pin", -1)
                in2 = w.get("in2_pin", -1)
                pwm_pin = w.get("pwm_pin", -1)

                for pin in (in1, in2, pwm_pin):
                    if pin >= 0:
                        if self._export_gpio(pin):
                            self._set_gpio_direction(pin, "low")
                        self._force_pin_low(pin)

        # Start software PWM / watchdog worker thread
        self._running = True
        self._last_command_time = time.perf_counter()
        self._worker_thread = threading.Thread(
            target=self._worker_loop, daemon=True, name="L298NWorker"
        )
        self._worker_thread.start()
        return True

    def shutdown_hardware(self) -> None:
        self._running = False
        if self._worker_thread and self._worker_thread.is_alive():
            self._worker_thread.join(timeout=0.5)

        wheels = self.config.get("wheels", {})
        for w in wheels.values():
            for p in (w.get("in1_pin"), w.get("in2_pin"), w.get("pwm_pin")):
                if p is not None and p >= 0:
                    self._force_pin_low(p)

    # -----------------------------------------------------------------------
    # Actuator Control API
    # -----------------------------------------------------------------------
    def emergency_brake(self) -> None:
        """Trigger instant active emergency brake and clamp all duty cycles to 0."""
        with self._lock:
            self._emergency_stop = True
            self._target_duties = [0.0, 0.0, 0.0, 0.0]

        # Fast direct active brake (LOW/LOW on all direction and PWM pins)
        wheels = self.config.get("wheels", {})
        for w in wheels.values():
            for p in (w.get("in1_pin"), w.get("in2_pin"), w.get("pwm_pin")):
                if p is not None and p >= 0:
                    self._force_pin_low(p)

    def is_emergency_stopped(self) -> bool:
        return self._emergency_stop

    def reset_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stop = False
            self._watchdog_tripped = False
            self._last_command_time = time.perf_counter()

    def set_watchdog_enabled(self, enabled: bool) -> None:
        self._watchdog_enabled = enabled

    def set_raw_channel_duty(self, channel_idx: int, duty: float) -> None:
        """Directly command a raw H-bridge channel index (0..3). Used by Calibration Wizard."""
        if self._emergency_stop or channel_idx < 0 or channel_idx >= 4:
            return
        with self._lock:
            self._last_command_time = time.perf_counter()
            self._target_duties[channel_idx] = clamp(duty, -1.0, 1.0)

    def set_wheel_duties(self, fl: float, fr: float, rl: float, rr: float) -> None:
        """Set individual wheel duty cycles [-1.0, 1.0]."""
        if self._emergency_stop:
            return

        with self._lock:
            self._last_command_time = time.perf_counter()
            wheels = self.config.get("wheels", {})

            wheel_inputs = {"FL": fl, "FR": fr, "RL": rl, "RR": rr}
            for name, duty in wheel_inputs.items():
                cfg = wheels.get(name)
                if not cfg:
                    continue
                ch = cfg.get("channel", -1)
                if 0 <= ch < 4:
                    trim = cfg.get("trim", 1.0)
                    invert = cfg.get("invert", False)
                    applied = duty * trim
                    if invert:
                        applied = -applied
                    self._target_duties[ch] = clamp(applied, -1.0, 1.0)

    def set_duty_cycles(self, duty_left: float, duty_right: float) -> None:
        """
        Omni-Tank 2-DoF Differential command:
          Left bank  = Front-Left (FL) and Rear-Left (RL)
          Right bank = Front-Right (FR) and Rear-Right (RR)
        """
        if self._emergency_stop:
            return

        deadband = self.config.get("deadband", 0.15)
        dl = clamp(duty_left, -1.0, 1.0)
        dr = clamp(duty_right, -1.0, 1.0)

        # Stiction deadband compensation
        if abs(dl) > 1e-3:
            dl += deadband if dl > 0.0 else -deadband
        if abs(dr) > 1e-3:
            dr += deadband if dr > 0.0 else -deadband

        dl = clamp(dl, -1.0, 1.0)
        dr = clamp(dr, -1.0, 1.0)

        self.set_wheel_duties(fl=dl, fr=dr, rl=dl, rr=dr)

    # -----------------------------------------------------------------------
    # Background Software PWM and Watchdog Worker
    # -----------------------------------------------------------------------
    def _worker_loop(self) -> None:
        freq_hz = self.config.get("pwm_frequency_hz", 250)
        period_ns = int(1_000_000_000 / freq_hz)
        timeout_sec = self._watchdog_timeout_ms / 1000.0

        while self._running:
            loop_start = time.perf_counter_ns()

            # 1. Watchdog check
            now_sec = time.perf_counter()
            if self._watchdog_enabled and (
                now_sec - self._last_command_time > timeout_sec
            ):
                if not self._watchdog_tripped:
                    self._watchdog_tripped = True
                with self._lock:
                    self._target_duties = [0.0, 0.0, 0.0, 0.0]

            with self._lock:
                duties = list(self._target_duties)
                e_stopped = self._emergency_stop

            if e_stopped:
                duties = [0.0, 0.0, 0.0, 0.0]

            # 2. Update outputs: Direct-IN PWM mode (4-wire per board) or 3-pin mode
            if not self.is_simulated:
                wheels = self.config.get("wheels", {})
                pulse_pins: Dict[int, int] = {}  # pin -> on_time_ns

                for name, w in wheels.items():
                    ch = w.get("channel", -1)
                    d = duties[ch] if (0 <= ch < 4) else 0.0
                    in1 = w.get("in1_pin", -1)
                    in2 = w.get("in2_pin", -1)
                    pwm_p = w.get("pwm_pin", -1)
                    on_time = int(abs(d) * period_ns)

                    if pwm_p < 0:
                        # Direct-IN PWM Mode (ENA/ENB jumper caps ON)
                        if abs(d) < 1e-3:
                            self._write_gpio(in1, 0)
                            self._write_gpio(in2, 0)
                        elif d > 0.0:
                            # Forward: IN2 is LOW, IN1 pulses
                            self._write_gpio(in2, 0)
                            if on_time > 0:
                                pulse_pins[in1] = on_time
                            else:
                                self._write_gpio(in1, 0)
                        else:
                            # Reverse: IN1 is LOW, IN2 pulses
                            self._write_gpio(in1, 0)
                            if on_time > 0:
                                pulse_pins[in2] = on_time
                            else:
                                self._write_gpio(in2, 0)
                    else:
                        # 3-Pin Mode (Dedicated PWM on ENA/ENB)
                        if abs(d) < 1e-3:
                            self._write_gpio(in1, 0)
                            self._write_gpio(in2, 0)
                        elif d > 0.0:
                            self._write_gpio(in1, 1)
                            self._write_gpio(in2, 0)
                        else:
                            self._write_gpio(in1, 0)
                            self._write_gpio(in2, 1)

                        if on_time > 0:
                            pulse_pins[pwm_p] = on_time
                        else:
                            self._write_gpio(pwm_p, 0)

                # Set active pulsing pins HIGH at start of period
                for p in pulse_pins:
                    self._write_gpio(p, 1)

                # Wait until period expires, dropping pins to LOW as their duty threshold expires
                while True:
                    cur_ns = time.perf_counter_ns()
                    elapsed_ns = cur_ns - loop_start
                    if elapsed_ns >= period_ns:
                        break

                    for p, on_time in list(pulse_pins.items()):
                        if on_time > 0 and elapsed_ns >= on_time:
                            self._write_gpio(p, 0)
                            pulse_pins[p] = 0

                    time.sleep(0.001)

                # End of period: ensure all pulse pins are LOW
                for p in pulse_pins:
                    self._write_gpio(p, 0)
            else:
                # Simulated mode: sleep the full period
                time.sleep(period_ns / 1_000_000_000.0)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.emergency_brake()
        self.shutdown_hardware()
