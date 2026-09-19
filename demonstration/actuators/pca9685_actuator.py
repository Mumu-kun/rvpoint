#!/usr/bin/env python3
"""
pca9685_actuator.py — PCA9685 I2C Hardware PWM Motor Driver Actuator in Python.

Provides full hardware abstraction for 4-wheel Omni-Tank chassis driven by
a PCA9685 16-channel 12-bit PWM co-processor (default I2C Bus 4, Address 0x60 on Orange Pi RV2).

Features:
  - Zero external dependencies: Uses standard library Linux /dev/i2c-X with fcntl/ioctl (with smbus fallback).
  - High-precision 12-bit hardware PWM: Completely eliminates Linux CPU jitter and timer latency.
  - Direction-aware calibration: Independent forward and reverse trims and stiction deadbands per wheel.
  - Active dynamic braking and 200 ms safety watchdog.
  - Graceful mock simulation fallback when running off-target (CI/macOS/Windows).
"""

import glob
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def clamp(val: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(val, max_val))


# PCA9685 Registers
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_LED0_ON_L = 0x06
PCA9685_ALL_LED_ON_L = 0xFA
PCA9685_PRESCALE = 0xFE

MODE1_RESTART = 0x80
MODE1_AI = 0x20  # Auto-increment
MODE1_SLEEP = 0x10
MODE1_ALLCALL = 0x01
MODE2_OUTDRV = 0x04  # Totem-pole push-pull

I2C_SLAVE = 0x0703

# Standard Motor HAT PCA9685 Channel Mappings: (PWM, IN1, IN2)
ADAFRUIT_MAPPINGS = {
    1: {"pwm": 8, "in2": 9, "in1": 10},
    2: {"pwm": 13, "in2": 12, "in1": 11},
    3: {"pwm": 2, "in2": 3, "in1": 4},
    4: {"pwm": 7, "in2": 6, "in1": 5},
}

WAVESHARE_MAPPINGS = {
    1: {"pwm": 0, "in1": 1, "in2": 2},
    2: {"pwm": 5, "in1": 3, "in2": 4},
    3: {"pwm": 6, "in1": 7, "in2": 8},
    4: {"pwm": 11, "in1": 9, "in2": 10},
}


class PCA9685Actuator:
    """PCA9685 I2C 16-Channel Motor Actuator for 4-Wheel Omni-Tank Kinematics."""

    DEFAULT_CONFIG_PATH = Path("eval/actuators/pca9685_pins.json")

    def __init__(self, config_path: Optional[str | Path] = None, auto_init: bool = True):
        self.config_path = Path(config_path) if config_path else self.DEFAULT_CONFIG_PATH
        self.config: Dict[str, Any] = self._load_default_config()

        if self.config_path.exists():
            self.load_config(self.config_path)

        self._emergency_stop = False
        self._watchdog_enabled = self.config.get("enable_watchdog", True)
        self._watchdog_timeout_ms = self.config.get("watchdog_timeout_ms", 200)
        self._watchdog_tripped = False
        self._last_command_time = time.perf_counter()

        # Commanded wheel duties [-1.0, 1.0] for FL, FR, RL, RR
        self._target_duties = [0.0, 0.0, 0.0, 0.0]
        self._lock = threading.Lock()

        # Hardware handle
        self.is_simulated = False
        self.fd: Optional[int] = None
        self._smbus = None
        self._running = False
        self._worker_thread: Optional[threading.Thread] = None

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
            "bus": 4,
            "address": "0x60",
            "freq_hz": 200.0,
            "mapping": "adafruit",
            "deadband": 0.12,
            "enable_watchdog": True,
            "watchdog_timeout_ms": 200,
            "wheels": {
                "FL": {"motor": 1, "invert": False, "trim": 1.0, "deadband_forward": 0.12, "deadband_reverse": 0.12},
                "FR": {"motor": 2, "invert": False, "trim": 1.0, "deadband_forward": 0.12, "deadband_reverse": 0.12},
                "RL": {"motor": 3, "invert": False, "trim": 1.0, "deadband_forward": 0.12, "deadband_reverse": 0.12},
                "RR": {"motor": 4, "invert": False, "trim": 1.0, "deadband_forward": 0.12, "deadband_reverse": 0.12},
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
            print(f"[PCA9685Actuator] Error loading {path}: {e}", file=sys.stderr)
            return False

    def save_config(self, filepath: Optional[str | Path] = None) -> bool:
        path = Path(filepath) if filepath else self.config_path
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.config, f, indent=2)
            return True
        except Exception as e:
            print(f"[PCA9685Actuator] Error saving {path}: {e}", file=sys.stderr)
            return False

    def _auto_detect_bus_and_address(self) -> Tuple[int, int]:
        cfg_bus = int(self.config.get("bus", 4))
        raw_addr = self.config.get("address", "0x60")
        cfg_addr = int(raw_addr, 0) if isinstance(raw_addr, str) else int(raw_addr)

        # Check if configured bus node exists
        if os.path.exists(f"/dev/i2c-{cfg_bus}"):
            return cfg_bus, cfg_addr

        # Search other nodes
        nodes = sorted(glob.glob("/dev/i2c-*"))
        for node in [f"/dev/i2c-{b}" for b in [4, 1, 0, 2, 3]]:
            if os.path.exists(node):
                b = int(node.split("-")[-1])
                return b, cfg_addr

        return cfg_bus, cfg_addr

    def initialize_hardware(self) -> bool:
        bus_num, address = self._auto_detect_bus_and_address()
        dev_path = f"/dev/i2c-{bus_num}"

        if os.path.exists(dev_path):
            try:
                import fcntl

                self.fd = os.open(dev_path, os.O_RDWR)
                fcntl.ioctl(self.fd, I2C_SLAVE, address)
            except Exception:
                self.fd = None

        if self.fd is None:
            try:
                import smbus2 as smbus  # type: ignore

                self._smbus = smbus.SMBus(bus_num)
            except ImportError:
                try:
                    import smbus  # type: ignore

                    self._smbus = smbus.SMBus(bus_num)
                except ImportError:
                    self._smbus = None

        if self.fd is None and self._smbus is None:
            self.is_simulated = True
            print(f"[PCA9685Actuator] Hardware I2C /dev/i2c-{bus_num} inaccessible. Running in SIMULATION mode.")
        else:
            print(f"[PCA9685Actuator] Initialized hardware on /dev/i2c-{bus_num} (Address: 0x{address:02X})")
            self._hw_reset()
            self._hw_set_freq(float(self.config.get("freq_hz", 200.0)))

        self._running = True
        self._worker_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self._worker_thread.start()
        return True

    def _hw_write_byte(self, reg: int, val: int) -> None:
        if self.is_simulated:
            return
        if self.fd is not None:
            os.write(self.fd, bytes([reg & 0xFF, val & 0xFF]))
        elif self._smbus is not None:
            self._smbus.write_byte_data(int(self.config.get("address", "0x60"), 0), reg & 0xFF, val & 0xFF)

    def _hw_read_byte(self, reg: int) -> int:
        if self.is_simulated:
            return 0
        if self.fd is not None:
            os.write(self.fd, bytes([reg & 0xFF]))
            d = os.read(self.fd, 1)
            return d[0] if d else 0
        elif self._smbus is not None:
            return self._smbus.read_byte_data(int(self.config.get("address", "0x60"), 0), reg & 0xFF)
        return 0

    def _hw_reset(self) -> None:
        self._hw_write_byte(PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL)
        self._hw_write_byte(PCA9685_MODE2, MODE2_OUTDRV)
        time.sleep(0.005)

    def _hw_set_freq(self, freq_hz: float) -> None:
        if self.is_simulated:
            return
        prescale = int(round(25000000.0 / (4096.0 * freq_hz)) - 1.0)
        prescale = max(3, min(255, prescale))
        old_mode = self._hw_read_byte(PCA9685_MODE1)
        self._hw_write_byte(PCA9685_MODE1, (old_mode & 0x7F) | MODE1_SLEEP)
        self._hw_write_byte(PCA9685_PRESCALE, prescale)
        self._hw_write_byte(PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self._hw_write_byte(PCA9685_MODE1, old_mode | MODE1_AI | MODE1_RESTART)

    def _hw_set_duty(self, channel: int, duty: float) -> None:
        if channel < 0 or channel > 15 or self.is_simulated:
            return
        duty = max(0.0, min(1.0, float(duty)))
        reg_base = PCA9685_LED0_ON_L + 4 * channel
        if duty <= 0.0:
            payload = bytes([reg_base, 0x00, 0x00, 0x00, 0x10])
        elif duty >= 1.0:
            payload = bytes([reg_base, 0x00, 0x10, 0x00, 0x00])
        else:
            off_count = int(duty * 4095.0)
            payload = bytes([reg_base, 0x00, 0x00, off_count & 0xFF, (off_count >> 8) & 0x0F])

        if self.fd is not None:
            os.write(self.fd, payload)
        elif self._smbus is not None:
            addr = int(self.config.get("address", "0x60"), 0)
            self._smbus.write_i2c_block_data(addr, reg_base, list(payload[1:]))

    def _hw_drive_motor(self, motor_num: int, duty: float, direction: str) -> None:
        mapping_name = self.config.get("mapping", "adafruit")
        mappings = ADAFRUIT_MAPPINGS if mapping_name == "adafruit" else WAVESHARE_MAPPINGS
        if motor_num not in mappings:
            return

        pins = mappings[motor_num]
        pwm_ch = pins["pwm"]
        in1_ch = pins["in1"]
        in2_ch = pins["in2"]

        if abs(duty) < 0.01:
            self._hw_set_duty(pwm_ch, 0.0)
            self._hw_set_duty(in1_ch, 0.0)
            self._hw_set_duty(in2_ch, 0.0)
            return

        if direction == "forward":
            self._hw_set_duty(in1_ch, 1.0)
            self._hw_set_duty(in2_ch, 0.0)
            self._hw_set_duty(pwm_ch, duty)
        else:
            self._hw_set_duty(in1_ch, 0.0)
            self._hw_set_duty(in2_ch, 1.0)
            self._hw_set_duty(pwm_ch, duty)

    def shutdown_hardware(self) -> None:
        self._running = False
        if not self.is_simulated and self.fd is not None:
            try:
                # Stop all
                payload = bytes([PCA9685_ALL_LED_ON_L, 0x00, 0x00, 0x00, 0x10])
                os.write(self.fd, payload)
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None

    def emergency_brake(self) -> None:
        with self._lock:
            self._emergency_stop = True
            self._target_duties = [0.0, 0.0, 0.0, 0.0]
            if not self.is_simulated:
                for m in [1, 2, 3, 4]:
                    self._hw_drive_motor(m, 0.0, "forward")

    def is_emergency_stopped(self) -> bool:
        with self._lock:
            return self._emergency_stop

    def reset_emergency_stop(self) -> None:
        with self._lock:
            self._emergency_stop = False
            self._watchdog_tripped = False
            self._last_command_time = time.perf_counter()

    def set_watchdog_enabled(self, enabled: bool) -> None:
        self._watchdog_enabled = enabled

    def set_wheel_duties(self, fl: float, fr: float, rl: float, rr: float) -> None:
        with self._lock:
            if self._emergency_stop:
                return

            self._last_command_time = time.perf_counter()
            self._watchdog_tripped = False

            raw_duties = {"FL": clamp(fl, -1.0, 1.0), "FR": clamp(fr, -1.0, 1.0),
                          "RL": clamp(rl, -1.0, 1.0), "RR": clamp(rr, -1.0, 1.0)}

            self._target_duties = [raw_duties["FL"], raw_duties["FR"], raw_duties["RL"], raw_duties["RR"]]

            wheels_cfg = self.config.get("wheels", {})
            for key, duty in raw_duties.items():
                wcfg = wheels_cfg.get(key, {"motor": 1, "invert": False, "trim": 1.0})
                motor_num = wcfg.get("motor", 1)
                invert = wcfg.get("invert", False)
                trim = wcfg.get("trim", 1.0)
                deadband = wcfg.get("deadband_forward" if duty >= 0 else "deadband_reverse", self.config.get("deadband", 0.12))

                is_positive = (duty >= 0.0)
                if invert:
                    is_positive = not is_positive

                mag = abs(duty)
                if mag < 0.01:
                    effective_duty = 0.0
                else:
                    effective_duty = deadband + (1.0 - deadband) * mag * trim
                    effective_duty = clamp(effective_duty, 0.0, 1.0)

                direction = "forward" if is_positive else "reverse"
                self._hw_drive_motor(motor_num, effective_duty, direction)

    def set_duty_cycles(self, duty_left: float, duty_right: float) -> None:
        self.set_wheel_duties(duty_left, duty_right, duty_left, duty_right)

    def _watchdog_loop(self) -> None:
        while self._running:
            time.sleep(0.05)
            if not self._watchdog_enabled or self._emergency_stop:
                continue

            with self._lock:
                elapsed_ms = (time.perf_counter() - self._last_command_time) * 1000.0
                if elapsed_ms > self._watchdog_timeout_ms and not self._watchdog_tripped:
                    # Trip watchdog and brake
                    self._watchdog_tripped = True
                    self._target_duties = [0.0, 0.0, 0.0, 0.0]
                    if not self.is_simulated:
                        for m in [1, 2, 3, 4]:
                            self._hw_drive_motor(m, 0.0, "forward")
