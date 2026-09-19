#!/usr/bin/env python3
"""
test_pca9685.py — Zero-Dependency PCA9685 I2C Motor & Calibration Wizard.

Designed for Orange Pi RV2 (/dev/i2c-4, addr 0x60/0x40) and Raspberry Pi (/dev/i2c-1).
Supports interactive calibration, polarity flipping, directional verification, and teleoperation.

Features:
  1. Guided Calibration Wizard (--wizard):
     Sequentially pulses each motor, asks which wheel moved (FL, FR, RL, RR)
     and whether it rotated Forward or Reverse. Saves configuration to
     eval/actuators/pca9685_pins.json.
  2. Individual Wheel Polarity Test (--test individual):
     Tests each wheel: Forward (1.5s) -> Brake -> Reverse (1.5s) -> Brake.
  3. Directional Motions Test (--test directional):
     Forward -> Reverse -> Pivot Left -> Pivot Right under 4-wheel Omni-Tank kinematics.
  4. Interactive Terminal Teleoperation (--teleop):
     Real-time keyboard driving (WASD + Spacebar Brake).
  5. Single Motor / Channel Test (--motor <1-4> or --channel <0-15> --duty <0.0-1.0>):
     Quick verification pulse.
  6. Breakaway Stiction Sweep (--sweep):
     Ramps duty from 5% to 50% to observe gearbox breakaway threshold.
  7. Emergency Stop (--stop):
     Immediately shuts off all channels.

Usage Examples:
  # 1. Run guided calibration wizard to identify FL, FR, RL, RR and polarities:
  python3 demonstration/test_pca9685.py --wizard

  # 2. Verify individual wheel forward/reverse polarities:
  python3 demonstration/test_pca9685.py --test individual

  # 3. Test 4-wheel directional movements (Forward, Reverse, Pivot Left, Pivot Right):
  python3 demonstration/test_pca9685.py --test directional

  # 4. Drive via keyboard (WASD, Space=Brake, Q=Quit):
  python3 demonstration/test_pca9685.py --teleop
"""

import argparse
import glob
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any

# PCA9685 Register Definitions
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_LED0_ON_L = 0x06
PCA9685_LED0_ON_H = 0x07
PCA9685_LED0_OFF_L = 0x08
PCA9685_LED0_OFF_H = 0x09
PCA9685_ALL_LED_ON_L = 0xFA
PCA9685_ALL_LED_ON_H = 0xFB
PCA9685_ALL_LED_OFF_L = 0xFC
PCA9685_ALL_LED_OFF_H = 0xFD
PCA9685_PRESCALE = 0xFE

# MODE1 Bits
MODE1_RESTART = 0x80
MODE1_AI = 0x20  # Auto-Increment
MODE1_SLEEP = 0x10  # Sleep mode (oscillator off)
MODE1_ALLCALL = 0x01

# MODE2 Bits
MODE2_OUTDRV = 0x04  # Totem-pole output (push-pull)

# Linux I2C ioctl
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

CONFIG_FILE = Path("eval/actuators/pca9685_pins.json")


def scan_i2c_bus(bus_num: int) -> List[int]:
    """Scans standard 7-bit I2C addresses (0x03 to 0x77) on Linux."""
    found: List[int] = []
    dev_path = f"/dev/i2c-{bus_num}"
    if not os.path.exists(dev_path):
        return found

    try:
        import fcntl

        fd = os.open(dev_path, os.O_RDWR)
        for addr in range(0x03, 0x78):
            try:
                fcntl.ioctl(fd, I2C_SLAVE, addr)
                os.read(fd, 1)
                found.append(addr)
            except Exception:
                pass
        os.close(fd)
    except Exception:
        pass

    return found


def auto_detect_bus_and_address() -> Tuple[int, int]:
    """Searches for PCA9685 (0x60 or 0x40) across available /dev/i2c-* nodes."""
    nodes = sorted(glob.glob("/dev/i2c-*"))
    bus_nums: List[int] = []
    for node in nodes:
        try:
            bus_nums.append(int(node.split("-")[-1]))
        except ValueError:
            pass

    priority_order = [4, 1, 0, 2, 3] + [b for b in bus_nums if b not in [4, 1, 0, 2, 3]]
    for bus in priority_order:
        dev_path = f"/dev/i2c-{bus}"
        if not os.path.exists(dev_path):
            continue
        addrs = scan_i2c_bus(bus)
        if 0x60 in addrs:
            return bus, 0x60
        if 0x40 in addrs:
            return bus, 0x40

    return (4 if os.path.exists("/dev/i2c-4") else 1), 0x60


class PCA9685Driver:
    """Zero-dependency hardware controller for PCA9685 over Linux /dev/i2c-X."""

    def __init__(self, bus_num: int = 4, address: int = 0x60, freq_hz: float = 200.0):
        self.bus_num = bus_num
        self.address = address
        self.freq_hz = freq_hz
        self.is_simulated = False
        self.fd: Optional[int] = None
        self._smbus = None

        dev_path = f"/dev/i2c-{bus_num}"
        if os.path.exists(dev_path):
            try:
                import fcntl

                self.fd = os.open(dev_path, os.O_RDWR)
                fcntl.ioctl(self.fd, I2C_SLAVE, self.address)
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
            print(f"[PCA9685] Running in SIMULATION mode (No hardware /dev/i2c-{bus_num} available).")
            return

        self.reset()
        self.set_pwm_freq(freq_hz)

    def write_byte(self, reg: int, val: int) -> None:
        if self.is_simulated:
            return
        if self.fd is not None:
            os.write(self.fd, bytes([reg & 0xFF, val & 0xFF]))
        elif self._smbus is not None:
            self._smbus.write_byte_data(self.address, reg & 0xFF, val & 0xFF)

    def read_byte(self, reg: int) -> int:
        if self.is_simulated:
            return 0x00
        if self.fd is not None:
            os.write(self.fd, bytes([reg & 0xFF]))
            data = os.read(self.fd, 1)
            return data[0] if data else 0
        elif self._smbus is not None:
            return self._smbus.read_byte_data(self.address, reg & 0xFF)
        return 0

    def reset(self) -> None:
        if self.is_simulated:
            return
        self.write_byte(PCA9685_MODE1, MODE1_AI | MODE1_ALLCALL)
        self.write_byte(PCA9685_MODE2, MODE2_OUTDRV)
        time.sleep(0.005)

    def set_pwm_freq(self, freq_hz: float) -> None:
        self.freq_hz = freq_hz
        if self.is_simulated:
            return

        prescale = int(round(25000000.0 / (4096.0 * freq_hz)) - 1.0)
        prescale = max(3, min(255, prescale))

        old_mode = self.read_byte(PCA9685_MODE1)
        sleep_mode = (old_mode & 0x7F) | MODE1_SLEEP
        self.write_byte(PCA9685_MODE1, sleep_mode)
        self.write_byte(PCA9685_PRESCALE, prescale)
        self.write_byte(PCA9685_MODE1, old_mode)
        time.sleep(0.005)
        self.write_byte(PCA9685_MODE1, old_mode | MODE1_AI | MODE1_RESTART)

    def set_duty(self, channel: int, duty: float) -> None:
        duty = max(0.0, min(1.0, float(duty)))
        if channel < 0 or channel > 15:
            raise ValueError(f"Channel {channel} out of range (0-15)")

        if self.is_simulated:
            return

        reg_base = PCA9685_LED0_ON_L + 4 * channel
        if duty <= 0.0:
            payload = bytes([reg_base, 0x00, 0x00, 0x00, 0x10])
        elif duty >= 1.0:
            payload = bytes([reg_base, 0x00, 0x10, 0x00, 0x00])
        else:
            off_count = int(duty * 4095.0)
            payload = bytes([
                reg_base,
                0x00,
                0x00,
                off_count & 0xFF,
                (off_count >> 8) & 0x0F,
            ])

        if self.fd is not None:
            os.write(self.fd, payload)
        elif self._smbus is not None:
            self._smbus.write_i2c_block_data(self.address, reg_base, list(payload[1:]))

    def set_digital(self, channel: int, state: bool) -> None:
        self.set_duty(channel, 1.0 if state else 0.0)

    def drive_motor(
        self,
        motor_num: int,
        duty: float,
        direction: str = "forward",
        mapping_name: str = "adafruit",
    ) -> None:
        mappings = ADAFRUIT_MAPPINGS if mapping_name == "adafruit" else WAVESHARE_MAPPINGS
        if motor_num not in mappings:
            raise ValueError(f"Motor {motor_num} not found. Choose 1, 2, 3, or 4.")

        pins = mappings[motor_num]
        pwm_ch = pins["pwm"]
        in1_ch = pins["in1"]
        in2_ch = pins["in2"]

        if abs(duty) < 0.01:
            self.set_duty(pwm_ch, 0.0)
            self.set_digital(in1_ch, False)
            self.set_digital(in2_ch, False)
            return

        if direction.lower() in ["forward", "fwd", "f"]:
            self.set_digital(in1_ch, True)
            self.set_digital(in2_ch, False)
            self.set_duty(pwm_ch, duty)
        elif direction.lower() in ["reverse", "rev", "r"]:
            self.set_digital(in1_ch, False)
            self.set_digital(in2_ch, True)
            self.set_duty(pwm_ch, duty)
        else:
            raise ValueError(f"Unknown direction '{direction}'. Use 'forward' or 'reverse'.")

    def brake_motor(self, motor_num: int, mapping_name: str = "adafruit") -> None:
        self.drive_motor(motor_num, 0.0, "forward", mapping_name)

    def stop_all(self) -> None:
        if self.is_simulated:
            return
        payload = bytes([PCA9685_ALL_LED_ON_L, 0x00, 0x00, 0x00, 0x10])
        if self.fd is not None:
            os.write(self.fd, payload)
        elif self._smbus is not None:
            self._smbus.write_i2c_block_data(self.address, PCA9685_ALL_LED_ON_L, list(payload[1:]))

    def close(self) -> None:
        self.stop_all()
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
            self.fd = None


# ---------------------------------------------------------------------------
# Calibration Configuration Manager
# ---------------------------------------------------------------------------
def load_wheel_config(config_path: Path = CONFIG_FILE) -> Dict[str, Any]:
    if config_path.exists():
        try:
            with open(config_path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass

    return {
        "mapping": "adafruit",
        "wheels": {
            "FL": {"motor": 1, "invert": False, "trim": 1.0},
            "FR": {"motor": 2, "invert": False, "trim": 1.0},
            "RL": {"motor": 3, "invert": False, "trim": 1.0},
            "RR": {"motor": 4, "invert": False, "trim": 1.0},
        },
    }


def save_wheel_config(config: Dict[str, Any], config_path: Path = CONFIG_FILE) -> bool:
    try:
        config_path.parent.mkdir(parents=True, exist_ok=True)
        with open(config_path, "w", encoding="utf-8") as f:
            json.dump(config, f, indent=2)
        return True
    except Exception as e:
        print(f"Error saving {config_path}: {e}", file=sys.stderr)
        return False


def drive_wheel(
    driver: PCA9685Driver,
    config: Dict[str, Any],
    wheel_key: str,
    duty: float,
    mapping_name: str,
) -> None:
    """Drives a specific wheel (FL, FR, RL, RR) with direction and invert applied."""
    wcfg = config["wheels"].get(wheel_key, {"motor": 1, "invert": False, "trim": 1.0})
    motor_num = wcfg.get("motor", 1)
    invert = wcfg.get("invert", False)
    trim = wcfg.get("trim", 1.0)

    effective_duty = max(0.0, min(1.0, abs(duty) * trim))
    if effective_duty < 0.01:
        driver.brake_motor(motor_num, mapping_name)
        return

    # Determine direction
    is_positive = (duty >= 0.0)
    if invert:
        is_positive = not is_positive

    direction = "forward" if is_positive else "reverse"
    driver.drive_motor(motor_num, effective_duty, direction, mapping_name)


# ---------------------------------------------------------------------------
# Guided Calibration Wizard
# ---------------------------------------------------------------------------
def run_wizard(driver: PCA9685Driver, mapping_name: str, test_duty: float = 0.40) -> None:
    print("\n" + "=" * 65)
    print("      PCA9685 GUIDED WHEEL & POLARITY CALIBRATION WIZARD         ")
    print("=" * 65)
    print("\nThis wizard will pulse each motor terminal (1, 2, 3, 4) one by one.")
    print("For each motor, you will tell the wizard:")
    print("  1. Which physical wheel spun: FL, FR, RL, or RR?")
    print("  2. Did it rotate FORWARD (toward front of car) or REVERSE?")
    print("\nSAFETY NOTICE: Place vehicle on a stand so wheels spin freely!")

    input("\nPress [ENTER] to start...")

    wheel_keys = ["FL", "FR", "RL", "RR"]
    wheel_labels = [
        "Front-Left (FL)",
        "Front-Right (FR)",
        "Rear-Left (RL)",
        "Rear-Right (RR)",
    ]

    new_wheels: Dict[str, Dict[str, Any]] = {
        "FL": {"motor": 1, "invert": False, "trim": 1.0},
        "FR": {"motor": 2, "invert": False, "trim": 1.0},
        "RL": {"motor": 3, "invert": False, "trim": 1.0},
        "RR": {"motor": 4, "invert": False, "trim": 1.0},
    }

    assigned_wheels = set()

    for m in [1, 2, 3, 4]:
        while True:
            print("\n" + "-" * 65)
            print(f">>> Pulsing Motor {m} FORWARD for 1.5s at {int(test_duty * 100)}% duty...")
            print("-" * 65)

            driver.drive_motor(m, test_duty, "forward", mapping_name)
            time.sleep(1.5)
            driver.brake_motor(m, mapping_name)

            print("\nWhich wheel just spun?")
            print("  [1] Front-Left  (FL)")
            print("  [2] Front-Right (FR)")
            print("  [3] Rear-Left   (RL)")
            print("  [4] Rear-Right  (RR)")
            print("  [p] Pulse Motor " + str(m) + " again for 1.5s")
            print("  [s] Skip this motor")

            choice_str = input("Selection [1-4, p, s]: ").strip().lower()
            if choice_str == "p":
                continue
            if choice_str == "s":
                print(f"Skipping Motor {m}.")
                break

            try:
                choice = int(choice_str)
                if 1 <= choice <= 4:
                    target_key = wheel_keys[choice - 1]
                    target_label = wheel_labels[choice - 1]
                    break
            except Exception:
                pass
            print("Invalid selection. Enter 1, 2, 3, 4, p, or s.")

        if choice_str == "s":
            continue

        dir_choice = ""
        while dir_choice not in ["f", "forward", "r", "reverse"]:
            dir_choice = input(f"Did {target_label} rotate FORWARD or REVERSE? [f/r]: ").strip().lower()

        is_reverse = dir_choice.startswith("r")
        new_wheels[target_key]["motor"] = m
        new_wheels[target_key]["invert"] = is_reverse
        assigned_wheels.add(target_key)

        print(f"\n✅ Assigned Motor {m} -> {target_label} (Invert Polarity: {'YES (Reversed)' if is_reverse else 'NO (Normal)'})")

    # Summary
    print("\n" + "=" * 65)
    print(">>> CALIBRATION SUMMARY <<<")
    print("=" * 65)
    config = {
        "mapping": mapping_name,
        "wheels": new_wheels,
    }
    for key, label in zip(wheel_keys, wheel_labels):
        w = new_wheels[key]
        print(f"  {label:<18}: Motor {w['motor']} | Invert: {'YES' if w['invert'] else 'NO':<3} | Trim: {w['trim']:.2f}")

    if save_wheel_config(config):
        print(f"\n[Saved] Configuration saved to {CONFIG_FILE} successfully!")
    else:
        print(f"\n[Error] Could not write configuration to {CONFIG_FILE}.", file=sys.stderr)

    # All-forward verification test
    confirm = input("\nRun simultaneous 2.0s FORWARD verification spin on all 4 wheels? [Y/n]: ").strip().lower()
    if confirm in ("", "y", "yes"):
        print("\n[VERIFICATION] Spinning all 4 wheels FORWARD for 2.0s...")
        for key in wheel_keys:
            drive_wheel(driver, config, key, test_duty, mapping_name)
        time.sleep(2.0)
        driver.stop_all()
        print("[VERIFICATION] All motors stopped. If all wheels spun forward, calibration is COMPLETE!")


# ---------------------------------------------------------------------------
# Individual Wheel Polarity Test
# ---------------------------------------------------------------------------
def run_individual_test(driver: PCA9685Driver, mapping_name: str, test_duty: float = 0.40) -> None:
    config = load_wheel_config()
    print("\n" + "=" * 65)
    print(">>> INDIVIDUAL WHEEL POLARITY VERIFICATION TEST <<<")
    print("=" * 65)
    print(f"Loaded config from {CONFIG_FILE} (Mapping: {mapping_name})")
    print("Testing each wheel: Forward (1.5s) -> Brake (0.5s) -> Reverse (1.5s) -> Brake\n")

    wheel_keys = ["FL", "FR", "RL", "RR"]
    wheel_labels = ["Front-Left (FL)", "Front-Right (FR)", "Rear-Left (RL)", "Rear-Right (RR)"]

    for key, label in zip(wheel_keys, wheel_labels):
        w = config["wheels"].get(key, {"motor": 1, "invert": False})
        print(f"\n=== Testing {label} (Motor {w.get('motor')}, Invert: {w.get('invert')}) ===")

        # Forward
        print(f"  -> FORWARD (+{int(test_duty * 100)}%)... ", end="", flush=True)
        drive_wheel(driver, config, key, test_duty, mapping_name)
        time.sleep(1.5)
        drive_wheel(driver, config, key, 0.0, mapping_name)
        print("[BRAKE]")
        time.sleep(0.5)

        # Reverse
        print(f"  -> REVERSE (-{int(test_duty * 100)}%)... ", end="", flush=True)
        drive_wheel(driver, config, key, -test_duty, mapping_name)
        time.sleep(1.5)
        drive_wheel(driver, config, key, 0.0, mapping_name)
        print("[BRAKE]")
        time.sleep(0.5)

    print("\nIndividual wheel test complete.")


# ---------------------------------------------------------------------------
# Directional Motion Test
# ---------------------------------------------------------------------------
def run_directional_test(driver: PCA9685Driver, mapping_name: str, test_duty: float = 0.45) -> None:
    config = load_wheel_config()
    print("\n" + "=" * 65)
    print(">>> 4-WHEEL DIRECTIONAL MOTIONS TEST (Omni-Tank Kinematics) <<<")
    print("=" * 65)
    print("Sequence: Forward -> Brake -> Reverse -> Brake -> Pivot Left -> Brake -> Pivot Right")

    wheel_keys = ["FL", "FR", "RL", "RR"]

    def set_all(d_fl: float, d_fr: float, d_rl: float, d_rr: float, duration: float, name: str):
        print(f"\n[ACTION] {name} ({duration}s)...")
        drive_wheel(driver, config, "FL", d_fl, mapping_name)
        drive_wheel(driver, config, "FR", d_fr, mapping_name)
        drive_wheel(driver, config, "RL", d_rl, mapping_name)
        drive_wheel(driver, config, "RR", d_rr, mapping_name)
        time.sleep(duration)
        driver.stop_all()
        time.sleep(0.6)

    # 1. Forward
    set_all(test_duty, test_duty, test_duty, test_duty, 1.8, "FORWARD")
    # 2. Reverse
    set_all(-test_duty, -test_duty, -test_duty, -test_duty, 1.8, "REVERSE")
    # 3. Pivot Left (Left wheels reverse, Right wheels forward)
    set_all(-test_duty, test_duty, -test_duty, test_duty, 1.5, "PIVOT LEFT (Counter-Clockwise)")
    # 4. Pivot Right (Left wheels forward, Right wheels reverse)
    set_all(test_duty, -test_duty, test_duty, -test_duty, 1.5, "PIVOT RIGHT (Clockwise)")

    print("\nDirectional verification completed successfully.")


# ---------------------------------------------------------------------------
# Terminal Teleoperation (Keyboard Driving)
# ---------------------------------------------------------------------------
class KeyboardReader:
    def __init__(self):
        self.is_windows = os.name == "nt"
        self._orig_termios = None

    def __enter__(self):
        if not self.is_windows:
            try:
                import termios
                import tty

                if sys.stdin.isatty():
                    self._orig_termios = termios.tcgetattr(sys.stdin.fileno())
                    tty.setcbreak(sys.stdin.fileno())
            except Exception:
                pass
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not self.is_windows and self._orig_termios is not None:
            try:
                import termios

                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._orig_termios)
            except Exception:
                pass

    def get_char(self) -> Optional[str]:
        if self.is_windows:
            import msvcrt

            if msvcrt.kbhit():
                try:
                    return msvcrt.getch().decode("utf-8")
                except Exception:
                    return None
            return None
        else:
            import select

            if select.select([sys.stdin], [], [], 0.05)[0]:
                return sys.stdin.read(1)
            return None


def run_teleop(driver: PCA9685Driver, mapping_name: str, base_duty: float = 0.50) -> None:
    config = load_wheel_config()
    print("\n" + "=" * 65)
    print(">>> INTERACTIVE TERMINAL TELEOPERATION (WASD) <<<")
    print("=" * 65)
    print("Controls:")
    print("  [W] Forward           [S] Reverse")
    print("  [A] Pivot Left        [D] Pivot Right")
    print("  [SPACE] Instant Brake [Q / ESC] Quit Teleop")
    print(f"Current Speed: {int(base_duty * 100)}% duty\n")

    current_motion = "STOP"

    def apply_motion(cmd: str):
        nonlocal current_motion
        current_motion = cmd
        if cmd == "FORWARD":
            drive_wheel(driver, config, "FL", base_duty, mapping_name)
            drive_wheel(driver, config, "FR", base_duty, mapping_name)
            drive_wheel(driver, config, "RL", base_duty, mapping_name)
            drive_wheel(driver, config, "RR", base_duty, mapping_name)
        elif cmd == "REVERSE":
            drive_wheel(driver, config, "FL", -base_duty, mapping_name)
            drive_wheel(driver, config, "FR", -base_duty, mapping_name)
            drive_wheel(driver, config, "RL", -base_duty, mapping_name)
            drive_wheel(driver, config, "RR", -base_duty, mapping_name)
        elif cmd == "LEFT":
            drive_wheel(driver, config, "FL", -base_duty, mapping_name)
            drive_wheel(driver, config, "FR", base_duty, mapping_name)
            drive_wheel(driver, config, "RL", -base_duty, mapping_name)
            drive_wheel(driver, config, "RR", base_duty, mapping_name)
        elif cmd == "RIGHT":
            drive_wheel(driver, config, "FL", base_duty, mapping_name)
            drive_wheel(driver, config, "FR", -base_duty, mapping_name)
            drive_wheel(driver, config, "RL", base_duty, mapping_name)
            drive_wheel(driver, config, "RR", -base_duty, mapping_name)
        else:
            driver.stop_all()

    with KeyboardReader() as reader:
        last_key_time = time.time()
        while True:
            char = reader.get_char()
            now = time.time()

            if char:
                char_lower = char.lower()
                if char_lower == "w":
                    apply_motion("FORWARD")
                    print(f"\r[STATUS] FORWARD     (Duty: {int(base_duty * 100)}%)  ", end="", flush=True)
                elif char_lower == "s":
                    apply_motion("REVERSE")
                    print(f"\r[STATUS] REVERSE     (Duty: {int(base_duty * 100)}%)  ", end="", flush=True)
                elif char_lower == "a":
                    apply_motion("LEFT")
                    print(f"\r[STATUS] PIVOT LEFT  (Duty: {int(base_duty * 100)}%)  ", end="", flush=True)
                elif char_lower == "d":
                    apply_motion("RIGHT")
                    print(f"\r[STATUS] PIVOT RIGHT (Duty: {int(base_duty * 100)}%)  ", end="", flush=True)
                elif char == " " or char_lower == "x":
                    apply_motion("STOP")
                    print("\r[STATUS] BRAKE                                    ", end="", flush=True)
                elif char_lower in ("q", "\x1b"):  # ESC
                    apply_motion("STOP")
                    print("\nExiting teleoperation...")
                    break
                last_key_time = now

            # 300ms watchdog deadman timeout
            if current_motion != "STOP" and (now - last_key_time > 0.35):
                apply_motion("STOP")
                print("\r[STATUS] WATCHDOG STOP                            ", end="", flush=True)

            time.sleep(0.02)


# ---------------------------------------------------------------------------
# Main CLI Router
# ---------------------------------------------------------------------------
def main():
    detected_bus, detected_addr = auto_detect_bus_and_address()

    parser = argparse.ArgumentParser(
        description="PCA9685 I2C Hardware PWM Motor Driver & Calibration Wizard"
    )
    parser.add_argument("--bus", type=int, default=detected_bus, help=f"I2C bus number (auto-detected: {detected_bus})")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=detected_addr, help=f"I2C address in hex (auto-detected: 0x{detected_addr:02X})")
    parser.add_argument("--freq", type=float, default=200.0, help="PWM frequency in Hz (default: 200 Hz)")
    parser.add_argument("--mapping", choices=["adafruit", "waveshare"], default="adafruit", help="Motor HAT pinout mapping (default: adafruit)")
    parser.add_argument("--wizard", action="store_true", help="Launch interactive FL/FR/RL/RR polarity calibration wizard")
    parser.add_argument("--teleop", action="store_true", help="Interactive WASD keyboard driving")
    parser.add_argument("--scan", action="store_true", help="Scan I2C bus to find detected chips")
    parser.add_argument("--motor", type=int, choices=[1, 2, 3, 4], help="Motor HAT motor number [1, 2, 3, 4]")
    parser.add_argument("--dir", choices=["forward", "reverse"], default="forward", help="Direction for --motor (default: forward)")
    parser.add_argument("--channel", type=int, help="Raw PCA9685 PWM channel [0-15]")
    parser.add_argument("--duty", type=float, default=None, help="Target duty cycle [0.0 - 1.0]")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration in seconds to run motor test (default: 2.0s)")
    parser.add_argument("--sweep", action="store_true", help="Sweep duty from 5%% to 50%% to find breakaway torque")
    parser.add_argument("--test", choices=["individual", "directional", "all-motors"], help="Run automated test suite")
    parser.add_argument("--stop", action="store_true", help="Send immediate failsafe e-stop to all channels")

    args = parser.parse_args()

    # 1. Bus Scan Mode
    if args.scan:
        print(f"\n[Scanning /dev/i2c-{args.bus} for active I2C slave devices...]")
        detected = scan_i2c_bus(args.bus)
        if not detected:
            print("❌ No I2C devices detected on this bus.")
        else:
            print(f"✅ Found {len(detected)} device(s): " + ", ".join(f"0x{a:02X}" for a in detected))
            if 0x60 in detected:
                print("   👉 0x60: PCA9685 Motor Driver HAT ready!")
            if 0x70 in detected:
                print("   👉 0x70: PCA9685 All-Call broadcast verified!")
        return

    # Initialize Driver
    driver = PCA9685Driver(bus_num=args.bus, address=args.address, freq_hz=args.freq)

    def handle_exit(signum, frame):
        print("\n[Safe Stop] Interrupted. Stopping all motor outputs...")
        driver.stop_all()
        driver.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    try:
        if args.stop:
            print("[Action] Stopping all channels immediately...")
            driver.stop_all()
            print("Done. All outputs disabled.")
            return

        if args.wizard:
            duty = args.duty if args.duty is not None else 0.40
            run_wizard(driver, args.mapping, test_duty=duty)
            return

        if args.teleop:
            duty = args.duty if args.duty is not None else 0.50
            run_teleop(driver, args.mapping, base_duty=duty)
            return

        if args.test == "individual":
            duty = args.duty if args.duty is not None else 0.40
            run_individual_test(driver, args.mapping, test_duty=duty)
            return

        if args.test == "directional":
            duty = args.duty if args.duty is not None else 0.45
            run_directional_test(driver, args.mapping, test_duty=duty)
            return

        if args.test == "all-motors":
            duty = args.duty if args.duty is not None else 0.40
            print(f"\n[All Motors Sequence] Testing Motors 1-4 at {duty * 100:.0f}% duty...")
            for m in [1, 2, 3, 4]:
                print(f"  ▶ Motor {m} FORWARD ({args.duration:.1f}s)...")
                driver.drive_motor(m, duty, "forward", args.mapping)
                time.sleep(args.duration)
                driver.brake_motor(m, args.mapping)
                time.sleep(0.3)
                print(f"  ◀ Motor {m} REVERSE ({args.duration:.1f}s)...")
                driver.drive_motor(m, duty, "reverse", args.mapping)
                time.sleep(args.duration)
                driver.brake_motor(m, args.mapping)
                time.sleep(0.3)
            print("All motors test finished.")
            return

        if args.motor is not None:
            duty = args.duty if args.duty is not None else 0.40
            print(f"\n[Motor {args.motor} Pulse] Running {args.dir.upper()} at {duty * 100:.1f}% duty for {args.duration:.1f}s...")
            driver.drive_motor(args.motor, duty, args.dir, args.mapping)
            time.sleep(args.duration)
            driver.brake_motor(args.motor, args.mapping)
            print(f"Motor {args.motor} stopped.")
            return

        if args.channel is not None:
            duty = args.duty if args.duty is not None else 0.40
            print(f"\n[Raw Channel {args.channel} Pulse] Pulsing at {duty * 100:.1f}% duty for {args.duration:.1f}s...")
            driver.set_duty(args.channel, duty)
            time.sleep(args.duration)
            driver.set_duty(args.channel, 0.0)
            print(f"Channel {args.channel} stopped.")
            return

        # Default fallback: run wizard prompt
        print("\nNo specific action specified.")
        print("To run the guided calibration wizard, execute:")
        print("  python3 demonstration/test_pca9685.py --wizard")

    finally:
        driver.stop_all()
        driver.close()


if __name__ == "__main__":
    main()
