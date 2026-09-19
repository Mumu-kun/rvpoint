#!/usr/bin/env python3
"""
test_pca9685.py — Zero-Dependency PCA9685 I2C Motor & PWM Driver Verification Tool.

Auto-detects PCA9685 on Orange Pi RV2 (Bus 4, Address 0x60/0x40) or Raspberry Pi (Bus 1).
Supports both standard PCA9685 raw PWM channels and integrated Motor HATs (Adafruit / Waveshare).

Features:
  1. Auto-Bus & Address Detection:
     Automatically searches /dev/i2c-4, /dev/i2c-1, etc. for 0x60 or 0x40.
  2. Motor HAT Testing (--motor <1-4> --dir <forward|reverse> --duty <0.0-1.0>):
     Drives H-bridge DC motors directly via PCA9685 PWM + direction channels.
  3. Raw PWM Channel Test (--channel <0-15> --duty <0.0-1.0>):
     Tests an individual PCA9685 pin (e.g. for external L298N ENA/IN pins or servos).
  4. Stiction Breakaway Sweep (--sweep):
     Ramps duty cycle from 5% to 50% to find mechanical gearbox breakaway torque.
  5. Sequential Verification (--test individual | all-motors):
     Automated sequence testing all motor channels one by one.
  6. Emergency Stop (--stop):
     Immediately writes ALL_LED_OFF to kill power to all 16 channels.

Usage Examples on Orange Pi RV2:
  # Scan and auto-detect:
  python3 demonstration/test_pca9685.py --scan

  # Test Motor 1 forward at 40% duty for 2 seconds:
  python3 demonstration/test_pca9685.py --motor 1 --dir forward --duty 0.40

  # Test Motor 1 reverse:
  python3 demonstration/test_pca9685.py --motor 1 --dir reverse --duty 0.40

  # Test all 4 motors in sequence:
  python3 demonstration/test_pca9685.py --test all-motors

  # Test raw PWM Channel (e.g. Channel 0 at 50% duty):
  python3 demonstration/test_pca9685.py --channel 0 --duty 0.50

  # Emergency stop:
  python3 demonstration/test_pca9685.py --stop
"""

import argparse
import glob
import os
import signal
import sys
import time
from typing import Dict, List, Optional, Tuple

# PCA9685 Register Map
PCA9685_MODE1 = 0x00
PCA9685_MODE2 = 0x01
PCA9685_SUBADR1 = 0x02
PCA9685_SUBADR2 = 0x03
PCA9685_SUBADR3 = 0x04
PCA9685_ALLCALLADR = 0x05
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

# Standard Motor HAT PCA9685 Channel Mappings: (PWM_CH, IN1_CH, IN2_CH)
# Adafruit Motor HAT v2 pinout (default for address 0x60)
ADAFRUIT_MAPPINGS = {
    1: {"pwm": 8, "in2": 9, "in1": 10},
    2: {"pwm": 13, "in2": 12, "in1": 11},
    3: {"pwm": 2, "in2": 3, "in1": 4},
    4: {"pwm": 7, "in2": 6, "in1": 5},
}

# Waveshare Motor Driver HAT pinout
WAVESHARE_MAPPINGS = {
    1: {"pwm": 0, "in1": 1, "in2": 2},
    2: {"pwm": 5, "in1": 3, "in2": 4},
    3: {"pwm": 6, "in1": 7, "in2": 8},
    4: {"pwm": 11, "in1": 9, "in2": 10},
}


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
    """Automatically searches for PCA9685 (0x60 or 0x40) across available /dev/i2c-* nodes."""
    nodes = sorted(glob.glob("/dev/i2c-*"))
    bus_nums: List[int] = []
    for node in nodes:
        try:
            bus_nums.append(int(node.split("-")[-1]))
        except ValueError:
            pass

    # Prioritize bus 4 (Orange Pi RV2) then bus 1 (Raspberry Pi)
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

    # Default fallback if not found or off-target
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
            except Exception as e:
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
        """Sets duty cycle [0.0 to 1.0] for a channel [0..15]."""
        duty = max(0.0, min(1.0, float(duty)))
        if channel < 0 or channel > 15:
            raise ValueError(f"Channel {channel} out of range (0-15)")

        if self.is_simulated:
            print(f"[SIM] Ch {channel:02d} duty set to {duty * 100.0:5.1f}%")
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
        """Sets a channel to digital HIGH (1.0) or LOW (0.0)."""
        self.set_duty(channel, 1.0 if state else 0.0)

    def drive_motor(
        self,
        motor_num: int,
        duty: float,
        direction: str = "forward",
        mapping_name: str = "adafruit",
    ) -> None:
        """Drives an H-bridge DC motor on a Motor HAT."""
        mappings = ADAFRUIT_MAPPINGS if mapping_name == "adafruit" else WAVESHARE_MAPPINGS
        if motor_num not in mappings:
            raise ValueError(f"Motor {motor_num} not found. Choose 1, 2, 3, or 4.")

        pins = mappings[motor_num]
        pwm_ch = pins["pwm"]
        in1_ch = pins["in1"]
        in2_ch = pins["in2"]

        if abs(duty) < 0.01:
            # Active brake
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

    def stop_all(self) -> None:
        """Immediately turns off all 16 PWM output channels."""
        if self.is_simulated:
            print("[SIM] Stopped all channels.")
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


def print_i2c_table(found: List[int]) -> None:
    print("\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f")
    for row in range(0x00, 0x80, 0x10):
        line = f"{row:02x}: "
        for col in range(16):
            addr = row + col
            if addr < 0x03 or addr > 0x77:
                line += "   "
            elif addr in found:
                line += f"{addr:02x} "
            else:
                line += "-- "
        print(line)
    print()


def main():
    detected_bus, detected_addr = auto_detect_bus_and_address()

    parser = argparse.ArgumentParser(
        description="PCA9685 I2C Hardware PWM Motor Driver Verification Tool"
    )
    parser.add_argument("--bus", type=int, default=detected_bus, help=f"I2C bus number (auto-detected: {detected_bus})")
    parser.add_argument("--address", type=lambda x: int(x, 0), default=detected_addr, help=f"I2C address in hex (auto-detected: 0x{detected_addr:02X})")
    parser.add_argument("--freq", type=float, default=200.0, help="PWM frequency in Hz (default: 200 Hz)")
    parser.add_argument("--mapping", choices=["adafruit", "waveshare"], default="adafruit", help="Motor HAT pinout mapping (default: adafruit)")
    parser.add_argument("--scan", action="store_true", help="Scan I2C bus to find detected chips")
    parser.add_argument("--motor", type=int, choices=[1, 2, 3, 4], help="Motor HAT motor number [1, 2, 3, 4]")
    parser.add_argument("--dir", choices=["forward", "reverse"], default="forward", help="Direction for --motor (default: forward)")
    parser.add_argument("--channel", type=int, help="Raw PCA9685 PWM channel [0-15]")
    parser.add_argument("--duty", type=float, default=None, help="Target duty cycle [0.0 - 1.0] (default: 0.40)")
    parser.add_argument("--duration", type=float, default=2.0, help="Duration in seconds to run motor test (default: 2.0s)")
    parser.add_argument("--sweep", action="store_true", help="Sweep duty from 5%% to 50%% to find breakaway torque")
    parser.add_argument("--test", choices=["individual", "directional", "all-motors"], help="Run automated motor sequence")
    parser.add_argument("--stop", action="store_true", help="Send immediate failsafe e-stop to all channels")

    args = parser.parse_args()

    print("\n" + "=" * 65)
    print("      PCA9685 I2C 16-Channel 12-Bit PWM Driver Diagnostic Tool   ")
    print(f"      Bus: /dev/i2c-{args.bus} | Address: 0x{args.address:02X} | Frequency: {args.freq:.1f} Hz")
    print("=" * 65)

    # 1. Bus Scan Mode
    if args.scan:
        print(f"\n[Scanning /dev/i2c-{args.bus} for active I2C slave devices...]")
        detected = scan_i2c_bus(args.bus)
        print_i2c_table(detected)

        if not detected:
            print("❌ No I2C devices detected on this bus.")
        else:
            print(f"✅ Found {len(detected)} device(s): " + ", ".join(f"0x{a:02X}" for a in detected))
            if 0x60 in detected:
                print("   👉 0x60: PCA9685 Motor Driver HAT detected! (Target chip ready)")
            if 0x70 in detected:
                print("   👉 0x70: PCA9685 All-Call address verified!")
            if 0x40 in detected:
                print("   👉 0x40: Standard PCA9685 default address verified!")
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
        # 2. Immediate Stop
        if args.stop:
            print("[Action] Stopping all channels immediately...")
            driver.stop_all()
            print("Done. All outputs disabled.")
            return

        # 3. Motor HAT Single Motor Drive
        if args.motor is not None:
            duty = args.duty if args.duty is not None else 0.40
            print(f"\n[Motor HAT Test] Running Motor {args.motor} ({args.dir.upper()}) at {duty * 100:.1f}% duty for {args.duration:.1f}s...")
            print(f"  Mapping: {args.mapping.upper()} (PWM + IN1 + IN2)")
            driver.drive_motor(args.motor, duty, args.dir, args.mapping)
            time.sleep(args.duration)
            driver.drive_motor(args.motor, 0.0, args.dir, args.mapping)
            print(f"Motor {args.motor} test completed.")
            return

        # 4. Test All Motors in Sequence (Motor HAT)
        if args.test == "all-motors":
            duty = args.duty if args.duty is not None else 0.40
            print(f"\n[All Motors Sequence] Testing Motors 1, 2, 3, 4 sequentially at {duty * 100:.0f}% duty...")
            for m in [1, 2, 3, 4]:
                print(f"  ▶ Motor {m} FORWARD ({args.duration:.1f}s)...")
                driver.drive_motor(m, duty, "forward", args.mapping)
                time.sleep(args.duration)
                driver.drive_motor(m, 0.0, "forward", args.mapping)
                time.sleep(0.4)
                print(f"  ◀ Motor {m} REVERSE ({args.duration:.1f}s)...")
                driver.drive_motor(m, duty, "reverse", args.mapping)
                time.sleep(args.duration)
                driver.drive_motor(m, 0.0, "reverse", args.mapping)
                time.sleep(0.4)
            print("All motors test finished successfully.")
            return

        # 5. Breakaway Stiction Sweep (Raw Channel or Motor)
        if args.sweep:
            ch = args.channel if args.channel is not None else 0
            print(f"\n[Breakaway Torque Sweep] Ramping Channel {ch} from 5% to 50% duty...")
            print("Observe the motor shaft to note the exact duty cycle where rotation begins.\n")
            for pct in range(5, 51, 3):
                duty = pct / 100.0
                driver.set_duty(ch, duty)
                print(f"  → Duty: {pct:2d}% ({duty:.2f}) - Observing motor...", end="\r", flush=True)
                time.sleep(0.4)
            print(f"\n  → Reached 50% duty. Holding for 1.0s...")
            time.sleep(1.0)
            driver.set_duty(ch, 0.0)
            print("Sweep complete. Channel returned to 0% duty.")
            return

        # 6. Raw Channel Pulse Test
        if args.channel is not None:
            duty = args.duty if args.duty is not None else 0.40
            ch = args.channel
            print(f"\n[Raw Channel Pulse] Pulsing Channel {ch} at {duty * 100:.1f}% duty for {args.duration:.1f}s...")
            driver.set_duty(ch, duty)
            time.sleep(args.duration)
            driver.set_duty(ch, 0.0)
            print(f"Channel {ch} test complete.")
            return

        # 7. Default behavior if no specific flags passed: test channels 0-3
        duty = args.duty if args.duty is not None else 0.35
        print("\n[Default Multi-Channel Pulse] Testing channels 0, 1, 2, 3 sequentially...")
        for ch in [0, 1, 2, 3]:
            print(f"  ▶ Pulsing Channel {ch} for 1.0s...")
            driver.set_duty(ch, duty)
            time.sleep(1.0)
            driver.set_duty(ch, 0.0)
            time.sleep(0.3)
        print("Done. Use --motor <1-4> or --channel <0-15> for specific testing.")

    finally:
        driver.stop_all()
        driver.close()


if __name__ == "__main__":
    main()
