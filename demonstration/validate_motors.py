#!/usr/bin/env python3
"""
validate_motors.py — Python-Based Axle-Partitioned Dual L298N Validation Tool & Calibration Wizard.

Full feature parity with the C++ prototype:
  1. Guided Calibration Wizard (--wizard):
     Interactive channel-by-channel pulse and polarity identification. Automatically saves
     to eval/actuators/l298n_pins.json.
  2. Individual Wheel Polarity Test (--test individual):
     Sequential forward/reverse check on FL, FR, RL, RR.
  3. Directional Motions Test (--test directional):
     Forward -> Reverse -> Pivot Left -> Pivot Right under Omni-Tank kinematics.
  4. Stiction / Deadband Sweep (--test sweep):
     5% to 50% ramp to measure static gearbox breakaway torque.
  5. Interactive Terminal Teleoperation (--teleop):
     Real-time keyboard driving (WASD + Spacebar Brake) with 200 ms safety watchdog.

Usage:
  python3 demonstration/validate_motors.py --wizard
  python3 demonstration/validate_motors.py --test individual --duty 0.25
  python3 demonstration/validate_motors.py --test directional
  python3 demonstration/validate_motors.py --test sweep
  python3 demonstration/validate_motors.py --teleop
"""

import argparse
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional, Dict, Any, Tuple

# Add project root to sys.path so demonstration.actuators can be imported
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from demonstration.actuators.l298n_actuator import DualL298NActuator


def print_banner() -> None:
    print("\n" + "=" * 65)
    print("   RVPoint Python Dual L298N Motor Validation Tool (Omni-Tank)   ")
    print("      Ch A/B Front Board + Ch A/B Rear Board Hardware Bridge      ")
    print("=" * 65 + "\n")


# ---------------------------------------------------------------------------
# Cross-Platform Non-Blocking Keyboard Reader
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

                termios.tcsetattr(
                    sys.stdin.fileno(), termios.TCSADRAIN, self._orig_termios
                )
            except Exception:
                pass

    def get_char(self) -> Optional[str]:
        if self.is_windows:
            import msvcrt

            if msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    return ch.decode("utf-8")
                except Exception:
                    return None
            return None
        else:
            import select

            if select.select([sys.stdin], [], [], 0.05)[0]:
                return sys.stdin.read(1)
            return None


# ---------------------------------------------------------------------------
# Mode 1: Calibration Wizard
# ---------------------------------------------------------------------------
def run_wizard(
    actuator: DualL298NActuator, config_path: Path, test_duty: float
) -> None:
    print_banner()
    print(">>> GUIDED CHANNEL & POLARITY CALIBRATION WIZARD <<<\n")
    print("This wizard will pulse each of the 4 physical H-bridge channels")
    print("one by one so you can identify which motor is connected to which channel")
    print("and whether the wiring polarity is correct.\n")
    print("SAFETY NOTICE: Put vehicle on a stand so wheels can spin freely!\n")

    input("Press [ENTER] to begin...")
    actuator.set_watchdog_enabled(False)

    ch_names = [
        "Channel 1 (Board 1 Front, Channel A)",
        "Channel 2 (Board 1 Front, Channel B)",
        "Channel 3 (Board 2 Rear,  Channel A)",
        "Channel 4 (Board 2 Rear,  Channel B)",
    ]

    wheel_keys = ["FL", "FR", "RL", "RR"]
    wheel_labels = [
        "Front-Left (FL)",
        "Front-Right (FR)",
        "Rear-Left (RL)",
        "Rear-Right (RR)",
    ]

    for ch in range(4):
        choice = None
        while choice is None:
            print("\n" + "-" * 65)
            print(
                f">>> Pulsing {ch_names[ch]} for 1.0s at {int(test_duty * 100)}% duty..."
            )
            print("-" * 65)

            # 1. Pulse for exactly 1.0 second
            actuator.set_raw_channel_duty(ch, test_duty)
            time.sleep(3.0)

            # 2. Hard stop immediately so wheel does NOT spin while you are reading!
            actuator.set_raw_channel_duty(ch, 0.0)
            actuator.emergency_brake()
            actuator.reset_emergency_stop()

            print("\nWhich wheel just spun?")
            print("  [1] Front-Left  (FL)")
            print("  [2] Front-Right (FR)")
            print("  [3] Rear-Left   (RL)")
            print("  [4] Rear-Right  (RR)")
            print("  [p] Pulse this channel again for 1.0s")
            print("  [0] None / Not moving")

            try:
                choice_str = input("Selection [0-4 or p]: ").strip().lower()
                if choice_str == "p":
                    continue
                choice = int(choice_str) if choice_str else 0
            except Exception:
                choice = 0

        if choice < 1 or choice > 4:
            print(f"(!) Channel {ch + 1} skipped or not connected.")
            continue

        dir_choice = (
            input("\nWas the wheel spinning FORWARD or REVERSE? [f/r]: ")
            .strip()
            .lower()
        )
        is_reverse = dir_choice.startswith("r")

        target_wheel_key = wheel_keys[choice - 1]
        wheel_cfg = actuator.config["wheels"][target_wheel_key]
        wheel_cfg["channel"] = ch
        wheel_cfg["invert"] = is_reverse

        print(
            f"==> Assigned {ch_names[ch]} -> {wheel_labels[choice - 1]} (Invert: {'YES' if is_reverse else 'NO'})"
        )

    print("\n" + "=" * 65)
    print(">>> CALIBRATION SUMMARY <<<")
    print("=" * 65)
    for key, label in zip(wheel_keys, wheel_labels):
        w = actuator.config["wheels"][key]
        print(f"  {label}: Channel {w['channel']}, Invert: {w['invert']}")

    if actuator.save_config(config_path):
        print(f"\nSuccessfully saved updated configuration to: {config_path}")
    else:
        print(f"\nFailed to save configuration to: {config_path}", file=sys.stderr)

    confirm = (
        input(
            "\nRun simultaneous 1.5s forward verification spin on all wheels? [Y/n]: "
        )
        .strip()
        .lower()
    )
    if confirm in ("", "y", "yes"):
        print("\n[TEST] Spinning all 4 wheels FORWARD for 1.5s...")
        actuator.set_wheel_duties(test_duty, test_duty, test_duty, test_duty)
        time.sleep(1.5)
        actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)
        print("[TEST] Done. Active brake engaged.")

    actuator.set_watchdog_enabled(True)


# ---------------------------------------------------------------------------
# Mode 2: Individual Wheel Polarity Test
# ---------------------------------------------------------------------------
def run_individual_test(actuator: DualL298NActuator, duty: float) -> None:
    print_banner()
    print(">>> INDIVIDUAL WHEEL POLARITY TEST <<<\n")
    print("Testing each wheel: Forward (1.5s) -> Pause (0.5s) -> Reverse (1.5s)\n")

    actuator.set_watchdog_enabled(False)
    wheel_keys = ["FL", "FR", "RL", "RR"]
    names = ["Front-Left (FL)", "Front-Right (FR)", "Rear-Left (RL)", "Rear-Right (RR)"]

    try:
        for i, (key, name) in enumerate(zip(wheel_keys, names)):
            print(f"=== Testing {name} ===")

            # Forward
            print(f"  -> FORWARD (+{int(duty * 100)}%)... ", end="", flush=True)
            duties = [0.0, 0.0, 0.0, 0.0]
            duties[i] = duty
            actuator.set_wheel_duties(*duties)
            time.sleep(1.5)

            # Stop
            actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)
            print("STOP.")
            time.sleep(0.5)

            # Reverse
            print(f"  -> REVERSE (-{int(duty * 100)}%)... ", end="", flush=True)
            duties[i] = -duty
            actuator.set_wheel_duties(*duties)
            time.sleep(1.5)

            # Stop
            actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)
            print("STOP.\n")
            time.sleep(0.5)
    finally:
        actuator.set_watchdog_enabled(True)

    print("[DONE] Individual wheel test completed.")


# ---------------------------------------------------------------------------
# Mode 3: Directional Motions Test
# ---------------------------------------------------------------------------
def run_directional_test(actuator: DualL298NActuator, duty: float) -> None:
    print_banner()
    print(">>> DIRECTIONAL MOTIONS TEST (Omni-Tank 2-DoF) <<<\n")
    print("Executing: Forward -> Reverse -> Pivot Left -> Pivot Right (1.5s each)\n")

    actuator.set_watchdog_enabled(False)
    moves = [
        ("FORWARD", duty, duty),
        ("REVERSE", -duty, -duty),
        ("PIVOT LEFT", -duty, duty),
        ("PIVOT RIGHT", duty, -duty),
    ]

    try:
        for label, dl, dr in moves:
            print(f">>> {label} (Left: {dl:.2f}, Right: {dr:.2f})... ", end="", flush=True)
            actuator.set_duty_cycles(dl, dr)
            time.sleep(1.5)
            actuator.set_duty_cycles(0.0, 0.0)
            print("STOP.")
            time.sleep(0.5)
    finally:
        actuator.set_watchdog_enabled(True)

    print("\n[DONE] Directional motion test completed.")


# ---------------------------------------------------------------------------
# Mode 4: Stiction / Deadband Sweep
# ---------------------------------------------------------------------------
def run_sweep_test(actuator: DualL298NActuator, config_path: Path) -> None:
    print_banner()
    print(">>> STICTION DEADBAND CALIBRATION SWEEP <<<\n")
    print("Ramping forward duty from 5% to 50% in steps of 5%.")
    print("Observe wheels and indicate when motor rotation begins.\n")

    actuator.set_watchdog_enabled(False)
    found_deadband = 0.15
    detected = False

    for pct in range(5, 55, 5):
        d = pct / 100.0
        print(f"Testing duty: {pct}% ({d:.2f})... ", end="", flush=True)
        actuator.set_wheel_duties(d, d, d, d)
        time.sleep(3.0)
        actuator.set_wheel_duties(0.0, 0.0, 0.0, 0.0)

        ans = input(f"\nDid all wheels start turning? [y/N/q]: ").strip().lower()
        if ans == "q":
            break
        if ans == "y":
            found_deadband = d
            detected = True
            print(f"\n==> Stiction break threshold detected at: {pct}% ({d:.2f} duty)!")
            break

    if detected:
        actuator.config["deadband"] = found_deadband
        actuator.save_config(config_path)
        print(
            f"Updated configuration deadband to {found_deadband} and saved to {config_path}"
        )

    actuator.set_watchdog_enabled(True)


# ---------------------------------------------------------------------------
# Mode 5: Interactive Terminal Teleoperation
# ---------------------------------------------------------------------------
def run_teleop(actuator: DualL298NActuator) -> None:
    print_banner()
    print(">>> INTERACTIVE TERMINAL TELEOPERATION (Python) <<<\n")
    print("Controls:")
    print("  [W] Drive Forward")
    print("  [S] Drive Reverse")
    print("  [A] Pivot Left")
    print("  [D] Pivot Right")
    print("  [SPACE] Instant Active Emergency Brake")
    print("  [+] Increase Speed Step")
    print("  [-] Decrease Speed Step")
    print("  [Q] Exit Teleoperation\n")
    print("Safety Watchdog: Auto-brakes within 200 ms if no key is held.")
    print("Starting teleop loop...\n")

    base_speed = 0.35

    with KeyboardReader() as reader:
        try:
            while True:
                ch = reader.get_char()
                if ch:
                    ch_lower = ch.lower()
                    if ch_lower == "q":
                        break
                    elif ch_lower == "w":
                        actuator.set_duty_cycles(base_speed, base_speed)
                    elif ch_lower == "s":
                        actuator.set_duty_cycles(-base_speed, -base_speed)
                    elif ch_lower == "a":
                        actuator.set_duty_cycles(-base_speed, base_speed)
                    elif ch_lower == "d":
                        actuator.set_duty_cycles(base_speed, -base_speed)
                    elif ch == " ":
                        actuator.emergency_brake()
                    elif ch in ("+", "="):
                        base_speed = min(1.0, base_speed + 0.05)
                        print(f"Speed: {base_speed:.2f}")
                    elif ch in ("-", "_"):
                        base_speed = max(0.15, base_speed - 0.05)
                        print(f"Speed: {base_speed:.2f}")

                time.sleep(0.02)
        except KeyboardInterrupt:
            pass

    actuator.emergency_brake()
    print("\nTeleoperation terminated safely.")


# ---------------------------------------------------------------------------
# Main Entry Point
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="RVPoint Dual L298N Motor Validation Tool (Python)"
    )
    parser.add_argument(
        "--wizard",
        action="store_true",
        help="Guided interactive channel & polarity calibration",
    )
    parser.add_argument(
        "--test",
        choices=["individual", "directional", "sweep"],
        help="Run automated test routine",
    )
    parser.add_argument(
        "--teleop", action="store_true", help="Interactive terminal keyboard driving"
    )
    parser.add_argument(
        "--config",
        default="eval/actuators/l298n_pins.json",
        help="Path to l298n_pins.json",
    )
    parser.add_argument(
        "--duty", type=float, default=0.30, help="Test duty cycle [0.1, 1.0]"
    )
    parser.add_argument(
        "--hw-pwm", action="store_true", help="Force Linux sysfs hardware PWM mode"
    )
    parser.add_argument(
        "--sw-pwm", action="store_true", help="Force software timer GPIO PWM mode"
    )

    args = parser.parse_args()

    config_path = Path(args.config)
    actuator = DualL298NActuator(config_path=config_path)

    if args.hw_pwm:
        actuator.config["use_hardware_pwm"] = True
    elif args.sw_pwm:
        actuator.config["use_hardware_pwm"] = False

    def sig_handler(sig, frame):
        print(f"\n[SAFETY] Interrupted by signal {sig}. Active E-Brake engaged!")
        actuator.emergency_brake()
        actuator.shutdown_hardware()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        if args.wizard or (not args.test and not args.teleop):
            run_wizard(actuator, config_path, args.duty)
        elif args.test == "individual":
            run_individual_test(actuator, args.duty)
        elif args.test == "directional":
            run_directional_test(actuator, args.duty)
        elif args.test == "sweep":
            run_sweep_test(actuator, config_path)
        elif args.teleop:
            run_teleop(actuator)
    finally:
        actuator.emergency_brake()
        actuator.shutdown_hardware()


if __name__ == "__main__":
    main()
