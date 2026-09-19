#!/usr/bin/env python3
"""
validate_motors.py — PCA9685 Motor Validation & Calibration Tool.

Replaced deprecated L298N driver with PCA9685 I2C hardware PWM driver.
Forwards directly to demonstration/test_pca9685.py.
"""

import sys
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from demonstration.test_pca9685 import main

if __name__ == "__main__":
    main()
