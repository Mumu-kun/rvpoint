# Orange Pi RV2 (SpacemiT K1) 26-Pin GPIO & Dual L298N Wiring Guide

The **Orange Pi RV2** features a **26-pin expansion header (2x13 pins)**. This guide details the **Column-Separated 8-Wire Direct-IN PWM** wiring layout, where Board 1 connects entirely along the **Inner Column** and Board 2 connects entirely along the **Outer Column**.

---

## 1. Column-Separated 26-Pin Header Diagram

- **Inner Column (Odd pins)**: Entirely dedicated to **Board 1 (Front Axle)**.
- **Outer Column (Even pins)**: Entirely dedicated to **Board 2 (Rear Axle)**.
- **Zero cross-over**: No zigzagging between columns!

```text
                        [ORANGE PI RV2 26-PIN HEADER]
                        
           [INNER COLUMN: BOARD 1]         [OUTER COLUMN: BOARD 2]
           
                     [ CAP: 3.3V ]  ( 1)  ( 2)  [ CAP: 5V ]
                                    ( 3)  ( 4)  [ CAP: 5V ]
                                    ( 5)  ( 6) <=== [COMMON GND] (Battery (-) & L298N GNDs)
                 Board 1 [IN1] ===> ( 7)  ( 8) <=== Board 2 [IN1]
                  [GND / Alt GND]   ( 9)  (10) <=== Board 2 [IN2]
                 Board 1 [IN2] ===> (11)  (12) <=== Board 2 [IN3]
                 Board 1 [IN3] ===> (13)  (14)      [GND / Alt GND]
                 Board 1 [IN4] ===> (15)  (16) <=== Board 2 [IN4]
                     [ CAP: 3.3V ]  (17)  (18) 
                                    (19)  (20) 
                                    (21)  (22) 
                                    (23)  (24) 
                                    (25)  (26) 
```

---

## 2. Pins to Cover / Protect

Cover these 4 power pins before plugging:
* **Pins 2 & 4 (5V Power)**: Cover with tape or empty DuPont housing.
* **Pins 1 & 17 (3.3V Power)**: Cover to protect the internal SoC PMIC.
* **Both L298N `+5V` Screw Terminals**: Must have **nothing connected**.

---

## 3. Detailed Wiring Connection Table

Leave the **black jumper caps on `ENA` and `ENB` INSTALLED** on both L298N modules.

### A. Common Ground (MANDATORY)
| From | Connects to Orange Pi RV2 | Function |
| :--- | :--- | :--- |
| **Battery (-) Negative** | **Pin 6 (GND)** | Shared reference ground |
| **Front L298N `GND` Terminal** | **Pin 6 (GND)** | Logic return path |
| **Rear L298N `GND` Terminal** | **Pin 6 or Pin 9 or Pin 14 (GND)** | Logic return path |

---

### B. Board 1: Front Axle Module (All on INNER Column)
Just plug your 4 wires down the **Inner Column**, skipping Pin 9 in the middle:

| Front L298N Wire | Calibrated Wheel | Inner Column Pin # | Linux Sysfs GPIO |
| :--- | :--- | :--- | :--- |
| **`IN1`** | Front-Right (FR) Dir 1 | **Pin 7** | `gpio74` |
| *(Pin 9 is GND)* | *(Skip or use as ground)* | — | — |
| **`IN2`** | Front-Right (FR) Dir 2 | **Pin 11** | `gpio71` |
| **`IN3`** | Front-Left (FL) Dir 1 | **Pin 13** | `gpio72` |
| **`IN4`** | Front-Left (FL) Dir 2 | **Pin 15** | `gpio73` |

---

### C. Board 2: Rear Axle Module (All on OUTER Column)
Plug your 4 wires down the **Outer Column**, skipping Pin 14 in the middle:

| Rear L298N Wire | Calibrated Wheel | Outer Column Pin # | Linux Sysfs GPIO |
| :--- | :--- | :--- | :--- |
| **`IN1`** | Rear-Right (RR) Dir 1 | **Pin 8** | `gpio47` |
| **`IN2`** | Rear-Right (RR) Dir 2 | **Pin 10** | `gpio48` |
| **`IN3`** | Rear-Left (RL) Dir 1 | **Pin 12** | `gpio70` |
| *(Pin 14 is GND)* | *(Skip or use as ground)* | — | — |
| **`IN4`** | Rear-Left (RL) Dir 2 | **Pin 16** | `gpio91` |

---

## 4. First-Time Verification

1. Place the vehicle on a stand (wheels off the ground).
2. Boot the Orange Pi RV2 via USB-C, then turn on the motor battery.
3. Run the wizard:
   ```bash
   python3 demonstration/validate_motors.py --wizard
   ```
   Follow the prompts to identify each spinning wheel and direction. The wizard will update `eval/actuators/l298n_pins.json` automatically!
