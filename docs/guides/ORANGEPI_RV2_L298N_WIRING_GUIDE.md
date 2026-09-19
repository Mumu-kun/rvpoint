# Orange Pi RV2 (SpacemiT K1) 26-Pin GPIO & Dual L298N Wiring Guide

The **Orange Pi RV2** features a **26-pin expansion header (2x13 pins)**. This guide details the **Column-Separated 12-Wire Enable-Pin (Drive-Coast)** wiring layout:
- **Board 1 (Front Axle)** connects entirely along the **Inner Column (Odd Pins)**.
- **Board 2 (Rear Axle)** connects entirely along the **Outer Column (Even Pins)**.
- **Zero cross-overs**: All wires stay in their respective column!

> [!IMPORTANT]
> **Remove the 4 black jumper caps on `ENA` and `ENB`** on both L298N driver boards. Connecting PWM to the enable pins switches the H-bridge from Active Dynamic Braking (which causes stiction lockup) to **Free-Running Coast (Drive-Coast)**, preserving rotational inertia and synchronizing all 4 wheels.

---

## 1. Column-Separated 26-Pin Header Diagram

```text
                        [ORANGE PI RV2 26-PIN HEADER]

           [INNER COLUMN: BOARD 1]         [OUTER COLUMN: BOARD 2]

                     [ CAP: 3.3V ]  ( 1)  ( 2)  [ CAP: 5V ]
                                    ( 3)  ( 4)  [ CAP: 5V ]
                                    ( 5)  ( 6) <=== [COMMON GND] (Battery (-) & L298N GNDs)
                 Board 1 [IN1] ===> ( 7)  ( 8) <=== Board 2 [IN1]
                   [GND / Alt GND]  ( 9)  (10) <=== Board 2 [IN2]
                 Board 1 [IN2] ===> (11)  (12) <=== Board 2 [IN3]
                 Board 1 [IN3] ===> (13)  (14)      [GND / Alt GND]
                 Board 1 [IN4] ===> (15)  (16) <=== Board 2 [IN4]
           [CAP: 3.3V / DO NOT USE] (17)  (18) <=== Board 2 [ENA] (RR Enable, gpio92)
     Board 1 [ENA] (FR, gpio77) ==> (19)  (20)      [GND / Alt GND]
     Board 1 [ENB] (FL, gpio78) ==> (21)  (22) <=== Board 2 [ENB] (RL Enable, gpio49)
                                    (23)  (24)
                                    (25)  (26)
```

---

## 2. Pins to Cover / Protect

Cover these power pins before plugging:
* **Pins 2 & 4 (5V Power)**: Cover with tape or empty DuPont housing.
* **Pins 1 & 17 (3.3V Power)**: Cover with tape to protect the internal SoC PMIC.
* **Both L298N `+5V` Screw Terminals**: Must have **nothing connected**.

---

## 3. Detailed Wiring Connection Table

### A. Common Ground (MANDATORY)
| From | Connects to Orange Pi RV2 | Function |
| :--- | :--- | :--- |
| **Battery (-) Negative** | **Pin 6 (GND)** | Shared reference ground |
| **Front L298N `GND` Terminal** | **Pin 6 (GND)** | Logic return path |
| **Rear L298N `GND` Terminal** | **Pin 6 or Pin 9 or Pin 14 (GND)** | Logic return path |

---

### B. Board 1: Front Axle Module (All on INNER Column / Odd Pins)
Plug wires down the **Inner Column**:

| Front L298N Wire | Target Wheel | Inner Column Pin # | Linux Sysfs GPIO | Function |
| :--- | :--- | :--- | :--- | :--- |
| **`IN1`** | Front-Right (FR) | **Pin 7** | `gpio74` | Direction 1 |
| *(Pin 9 is GND)* | *(Skip or use as ground)* | — | — | — |
| **`IN2`** | Front-Right (FR) | **Pin 11** | `gpio71` | Direction 2 |
| **`IN3`** | Front-Left (FL) | **Pin 13** | `gpio72` | Direction 1 |
| **`IN4`** | Front-Left (FL) | **Pin 15** | `gpio73` | Direction 2 |
| *(Pin 17 is 3.3V)* | *(COVER WITH TAPE)* | — | — | — |
| **`ENA`** | Front-Right (FR) | **Pin 19** | `gpio77` | Speed PWM (Coast) |
| **`ENB`** | Front-Left (FL) | **Pin 21** | `gpio78` | Speed PWM (Coast) |

---

### C. Board 2: Rear Axle Module (All on OUTER Column / Even Pins)
Plug wires down the **Outer Column**:

| Rear L298N Wire | Target Wheel | Outer Column Pin # | Linux Sysfs GPIO | Function |
| :--- | :--- | :--- | :--- | :--- |
| **`IN1`** | Rear-Right (RR) | **Pin 8** | `gpio47` | Direction 1 |
| **`IN2`** | Rear-Right (RR) | **Pin 10** | `gpio48` | Direction 2 |
| **`IN3`** | Rear-Left (RL) | **Pin 12** | `gpio70` | Direction 1 |
| *(Pin 14 is GND)* | *(Skip or use as ground)* | — | — | — |
| **`IN4`** | Rear-Left (RL) | **Pin 16** | `gpio91` | Direction 2 |
| **`ENA`** | Rear-Right (RR) | **Pin 18** | `gpio92` | Speed PWM (Coast) |
| *(Pin 20 is GND)* | *(Skip or use as ground)* | — | — | — |
| **`ENB`** | Rear-Left (RL) | **Pin 22** | `gpio49` | Speed PWM (Coast) |

---

## 4. First-Time Verification

1. Place the vehicle on a stand (wheels off the ground).
2. Ensure the 4 black jumper caps on `ENA` and `ENB` are removed.
3. Turn on the motor battery and run the directional test:
   ```bash
   sudo ./validate_motors --test directional --duty 0.45
   ```
4. Or interactive WASD teleoperation:
   ```bash
   sudo ./validate_motors --teleop
   ```
