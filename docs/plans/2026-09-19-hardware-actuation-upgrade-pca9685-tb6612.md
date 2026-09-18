# Architectural Plan: PCA9685 + Dual TB6612 Motor Driver Migration

**Date:** 2026-09-19  
**Status:** ready-for-hardware  
**Target Hardware:** Orange Pi RV2 (SpacemiT K1 RISC-V SoC), PCA9685 + TB6612 FeatherWing, 4WD Mecanum Chassis  
**Context:** Upgrading from dual L298N Darlington drivers to an integrated 12-bit I2C MOSFET driver board once sourced from RoboticsBD.

---

## 1. Executive Summary & Problem Formulation

The prototype rover currently uses two legacy **L298N** dual H-bridge driver modules to actuate its 4-wheel Mecanum chassis. While functional during initial bring-up, the dual L298N architecture introduces critical bottlenecks for indoor LiDAR SLAM and autonomous navigation:

1. **Bipolar Transistor (BJT) Voltage Drop**: The L298N drops $\sim 2.5\text{--}2.8\,\text{V}$ internally as heat across its output Darlington transistors. On a 2S Li-ion pack ($7.4\,\text{V}$ nominal), the motors receive only $4.6\text{--}4.9\,\text{V}$, causing severe low-speed stiction and stall outs.
2. **Wiring Complexity & GPIO Saturation**: 4 motors require 12 discrete GPIO wires (`IN1..IN4`, `ENA`, `ENB` $\times 2$), cluttering the chassis and exposing the system to loose Dupont jumper faults under vibration.
3. **Software PWM Jitter & CPU Overhead**: Software PWM bit-banged via Linux sysfs `/sys/class/gpio` introduces timing jitter, thread preemption latency, and CPU load on the SpacemiT K1 SoC.
4. **Size & Weight**: Two L298N boards weigh $\approx 65\,\text{g}$ and occupy significant top-deck footprint.

### Sourced Solution: PCA9685 + TB6612 FeatherWing Compatible
By procuring the **[DC Motor + Stepper Motor Driver PCA9685 + TB6612 FeatherWing](https://store.roboticsbd.com/stepper-motor-driver/3514-dc-motor-stepper-motor-driver-pca9685-tb6612-featherwing-compatible-robotics-bangladesh.html)** (RoboticsBD SKU `RBD-3514`), we consolidate the entire actuation subsystem into a single, high-efficiency board:

| Metric | Dual L298N (Current) | PCA9685 + TB6612 FeatherWing (Planned) | Architectural Benefit |
| :--- | :--- | :--- | :--- |
| **Output Drivers** | Bipolar Darlington ($V_{\text{drop}} \approx 2.5\text{V}$) | Dual TB6612FNG Power MOSFETs ($R_{DS(\text{on})} \approx 0.5\,\Omega$) | Efficient 2S operation; no heatsinks; zero thermal throttling |
| **Control Interface** | 12 GPIO lines (Software PWM / Sysfs) | **2 I2C lines** (`SDA`, `SCL`) | 10 GPIOs freed; zero kernel bit-banging |
| **PWM Resolution** | 8-bit ($\sim 100\text{--}255$ steps, jittery) | **12-bit ($4096$ steps, hardware locked)** | Silky creeping speed ($< 0.05\,\text{m/s}$) for LiDAR mapping |
| **Board Weight** | $\approx 65\,\text{g}$ | **$4.6\,\text{g}$** | $>60\,\text{g}$ payload savings for battery/sensors |
| **Board Dimensions**| $43 \times 43\,\text{mm}$ ($\times 2$ boards) | $50.8 \times 22.9 \times 1.6\,\text{mm}$ (1 board) | Fits compactly beneath the Orange Pi RV2 mounting plate |
| **Motor Supply ($V_{\text{motor}}$)** | $5\text{--}35\,\text{V}$ (Inefficient below 9V) | **$4.5\text{--}13.5\,\text{V}$** | Native support for both 2S ($7.4\,\text{V}$) and 3S ($11.1\,\text{V}$) packs |

---

## 2. Hardware Architecture & Electrical Integration

```
   +-------------------------------------------------------------+
   |                  Orange Pi RV2 (SpacemiT K1)                |
   |                                                             |
   |   [Pin 1: 3.3V]    [Pin 3: I2C-SDA]   [Pin 5: I2C-SCL]     [Pin 6: GND]
   +---------+-----------------+------------------+-------------------+
             |                 |                  |                   |
             | (3.3V Logic)    | (SDA)            | (SCL)             | (Ground)
             v                 v                  v                   v
   +---------+-----------------+------------------+-------------------+
   |  [3V]                [SDA]              [SCL]              [GND] |
   |                                                                 |
   |              PCA9685 16-Channel 12-Bit I2C PWM Controller       |
   |                                                                 |
   |      +---------------------------------------------------+      |
   |      | Dual TB6612FNG Dual H-Bridge Motor Drivers (MOSFET) |      |
   |      +---------------------------------------------------+      |
   |                                                                 |
   |  [Power In: +  -]     [ M1 ]       [ M2 ]       [ M3 ]   [ M4 ] |
   +-------+-------+---------+------------+------------+--------+----+
           ^       ^         |            |            |        |
           |       |         v            v            v        v
  [Battery +]     [Battery -] FL Wheel   FR Wheel    RL Wheel  RR Wheel
 (2S 7.4V or 3S 11.1V)     (Mecanum)   (Mecanum)   (Mecanum) (Mecanum)
```

### 2.1. Wiring Pinout Table

| FeatherWing Header Pin | Orange Pi RV2 Header | Signal Description | Electrical Note |
| :--- | :--- | :--- | :--- |
| **`3V`** | **Pin 1 (3.3V)** | Logic Supply & I2C Pullup | **Never connect to 5V!** K1 SoC requires 3.3V I2C levels. |
| **`GND`** | **Pin 6, 9, 14, 20 (GND)** | Common Digital Ground | Star-grounded with battery negative. |
| **`SDA`** | **Pin 3 (I2C_SDA)** | I2C Serial Data line | Hardware I2C (default bus `/dev/i2c-0` or `/dev/i2c-2`). |
| **`SCL`** | **Pin 5 (I2C_SCL)** | I2C Serial Clock line | Hardware I2C clock (up to 400 kHz Fast Mode). |
| **Motor Power `+`** | **Battery Pack Positive (+)** | Motor Armature Supply | $2\text{S} = 7.4\text{V}\text{--}8.4\text{V}$; $3\text{S} = 11.1\text{V}\text{--}12.6\text{V}$. |
| **Motor Power `-`** | **Battery Pack Negative (-)** | Motor Armature Ground | Dedicated heavy-gauge wire directly to battery. |

### 2.2. Onboard Channel Mapping (Adafruit Standard)
The PCA9685 outputs are internally routed to the two TB6612 chips without external wiring:

- **Motor 1 (Front-Left)**:
  - `PWM`: PCA9685 Channel 8
  - `IN2`: PCA9685 Channel 9
  - `IN1`: PCA9685 Channel 10
- **Motor 2 (Front-Right)**:
  - `PWM`: PCA9685 Channel 13
  - `IN2`: PCA9685 Channel 12
  - `IN1`: PCA9685 Channel 11
- **Motor 3 (Rear-Left)**:
  - `PWM`: PCA9685 Channel 2
  - `IN2`: PCA9685 Channel 3
  - `IN1`: PCA9685 Channel 4
- **Motor 4 (Rear-Right)**:
  - `PWM`: PCA9685 Channel 7
  - `IN2`: PCA9685 Channel 6
  - `IN1`: PCA9685 Channel 5

*Default I2C Address:* `0x60` (Configurable `0x60`..`0x7F` via solder jumpers A0..A4).

---

## 3. Kinematics & Mecanum Topology

With 4 independently controlled channels, the vehicle transitions from a constrained 2-channel tank steering model to a **true 3-DoF Holonomic Omnidirectional platform**:

$$\begin{bmatrix} v_{FL} \\ v_{FR} \\ v_{RL} \\ v_{RR} \end{bmatrix} = \frac{1}{r} \begin{bmatrix} 1 & -1 & -(l_x + l_y) \\ 1 & 1 & (l_x + l_y) \\ 1 & 1 & -(l_x + l_y) \\ 1 & -1 & (l_x + l_y) \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Where:
- $v_x$: Forward/reverse linear velocity
- $v_y$: Lateral strafe velocity (crabbing left/right)
- $\omega_z$: Yaw rotation velocity
- $l_x, l_y$: Half-wheelbase and half-track dimensions
- $r$: Wheel radius ($30\,\text{mm}$ for standard $60\,\text{mm}$ Mecanum wheels)

### Mecanum Physical Orientation Invariant ("Top-View X")
All four wheels must be mounted such that when viewed from above:
- The rollers on the top surface point towards the vehicle center, forming an **'X'**.
- If the rollers form an **'O'**, the vehicle will be unable to generate lateral strafe traction and will rotate uncontrollably.

---

## 4. Software Architecture & Implementation Plan

> [!IMPORTANT]
> **Workspace Boundary Invariant (ADR-0007 / AGENTS.md):**  
> `src/` compiles into `librvpoint.a` and is a **100% pure algorithm/perception library** with zero hardware/platform dependencies. It only contains the pure abstract interface `MotorActuator` and `MockMotorActuator` for offline simulation.  
> All concrete physical hardware drivers belong strictly in:
> - `eval/actuators/` (for C++ hardware evaluation and perception-control pipelines)
> - `demonstration/actuators/` (for live Python teleoperation and diagnostic daemons)

### Phase 1: Hardware I2C Driver Adapter (`eval/actuators/`)
Implement a concrete Linux I2C actuator class conforming to the `MotorActuator` interface:

1. **`eval/actuators/pca9685_actuator.h` & `eval/actuators/pca9685_actuator.cpp`**:
   - Implements `MotorActuator` interface from `src/control/actuators/motor_actuator.h`.
   - Directly accesses `/dev/i2c-X` via standard Linux `open()`, `ioctl(I2C_SLAVE, 0x60)`, and `write()`.
   - PCA9685 initialization: Configure `MODE1` (auto-increment `AI=1`, normal mode), set prescaler for $200\,\text{Hz}$ motor PWM.
   - `set_wheel_speeds(float v_fl, float v_fr, float v_rl, float v_rr)`: Converts normalized duty cycles $[-1.0, 1.0]$ to 12-bit register values:
     - Forward ($v > 0$): `IN1 = ON`, `IN2 = OFF`, `PWM = round(v * 4095)`
     - Reverse ($v < 0$): `IN1 = OFF`, `IN2 = ON`, `PWM = round(|v| * 4095)`
     - Active Brake ($v = 0$): `IN1 = ON`, `IN2 = ON`, `PWM = 0` (TB6612 short brake)
     - Coast / E-stop: `IN1 = OFF`, `IN2 = OFF`, `PWM = 0` (free-wheeling coast)
   - Built-in fail-safe safety watchdog thread ($200\,\text{ms}$ deadman timeout).

### Phase 2: Python Teleop & Calibration Integration (`demonstration/`)
1. **`demonstration/actuators/pca9685_actuator.py`**:
   - Lightweight pure-Python implementation using either `smbus2` or raw `/dev/i2c-*` ioctl (standard library only).
   - Drop-in interface matching `DualL298NActuator`:
     - `set_duty_cycles(fl, fr, rl, rr)`
     - `emergency_brake()`
     - `reset_emergency_stop()`
2. **Update `demonstration/teleop_server.py`**:
   - Add hardware backend auto-detection:
     ```python
     if os.path.exists("/dev/i2c-0") or os.path.exists("/dev/i2c-2"):
         # Attempt PCA9685 probe at 0x60
         actuator = Pca9685Tb6612Actuator(bus=0, address=0x60)
     else:
         actuator = DualL298NActuator(...)
     ```
   - Update Web UI: Expose 4 independent wheel trim sliders (`FL`, `FR`, `RL`, `RR`) with live stiction feedforward tuning across 4096 levels.

### Phase 3: Autonomous Vehicle Pipeline Integration (`eval/pipelines/`)
1. **`eval/pipelines/pipeline_iphone_vehicle.cpp`**:
   - Utilize 3-DoF holonomic evasion:
     - When `ForwardCorridorSafetyFilter` detects an obstacle dead ahead, instead of coming to a dead stop and executing an expensive 90° pivot, the vehicle can execute a **lateral strafe evasion** ($v_y \neq 0, \omega_z = 0$) while maintaining its camera orientation towards the target.
   - Retains the fail-safe hardware watchdog thread monitoring TCP/UDP stream heartbeats ($< 250\,\text{ms}$).

---

## 5. Calibration & Commissioning Protocol

When the board arrives and is wired up:

1. **I2C Bus Detection**:
   ```bash
   # Install i2c-tools if needed
   sudo apt-get install -y i2c-tools
   # Probe I2C bus 0 or 2
   sudo i2cdetect -y -r 0
   # Verify device responds at address 0x60
   ```
2. **Motor Direction & Polarity Verification**:
   Run single-wheel verification pulse tests via `teleop_server.py` to ensure motor wire polarities:
   - Command $+0.30$ duty to M1 $\rightarrow$ Front-Left wheel rotates forward.
   - Command $+0.30$ duty to M2 $\rightarrow$ Front-Right wheel rotates forward.
   - Command $+0.30$ duty to M3 $\rightarrow$ Rear-Left wheel rotates forward.
   - Command $+0.30$ duty to M4 $\rightarrow$ Rear-Right wheel rotates forward.
   *(If any wheel turns backwards, simply swap the two motor wires in that channel's screw terminal).*
3. **Deadband & Stiction Sweep**:
   - Sweep 12-bit PWM values from 0 up to 1000 in steps of 20 to determine the minimum breakaway torque $u_{\text{db}}$ for each wheel on the test surface.
   - Save calibration parameters to `eval/actuators/pca9685_calibration.json`.

---

## 6. Bill of Materials & Procurement Status

| Item | Model / Spec | Source / Vendor | Status | Unit Cost |
| :--- | :--- | :--- | :--- | :--- |
| **Motor Driver Board** | PCA9685 + Dual TB6612 FeatherWing | [RoboticsBD #3514](https://store.roboticsbd.com/stepper-motor-driver/3514-dc-motor-stepper-motor-driver-pca9685-tb6612-featherwing-compatible-robotics-bangladesh.html) | Pending Order | 1,390 BDT |
| **Motors** | 4x TT Geared DC Motors (Metal Gears, 1:90 or 1:48) | Local hobby / RoboticsBD | Pending / Optional | ~250 BDT ea |
| **Battery Supply** | 2S Li-ion Pack (7.4V nominal, 2500mAh 18650) | In possession / standard pack | Ready | - |
| **Wiring** | 4x Female-to-Female Dupont wires (I2C) | Workbench stock | Ready | - |
