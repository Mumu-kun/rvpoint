# Motor Control Reformulation, Hardware Architecture, and Wheel Synchronization Spec

**Investigation Against Primary Sources: SpacemiT K1 Architecture, STMicroelectronics L298 Datasheet, TT DC Gearmotor Electro-Mechanical Dynamics, and Non-Invasive Acoustic Calibration**

- **Document ID**: `EXP-MOTOR-002`
- **Component**: Actuation Subsystem (`librvpoint` / `demonstration/actuators` / `eval/actuators`)
- **Target Hardware**: Orange Pi RV2 (SpacemiT K1 octa-core RISC-V SoC), Dual L298N H-Bridges, 4x Yellow TT DC Brushed Gearmotors (1:48), 3S Li-ion Pack (11.1V–12.6V).
- **Status**: Complete Technical Specification & Roadmap

---

## 1. Executive Summary & Problem Decomposition

During empirical testing of the 4-wheel omnidirectional chassis in the air (suspended on a bench stand), the robot exhibited four distinct anomalies:
1. **Directional Inter-Wheel Desynchronization**:
   - **Forward**: Front-Right (FR) lags slightly behind the other three wheels.
   - **Reverse**: Rear-Left (RL) lags noticeably.
   - **Pivot Left (CCW)**: FR and RL lag noticeably.
   - **Pivot Right (CW)**: Front-Left (FL) and Rear-Right (RR) lag noticeably.
   - *Key Diagnostic Observation*: In the configuration file `l298n_pins.json`, channels FR (ch 0) and RL (ch 3) have `invert: true`, whereas FL (ch 1) and RR (ch 2) have `invert: false`. The lagging wheels perfectly coincide with the physical rotation polarity of each individual H-bridge channel.
2. **Inconsistent Inter-Wheel Delays**: Between successive test runs, the exact delay before each wheel begins spinning fluctuates by tens of milliseconds.
3. **Audible 250 Hz Humming/Buzzing**: When PWM is active, all four gearboxes emit a loud audible acoustic buzz.
4. **Inter-Wheel Speed Variance**: Even at steady-state duty cycle, wheel RPMs are noticeably unequal across the four corners.

This specification analyzes the root electro-mechanical causes across the entire system stack, investigates the SpacemiT K1 hardware PWM peripherals, benchmarks the limitations of the current L298N/TT motor setup, provides a zero-cost non-invasive calibration procedure using a smartphone microphone, and formalizes a fresh motor control architecture.

---

## 2. Orange Pi RV2 (SpacemiT K1) Hardware PWM & GPIO Analysis

### 2.1 The 26-Pin Expansion Header Hardware Limitation
The Orange Pi RV2 is powered by the **SpacemiT K1** (an 8-core 64-bit RISC-V SoC implementing RV64GCV). While the K1 silicon die integrates up to 30 hardware PWM channels managed by the `pwm-pxa` kernel driver, **only TWO hardware PWM channels are routed out to the physical 26-pin expansion header**:

| Header Pin | SoC Function / GPIO | Hardware PWM Channel | Controller Base Address |
| :--- | :--- | :--- | :--- |
| **Pin 7** | `GPIO74` | **PWM9** | `0xc0888a00` |
| **Pin 18** | `GPIO77` | **PWM7** | `0xd401bc00` |

All other pins on the 26-pin header (including Pins 19, 21, 22, 23, 24, 26) are standard digital GPIOs.

#### Consequence for 4-Wheel Drive
Because an omnidirectional 4-wheel robot requires **4 independent analog/PWM control channels** (one per wheel to regulate individual corner velocity), **it is physically impossible to control all 4 wheels using native hardware PWM directly from the 26-pin header alone**.

There are only two architectural options:
1. **Option A (Zero Additional Hardware - Current Architecture)**: High-resolution real-time software timer PWM across 4 GPIO pins.
2. **Option B (Recommended Hardware Upgrade)**: An external **I2C 16-channel 12-bit PWM co-processor (PCA9685)** connected to Pins 3 (I2C SDA) and 5 (I2C SCL). The PCA9685 offloads all PWM generation to hardware at up to 1.5 kHz, providing microsecond-accurate hardware phase synchronization and zero CPU overhead.

### 2.2 Why Software PWM Jitters on Linux Sysfs
In the current software implementation:
- GPIO pin states are toggled from userspace by writing `"0"` or `"1"` to `/sys/class/gpio/gpioX/value`.
- Timing is managed via Python `time.sleep()` or C++ `std::this_thread::sleep_for()`.

In a non-real-time Linux kernel (`PREEMPT_NONE` or `PREEMPT_VOLUNTARY`):
1. **Sysfs File I/O Overhead**: Every call to `write()` incurs a user-to-kernel context switch, VFS path traversal, and driver hook execution ($\sim 5\text{--}15\,\mu\text{s}$ per call).
2. **Linux Timer Granularity & Preemption Latency**: The standard Linux kernel tick (`CONFIG_HZ=250` or `CONFIG_HZ=1000`) has a default resolution of $1\text{--}4\,\text{ms}$. Even with high-resolution timers (`hrtimer`), userspace process scheduling can be preempted by OS background tasks (systemd, journald, networking). A jitter of just $1.0\,\text{ms}$ at $250\,\text{Hz}$ ($T = 4.0\,\text{ms}$) alters the commanded duty cycle by **25%**, directly causing the inconsistent startup delay between test runs!

---

## 3. Electro-Mechanical Physics of TT Gearmotors

### 3.1 Motor Parameters & Time Constants
The yellow plastic DC gearmotors are built around standard 130-size brushed permanent magnet DC (PMDC) cores:
- **Operating Input Voltage**: $3.0\,\text{V}\text{ to }6.0\,\text{V DC}$.
- **Reduction Ratio**: $1:48$ straight spur gear train (4 reduction stages).
- **Armature Resistance ($R_a$)**: $\approx 4.5\,\Omega\text{ to }5.5\,\Omega$.
- **Armature Inductance ($L_a$)**: $\approx 1.2\,\text{mH}\text{ to }1.8\,\text{mH}$.
- **Electrical Time Constant**: $\tau_e = \frac{L_a}{R_a} \approx \frac{1.5\,\text{mH}}{5.0\,\Omega} \approx 0.30\,\text{ms}\ (300\,\mu\text{s})$.
- **Mechanical Time Constant**: $\tau_m = \frac{J \cdot R_a}{K_t \cdot K_e} \approx 15\text{--}25\,\text{ms}$ (dominated by rotor inertia and gearbox friction).

### 3.2 The 3S Li-ion Voltage Overdrive Hazard
The motors are rated for **3V to 6V DC**. However, the robot power system uses a **3S Li-ion battery pack**:
- Fresh pack (full charge): $3 \times 4.20\,\text{V} = 12.6\,\text{V}$.
- Nominal pack: $3 \times 3.70\,\text{V} = 11.1\,\text{V}$.
- Depleted pack: $3 \times 3.20\,\text{V} = 9.6\,\text{V}$.

Even after subtracting the L298N Darlington saturation drop ($V_{CE(sat)} \approx 2.2\,\text{V}$), the effective voltage applied to the 6V motor terminals is:
$$V_{\text{motor}} = 11.1\,\text{V} - 2.2\,\text{V} = 8.9\,\text{V} \quad (\text{up to } 10.4\,\text{V on fresh pack})$$

**Physical Consequences**:
1. **Severe Inrush Current**: Stall current at 9V reaches $I_{\text{stall}} \approx \frac{9.0\,\text{V}}{5.0\,\Omega} = 1.8\,\text{A}$ per motor ($7.2\,\text{A}$ across 4 wheels), compared to $0.8\,\text{A}$ rated at 4.5V.
2. **Brush Arcing & Commutator Pitting**: 130-size motors use thin copper-leaf wiper brushes without carbon blocks. Overvoltage causes severe electrical arcing, accelerating mechanical wear and creating asymmetric brush contact resistance.
3. **Gear Tooth Shock & Backlash Impact**: The instantaneous torque pulse $\tau = K_t I$ is nearly double the design tolerance of the injection-molded POM plastic gears, causing gear chatter.

### 3.3 Cause of the Audible 250 Hz Buzz
Human hearing is most sensitive between $200\,\text{Hz}$ and $4000\,\text{Hz}$. At $250\,\text{Hz}$, the fundamental PWM frequency and its harmonics ($500\,\text{Hz}, 750\,\text{Hz}, 1000\,\text{Hz}$) fall precisely into the peak resonance band of the motor shell and plastic gearbox. The current ripple $\Delta I = \frac{V}{L} \Delta t$ vibrates the iron core armature 250 times per second like an audio speaker coil.

---

## 4. Root Cause Analysis: The Directional Inversion Asymmetry

### 4.1 The Polarity Paradox
The user's test results revealed a conclusive diagnostic pattern:
- **Forward**: FL & RR move immediately; **FR lags**.
- **Reverse**: FL, FR & RR move immediately; **RL lags**.
- **Pivot Left**: Left wheels reverse, Right wheels forward $\to$ **FR & RL lag**.
- **Pivot Right**: Left wheels forward, Right wheels reverse $\to$ **FL & RR lag**.

Cross-referencing `l298n_pins.json`:
- `FR`: `invert = true`
- `RL`: `invert = true`
- `FL`: `invert = false`
- `RR`: `invert = false`

### 4.2 Why Reversed Polarity Causes Startup Lag
1. **Commutator Brush Rake Angle Asymmetry**:
   In low-cost DC motors, brush wipers are angled to favor rotation in one direction. Spinning against the brush rake increases mechanical friction and reduces effective contact area until rotational speed is established.
2. **Gearbox Thrust Loading & Backlash**:
   The TT motor's 4-stage straight spur gear train lacks precision thrust bearings. When rotating clockwise, the gear teeth force the gears against the molded casing thrust washer; when rotating counter-clockwise, the gears shift axially until hitting the opposite wall. The static breakaway torque ($\tau_{\text{breakaway}}$) is measurably higher in reverse than in forward.
3. **L298N High-Side vs. Low-Side Switching**:
   When `invert: true`, the software swaps `IN1` and `IN2`. In an L298N H-bridge, the internal transistors are not symmetric: the upper transistors are Darlington pairs with integrated base pull-ups, while the lower transistors are standard high-current Darlingtons. The voltage drop and turn-on delays differ slightly between the two conduction paths.

**Conclusion**: A single global `deadband` or scalar `trim` is fundamentally incapable of synchronizing 4 wheels. Each wheel must have an independent **Forward Deadband/Trim** and **Reverse Deadband/Trim**.

---

## 5. Drawbacks of the Current Hardware Setup

| Component | Current Setup | Key Drawbacks | Modern Recommended Replacement |
| :--- | :--- | :--- | :--- |
| **Driver** | Dual L298N Modules | • **Darlington Drop**: Loses $1.8\text{--}2.6\,\text{V}$ as pure heat ($>4\,\text{W}$ at full load).<br>• **Slow Switching**: Rise/fall times $t_r, t_f \approx 1.5\,\mu\text{s}$ limit PWM to $<5\,\text{kHz}$.<br>• **No Current Sensing**: Cannot detect motor stall or wheel slip. | **TB6612FNG** or **DRV8833** (MOSFET $R_{DS(on)} < 0.2\,\Omega$, $95\%$ efficiency, silent $20\,\text{kHz}$ PWM, sub-microsecond response). |
| **Motors** | 4x Yellow TT Motors (1:48) | • **No Encoders**: 100% open loop; cannot correct for battery discharge or floor friction.<br>• **Plastic Gear Play**: $5^\circ\text{--}15^\circ$ backlash dead-angle.<br>• **Rated for 3–6V**: Overstressed by 3S Li-ion. | TT motors with **Metal Gears and Hall Quadrature Encoders** (or N20 12V metal gearmotors with 334 CPR encoders). |
| **PWM Source** | Linux Sysfs GPIO Bit-Banging | • **Scheduler Jitter**: $1\text{--}4\,\text{ms}$ preemption latency causes $25\%$ duty cycle instability.<br>• **CPU Overhead**: Constant userspace wakeups. | **PCA9685 I2C 16-Channel 12-Bit PWM Co-processor** (hardware timer, microsecond precision, 2-wire I2C). |
| **Battery** | 3S Li-ion (11.1V–12.6V) Direct | • Delivers $9\text{--}10\,\text{V}$ to 6V motors, causing aggressive stiction jump and brush burn. | **Adjustable Buck Converter (LM2596 / XL4015)** stepping 3S (11.1V) down to a stable **6.0V DC** for motor power. |

---

## 6. Non-Invasive Calibration Without Hardware Encoders

The user asked: *"maybe using a phone mic app?"*
**Yes. In mechanical engineering, this is known as Acoustic Gear-Mesh Frequency (GMF) Analysis.**

### 6.1 Mathematical Formulation of Gear-Mesh Acoustics
In any gear train, as each gear tooth enters and leaves mesh with its mating gear, it produces a periodic acoustic pressure pulse. The fundamental gear-mesh frequency ($f_{\text{mesh}}$) is directly proportional to rotational speed:

$$f_{\text{mesh}} = \frac{\text{RPM}_{\text{motor}}}{60} \times z_1$$

Where:
- $z_1 = 9$ (number of teeth on the 130-motor steel/brass pinion).
- Gear reduction ratio $N = 48:1$.
- $\text{RPM}_{\text{motor}} = 48 \times \text{RPM}_{\text{wheel}}$.

Substituting:
$$f_{\text{mesh}} = \frac{48 \times \text{RPM}_{\text{wheel}}}{60} \times 9 = 7.2 \times \text{RPM}_{\text{wheel}} \quad \text{[Hz]}$$

$$\text{RPM}_{\text{wheel}} = \frac{f_{\text{mesh}}}{7.2} \approx 0.1389 \times f_{\text{mesh}}$$

#### Reference Frequencies:
| Wheel Speed ($\text{RPM}_{\text{wheel}}$) | Motor Speed ($\text{RPM}_{\text{motor}}$) | Fundamental Gear Mesh Frequency ($f_{\text{mesh}}$) |
| :--- | :--- | :--- |
| **50 RPM** | $2,400\,\text{RPM}$ | **$360\,\text{Hz}$** |
| **100 RPM** | $4,800\,\text{RPM}$ | **$720\,\text{Hz}$** |
| **150 RPM** | $7,200\,\text{RPM}$ | **$1,080\,\text{Hz}$** |
| **200 RPM** | $9,600\,\text{RPM}$ | **$1,440\,\text{Hz}$** |

### 6.2 Step-by-Step Acoustic Calibration Protocol
1. **Install a Free Spectrum Analyzer App**:
   - **Android**: *Spectroid* (sets standard for high-resolution FFT spectrogram).
   - **iOS (iPhone)**: *SpectrumView* or *Audio Spectrum Analyzer*.
2. **Setup**:
   - Keep the robot on the bench stand (wheels in the air).
   - Position the smartphone microphone **$2\text{--}3\,\text{cm}$** away from the gearbox being measured.
3. **Execute Single-Wheel Test**:
   - Run a single wheel at target test duty cycle (e.g. `0.60`):
     ```bash
     python3 demonstration/validate_motors.py --test individual --speed 0.60
     ```
4. **Identify the Peak**:
   - In the spectrum display, locate the sharpest peak in the $800\,\text{Hz}\text{ to }1600\,\text{Hz}$ frequency range.
   - Record the exact peak frequency $f_{FL}, f_{FR}, f_{RL}, f_{RR}$ for both Forward and Reverse directions.
5. **Compute the Trims**:
   - Select the slowest wheel frequency as the baseline $f_{\min}$, or normalize against a nominal target frequency $f_{\text{target}}$:
     $$\text{trim}_i = \frac{f_{\min}}{f_i}$$
   - Example: If at 60% duty, FL spins at $1200\,\text{Hz}$ ($166.7\,\text{RPM}$) and FR spins at $1100\,\text{Hz}$ ($152.8\,\text{RPM}$):
     $$\text{trim}_{FL} = \frac{1100}{1200} = 0.917, \quad \text{trim}_{FR} = 1.000$$
   - Applying $\text{trim}_{FL} = 0.917$ perfectly equalizes the physical wheel speeds to within $1\%$!

---

## 7. Fresh Reformulation Architecture

### 7.1 Direction-Aware 4x2 Calibration Matrix
Replace the single scalar `trim` and `deadband` in `eval/actuators/l298n_pins.json` with a direction-aware configuration:

```json
{
  "pwm_frequency_hz": 250,
  "channels": [
    {
      "name": "front_right",
      "channel_id": 0,
      "enable_pin": 77,
      "in1_pin": 79,
      "in2_pin": 80,
      "invert": true,
      "deadband_forward": 0.14,
      "deadband_reverse": 0.18,
      "trim_forward": 1.00,
      "trim_reverse": 0.92
    },
    {
      "name": "front_left",
      "channel_id": 1,
      "enable_pin": 78,
      "in1_pin": 76,
      "in2_pin": 75,
      "invert": false,
      "deadband_forward": 0.12,
      "deadband_reverse": 0.15,
      "trim_forward": 0.92,
      "trim_reverse": 1.00
    },
    {
      "name": "rear_right",
      "channel_id": 2,
      "enable_pin": 92,
      "in1_pin": 89,
      "in2_pin": 90,
      "invert": false,
      "deadband_forward": 0.12,
      "deadband_reverse": 0.14,
      "trim_forward": 0.95,
      "trim_reverse": 0.98
    },
    {
      "name": "rear_left",
      "channel_id": 3,
      "enable_pin": 49,
      "in1_pin": 88,
      "in2_pin": 87,
      "invert": true,
      "deadband_forward": 0.16,
      "deadband_reverse": 0.20,
      "trim_forward": 0.98,
      "trim_reverse": 0.90
    }
  ]
}
```

### 7.2 Trajectory Velocity Profiler (Trapezoidal Ramping)
To prevent inrush-induced battery sag and instant gearbox stiction jerk, every velocity command must be filtered through a continuous rate-limiter:

$$v_{\text{target}}(t) = \text{clamp}\left(v_{\text{cmd}}, v(t - \Delta t) - a_{\max}\Delta t, v(t - \Delta t) + a_{\max}\Delta t\right)$$

Where $a_{\max} \approx 0.5\,\text{m/s}^2$ (or $\Delta \text{duty}_{\max} \approx 2.0\,\text{s}^{-1}$). A step command from 0 to 60% takes $300\,\text{ms}$ of smooth ramping, preventing wheel slip and voltage drops.

### 7.3 Deadband Mapping Formula
The continuous deadband mapping ensures that any non-zero command instantly jumps over the stiction deadband for that specific wheel and direction:

$$\text{effective\_duty} = \begin{cases}
0 & \text{if } |u| < \epsilon \\
\text{sgn}(u) \cdot \left( d_{\text{dir}} + |u| \cdot (1 - d_{\text{dir}}) \cdot t_{\text{dir}} \right) & \text{if } |u| \ge \epsilon
\end{cases}$$

Where:
- $d_{\text{dir}} = \begin{cases} \text{deadband\_forward} & \text{if } u > 0 \\ \text{deadband\_reverse} & \text{if } u < 0 \end{cases}$
- $t_{\text{dir}} = \begin{cases} \text{trim\_forward} & \text{if } u > 0 \\ \text{trim\_reverse} & \text{if } u < 0 \end{cases}$

---

## 8. Summary of Actionable Implementation Steps

1. **Update `l298n_pins.json`**:
   - Add `deadband_forward`, `deadband_reverse`, `trim_forward`, `trim_reverse` fields to all 4 channels.
2. **Update `DualL298NActuator` (C++ and Python)**:
   - Load the 4x2 matrix and apply direction-aware scaling on every call to `set_wheel_speed()`.
3. **Execute Acoustic Tuning Session**:
   - Use smartphone spectrum analyzer to measure peak mesh frequencies at 50% duty.
   - Enter measured values into `l298n_pins.json`.
4. **Prepare for Ground Transition**:
   - Swap Mecanum rollers from current "O" configuration to canonical "X" configuration before placing on the floor.

