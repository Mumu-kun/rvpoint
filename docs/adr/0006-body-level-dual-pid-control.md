# Body-Level Dual PID Control Architecture

Implement a body-level decoupled dual PID controller (Linear Velocity PID + Angular Yaw Rate PID) driven by filtered ARKit VIO odometry observables, rather than wheel-level velocity loops.

## Status
Accepted

## Context
Standard mobile robots close velocity loops on individual wheel motor shafts using high-resolution optical quadrature encoders. In our system, the Orange Pi RV2 drives discrete H-bridges directly without wheel encoders, but receives 60 Hz 6-DoF VIO pose estimates from an onboard iPhone 14 Pro. The VIO measures vehicle rigid-body motion ($v_x, \omega_z$) in the spatial coordinate frame, but cannot measure individual wheel speeds.

## Decision
Implement a **Body-Level Decoupled Dual PID Controller**:
1. **Linear Velocity Loop**: Regulates forward vehicle speed $e_v = v_{\text{cmd}} - \hat{v}_{\text{actual}}$ to produce base throttle $u_v$.
2. **Angular Yaw Rate Loop**: Regulates rotational heading velocity $e_\omega = \omega_{\text{cmd}} - \hat{\omega}_{\text{actual}}$ to produce differential steering torque $u_\omega$.
3. **Feed-Forward & Stiction Compensation**: Add static feed-forward mapping $u_{\text{ff}}(v) = K_{\text{ff}} \cdot v$ and deadband compensation $u_{\text{stiction}} \cdot \operatorname{sgn}(u)$ to overcome low-speed static motor friction.
4. **Actuation Mixer**: Mix the outputs directly to the left and right H-bridge channels:
   $$u_{\text{left}} = \operatorname{clamp}(u_v - u_\omega, -1.0, 1.0)$$
   $$u_{\text{right}} = \operatorname{clamp}(u_v + u_\omega, -1.0, 1.0)$$
5. **Anti-Windup**: Implement conditional back-calculation anti-windup clamping to prevent integrator windup when PWM duty saturates at $\pm 100\%$.

## Consequences
- **Positive**: Directly observable from iPhone VIO; naturally handles differential tank and zero-radius pivot turns; requires zero wheel encoder hardware.
- **Negative**: Relies on smooth derivative filtering of VIO poses to prevent high-frequency noise injection into the D term.

