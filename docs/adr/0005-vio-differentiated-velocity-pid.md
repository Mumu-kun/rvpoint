# VIO-Differentiated Velocity PID Speed Regulation

Regulate motor speed and compensate for battery voltage sag and wheel slip using software-closed PID loops driven by sensor-timestamp-differentiated iPhone ARKit VIO odometry.

## Status
Accepted

## Context
DC motors powered by discrete H-bridges without hardware optical wheel encoders exhibit non-linear velocity responses. As battery voltage drops from 12.6V to 11.1V during a run, fixed PWM duty cycles cause the vehicle to slow down. Furthermore, variations in floor surface (tile vs carpet) and wheel slippage cause discrepancies between commanded and actual velocity.

## Decision
Compute actual ground-truth linear velocity $v_{\text{actual}}$ and yaw velocity $\omega_{\text{actual}}$ from consecutive ARKit VIO poses:
1. Compute longitudinal displacement in the vehicle's body frame:
   $$\Delta \mathbf{t}_k^{\text{body}} = \mathbf{R}_k^T (\mathbf{t}_k - \mathbf{t}_{k-1}), \quad v_{\text{raw}, k} = \frac{\Delta x_k^{\text{body}}}{\Delta t_{\text{sensor}}}$$
   *(Using body-frame $\Delta x$ preserves directional sign for forward vs. reverse driving, unlike euclidean norms).*
2. Differentiate strictly against the **iPhone's hardware shutter timestamp** ($\Delta t_{\text{sensor}}$ transmitted in the UDP header), completely eliminating Wi-Fi network transmission jitter.
3. Smooth raw velocities through a recursive exponential $\alpha$-$\beta$ filter ($\alpha_v = 0.25, \alpha_\omega = 0.30$) before feeding them into the Dual PID controller.

## Consequences
- **Positive**: Eliminates the hardware cost, wiring complexity, and GPIO interrupt overhead of physical optical encoders; guarantees consistent $0.4\,\text{m/s}$ cruising speed across battery discharge cycles; immune to network latency jitter.
- **Negative**: Requires smooth low-pass filtering on differentiated VIO poses to prevent high-frequency derivative kick.
