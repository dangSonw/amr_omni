# Comprehensive Theoretical Specifications & Algorithmic Models
**Project**: AMR Omni Mecanum AGV Calibration & State Estimation Upgrade  
**Author**: `teamwork_preview_spec_miner_survey_1`  
**Date**: 2026-09-19  
**Specification Source**: `/home/sonev/amr_omni/temp/docs` (`amr_omni.docx`, `Encoder.docx`, `Calib.docx`, `Calib_2.docx`) & `/home/sonev/teamwork_projects/amr_omni_calib/ORIGINAL_REQUEST.md`

---

## 1. Executive Summary

This document extracts and formalizes the mathematical models, algorithmic derivations, error propagation mechanisms, and configuration standards for the Mecanum AGV (`amr_omni`) upgrade across Requirements R1 through R5. All formulas are rigorously derived from authoritative reference documents and industrial benchmarks (ODrive Robotics, LinuxCNC, ETH Zurich Kalibr, Tedaldi et al. ICRA 2014, ST AN4508, IEEE Std 952-1997, and REP-103).

---

## 2. Features Discovered

| # | Category | Feature | Description | Inputs | Outputs | Error Behavior | Discovered Via |
|---|----------|---------|-------------|--------|---------|----------------|----------------|
| 1 | R1 Velocity Estimation | Second-Order PLL Tracking Observer | Continuous/discrete 2nd-order phase-locked loop tracking position & velocity without steady-state phase lag | Raw encoder counts $\theta_{meas}$, sample period $T_s$, bandwidth $\omega_{pll}$ | Filtered position $\hat{\theta}$, filtered velocity $\hat{\omega}$ | Stable for $T_s \omega_{pll} < 1$; bounds high-frequency quantization noise | `Encoder.docx` §1.1, §3 |
| 2 | R1 Velocity Estimation | Hybrid M/T Velocity Estimation | LinuxCNC dual-regime estimator measuring pulse count $\Delta m$ and timer clocks $N_{timer}$ across edge-synchronized window | Encoder edges, high-frequency timer clock $f_{clk}$ | Angular velocity $\omega_{M/T}$ with relative error $< 0.1\%$ | Requires zero-speed watchdog timeout when pulse train ceases | `Encoder.docx` §1.1, §3.3 |
| 3 | R1 Hardware Counter | 16-Bit Timer Rollover Safe Difference | Two's complement integer cast computing correct signed count differences across 16-bit register overflows | Raw `CNT` register value ($0 \dots 65535$) | Signed step count `int16_t` $\Delta \text{counts}$ | Fails if rotor moves $\ge 32768$ counts between successive samples | `Encoder.docx` §4.2 |
| 4 | R1 Hardware Counter | Hardware Quadrature 4X & Input Filter | STM32 timer hardware `TIM_ENCODERMODE_TI12` with digital debouncing filter `IC1F=8` | Quadrature channels A and B | Hardware-accumulated pulse count | Filters pulses shorter than 8 core clock periods | `Encoder.docx` §4.1 |
| 5 | R1 Kinematics | Mecanum Inverse Kinematics (IK) | Analytical projection from chassis twist $(v_x, v_y, \omega_z)$ to 4 individual wheel angular velocities | Chassis twist vector $[v_x, v_y, \omega_z]^T$, geometry $(L, W, r)$ | Wheel speed vector $[\omega_1, \omega_2, \omega_3, \omega_4]^T$ | Clamped/scaled by maximum wheel speed; validates finite inputs | `amr_omni.docx` §9.3, `kinematics.py` |
| 6 | R1 Kinematics | Mecanum Forward Kinematics (FK) | Moore-Penrose pseudoinverse reconstruction of chassis twist from 4 wheel velocities | Wheel speed vector $[\omega_1, \omega_2, \omega_3, \omega_4]^T$, geometry $(L, W, r)$ | Reconstructed twist $[v_x, v_y, \omega_z]^T$ | Rejects non-finite inputs; enforces consistency $FK(IK(v)) = v$ | `amr_omni.docx` §9.3, `kinematics.py` |
| 7 | R1 Kinematics | Wheel Radius & Geometry Calibration | In-situ compensation for uneven wheel radii and effective track width via straight and rotational motion | Commanded travel distance, measured laser/tape distance | Scale factors $k_x, k_y, k_\theta$ adjusting $r_i$ and $R$ | Prevents parasitic yaw during straight runs and longitudinal drift during strafing | `amr_omni.docx` §9.4, `Encoder.docx` |
| 8 | R2 IMU Intrinsic | Tedaldi 12-Parameter IMU Model | Full physical model resolving 3 scale factors, 3 biases, and 6 non-orthogonal misalignment angles | Raw accelerometer $a_{raw}$ and gyro $\omega_{raw}$ measurements | Calibrated linear acceleration $a_{calib}$, angular velocity $\omega_{calib}$ | Inverts non-orthogonal matrix $T$; requires uncorrupted sensor data | `Calib.docx` §2.1 |
| 9 | R2 IMU Intrinsic | Turntable-Free Gravity Optimization | Levenberg-Marquardt solver exploiting gravity magnitude invariance $\|a_{true}\| = \|g\|$ across $M$ static poses | Multi-orientation static acceleration samples $a_k$ | Parameter estimates $S_a, T_a, b_a$ | Diverges if orientation poses do not span 3D space sufficiently | `Calib.docx` §2.1 (`imu_tk`) |
| 10 | R2 IMU Intrinsic | Sliding-Window Static Detector | Variance-based statistical filter identifying when IMU is stationary on table | Sliding buffer of accel & gyro samples ($N_w$ samples) | Boolean flag `is_static`, extracted static intervals $[t_{start}, t_{end}]$ | Rejects vibration/transient disturbance above threshold | `Calib.docx` §2.1 (`imu_tk`) |
| 11 | R2 IMU Intrinsic | Lie Group $SO(3)$ Gyro Calibration | Numerical integration of angular velocities between static orientations to align with gravity rotation | Gyro rate stream during reorientation between static poses | Parameter estimates $S_g, T_g, b_g$ | Requires accurate accelerometer calibration as reference baseline | `Calib.docx` §2.1 (`imu_tk`) |
| 12 | R2 IMU Intrinsic | ST AN4508 6-Position Static Calibration | Closed-form least-squares estimation of scale factors and biases along 6 orthogonal gravity orientations | Average readings from $+X, -X, +Y, -Y, +Z, -Z$ facing vertical | Diagonal scale matrix $S_a$ and bias vector $b_a$ | Ignores cross-axis misalignment; sensitive to surface leveling error | `Calib.docx` §2.1 (`imu_calib`) |
| 13 | R2 IMU Intrinsic | Static Gyroscope Zero-Rate Bias Nulling | Online/offline sample averaging during stationary state to eliminate angular drift | Stationary gyro rate samples $\omega(t)$ over duration $\Delta t \ge 10\text{ s}$ | Bias vector $b_g = [\bar{\omega}_x, \bar{\omega}_y, \bar{\omega}_z]^T$ | Fails if vehicle is moving during calibration; residual drift $< 0.05^\circ/\text{s}$ | `amr_omni.docx` §9.4, `Calib.docx` |
| 14 | R2 IMU Intrinsic | Allan Variance Noise Characterization | Time-domain cluster analysis identifying White Noise (ARW/VRW) and Bias Instability (BI) | Multi-hour static IMU data log ($\ge 3\text{ hours}$ at $\ge 200\text{ Hz}$) | Allan deviation curve $\sigma(\tau)$, noise parameters $N_g, K_g, N_a, K_a, B$ | Sensitive to ambient temperature fluctuations; requires warm-up | `Calib.docx` §2.1 (`imu_utils`) |
| 15 | R2 Coordinate System | REP-103 ENU Coordinate Frame Alignment | Rigid mapping of IMU sensor axes to ROS standard East-North-Up body frame with gravity compensation | Raw sensor coordinate stream | Conforming ENU acceleration and angular velocity ($Z$ up, CCW positive) | Flags negative/inverted axis mapping; offsets $+9.80665\text{ m/s}^2$ on $Z$ | `amr_omni.docx` §9.2, REP-103 |
| 16 | R3 Multi-Sensor | Spatial Lever-Arm Kinematics | Rigid-body kinematic transformation compensating for sensor offset $p_B^S$ from rotation center | Sensor translation vector $p_B^S$, chassis twist and angular acceleration | Real linear velocity $v_S$ and centripetal/Coriolis acceleration $a_S$ | Eliminates false centripetal acceleration $a_c = \omega^2 \|p\|$ in EKF | `Calib_2.docx` §1 |
| 17 | R3 Multi-Sensor | Continuous-Time B-Spline Trajectory | Parametric continuous representation $T(t) \in SE(3)$ decoupling asynchronous sensor timestamps | Asynchronous timestamped measurements from IMU, LiDAR, Encoders | Analytically differentiable pose, velocity, and acceleration at any $t$ | Minimizes cumulative B-spline error without linear interpolation artifacts | `Calib_2.docx` §2 (Kalibr) |
| 18 | R3 Multi-Sensor | Temporal Latency Joint Optimization | Co-estimation of communication/filtering latency $\Delta t$ alongside spatial extrinsics $T_B^S$ | Matched multi-sensor data streams during excitation motion | Optimal time offset $\Delta t$ (microsecond precision) | Prevents point-cloud distortion and EKF update step discontinuities | `Calib_2.docx` §2 (Kalibr) |
| 19 | R3 State Estimation | Single TF Authority Architecture | Strict ownership rule ensuring only one node broadcasts `odom -> base_link` | Filtered state from `ekf_node` | Broadcast TF transform `odom -> base_link` | Eliminates TF flickering and jump warnings in Nav2 | `amr_omni.docx` §3.4, §9.2, §10.2 |
| 20 | R3 State Estimation | Empirical EKF Covariance Tuning | Population of non-zero measurement covariances $\mathbf{R}$ and process noise $\mathbf{Q}$ in `ekf.yaml` | Empirical variance from rosbag logs, Allan variance, and field factor ($1.5\times - 2.0\times$) | Tuned covariance matrices in `nav_msgs/Odometry`, `sensor_msgs/Imu`, `ekf.yaml` | Prevents filter overconfidence, numerical singularity, and divergence | `amr_omni.docx` §8.2, §9.2 |
| 21 | R3 Perception | Chassis Blind-Spot Laser Box Filter | Bounding-box spatial filtering stripping scan rays striking robot bodywork | Raw `sensor_msgs/LaserScan`, chassis box bounds | Cleaned `sensor_msgs/LaserScan` with masked body points replaced by NaN | Preserves valid long-range obstacle returns without phantom collisions | `amr_omni.docx` §9.4, `laser_filter.yaml` |
| 22 | R4 Web & Comm | Binary Serial Hardware Contract | Deterministic packet protocol with header, fixed-point integer scaling, XOR checksum, and tail | Float commands, motor speeds, sensor states | Packed binary frames (e.g. 24 bytes) | Rejects corrupted packets via XOR checksum; clamps inputs against overflow | `amr_omni.docx` §2.2, §7.2, §10.2 |
| 23 | R4 Web & Comm | Communication Watchdog & Safety Gate | Dual-sided timeout monitoring (200 ms STM32 hardware, 250 ms ROS 2 node) | Heartbeat and packet reception timestamps | Safe zero velocity command, hardware motor brake engagement | Activates immediate full stop on cable disconnect or process crash | `amr_omni.docx` §2.1, §4.4, §11.1 |
| 24 | R4 Web & Comm | Web UI Calibration Control & Visualization | REST/WebSocket dashboard triggering routines and receiving computed matrices for YAML storage | Operator UI trigger clicks, serial telemetry feedback | Real-time progress, residual plots, updated ROS 2 YAML configuration | Prevents unsafe calibration during vehicle motion; displays convergence status | `ORIGINAL_REQUEST.md` R4 |

---

## 3. Edge Cases

| # | Feature | Input Condition | Observed / Modeled Behavior | Mathematical / Algorithmic Mitigation |
|---|---------|-----------------|-----------------------------|---------------------------------------|
| 1 | PLL Observer | Motor fully stopped ($\omega = 0$, $\Delta \theta = 0$) | Position estimate converges exactly to integer counts; velocity estimate decays asymptotically to 0 without steady-state chatter. | Continuous integration $\dot{\hat{\omega}} = k_i e$ maintains zero velocity without numerical hunting. |
| 2 | PLL Observer | High acceleration step ($a \gg 0$, e.g. wheel stall or sudden brake) | Transient position tracking error occurs; theoretical steady-state error is $e_{ss} = a / k_i = a / \omega_{pll}^2$. | Select bandwidth $\omega_{pll} \ge 100\text{ rad/s}$ so that lag at $a = 2\text{ m/s}^2$ remains $< 0.2\text{ mm}$ on wheel rim. |
| 3 | PLL Observer | Sample time violation ($T_s \cdot \omega_{pll} \ge 1$) | Discrete Euler integration poles move outside the unit circle $|z| > 1$, resulting in violent numerical explosion. | Restrict observer bandwidth to $\omega_{pll} \le \frac{1}{5 T_s}$. For $T_s = 10\text{ ms}$, limit $\omega_{pll} \le 20\text{ rad/s}$; for $T_s = 1\text{ ms}$, limit $\omega_{pll} \le 200\text{ rad/s}$. |
| 4 | M/T Method | Motor decelerates to zero speed | Interval between encoder edges extends to infinity ($N_{clk} \to \infty$); system stalls on previous velocity if not timed out. | Implement watchdog timeout $T_{timeout} = 50\text{ ms}$. If timer exceeds $T_{timeout}$ without edge, force $\omega \to 0$. |
| 5 | Timer Rollover | Rotor rotates across boundary ($65535 \to 0$ or $0 \to 65535$) | Raw difference is $-65535$ or $+65535$. Direct arithmetic causes false maximum velocity spike. | Cast difference to signed 16-bit integer: `(int16_t)(curr - last)`. Two's complement wrap yields exact step $\pm 1$. |
| 6 | Timer Rollover | Velocity exceeds Nyquist limit ($|\Delta \text{counts}| \ge 32768$) | Step count aliases and reverses sign. | Ensure timer sampling frequency $f_s$ satisfies $f_s > \frac{CPR \cdot RPM_{max}}{60 \cdot 32768}$. For $CPR=4000, RPM=1000$, minimum $f_s \approx 2\text{ Hz}$ (well below 100 Hz). |
| 7 | Kinematics | Input twist contains `NaN` or `Inf` (e.g. from upstream topic) | Matrix multiplication propagates `NaN` to all four motor speed commands, destroying PID integrator. | Guard check `isfinite()` on all twist components; if invalid, reject packet and command immediate zero velocity. |
| 8 | Kinematics | Commanded twist exceeds physical wheel speed saturation limit | Wheels that saturate first distort the direction of chassis motion, causing severe path departure. | Scale all 4 wheel speeds uniformly by $S = \max\left(1.0, \frac{\max_i |\omega_i|}{\omega_{max}}\right)$, preserving velocity vector orientation. |
| 9 | Kinematics | Asymmetric wheel radii ($r_i \neq r_{nominal}$) | Pure forward command $(v_x, 0, 0)$ generates unintended parasitic yaw velocity $\omega_z \neq 0$. | Diagonal calibration matrix $\mathbf{K}_r = \mathrm{diag}(k_1, k_2, k_3, k_4)$ applied to forward kinematics. |
| 10 | IMU Calibration | Sensor placed on vibrating surface during static calibration | Vibration inflates windowed variance $\sigma_a^2 > \gamma_a$, causing static detector to reject all samples. | Set static detector variance thresholds $\gamma_a, \gamma_\omega$ above ambient floor; mount IMU on vibration isolation mass during procedure. |
| 11 | IMU Calibration | Gravity vector collinear during 6-position calibration (duplicate faces) | Matrix in least-squares becomes rank-deficient ($\det(\mathbf{A}^T \mathbf{A}) \to 0$), causing matrix inversion explosion. | Require user to confirm all 6 distinct orthogonal faces; verify rank and condition number of calibration observation matrix. |
| 12 | IMU Calibration | Temperature drift during multi-hour Allan variance measurement | Thermal expansion and silicon drift create spurious low-frequency slope, obscuring true Bias Instability minimum. | Enforce mandatory 30–45 minute warm-up period to reach thermal equilibrium before logging Allan variance data. |
| 13 | Coordinate Frame | Inverted IMU mounting axis ($Z$ pointing downward) | Accelerometer measures $-9.81\text{ m/s}^2$; EKF gravity removal adds another $-9.81\text{ m/s}^2$, doubling error to $-19.62\text{ m/s}^2$. | Validate coordinate transformation matrix $R_{sensor}^{body}$ against REP-103; verify positive $+9.81\text{ m/s}^2$ on $Z$ at rest. |
| 14 | Lever-Arm | Rapid in-place rotation ($\omega_z = 2.0\text{ rad/s}$) with offset IMU ($p = 5\text{ cm}$) | Centripetal acceleration $a_c = \omega_z^2 p = 0.2\text{ m/s}^2$ perceived as forward vehicle acceleration by EKF. | Compensate IMU acceleration measurement via $a_B = a_S - \omega_B \times (\omega_B \times p_B^S) - \dot{\omega}_B \times p_B^S$. |
| 15 | Temporal Latency | Asynchronous message arrival with $50\text{ ms}$ transmission delay | EKF fuses IMU rotation with delayed wheel twist, producing spatial distortion $\Delta s = 7.5\text{ cm}$ and coordinate jump. | Implement timestamp offset compensation in ROS 2 (`transform_time_offset`) or co-estimate $\Delta t$ using continuous-time B-spline. |
| 16 | EKF Fusion | Covariance matrix diagonal elements set to 0.0 | Kalman gain computation $K = P H^T (H P H^T + R)^{-1}$ encounters singular matrix division; filter diverges to `NaN`. | Enforce strict non-zero lower bounds on all covariance diagonals ($\sigma^2 \ge 10^{-6}$). |
| 17 | EKF Fusion | Duplicate TF broadcast from multiple nodes | Robot pose in RViz/Nav2 alternates rapidly between two positions (TF flickering), causing planner trajectory abort. | Enforce Single Authority Rule: only `ekf_node` broadcasts `odom -> base_link`; simulator and hardware bridge have `publish_tf: false`. |
| 18 | Serial Contract | Integer overflow during fixed-point encoding ($v_x = 40.0\text{ m/s} \times 1000$) | Value $40000$ overflows `int16_t` ($[-32768, 32767]$), wrapping to $-25536$, commanding full reverse speed! | Strict clamping `fmaxf(-32.0f, fminf(32.0f, v))` before multiplication and conversion to `int16_t`. |

---

## 4. Module 1: R1 Motor Velocity Estimation & Kinematics Consistency

### 4.1. Second-Order Phase-Locked Loop (PLL) Tracking Observer (ODrive)

#### 4.1.1. Motivation & Problem Formulation
In digital motion control, the instantaneous position is measured by a quadrature encoder as an integer pulse count $\theta_m[k] \in \mathbb{Z}$. Estimating velocity by backward Euler differentiation:
$$\omega_{diff}[k] = \frac{\theta_m[k] - \theta_m[k-1]}{T_s}$$
suffers from severe quantization noise variance:
$$\sigma_{\omega}^2 = \frac{2 \sigma_q^2}{T_s^2} = \frac{2 \cdot (1/12 \text{ counts}^2)}{T_s^2} = \frac{1}{6 T_s^2}$$
As $T_s \to 0$, quantization noise diverges. Cascading a 1st-order Low-Pass Filter (LPF) introduces phase lag $\phi(\omega) = -\arctan(\omega / \omega_c)$, which degrades phase margin and limits the maximum achievable PID loop gains.

ODrive replaces simple differentiation with a continuous-time 2nd-Order PLL Tracking Observer, which treats encoder pulse arrivals as phase measurements in a phase-locked loop.

#### 4.1.2. Continuous-Time State-Space Formulation
Let the state vector be:
$$\mathbf{x}(t) = \begin{bmatrix} \hat{\theta}(t) \\ \hat{\omega}(t) \end{bmatrix}$$
where $\hat{\theta}(t)$ is the estimated angular position (in radians or turns) and $\hat{\omega}(t)$ is the estimated angular velocity (in rad/s or turns/s).

Let the measurement input be $y(t) = \theta_m(t)$. The observer dynamics are given by:
$$\begin{aligned}
\dot{\hat{\theta}}(t) &= \hat{\omega}(t) + k_p \left( \theta_m(t) - \hat{\theta}(t) \right) \\
\dot{\hat{\omega}}(t) &= k_i \left( \theta_m(t) - \hat{\theta}(t) \right)
\end{aligned}$$
where $k_p$ is the proportional observer gain and $k_i$ is the integral observer gain.

In matrix form:
$$\dot{\mathbf{x}}(t) = \begin{bmatrix} -k_p & 1 \\ -k_i & 0 \end{bmatrix} \mathbf{x}(t) + \begin{bmatrix} k_p \\ k_i \end{bmatrix} y(t)$$

#### 4.1.3. Transfer Function Analysis
Taking the Laplace transform with zero initial conditions:
$$s \hat{\Theta}(s) = \hat{\Omega}(s) + k_p \left( \Theta_m(s) - \hat{\Theta}(s) \right)$$
$$s \hat{\Omega}(s) = k_i \left( \Theta_m(s) - \hat{\Theta}(s) \right)$$
From the second equation, $\hat{\Omega}(s) = \frac{k_i}{s} \left( \Theta_m(s) - \hat{\Theta}(s) \right)$. Substituting into the first equation:
$$s \hat{\Theta}(s) = \frac{k_i}{s} \left( \Theta_m(s) - \hat{\Theta}(s) \right) + k_p \left( \Theta_m(s) - \hat{\Theta}(s) \right)$$
$$\left( s^2 + k_p s + k_i \right) \hat{\Theta}(s) = \left( k_p s + k_i \right) \Theta_m(s)$$

Thus, the closed-loop position estimation transfer function is:
$$H_{pos}(s) = \frac{\hat{\Theta}(s)}{\Theta_m(s)} = \frac{k_p s + k_i}{s^2 + k_p s + k_i}$$

The velocity estimation transfer function relating true position $\Theta_m(s)$ to estimated velocity $\hat{\Omega}(s)$ is:
$$H_{vel}(s) = \frac{\hat{\Omega}(s)}{\Theta_m(s)} = \frac{k_i s}{s^2 + k_p s + k_i}$$
Since true velocity is $\Omega(s) = s \Theta_m(s)$, the transfer function from true velocity to estimated velocity is:
$$\frac{\hat{\Omega}(s)}{\Omega(s)} = \frac{k_i}{s^2 + k_p s + k_i}$$

#### 4.1.4. Steady-State Tracking Performance (Zero Phase Lag)
Consider a ramp position input $\theta_m(t) = \omega_0 t \cdot u(t) \iff \Theta_m(s) = \frac{\omega_0}{s^2}$ (constant angular velocity).

The position tracking error is $e(t) = \theta_m(t) - \hat{\theta}(t)$:
$$E(s) = \Theta_m(s) - \hat{\Theta}(s) = \left( 1 - \frac{k_p s + k_i}{s^2 + k_p s + k_i} \right) \Theta_m(s) = \frac{s^2}{s^2 + k_p s + k_i} \cdot \frac{\omega_0}{s^2} = \frac{\omega_0}{s^2 + k_p s + k_i}$$
Applying the Final Value Theorem:
$$e_{ss} = \lim_{t \to \infty} e(t) = \lim_{s \to 0} s E(s) = \lim_{s \to 0} \frac{s \omega_0}{s^2 + k_p s + k_i} = 0$$
The steady-state position tracking error for any constant velocity is identically **zero**.

The steady-state velocity estimate is:
$$\lim_{t \to \infty} \hat{\omega}(t) = \lim_{s \to 0} s \hat{\Omega}(s) = \lim_{s \to 0} s \left( \frac{k_i s}{s^2 + k_p s + k_i} \cdot \frac{\omega_0}{s^2} \right) = \lim_{s \to 0} \frac{k_i \omega_0}{s^2 + k_p s + k_i} = \frac{k_i \omega_0}{k_i} = \omega_0$$
The observer tracks the velocity with **zero steady-state lag**.

For an acceleration ramp $\theta_m(t) = \frac{1}{2} a_0 t^2 \iff \Theta_m(s) = \frac{a_0}{s^3}$:
$$e_{ss} = \lim_{s \to 0} s \left( \frac{s^2}{s^2 + k_p s + k_i} \cdot \frac{a_0}{s^3} \right) = \frac{a_0}{k_i}$$
The steady-state position lag during constant acceleration is finite and inversely proportional to $k_i$.

#### 4.1.5. Observer Gain Synthesis
Matching the denominator $s^2 + k_p s + k_i$ to the standard 2nd-order characteristic polynomial:
$$s^2 + 2 \zeta \omega_n s + \omega_n^2$$
where $\omega_n = \omega_{pll}$ is the observer bandwidth in rad/s, and $\zeta$ is the damping ratio.

To prevent overshoot in the velocity estimate, critical damping $\zeta = 1.0$ is chosen:
$$\begin{aligned}
k_p &= 2 \zeta \omega_{pll} = 2 \omega_{pll} \\
k_i &= \omega_{pll}^2 = \frac{1}{4} k_p^2
\end{aligned}$$
*(ODrive industrial standard: `pll_kp_ = 2.0f * pll_bandwidth; pll_ki_ = 0.25f * (pll_kp_ * pll_kp_);`)*

#### 4.1.6. Discrete-Time Algorithmic Implementation (for STM32)
Discretizing with sampling period $T_s$ using forward-Euler state prediction and backward-Euler correction:

At each sampling step $k$:
1. **Prediction step**:
   $$\hat{\theta}_{pred}[k] = \hat{\theta}[k-1] + T_s \cdot \hat{\omega}[k-1]$$
2. **Measurement residual** (handling wrap-around if measuring within one electrical/mechanical turn):
   $$\Delta \theta[k] = \theta_{meas}[k] - \hat{\theta}_{pred}[k]$$
3. **Correction step**:
   $$\hat{\theta}[k] = \hat{\theta}_{pred}[k] + T_s \cdot k_p \cdot \Delta \theta[k]$$
   $$\hat{\omega}[k] = \hat{\omega}[k-1] + T_s \cdot k_i \cdot \Delta \theta[k]$$

**Stability condition**: The discrete state transition matrix is:
$$\mathbf{A}_d = \begin{bmatrix} 1 - T_s k_p & T_s \\ -T_s k_i & 1 \end{bmatrix}$$
The eigenvalues $\lambda$ solve $\det(\lambda \mathbf{I} - \mathbf{A}_d) = \lambda^2 - (2 - T_s k_p) \lambda + (1 - T_s k_p + T_s^2 k_i) = 0$.
For stability ($|\lambda| < 1$), the sampling interval and bandwidth must satisfy:
$$T_s \cdot \omega_{pll} < 1 \implies \omega_{pll} \le \frac{1}{5 T_s} \text{ (practical rule for robust margin)}$$
- If $T_s = 10\text{ ms}$ (100 Hz task): $\omega_{pll} \le 20\text{ rad/s}$.
- If $T_s = 1\text{ ms}$ (1 kHz task): $\omega_{pll} \le 200\text{ rad/s}$.

---

### 4.2. LinuxCNC Hybrid M/T Velocity Estimation Method

#### 4.2.1. Principle of M and T Methods
- **Frequency Method (M-Method)**: Measures pulse count $\Delta m$ during fixed sampling period $T_s$:
  $$\omega_M = \frac{2\pi \cdot \Delta m}{CPR \cdot T_s}$$
  Quantization error: $\delta \omega_M = \frac{2\pi}{CPR \cdot T_s}$. At low speed, $\Delta m \in \{0, 1\}$, producing velocity hunting.
- **Period Method (T-Method)**: Measures time interval $\Delta t = N_{clk} / f_{clk}$ between two consecutive encoder pulses using high-frequency timer clock $f_{clk}$:
  $$\omega_T = \frac{2\pi \cdot f_{clk}}{CPR \cdot N_{clk}}$$
  Quantization error: $\delta \omega_T \approx \frac{\omega^2 \cdot CPR}{2\pi f_{clk}}$. At high speed, $N_{clk}$ becomes small, degrading resolution.

#### 4.2.2. LinuxCNC Hybrid M/T Formulation
LinuxCNC synchronizes the sampling window boundaries strictly with encoder pulse edges over a nominal period $T_s$:
$$\omega_{M/T} = \frac{2\pi \cdot \Delta m}{CPR \cdot \left( \frac{N_{timer}}{f_{clk}} \right)}$$
where:
- $\Delta m$: exact integer number of encoder pulses elapsed between the first and last captured edge.
- $N_{timer}$: exact number of high-frequency timer clock cycles between those same edges.
- $f_{clk}$: timer clock frequency (e.g. 84 MHz on STM32F4).

**Relative Error**:
$$\frac{\delta \omega}{\omega} \le \frac{1}{N_{timer}} = \frac{1}{T_s \cdot f_{clk}}$$
For $T_s = 1\text{ ms}, f_{clk} = 84\text{ MHz} \implies \frac{\delta \omega}{\omega} \le \frac{1}{84000} \approx 0.0012\% (< 0.1\%)$.

#### 4.2.3. Zero-Speed Watchdog Timeout
When the motor stops, no new edges arrive, so $N_{timer}$ would not latch.
A watchdog counter tracks elapsed time $\Delta t_{elapsed}$ since the last edge:
$$\text{If } \Delta t_{elapsed} > T_{timeout} \text{ (e.g. } 50\text{ ms)}: \quad \omega = 0$$

---

### 4.3. 16-Bit Timer Rollover Math (Two's Complement)

On STM32 hardware timers configured in 4X Encoder Mode (`TIM_ENCODERMODE_TI12`), the 16-bit register `TIMx->CNT` ranges from $0$ to $65535$.
When rotating in the forward direction across zero: $65535 \to 0$.
When rotating in the reverse direction across zero: $0 \to 65535$.

Instead of servicing an interrupt on timer overflow/underflow, the exact signed difference is computed using two's complement modulo-$2^{16}$ arithmetic:
$$\Delta \text{counts} = (\text{int16\_t})\left( \text{CNT}_{current} - \text{CNT}_{previous} \right)$$
**Proof**:
- Forward rollover: $\text{CNT}_{prev} = 65530$, $\text{CNT}_{curr} = 5$.
  Unsigned difference: $(5 - 65530) \pmod{65536} = -65525 \equiv 11 = 0x000B$.
  `int16_t(0x000B) = +11`. (Exact)
- Reverse rollover: $\text{CNT}_{prev} = 5$, $\text{CNT}_{curr} = 65530$.
  Unsigned difference: $(65530 - 5) \pmod{65536} = 65525 = 0xFFF5$.
  `int16_t(0xFFF5) = -11`. (Exact)

*Condition*: $|\Delta \text{counts}| < 32768$ between consecutive samples.

---

### 4.4. Mecanum Kinematics Consistency & Wheel Radius Compensation

#### 4.4.1. Mecanum Geometry & Coordinate Definition
Let:
- $L$: longitudinal wheelbase (distance between front and rear axle centers).
- $W$: lateral track width (distance between left and right wheel centers).
- $r$: nominal wheel radius.
- $R = 0.5 \sqrt{L^2 + W^2}$: geometric distance from robot center to each wheel center.
- $\alpha = 45^\circ$: roller angle relative to wheel plane $\implies d = \sin(45^\circ) = \cos(45^\circ) = \sqrt{0.5} \approx 0.70710678$.

Wheel numbering (standard symmetric X-roller configuration):
- Wheel 1: Front-Right ($+L/2, -W/2$, roller angle $-45^\circ$)
- Wheel 2: Front-Left ($+L/2, +W/2$, roller angle $+45^\circ$)
- Wheel 3: Rear-Left ($-L/2, +W/2$, roller angle $+135^\circ$ or $-45^\circ$)
- Wheel 4: Rear-Right ($-L/2, -W/2$, roller angle $-135^\circ$ or $+45^\circ$)

#### 4.4.2. Inverse Kinematics (IK)
The mapping from body twist $\mathbf{v} = [v_x, v_y, \omega_z]^T$ to wheel angular velocities $\boldsymbol{\omega} = [\omega_1, \omega_2, \omega_3, \omega_4]^T$:
$$\begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix} = \frac{1}{r} \mathbf{J}_{geom} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \frac{1}{r} \begin{bmatrix} d & d & R \\ -d & d & R \\ -d & -d & R \\ d & -d & R \end{bmatrix} \begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix}$$

Individual equations:
$$\begin{aligned}
\omega_1 &= \frac{1}{r} \left( d \cdot v_x + d \cdot v_y + R \cdot \omega_z \right) \\
\omega_2 &= \frac{1}{r} \left( -d \cdot v_x + d \cdot v_y + R \cdot \omega_z \right) \\
\omega_3 &= \frac{1}{r} \left( -d \cdot v_x - d \cdot v_y + R \cdot \omega_z \right) \\
\omega_4 &= \frac{1}{r} \left( d \cdot v_x - d \cdot v_y + R \cdot \omega_z \right)
\end{aligned}$$

#### 4.4.3. Forward Kinematics (FK) via Moore-Penrose Pseudoinverse
Since there are 4 wheels and 3 DOF, the system is overdetermined. The least-squares solution is obtained via the Moore-Penrose pseudoinverse $\mathbf{J}_{geom}^\dagger = (\mathbf{J}_{geom}^T \mathbf{J}_{geom})^{-1} \mathbf{J}_{geom}^T$.

Compute $\mathbf{J}_{geom}^T \mathbf{J}_{geom}$:
$$\mathbf{J}_{geom}^T \mathbf{J}_{geom} = \begin{bmatrix} d & -d & -d & d \\ d & d & -d & -d \\ R & R & R & R \end{bmatrix} \begin{bmatrix} d & d & R \\ -d & d & R \\ -d & -d & R \\ d & -d & R \end{bmatrix}$$
Evaluating each entry:
$$\begin{aligned}
(1,1) &= d^2 + (-d)^2 + (-d)^2 + d^2 = 4 d^2 = 4 (0.5) = 2.0 \\
(1,2) &= d^2 - d^2 + d^2 - d^2 = 0 \\
(1,3) &= dR - dR - dR + dR = 0 \\
(2,2) &= d^2 + d^2 + (-d)^2 + (-d)^2 = 4 d^2 = 2.0 \\
(2,3) &= dR + dR - dR - dR = 0 \\
(3,3) &= R^2 + R^2 + R^2 + R^2 = 4 R^2
\end{aligned}$$
Thus, $\mathbf{J}_{geom}^T \mathbf{J}_{geom}$ is purely diagonal:
$$\mathbf{J}_{geom}^T \mathbf{J}_{geom} = \begin{bmatrix} 4 d^2 & 0 & 0 \\ 0 & 4 d^2 & 0 \\ 0 & 0 & 4 R^2 \end{bmatrix}$$
The inverse is:
$$(\mathbf{J}_{geom}^T \mathbf{J}_{geom})^{-1} = \begin{bmatrix} \frac{1}{4 d^2} & 0 & 0 \\ 0 & \frac{1}{4 d^2} & 0 \\ 0 & 0 & \frac{1}{4 R^2} \end{bmatrix}$$
Multiplying by $\mathbf{J}_{geom}^T$:
$$\mathbf{J}_{geom}^\dagger = \begin{bmatrix} \frac{1}{4 d^2} & 0 & 0 \\ 0 & \frac{1}{4 d^2} & 0 \\ 0 & 0 & \frac{1}{4 R^2} \end{bmatrix} \begin{bmatrix} d & -d & -d & d \\ d & d & -d & -d \\ R & R & R & R \end{bmatrix} = \frac{1}{4} \begin{bmatrix} \frac{1}{d} & -\frac{1}{d} & -\frac{1}{d} & \frac{1}{d} \\ \frac{1}{d} & \frac{1}{d} & -\frac{1}{d} & -\frac{1}{d} \\ \frac{1}{R} & \frac{1}{R} & \frac{1}{R} & \frac{1}{R} \end{bmatrix}$$
Therefore, the exact analytical Forward Kinematics equation is:
$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = r \cdot \mathbf{J}_{geom}^\dagger \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix} = \frac{r}{4} \begin{bmatrix} \frac{\omega_1 - \omega_2 - \omega_3 + \omega_4}{d} \\ \frac{\omega_1 + \omega_2 - \omega_3 - \omega_4}{d} \\ \frac{\omega_1 + \omega_2 + \omega_3 + \omega_4}{R} \end{bmatrix}$$

#### 4.4.4. Mathematical Consistency Proof
Substituting the Inverse Kinematics into Forward Kinematics:
$$\mathbf{v}_{reconstructed} = r \mathbf{J}_{geom}^\dagger \left( \frac{1}{r} \mathbf{J}_{geom} \mathbf{v} \right) = \left( \mathbf{J}_{geom}^\dagger \mathbf{J}_{geom} \right) \mathbf{v}$$
Since $\mathbf{J}_{geom}^\dagger \mathbf{J}_{geom} = (\mathbf{J}_{geom}^T \mathbf{J}_{geom})^{-1} (\mathbf{J}_{geom}^T \mathbf{J}_{geom}) = \mathbf{I}_{3 \times 3}$:
$$\mathbf{v}_{reconstructed} \equiv \mathbf{v}$$
For any finite twist $\mathbf{v}$, the mathematical round-trip consistency error is zero:
$$\| FK(IK(\mathbf{v})) - \mathbf{v} \| < 10^{-7} \text{ (float32)}, \quad < 10^{-15} \text{ (float64)}$$

#### 4.4.5. Wheel Radius Error Calibration Model
In physical reality, individual wheel radii deviate due to machining tolerances, tire compression, and wear:
$$r_i = r_{nom} \cdot (1 + \delta r_i) = r_{nom} \cdot k_i$$
Let $\mathbf{K}_r = \mathrm{diag}(k_1, k_2, k_3, k_4)$. The linear speed of each wheel rim is $v_{wheel, i} = r_i \omega_i = r_{nom} k_i \omega_i$.
The compensated forward kinematics becomes:
$$\begin{bmatrix} v_x \\ v_y \\ \omega_z \end{bmatrix} = \frac{r_{nom}}{4} \mathbf{J}_{geom}^\dagger \mathbf{K}_r \begin{bmatrix} \omega_1 \\ \omega_2 \\ \omega_3 \\ \omega_4 \end{bmatrix}$$

**In-Situ Calibration Procedure**:
1. **Longitudinal Test ($v_x = 0.3\text{ m/s}, v_y = 0, \omega_z = 0$)**: Commanded travel $D_{x, cmd} = 2.000\text{ m}$. Measure true ground distance $D_{x, true}$ with laser meter:
   $$k_x = \frac{D_{x, true}}{D_{x, cmd}}$$
2. **Lateral Test ($v_x = 0, v_y = 0.3\text{ m/s}, \omega_z = 0$)**: Commanded travel $D_{y, cmd} = 1.000\text{ m}$. Measure true distance $D_{y, true}$:
   $$k_y = \frac{D_{y, true}}{D_{y, cmd}}$$
3. **Rotational Test ($v_x = 0, v_y = 0, \omega_z = 0.5\text{ rad/s}$)**: Rotate $N = 5$ full revolutions ($\Theta_{cmd} = 10\pi\text{ rad}$). Measure true heading angle $\Theta_{true}$:
   $$k_\theta = \frac{\Theta_{true}}{\Theta_{cmd}}$$
Adjust nominal radius $r \leftarrow r \cdot k_x$, and effective geometry radius $R \leftarrow R \cdot \frac{k_x}{k_\theta}$.

---

## 5. Module 2: R2 IMU Intrinsic Calibration & Denoising on STM32

### 5.1. Tedaldi et al. (ICRA 2014) 12-Parameter IMU Error Model

#### 5.1.1. Physical Error Mechanisms
A low-cost MEMS 6-DoF inertial measurement unit exhibits three primary deterministic error sources:
1. **Scale Factor Errors** ($s_x, s_y, s_z$): Gain inaccuracies in internal capacitive sensing elements and ADC reference voltages.
2. **Non-Orthogonality / Cross-Axis Misalignment** ($\alpha_{ij}$): Non-perpendicularity among mechanical silicon proof masses due to packaging assembly tolerances.
3. **Deterministic Bias** ($b_x, b_y, b_z$): Constant null-voltage offsets caused by piezoresistive stress and DC amplifier drift.

#### 5.1.2. Accelerometer Formulation
Let $\mathbf{a}_{true}$ be the true specific force vector in an ideal orthogonal body frame. The measured raw acceleration $\mathbf{a}_{raw}$ is modeled by:
$$\mathbf{a}_{raw} = \mathbf{T}_a \mathbf{S}_a \mathbf{a}_{true} + \mathbf{b}_a + \mathbf{n}_a$$
where:
- $\mathbf{S}_a = \mathrm{diag}(s_{ax}, s_{ay}, s_{az})$: diagonal scale factor matrix.
- $\mathbf{T}_a$: non-orthogonal transformation matrix aligning the non-orthogonal sensing axes to the ideal orthogonal coordinate system:
  $$\mathbf{T}_a = \begin{bmatrix} 1 & 0 & 0 \\ -\alpha_{yz} & 1 & 0 \\ \alpha_{zy} & -\alpha_{zx} & 1 \end{bmatrix}$$
  where $\alpha_{ij}$ denotes the small angular misalignment between sensor axes.
- $\mathbf{b}_a = [b_{ax}, b_{ay}, b_{az}]^T$: static bias vector.
- $\mathbf{n}_a \sim \mathcal{N}(\mathbf{0}, \boldsymbol{\Sigma}_a)$: additive zero-mean Gaussian white noise.

Let $\mathbf{M}_a = \mathbf{T}_a \mathbf{S}_a$. The calibrated linear acceleration $\mathbf{a}_{calib}$ is recovered via:
$$\mathbf{a}_{calib} = \mathbf{M}_a^{-1} \left( \mathbf{a}_{raw} - \mathbf{b}_a \right)$$

#### 5.1.3. Gyroscope Formulation
Similarly, for the angular velocity:
$$\boldsymbol{\omega}_{raw} = \mathbf{T}_g \mathbf{S}_g \boldsymbol{\omega}_{true} + \mathbf{b}_g + \mathbf{n}_g$$
$$\boldsymbol{\omega}_{calib} = \mathbf{M}_g^{-1} \left( \boldsymbol{\omega}_{raw} - \mathbf{b}_g \right)$$
where $\mathbf{M}_g = \mathbf{T}_g \mathbf{S}_g$, with 3 scale factors, 3 biases, and 3/6 non-orthogonal angles.

---

### 5.2. Turntable-Free Gravity Optimization (Levenberg-Marquardt)

#### 5.2.1. Physical Invariance
When an IMU is stationary in a gravitational field, the true specific force magnitude is invariant to orientation:
$$\| \mathbf{a}_{true} \| = \| \mathbf{g} \| \approx 9.80665\text{ m/s}^2$$

#### 5.2.2. Cost Function for Accelerometer Optimization
Let $M$ be the number of distinct static poses ($M \ge 36$ to $50$ positions). During static interval $k \in \{1, \dots, M\}$, the mean measured raw acceleration is $\bar{\mathbf{a}}_k$.
The non-linear least-squares cost function is:
$$\mathcal{L}(\mathbf{M}_a, \mathbf{b}_a) = \sum_{k=1}^M \left( \| \mathbf{M}_a^{-1} (\bar{\mathbf{a}}_k - \mathbf{b}_a) \|^2 - g^2 \right)^2$$
This cost function is minimized using the Levenberg-Marquardt (LM) algorithm over the 9 parameters ($s_{ax}, s_{ay}, s_{az}, \alpha_{yz}, \alpha_{zy}, \alpha_{zx}, b_{ax}, b_{ay}, b_{az}$).

#### 5.2.3. Sliding-Window Static Detector
To automate static pose extraction without user button presses, a sliding window of length $N_w = 100$ samples ($1.0\text{ s}$ at 100 Hz) monitors sample variance:
$$s_a^2(t) = \frac{1}{N_w} \sum_{i=0}^{N_w - 1} \| \mathbf{a}[t - i] - \bar{\mathbf{a}} \|^2$$
$$s_\omega^2(t) = \frac{1}{N_w} \sum_{i=0}^{N_w - 1} \| \boldsymbol{\omega}[t - i] - \bar{\boldsymbol{\omega}} \|^2$$
Static state is declared if:
$$s_a^2(t) < \gamma_a \quad \text{and} \quad s_\omega^2(t) < \gamma_\omega$$
(Typical thresholds for MEMS: $\gamma_a = 0.005\text{ (m/s}^2)^2$, $\gamma_\omega = 10^{-4}\text{ (rad/s)}^2$).
Consecutive static samples exceeding $T_{static, min} = 2.0\text{ s}$ define static interval $k$.

#### 5.2.4. Gyroscope Numerical Integration on $SO(3)$
Between two static intervals $k$ and $k+1$, the IMU is rotated by hand.
1. From the calibrated accelerometer vectors $\bar{\mathbf{a}}_k$ and $\bar{\mathbf{a}}_{k+1}$, the gravity-based rotation $\Delta \mathbf{R}_{k, k+1}^{acc} \in SO(3)$ is determined.
2. The calibrated gyroscope measurement $\boldsymbol{\omega}_{calib}(t)$ is integrated on $SO(3)$:
   $$\mathbf{R}(t_{i+1}) = \mathbf{R}(t_i) \exp\left( \lfloor \boldsymbol{\omega}_{calib}(t_i) \times \rfloor \Delta t \right)$$
   where $\lfloor \mathbf{v} \times \rfloor$ is the skew-symmetric cross-product matrix.
3. The gyro parameter cost function minimizes the geodesic error on the Lie algebra $\mathfrak{so}(3)$:
   $$\mathcal{L}(\mathbf{M}_g, \mathbf{b}_g) = \sum_{k=1}^{M-1} \| \log\left( (\Delta \mathbf{R}_{k, k+1}^{acc})^T \Delta \mathbf{R}_{k, k+1}^{gyro} \right) \|^2$$

---

### 5.3. ST AN4508 6-Position Static Calibration (`dpkoch/imu_calib`)

For resource-constrained STM32 on-board computation where full LM optimization is too heavy, the ST AN4508 6-position method provides an exact closed-form linear least-squares solution.

#### 5.3.1. 6 Orthogonal Positions
The user places the robot chassis/IMU in 6 distinct positions aligned with the gravity vector:
1. $+X$ pointing up: $\mathbf{a}_{true} = [+g, 0, 0]^T$
2. $-X$ pointing up: $\mathbf{a}_{true} = [-g, 0, 0]^T$
3. $+Y$ pointing up: $\mathbf{a}_{true} = [0, +g, 0]^T$
4. $-Y$ pointing up: $\mathbf{a}_{true} = [0, -g, 0]^T$
5. $+Z$ pointing up: $\mathbf{a}_{true} = [0, 0, +g]^T$
6. $-Z$ pointing up: $\mathbf{a}_{true} = [0, 0, -g]^T$

#### 5.3.2. Closed-Form Scale and Bias Solution
Assuming uncoupled axes ($\mathbf{T}_a = \mathbf{I}$), each axis $j \in \{x, y, z\}$ satisfies:
$$\bar{a}_{j, +j} = s_j (+g) + b_j$$
$$\bar{a}_{j, -j} = s_j (-g) + b_j$$
Subtracting and adding the two equations yields:
$$\begin{aligned}
s_j &= \frac{\bar{a}_{j, +j} - \bar{a}_{j, -j}}{2 g} \\
b_j &= \frac{\bar{a}_{j, +j} + \bar{a}_{j, -j}}{2}
\end{aligned}$$
Calibrated measurement:
$$a_{calib, j} = \frac{a_{raw, j} - b_j}{s_j}$$

#### 5.3.3. Static Gyroscope Zero-Rate Bias Calibration
When the robot is held stationary for $T_{cal} \ge 10\text{ s}$ ($N$ samples at 100 Hz):
$$\mathbf{b}_g = \frac{1}{N} \sum_{i=1}^N \boldsymbol{\omega}_{raw}(t_i)$$
Residual drift acceptance threshold:
$$\| \mathbf{b}_g \| < 0.05^\circ/\text{s} \approx 8.72 \times 10^{-4}\text{ rad/s}$$

---

### 5.4. Allan Variance Analysis (IEEE Std 952-1997 / `imu_utils`)

#### 5.4.1. Mathematical Definition
Let $y(t)$ be the sensor rate output sampled at period $\tau_0$. The time series contains $N$ points.
Divide the record into $K = \lfloor N / m \rfloor$ clusters of duration $\tau = m \tau_0$.
The cluster average is:
$$\bar{y}_k(\tau) = \frac{1}{m} \sum_{i=1}^m y_{(k-1)m + i}$$
The Allan Variance $\sigma^2(\tau)$ is:
$$\sigma^2(\tau) = \frac{1}{2 (K - 1)} \sum_{k=1}^{K - 1} \left( \bar{y}_{k+1}(\tau) - \bar{y}_k(\tau) \right)^2$$
The Allan Deviation (ADEV) is $\sigma(\tau) = \sqrt{\sigma^2(\tau)}$.

#### 5.4.2. Extraction of Core Stochastic Parameters
On a log-log plot of $\sigma(\tau)$ vs $\tau$:
1. **White Noise / Random Walk (Slope = $-1/2$)**:
   - Gyroscope **Angle Random Walk (ARW)** $N_g$ (units: $\text{rad/s}/\sqrt{\text{Hz}}$):
     $$N_g = \sigma_{gyro}(\tau = 1\text{ s})$$
   - Accelerometer **Velocity Random Walk (VRW)** $N_a$ (units: $\text{m/s}^2/\sqrt{\text{Hz}}$):
     $$N_a = \sigma_{accel}(\tau = 1\text{ s})$$
2. **Bias Instability (BI, Slope = $0$)**:
   - The global minimum of the Allan curve at $\tau = \tau_{min}$:
     $$B = \frac{\sigma(\tau_{min})}{\sqrt{2 \ln 2 / \pi}} \approx \frac{\sigma(\tau_{min})}{0.6643}$$
3. **Rate Random Walk (RRW / Drift Rate, Slope = $+1/2$)**:
   - Gyroscope drift rate $K_g$ (units: $\text{rad/s}^2/\sqrt{\text{Hz}}$):
     $$K_g = \sqrt{3} \cdot \sigma_{gyro}(\tau = 3\text{ s})$$

#### 5.4.3. Lab-to-Field Covariance Inflation Rule
When operating on an AMR chassis with spinning motors and floor vibration:
$$\mathbf{R}_{field} = \alpha^2 \cdot \mathbf{R}_{lab}, \quad \text{where } \alpha \in [1.5, 2.0]$$
This prevents the EKF from becoming overconfident and diverging when high-frequency vibration enters the sensor passband.

---

### 5.5. REP-103 ENU Coordinate System Transformation & Gravity Compensation

ROS standard coordinate convention (REP-103):
- $+X$: Forward
- $+Y$: Left
- $+Z$: Up
- Yaw angle $\theta$: Positive Counter-Clockwise (CCW).

If the IMU chip is mounted in orientation $\mathbf{R}_{chip}^{body}$:
$$\mathbf{a}_{body} = \mathbf{R}_{chip}^{body} \mathbf{a}_{chip}, \quad \boldsymbol{\omega}_{body} = \mathbf{R}_{chip}^{body} \boldsymbol{\omega}_{chip}$$
At stationary rest on horizontal ground:
$$\mathbf{a}_{body, static} = \begin{bmatrix} 0 \\ 0 \\ +9.80665 \end{bmatrix} \text{ m/s}^2$$
When `imu0_remove_gravitational_acceleration: true` is set in `ekf.yaml`, `robot_localization` projects the estimated gravity vector onto the sensor frame and subtracts it, leaving $\mathbf{a}_{body} = \mathbf{0}$ at rest.

---

## 6. Module 3: R3 Multi-Sensor Extrinsics, Latency & EKF Covariance Tuning

### 6.1. Spatial Lever-Arm Kinematics

#### 6.1.1. Kinematic Transformation with Lever-Arm
Let $B$ denote the body frame (`base_link`, centered at the ground footprint center of rotation), and let $S$ denote a sensor frame (`imu_link` or `lidar_link`).
The rigid spatial transformation is:
$$\mathbf{T}_B^S = \begin{bmatrix} \mathbf{R}_B^S & \mathbf{p}_B^S \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$
where $\mathbf{p}_B^S = [p_x, p_y, p_z]^T$ is the lever-arm translation vector from $B$ to $S$.

For a rigid body moving with linear velocity $\mathbf{v}_B$, angular velocity $\boldsymbol{\omega}_B$, linear acceleration $\mathbf{a}_B$, and angular acceleration $\dot{\boldsymbol{\omega}}_B$:
- **Sensor Angular Velocity**:
  $$\boldsymbol{\omega}_S = \mathbf{R}_B^S \boldsymbol{\omega}_B$$
- **Sensor Linear Velocity**:
  $$\mathbf{v}_S = \mathbf{R}_B^S \left( \mathbf{v}_B + \boldsymbol{\omega}_B \times \mathbf{p}_B^S \right)$$
- **Sensor Linear Acceleration** (including Coriolis, centripetal, and tangential terms):
  $$\mathbf{a}_S = \mathbf{R}_B^S \left( \mathbf{a}_B + \dot{\boldsymbol{\omega}}_B \times \mathbf{p}_B^S + \boldsymbol{\omega}_B \times (\boldsymbol{\omega}_B \times \mathbf{p}_B^S) \right)$$

#### 6.1.2. Centripetal Error Quantification
When the robot rotates in place ($\mathbf{v}_B = \mathbf{0}, \mathbf{a}_B = \mathbf{0}, \boldsymbol{\omega}_B = [0, 0, \omega_z]^T$):
$$\mathbf{a}_c = \boldsymbol{\omega}_B \times (\boldsymbol{\omega}_B \times \mathbf{p}_B^S) = \begin{bmatrix} 0 \\ 0 \\ \omega_z \end{bmatrix} \times \begin{bmatrix} -\omega_z p_y \\ \omega_z p_x \\ 0 \end{bmatrix} = \begin{bmatrix} -\omega_z^2 p_x \\ -\omega_z^2 p_y \\ 0 \end{bmatrix}$$
Magnitude:
$$\| \mathbf{a}_c \| = \omega_z^2 \sqrt{p_x^2 + p_y^2}$$
*Numerical Example*: If an IMU is mounted $p = 5\text{ cm} = 0.05\text{ m}$ off-center, rotating at $\omega_z = 1.5\text{ rad/s}$ produces:
$$\| \mathbf{a}_c \| = (1.5)^2 \cdot 0.05 = 0.1125\text{ m/s}^2$$
If uncompensated, the EKF mistakes this centripetal acceleration for actual translational motion, resulting in false odometry drift.

---

### 6.2. Temporal Latency Compensation & Continuous-Time B-Spline (Kalibr)

#### 6.2.1. Spatial and Heading Error from Latency
Let $\Delta t$ be the time latency between sensor measurement acquisition and host processing (due to UART buffering, USB packetization, or driver scheduling).
At linear speed $v$ and angular speed $\omega$:
$$\Delta s = v \cdot \Delta t$$
$$\Delta \theta = \omega \cdot \Delta t$$
*Numerical Example*: At $v = 1.5\text{ m/s}$, a latency $\Delta t = 50\text{ ms} = 0.05\text{ s}$ introduces a position error $\Delta s = 7.5\text{ cm}$. At $\omega = 1.0\text{ rad/s}$, heading error $\Delta \theta = 0.05\text{ rad} \approx 2.86^\circ$.

#### 6.2.2. Continuous-Time B-Spline Trajectory Representation
Kalibr models the system trajectory as a continuous curve $\mathbf{T}(t) \in SE(3)$ parameterized by cumulative B-splines:
$$\mathbf{T}(t) = \mathbf{T}_0 \prod_{i=1}^N \exp\left( \tilde{B}_{i, d}(t) \cdot \boldsymbol{\Omega}_i \right)$$
where $\tilde{B}_{i, d}(t)$ are cumulative B-spline basis functions of degree $d$, and $\boldsymbol{\Omega}_i \in \mathfrak{se}(3)$ are Lie algebra control vertices.

**Advantages**:
1. Exact analytical time derivatives $\dot{\mathbf{T}}(t), \ddot{\mathbf{T}}(t)$ exist at *any* continuous time $t \in \mathbb{R}$.
2. Eliminates discrete interpolation noise between asynchronous sensors (e.g. IMU at 200 Hz, LiDAR at 10 Hz, Encoders at 50 Hz).
3. Latency $\Delta t$ is directly optimized as a continuous parameter in the cost function:
   $$\min_{\mathbf{T}_B^S, \Delta t, \mathbf{b}(t)} \sum \text{Residuals}$$

---

### 6.3. Single TF Authority Architecture

```
                 [ slam_toolbox / AMCL ]
                            │
                   map -> odom (TF)
                            ▼
                         [ odom ]
                            │
               odom -> base_link (TF)
               (EKF ekf_node ONLY!)
                            ▼
                      [ base_link ]
                            │
          base_link -> sensors (URDF Static TF)
                            ▼
              [ imu_link / lidar_link_1 ]
```

**Invariant Architectural Rules**:
1. **`map -> odom`**: Exclusively published by SLAM (`slam_toolbox`) or localization (`amcl`).
2. **`odom -> base_link`**: Exclusively published by `robot_localization` (`ekf_filter_node`).
3. **Driver & Simulator Discipline**: Both `stm32_simulator` and physical hardware drivers MUST set `publish_tf: false`.
4. **Sensor Extrinsics**: Transforms from `base_link` to `imu_link_1`, `lidar_link_1`, and `camera_link_1` are static and published via URDF / `robot_state_publisher`.

---

### 6.4. Measurement and Process Covariance Matrix Tuning

#### 6.4.1. Wheel Odometry Measurement Covariance $\mathbf{R}_{wheel}$
In `nav_msgs/Odometry` (`wheel/odom`), the twist covariance $6 \times 6$ matrix is:
$$\mathbf{R}_{twist} = \begin{bmatrix}
\sigma_{vx}^2 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{vy}^2 & 0 & 0 & 0 & 0 \\
0 & 0 & 10^6 & 0 & 0 & 0 \\
0 & 0 & 0 & 10^6 & 0 & 0 \\
0 & 0 & 0 & 0 & 10^6 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{\omega z}^2
\end{bmatrix}$$
Recommended values:
- $\sigma_{vx}^2 = 4.0 \times 10^{-4}\text{ (m/s)}^2 \implies \sigma_{vx} = 0.02\text{ m/s}$
- $\sigma_{vy}^2 = 9.0 \times 10^{-4}\text{ (m/s)}^2 \implies \sigma_{vy} = 0.03\text{ m/s}$ (higher due to Mecanum roller micro-slip)
- $\sigma_{\omega z}^2 = 4.0 \times 10^{-4}\text{ (rad/s)}^2 \implies \sigma_{\omega z} = 0.02\text{ rad/s}$
- Unused dimensions ($v_z, \omega_x, \omega_y$) set to $10^6$ (never 0).

#### 6.4.2. IMU Measurement Covariance $\mathbf{R}_{imu}$
In `sensor_msgs/Imu` (`imu/data`):
- Angular velocity covariance:
  $$\sigma_{gyro, z}^2 = \frac{N_g^2}{T_s} \times \alpha^2 \approx 1.0 \times 10^{-5}\text{ (rad/s)}^2$$
- Linear acceleration covariance:
  $$\sigma_{acc, xy}^2 = \frac{N_a^2}{T_s} \times \alpha^2 \approx 1.0 \times 10^{-3}\text{ (m/s}^2)^2$$

#### 6.4.3. Process Noise Covariance $\mathbf{Q}$ ($15 \times 15$)
In `ekf.yaml`, `process_noise_covariance` must be explicitly declared. Diagonal values for 2D Mecanum robot:
$$\begin{aligned}
\mathbf{Q}_{x, y} &= 1.0 \times 10^{-3}\text{ m}^2, \quad \mathbf{Q}_z = 1.0 \times 10^{-6}\text{ m}^2 \\
\mathbf{Q}_{roll, pitch} &= 1.0 \times 10^{-6}\text{ rad}^2, \quad \mathbf{Q}_{yaw} = 1.0 \times 10^{-3}\text{ rad}^2 \\
\mathbf{Q}_{vx, vy} &= 5.0 \times 10^{-3}\text{ (m/s)}^2, \quad \mathbf{Q}_{vz} = 1.0 \times 10^{-4}\text{ (m/s)}^2 \\
\mathbf{Q}_{vroll, vpitch} &= 1.0 \times 10^{-4}\text{ (rad/s)}^2, \quad \mathbf{Q}_{vyaw} = 2.0 \times 10^{-3}\text{ (rad/s)}^2 \\
\mathbf{Q}_{ax, ay} &= 1.0 \times 10^{-2}\text{ (m/s}^2)^2, \quad \mathbf{Q}_{az} = 1.0 \times 10^{-4}\text{ (m/s}^2)^2
\end{aligned}$$

---

### 6.5. Chassis Blind-Spot Laser Box Filter

To eliminate LiDAR beam reflections off the robot chassis, a `LaserScanBoxFilter` (`laser_filters`) is configured:
- Target frame: `base_link`
- Chassis dimensions: $0.1312\text{ m} \times 0.1312\text{ m}$ (wheelbase $\times$ track width).
- Filter bounding box:
  $$\begin{aligned}
  x &\in [-0.16, +0.16]\text{ m} \\
  y &\in [-0.16, +0.16]\text{ m} \\
  z &\in [-0.10, +0.50]\text{ m}
  \end{aligned}$$
Points falling inside this box are replaced by `NaN` or filtered out (`invert: false`), ensuring that robot structure does not appear as false obstacles in local costmaps.

---

## 7. Module 4: R4 Web UI Calibration Protocol & Serial Bridge Contract

### 7.1. Binary Serial Hardware Contract Specification

#### 7.1.1. Packet Structure
The communication between Jetson (Linux) and STM32 (MCU) utilizes a fixed-length binary frame:
```
[Header: 1B] [Frame ID: 1B] [Length: 1B] [Payload: NB] [Checksum XOR: 1B] [Tail: 1B]
```
- Header: `0x7B` (`{`)
- Tail: `0x7D` (`}`)
- Checksum: Bitwise XOR of all bytes between Header and Checksum:
  $$\text{Checksum} = \bigoplus_{i=\text{FrameID}}^{\text{LastPayloadByte}} \text{Byte}_i$$

#### 7.1.2. Velocity Command Frame (`Jetson -> STM32`, Frame ID = `0x01`, Length = 6 bytes)
Payload contains 3 signed 16-bit integers (`int16_t`, Little-Endian):
$$\text{payload}[0..1] = \text{int16\_t}(v_x \cdot 1000.0)$$
$$\text{payload}[2..3] = \text{int16\_t}(v_y \cdot 1000.0)$$
$$\text{payload}[4..5] = \text{int16\_t}(\omega_z \cdot 1000.0)$$
*Safety Clamping*: Each float must be clamped to $[-32.0, +32.0]$ before integer casting.

#### 7.1.3. Calibration Command Frame (`Jetson -> STM32`, Frame ID = `0x05`, Length = 2 bytes)
- `payload[0]`: Calibration Sub-Command:
  - `0x01`: Start Gyro Bias Nulling
  - `0x02`: Start Accel 6-Position Step (Face Index $1 \dots 6$)
  - `0x03`: Compute & Save Calibration Parameters to EEPROM/Flash
  - `0x04`: Abort / Reset to Factory Defaults
- `payload[1]`: Face Index / Parameter argument.

#### 7.1.4. Calibration Telemetry Report Frame (`STM32 -> Jetson`, Frame ID = `0x85`, Length = 26 bytes)
- `payload[0]`: Calibration Status (`0=IDLE, 1=SAMPLING, 2=COMPUTING, 3=SUCCESS, 4=FAILED`)
- `payload[1]`: Progress percentage ($0 \dots 100\%$)
- `payload[2..7]`: Gyro Bias Vector $[b_{gx}, b_{gy}, b_{gz}]$ (scaled $\times 10^5$, `int16_t` $\times 3$)
- `payload[8..13]`: Accel Bias Vector $[b_{ax}, b_{ay}, b_{az}]$ (scaled $\times 10^3$, `int16_t` $\times 3$)
- `payload[14..19]`: Accel Scale Factors $[s_{ax}, s_{ay}, s_{az}]$ (scaled $\times 10^4$, `uint16_t` $\times 3$)
- `payload[20..25]`: Residual Variance Metric (scaled $\times 10^6$, `uint16_t` $\times 3$)

---

### 7.2. Web UI Calibration State Machine & Auto-Persistence

```
[ Operator Web UI ]
       │  1. POST /api/calib/start { type: "gyro" / "accel" }
       ▼
[ FastAPI Backend ]
       │  2. Encode Serial Command (0x05)
       ▼
[ STM32 MCU ]
       │  3. Execute Real-Time Sampling & Matrix Computation
       │  4. Send Telemetry Report (0x85)
       ▼
[ FastAPI Backend ]
       │  5. Push WebSocket Telemetry to Web Frontend
       │  6. Auto-write computed parameters to config/imu_calib.yaml
       ▼
[ Operator Web UI ]
       - Real-time progress bar & error curve display
       - Instant visual verification of residuals
```

---

## 8. Recommended Parameters & Engineering Tolerances

| Parameter | Recommended Value | Engineering Tolerance / Validation Criteria | Source / Rationale |
|-----------|-------------------|---------------------------------------------|-------------------|
| Encoder Sample Time $T_s$ | $10\text{ ms}$ (100 Hz) or $1\text{ ms}$ (1 kHz) | Jitter $< 5\%$ | `Encoder.docx` §3 |
| PLL Observer Bandwidth $\omega_{pll}$ | $100.0\text{ rad/s}$ ($T_s \le 2\text{ ms}$) | Bound: $\omega_{pll} \le \frac{1}{5 T_s}$ | `Encoder.docx` §1.1 |
| PLL Proportional Gain $k_p$ | $2.0 \cdot \omega_{pll} = 200.0$ | $\zeta = 1.0$ critical damping | ODrive benchmark |
| PLL Integral Gain $k_i$ | $\omega_{pll}^2 = 10000.0$ | $k_i = 0.25 k_p^2$ | ODrive benchmark |
| M/T Method Clock $f_{clk}$ | $84\text{ MHz}$ (STM32 Timer) | Relative velocity error $< 0.1\%$ | `Encoder.docx` §3.3 |
| M/T Watchdog Timeout $T_{timeout}$ | $50\text{ ms}$ | Decays to 0 speed within 100 ms of stop | `Encoder.docx` §3.2 |
| Kinematics Consistency Error | $< 10^{-6}$ | $\|FK(IK(\mathbf{v})) - \mathbf{v}\| < 10^{-6}$ | `amr_omni.docx` §9.4 |
| Wheelbase $L$ | $0.1312\text{ m}$ | Mechanical measurement $\pm 0.5\text{ mm}$ | URDF `wheels.xacro` |
| Track Width $W$ | $0.1312\text{ m}$ | Mechanical measurement $\pm 0.5\text{ mm}$ | URDF `wheels.xacro` |
| Nominal Wheel Radius $r$ | $0.030\text{ m}$ (or URDF model) | Measured in-situ via 2 m laser travel test | `amr_omni.docx` §9.4 |
| Static Gyro Residual Drift | $< 0.05^\circ/\text{s}$ ($8.72 \times 10^{-4}\text{ rad/s}$) | Stationary vehicle over 60 seconds | `amr_omni.docx` §9.4 |
| Allan Variance Test Duration | $\ge 3\text{ to } 6\text{ hours}$ | Stationary room temperature $\pm 1^\circ\text{C}$ | `Calib.docx` §2.1 |
| Field Noise Multiplier $\alpha$ | $1.5\times - 2.0\times$ | Applied to Allan noise densities for EKF | `Calib.docx` §2.1 |
| EKF Filter Rate | $50.0\text{ Hz}$ | Execution period $20\text{ ms} \pm 1\text{ ms}$ | `ekf.yaml` |
| EKF Sensor Timeout | $0.2\text{ s}$ ($200\text{ ms}$) | Fallback to dead-reckoning prediction | `ekf.yaml` |
| STM32 Hardware Watchdog | $200\text{ ms}$ | Shuts off PWM if no command frame received | `amr_omni.docx` §4.4 |
| ROS 2 Command Watchdog | $250\text{ ms}$ | Commands zero velocity on input drop | `amr_omni.docx` §11.1 |
| Laser Chassis Box Bounds | $x, y \in [-0.16, +0.16]\text{ m}$ | Extends beyond wheels ($0.1312\text{ m}$) | `laser_filter.yaml` |

---
*End of Theoretical Specifications Document.*
