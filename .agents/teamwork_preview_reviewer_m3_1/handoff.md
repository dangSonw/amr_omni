# Review & Adversarial Challenge Report — Milestone 3: Extrinsics, Covariances, Single TF Authority & Laser Filter Masking

**Reviewer / Critic**: `teamwork_preview_reviewer_m3_1`  
**Working Directory**: `/home/sonev/amr_omni/.agents/teamwork_preview_reviewer_m3_1`  
**Target Milestone**: Milestone 3 (F3.1 – F3.5)  
**Parent Agent**: `parent` (`e684f9d6-654f-439a-9e8c-99049f9780b5`)  
**Date**: 2026-09-20T07:23:00Z  
**Verdict**: **APPROVE**

---

## 1. Observation

### 1.1 EKF Configuration (`src/omni_localization/config/ekf.yaml`)
- **Transform Timeout**: Line 10 specifies:
  ```yaml
  transform_timeout: 0.05
  ```
- **Single TF Authority**: Line 19 specifies:
  ```yaml
  publish_tf: true
  ```
- **Odometry Configuration (`odom0_config`)**: Lines 25–29:
  ```yaml
  odom0_config: [false, false, false,
                 false, false, false,
                 true,  true,  false,
                 false, false, true,
                 false, false, false]
  ```
  Verified via Python: Exactly 15 boolean elements. True indices: `[6, 7, 11]` ($v_x, v_y, \omega_z$). All other 12 indices are `false`.
- **IMU Configuration (`imu0_config`)**: Lines 37–41:
  ```yaml
  imu0_config: [false, false, false,
                false, false, true,
                false, false, false,
                false, false, true,
                true,  true,  false]
  ```
  Verified via Python: Exactly 15 boolean elements. True indices: `[5, 11, 12, 13]` ($\psi, \omega_z, a_x, a_y$). All other 11 indices are `false`.
- **Process Noise Covariance ($\mathbf{Q}$)**: Lines 51–67:
  - Exactly 225 elements ($15 \times 15$).
  - Symmetric ($\mathbf{Q} = \mathbf{Q}^T$).
  - Diagonals: `[0.05, 0.05, 1.0e-4, 1.0e-4, 1.0e-4, 0.03, 0.025, 0.025, 1.0e-4, 1.0e-4, 1.0e-4, 0.02, 0.01, 0.01, 0.01]`.
  - All diagonal elements strictly positive (min diagonal element = $1.0 \times 10^{-4} > 0$).
  - Strictly Positive Definite (minimum eigenvalue $\lambda_{\min} = 1.0 \times 10^{-4} > 0$).
  - Condition number $\kappa(\mathbf{Q}) = 500.0$.
- **Initial Estimate Covariance ($\mathbf{P}_0$)**: Lines 70–86:
  - Exactly 225 elements ($15 \times 15$).
  - Symmetric ($\mathbf{P}_0 = \mathbf{P}_0^T$).
  - Diagonals: `[1.0e-5, 1.0e-5, 0.1, 0.1, 0.1, 1.0e-5, 1.0e-3, 1.0e-3, 0.1, 0.1, 0.1, 1.0e-3, 0.01, 0.01, 0.01]`.
  - All diagonal elements strictly positive (min diagonal element = $1.0 \times 10^{-5} > 0$).
  - Strictly Positive Definite (minimum eigenvalue $\lambda_{\min} = 1.0 \times 10^{-5} > 0$).
  - Condition number $\kappa(\mathbf{P}_0) = 10000.0$.

### 1.2 Laser Filter Configuration (`src/omni_perception/config/laser_filter.yaml`)
- Lines 11–22:
  ```yaml
  filter2:
    name: footprint_filter
    type: laser_filters/LaserScanBoxFilter
    params:
      box_frame: base_link
      min_x: -0.135
      max_x: 0.135
      min_y: -0.135
      max_y: 0.135
      min_z: -0.10
      max_z: 0.50
      invert: false
  ```
  Verified via Python: Box boundaries are exactly $[-0.135, 0.135]$ m along X and Y, with `box_frame: base_link` and `invert: false`.

### 1.3 URDF Kinematic Tree and Sensor Extrinsics
- `src/omni_description/urdf/chassis.xacro` (Lines 17–20):
  ```xml
  <joint name="base_joint" type="fixed">
    <parent link="base_link"/><child link="${parent}"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  ```
  When instantiated by `omni.urdf.xacro` with `parent="base_footprint"`, the joint directs `base_link -> base_footprint`.
- `check_urdf` validation output:
  ```
  robot name is: simple_robot
  ---------- Successfully Parsed XML ---------------
  root Link: base_link has 8 child(ren)
      child(1):  base_footprint
      child(2):  camera_link_1
      child(3):  imu_link_1
          child(1):  imu_link
      child(4):  lidar_link_1
          child(1):  laser_link
      child(5):  omni_wheel_link_1 ...
  ```
  Root link is strictly `base_link`. There are zero multiple parents and zero closed kinematic cycles.
- `src/omni_description/urdf/sensors.xacro`:
  - `laser_link` fixed alias child to `lidar_link_1` (Lines 70–74).
  - `imu_link` fixed alias child to `imu_link_1` (Lines 77–81).

### 1.4 Single TF Authority Cross-Node Audit
- `src/omni_localization/config/ekf.yaml`: `publish_tf: true`.
- `src/omni_simulation/config/simulation.yaml`: `publish_tf: false` (Line 30).
- `src/omni_simulation/omni_simulation/stm32_simulator.py`: Line 1329 guards TF broadcasting with `if self.publish_tf:`.
- `src/omni_hardware/omni_hardware/stm32_bridge.py`: Contains zero TF broadcasters.
- Verified: `ekf_node` is the sole dynamic publisher of `odom -> base_link`.

### 1.5 Independent Test Execution
1. `python3 -m pytest tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py -v`:
   - Result: `25 passed, 139 warnings in 0.86s` (100% pass).
2. `python3 -m pytest tests/e2e/test_production_repo_readiness.py -v`:
   - Result: `7 passed, 139 warnings in 0.52s` (100% pass).
3. `python3 -m pytest -k "ekf or f3 or covariance or laser" tests/e2e -v`:
   - Result: `43 passed, 114 deselected in 0.78s` (100% pass).
4. `python3 -m pytest src/omni_description/test/test_description_files.py -v`:
   - Result: `2 passed in 0.02s` (100% pass).
5. `python3 -m pytest src/omni_bringup/test/test_launch_file.py -v`:
   - Result: `1 passed in 0.03s` (100% pass).
6. `python3 -m pytest tests/ -v` (Full repository test suite):
   - Result: `191 passed, 171 warnings in 13.31s` (100% pass, 0 failures, 0 errors).
7. `node .gitnexus/run.cjs detect-changes --repo amr_omni`:
   - Result: `Risk level: low, Affected processes: 0`.

---

## 2. Logic Chain

1. **EKF Filter Stability & Covariance Positivity**:
   - Zero diagonal elements in Kalman filter covariance matrices lead to mathematical degeneracy ($K \to 0$ or infinite confidence, locking the state and ignoring sensor innovations).
   - From Observation 1.1, all 15 diagonal elements of $\mathbf{Q}$ ($0.05, 0.05, 10^{-4}, \dots$) and $\mathbf{P}_0$ ($10^{-5}, \dots, 0.1, \dots$) are strictly $> 0$.
   - Matrix symmetry $\mathbf{Q} = \mathbf{Q}^T, \mathbf{P}_0 = \mathbf{P}_0^T$ and positive eigenvalues ($\lambda_{\min}(\mathbf{Q}) = 1.0 \times 10^{-4}$, $\lambda_{\min}(\mathbf{P}_0) = 1.0 \times 10^{-5}$) guarantee that both matrices are Symmetric Positive Definite (SPD).
   - Condition numbers $\kappa(\mathbf{Q}) = 500$ and $\kappa(\mathbf{P}_0) = 10^4$ are well conditioned for 64-bit floating point arithmetic, preventing numerical divergence during inversion of the innovation covariance $\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}$.

2. **Single TF Authority & Resolution of Dual-Parent Conflict**:
   - Prior to the worker's fix in `chassis.xacro`, `robot_state_publisher` published a static TF with parent `base_footprint` and child `base_link`. Concurrently, `ekf_node` published a dynamic TF with parent `odom` and child `base_link`.
   - In ROS 2 TF2, assigning two distinct parents (`base_footprint` and `odom`) to the same child frame (`base_link`) produces runtime `TF_MULTIPLE_PARENTS` errors and breaks frame lookup in Nav2.
   - Inverting `base_joint` to `parent="base_link"` and `child="base_footprint"` establishes `base_link` as the root of the URDF tree.
   - Combined with `publish_tf: true` in `ekf.yaml`, `publish_tf: false` in `simulation.yaml`, and zero TF publishers in `stm32_bridge.py`, `ekf_node` is the sole dynamic publisher of `odom -> base_link`.
   - The resulting TF tree `map -> odom -> base_link -> [base_footprint, sensors, wheels]` strictly conforms to REP-105 with zero cycles and zero multiple parents.

3. **Laser Footprint Filter Precision**:
   - The robot's outer envelope (wheelbase $0.1312$ m, track width $0.1312$ m, wheel radius $0.030$ m) extends to $|x|, |y| \le 0.0956$ m at wheel center plus roller outer clearance, reaching $\approx 0.130$ m.
   - Setting filter boundaries to $[-0.135, 0.135]$ m provides a tight 5 mm margin that completely masks chassis reflections while recovering the 25 mm perimeter erroneously masked by the uncalibrated $[-0.16, 0.16]$ m box.
   - Boundary conditions and grazing scans were independently validated in `test_boundary_laser_grazing.py` (5/5 passed).

4. **Forensic Integrity Verification**:
   - We inspected `tests/e2e/harness/extrinsics_oracle.py`, `ekf_sim_oracle.py`, and `laser_filter_oracle.py`.
   - All oracles execute real mathematical algorithms (numpy cross products for lever-arm coriolis/centrifugal effects, full Kalman prediction/update steps with Jacobians and Joseph form covariance update, polar ray-box intersection trigonometry).
   - No hardcoded test responses, dummy bypasses, or skipped assertions exist.
   - All 191 tests run and pass cleanly without mocks that stub out system logic.

---

## 3. Caveats

- **No Caveats**: All configuration files, URDF models, TF trees, and numerical properties have been verified via direct source inspection, mathematical checks, and automated tests.

---

## 4. Conclusion

Milestone 3 deliverables satisfy all requirements and acceptance criteria in `ORIGINAL_REQUEST.md` and `PROJECT.md`:
1. `src/omni_localization/config/ekf.yaml` is fully hardened with `transform_timeout: 0.05`, proper `odom0_config` and `imu0_config` masks, and full $15 \times 15$ strictly positive diagonal, symmetric positive definite covariance matrices.
2. `src/omni_perception/config/laser_filter.yaml` is calibrated to $[-0.135, 0.135]$ m, eliminating blind-spot reflection without clipping external obstacles.
3. Single TF Authority is enforced across URDF (`base_link -> base_footprint`), simulation (`publish_tf: false`), and hardware bridge (no TF).
4. Canonical aliases `laser_link` and `imu_link` guarantee full compatibility with REP-103 and Gazebo plugins.
5. All 191 automated tests in the repository pass 100%.

**Verdict**: **APPROVE**.

---

## 5. Verification Method

To independently verify this report:

1. **Verify M3 Feature Coverage**:
   ```bash
   python3 -m pytest tests/e2e/tier1_feature_coverage/test_f3_extrinsics_fusion.py -v
   ```
   *Expected*: `25 passed`.

2. **Verify Production Repository Readiness**:
   ```bash
   python3 -m pytest tests/e2e/test_production_repo_readiness.py -v
   ```
   *Expected*: `7 passed`.

3. **Verify All M3-Related Tests**:
   ```bash
   python3 -m pytest -k "ekf or f3 or covariance or laser" tests/e2e -v
   ```
   *Expected*: `43 passed, 114 deselected`.

4. **Verify URDF Tree and XML**:
   ```bash
   python3 -m pytest src/omni_description/test/test_description_files.py -v
   source /opt/ros/jazzy/setup.bash && source install/setup.bash && xacro src/omni_description/urdf/omni.urdf.xacro | check_urdf -
   ```
   *Expected*: `root Link: base_link has 8 child(ren)`.

5. **Verify Full System Test Suite**:
   ```bash
   python3 -m pytest tests/ -v
   ```
   *Expected*: `191 passed in ~13s`.

6. **Verify GitNexus Blast Radius**:
   ```bash
   node .gitnexus/run.cjs detect-changes --repo amr_omni
   ```
   *Expected*: `Risk level: low, Affected processes: 0`.
