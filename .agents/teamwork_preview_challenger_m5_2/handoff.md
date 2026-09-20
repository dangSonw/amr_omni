# Milestone 5 Phase 2 White-Box Adversarial Coverage Hardening Report

**Agent**: `teamwork_preview_challenger_m5_2`  
**Role**: critic, specialist (Empirical Challenger)  
**Scope**: ROS 2 Stack & Web Backend Subsystems (`omni_control`, `omni_localization`, Single TF Authority & URDF, `omni_perception`, `web/backend`)  
**Verdict**: **APPROVE**  

---

## 1. Observation

### 1.1 Target 1: `omni_control` Kinematics Kr Compensation & Singularity Avoidance
- File: `src/omni_control/omni_control/kinematics.py`
  - Lines 11-23 (`validate_twist`): Enforces `math.isfinite(val)` across all twist components `(vx_mps, vy_mps, wz_rad_s)`. Raises `ValueError` if non-finite.
  - Lines 30-35 (`inverse_kinematics`): Strictly rejects non-positive or non-finite `wheelbase_m`, `track_width_m`, and `max_wheel_speed_rad_s` with `ValueError`.
  - Lines 58-96 (`_parse_wheel_radius_correction`): Validates `wheel_radius_m > 0`, verifies `Kr` is positive and diagonal (shape `(4, 4)` or 4-element sequence), and verifies all elements are strictly positive (`arr <= 0.0` raises `ValueError`).
  - Lines 120-128 (`forward_kinematics`): Implements closed-form Moore-Penrose pseudo-inverse `pinv_matrix @ rim_speeds`.
- Empirical Test: `tests/stress/test_m5_2_whitebox_adversarial_hardening.py::TestOmniControlKinematicsAdversarial`
  - `test_kinematics_100k_monte_carlo_roundtrip`: 100,000 randomized velocity vectors in $[-10.0, 10.0]$ m/s and $[-25.0, 25.0]$ rad/s with perturbed $K_r \in [0.5, 2.0]$. Result:
    `[omni_control 100k MC] Completed in 4.63s | Max err: 1.832e-14 | Mean err: 2.212e-15`
    Total violations ($\ge 10^{-5}$): 0 (0.0000%).
  - `test_kinematics_singularity_avoidance`: Zero/negative geometries (`wheelbase <= 0`, `track_width <= 0`, `radius <= 0`, `max_wheel_speed <= 0`, `Kr <= 0`) consistently raise `ValueError`.
  - `test_kinematics_zero_nan_inf_strictness`: NaN and $\pm\infty$ inputs strictly raise `ValueError`. Valid extreme inputs produce 100% finite outputs (0 NaN/Inf).

### 1.2 Target 2: `omni_localization` EKF Covariances, Hard Braking & Vibration Noise
- File: `src/omni_localization/config/ekf.yaml`
  - Lines 51-67: 15x15 `process_noise_covariance` ($\mathbf{Q}$). Diagonal entries: `[0.05, 0.05, 1e-4, 1e-4, 1e-4, 0.03, 0.025, 0.025, 1e-4, 1e-4, 1e-4, 0.02, 0.01, 0.01, 0.01]`. Zero diagonal elements: 0.
  - Lines 70-86: 15x15 `initial_estimate_covariance` ($\mathbf{P}_0$). Diagonal entries: `[1e-5, 1e-5, 1e-1, 1e-1, 1e-1, 1e-5, 1e-3, 1e-3, 1e-1, 1e-1, 1e-1, 1e-3, 1e-2, 1e-2, 1e-2]`. Zero diagonal elements: 0.
- Empirical Test: `tests/stress/test_m5_2_whitebox_adversarial_hardening.py::TestOmniLocalizationEKFAdversarial`
  - `test_ekf_spd_condition_numbers`:
    - Symmetry: $\max |Q - Q^T| = 0.0$, $\max |P_0 - P_0^T| = 0.0$.
    - Eigenvalues: All strictly positive ($\min \lambda(Q) = 1.0 \times 10^{-4}$, $\min \lambda(P_0) = 1.0 \times 10^{-5}$).
    - Condition numbers: $\kappa(Q) = \lambda_{\max}/\lambda_{\min} = 500.0$; $\kappa(P_0) = \lambda_{\max}/\lambda_{\min} = 10,000.0$.
    - Cholesky decomposition: Factorization $L L^T = M$ succeeds with reconstruction residual $< 10^{-14}$.
  - `test_ekf_hard_braking_and_50hz_vibration_stress`:
    - Simulated 100 steps multi-axial maneuver ($v_x = 2.0$ m/s, $v_y = 1.5$ m/s, $\omega_z = 3.0$ rad/s).
    - Hard braking deceleration at $-7.5$ m/s² down to rest ($0.0$ m/s) over 17 steps.
    - 250 steps (5.0 seconds) of 50 Hz structural floor vibration noise ($8.0$ m/s² amplitude with phase jitter) + Gaussian shock noise.
    - Verified filter health at every single time step: 0 NaN/Inf, all eigenvalues of $\mathbf{P} > 0$, $\kappa(\mathbf{P})$ bounded ($\approx 3.0 \times 10^2 \ll 10^8$).

### 1.3 Target 3: Single TF Authority & URDF Tree Acyclicity
- Config & Code Observations:
  - `src/omni_localization/config/ekf.yaml` (line 19): `publish_tf: true`, `odom_frame: odom`, `base_link_frame: base_link`.
  - `src/omni_simulation/config/simulation.yaml` (line 30): `publish_tf: false`.
  - `src/omni_simulation/omni_simulation/stm32_simulator.py` (line 744, 864, 1329): Declares `publish_tf = False` default, and guards `self.tf_broadcaster.sendTransform` behind `if self.publish_tf:`.
  - `src/omni_hardware/omni_hardware/stm32_bridge.py`: Contains 0 instances of `TransformBroadcaster` or `sendTransform`.
  - All launch files in `src/`: 0 static transform publishers between `odom` and `base_link`.
- URDF Topology:
  - URDF generated via xacro (`xacro src/omni_description/urdf/omni.urdf.xacro`): 35 links, 34 joints.
  - `src/omni_description/urdf/chassis.xacro` (lines 17-20): `joint base_joint` has `<parent link="base_link"/><child link="${parent}"/>` where `${parent}` is `base_footprint`.
  - Graph traversal in `test_urdf_tree_acyclicity_and_base_link_root`:
    - Root links (in-degree 0): Exactly 1 (`['base_link']`).
    - `base_footprint` is a child of `base_link`.
    - Fully connected, strictly acyclic directed tree. Zero cycles, zero `TF_MULTIPLE_PARENTS`.

### 1.4 Target 4: `omni_perception` Laser Filter Chassis Masking
- File: `src/omni_perception/config/laser_filter.yaml`
  - Lines 12-23: `LaserScanBoxFilter` configured with `box_frame: base_link`, `min_x: -0.135`, `max_x: 0.135`, `min_y: -0.135`, `max_y: 0.135`, `min_z: -0.10`, `max_z: 0.50`, `invert: false`.
- Empirical Test: `tests/stress/test_m5_2_whitebox_adversarial_hardening.py::TestOmniPerceptionLaserFilterAdversarial`
  - `test_filter_adversarial_dense_polar_scan`: 20,000 synthetic laser scan points across 360° and distances $0.05$ m to $3.0$ m.
    - 768 points inside $[-0.135, 0.135]^2$: 100% masked to NaN (0 false inclusions).
    - 19,232 points outside: 100% preserved (0 false exclusions).
  - `test_filter_boundary_grazing_micrometer_resolution`: Tested boundary points at $\pm 0.135 \pm 10^{-6}$ m. Points $1\,\mu\text{m}$ inside are masked; points $1\,\mu\text{m}$ outside are preserved.

### 1.5 Target 5: `web/backend` Concurrent Atomic YAML Persistence
- File: `web/backend/app/services/calib_service.py`
  - Lines 68, 116: `tmp_file = target_dir / f"{target_file.name}.tmp.{uuid.uuid4().hex}"`.
  - Lines 86-88, 134-136: `f.flush()`, `os.fsync(f.fileno())`, `os.replace(tmp_file, target_file)`.
- Empirical Test: `tests/stress/test_m5_2_whitebox_adversarial_hardening.py::TestWebBackendAtomicYAMLAdversarial`
  - `test_high_concurrency_race_free_writes_and_zero_empty_reads`:
    - 20 concurrent writer threads (10 IMU writers + 10 Wheel writers) performing 50 iterations each = 1,000 total atomic writes.
    - 10 concurrent reader threads continuously reading and parsing both YAML files during writing (completed 1,228 reads).
    - Results:
      - Writer errors (`FileNotFoundError`, `OSError`): 0.
      - Reader errors (0-byte observations, `yaml.safe_load` returning `None`, missing dictionary keys): 0.
      - 100% schema integrity and race-free operation confirmed.

### 1.6 Full Test Suite Execution
- Command: `pytest tests/`
- Result:
  `233 passed, 176 warnings in 43.05s` (Exit Code: 0).
  Zero failed tests, zero regressions.

---

## 2. Logic Chain

1. **Kinematic Invertibility & Singularities**:
   - Mecanum kinematics relates body twist $\mathbf{v} = [v_x, v_y, \omega_z]^T$ to wheel speeds $\boldsymbol{\omega} = [w_1, w_2, w_3, w_4]^T$ via $\boldsymbol{\omega} = \mathbf{K}_r^{-1} \mathbf{M} \mathbf{v}$.
   - The forward kinematics reconstructs $\mathbf{v} = \mathbf{M}^{\dagger} \mathbf{K}_r \boldsymbol{\omega}$.
   - Substituting gives $\mathbf{v}' = \mathbf{M}^{\dagger} \mathbf{K}_r \mathbf{K}_r^{-1} \mathbf{M} \mathbf{v} = \mathbf{M}^{\dagger} \mathbf{M} \mathbf{v}$.
   - Because $\mathbf{M}$ has full column rank 3, $\mathbf{M}^{\dagger} \mathbf{M} = \mathbf{I}_{3 \times 3}$, mathematically guaranteeing $\|FK(IK(\mathbf{v})) - \mathbf{v}\| \equiv 0$ regardless of $\mathbf{K}_r$.
   - Empirically, across 100,000 extreme random Monte Carlo samples in $[-10, 10]$ m/s, the numerical roundtrip residual was bounded by $1.832 \times 10^{-14} \ll 1.0 \times 10^{-5}$.
   - Boundary checks in `validate_twist` and `_parse_wheel_radius_correction` strictly protect all denominators ($L, W, R, K_r, \omega_{\max}$), ensuring no singular division by zero or NaN generation.

2. **EKF Covariance Condition Numbers & Dynamic Stability**:
   - The continuous Riccati update of the Kalman covariance equation requires $\mathbf{Q} \succ 0$ and $\mathbf{P}_0 \succ 0$ to prevent covariance collapse into null subspaces.
   - Exact numerical audit of `src/omni_localization/config/ekf.yaml` proved non-zero diagonals on all 15 states, positive eigenvalues on all 15 modes, and condition numbers $\kappa(\mathbf{Q}) = 500.0$ and $\kappa(\mathbf{P}_0) = 10,000.0$, far below numerical instability limits ($10^{12}$).
   - During extreme stress (violent $-7.5$ m/s² hard braking and simulated 50 Hz structural floor vibration noise), the filter maintained $\mathbf{P} \succ 0$ with $\kappa(\mathbf{P}) \le 300$, demonstrating robust immunity to divergent covariance growth or matrix singularity.

3. **Single TF Authority & URDF Tree Inversion**:
   - In ROS 2, `TF_MULTIPLE_PARENTS` occurs when any frame has more than one incoming transform.
   - Traditional mobile robot URDFs place `base_footprint -> base_joint -> base_link`. When an external localization node publishes `odom -> base_link`, `base_link` has two parents (`base_footprint` and `odom`), triggering a split-brain tree or crash.
   - In `omni_description`, `base_joint` was inverted to `<parent link="base_link"/><child link="base_footprint"/>`. Thus, `base_link` is the single root link (in-degree 0) of the entire robot URDF model.
   - When `ekf_node` alone broadcasts `odom -> base_link` (`publish_tf: true`), `base_link` acquires exactly one parent (`odom`).
   - Audit across all packages confirmed `stm32_simulator` has `publish_tf: false`, `stm32_bridge` has no broadcaster, and launch files contain no duplicate static broadcasters, guaranteeing 100% Single TF Authority.

4. **Laser Filter Spatial Footprint Enclosure**:
   - The physical robot chassis and Mecanum wheels extend to $|x| \le 0.0956$ m and $|y| \le 0.0956$ m.
   - The calibrated box filter envelope $[-0.135, 0.135] \times [-0.135, 0.135]$ m provides a safe $\sim 39$ mm buffer around the chassis.
   - Across 20,000 synthetic polar scan rays, 0 chassis reflections escaped masking and 0 environmental obstacles outside the box were falsely masked. Boundary grazing verified $1\,\mu\text{m}$ transition fidelity.

5. **Concurrency & Atomic Persistence**:
   - File corruption and partial reads under concurrent file writing occur when processes overwrite a target file in-place or use static temporary filenames subject to race conditions.
   - `calib_service.py` prevents this through:
     1. `uuid.uuid4().hex` in temporary file paths, guaranteeing zero inter-thread namespace collisions.
     2. `f.flush()` followed by `os.fsync(f.fileno())`, ensuring data is physically committed to disk buffers.
     3. `os.replace()`, guaranteeing POSIX atomic directory entry update.
   - Under 1,000 concurrent writes across 20 threads and 1,228 concurrent reads across 10 threads, 0 empty reads, 0 file collision exceptions, and 0 partial dictionaries were observed.

---

## 3. Caveats

1. **Hardware-in-the-Loop Floor Dynamics**:
   - The 50 Hz floor vibration and hard braking tests were executed via high-fidelity 15-state dynamic software simulation (`Full15StateEKFSimulator` & `EKFSimOracle2D`). Physical hardware validation on the actual AGV chassis across physical tile gaps remains a field deployment task.
2. **Pytest Deprecation Warnings**:
   - 176 harmless warnings were emitted during the 233-test run, primarily related to Python 3.14 Starlette multipart imports and custom pytest mark registrations (`tier1`, `stress`). These do not affect functionality or numerical correctness.

---

## 4. Conclusion

All five Challenger Objectives have been thoroughly tested, stress-tested under white-box adversarial conditions, and empirically verified:
1. `omni_control`: Kinematics Kr consistency roundtrip error $< 1.832 \times 10^{-14} \ll 10^{-5}$, full singularity avoidance, and zero NaN/Inf.
2. `omni_localization`: EKF Q and P0 strictly SPD ($\kappa(Q)=500$, $\kappa(P_0)=10,000$), stable under multi-axial high-speed maneuvers, $-7.5$ m/s² hard braking, and 50 Hz floor vibration noise.
3. Single TF Authority & URDF: Exactly one broadcaster (`ekf_node`), URDF tree strictly acyclic with `base_link` root link, zero `TF_MULTIPLE_PARENTS`.
4. `omni_perception`: Footprint laser filter $[-0.135, 0.135]^2$ m completely masks chassis (0 false inclusions) while preserving all external obstacles (0 false exclusions).
5. `web/backend`: Atomic YAML persistence in `calib_service.py` is 100% race-free with zero empty reads across 1,000 concurrent writes and 1,200+ concurrent reads.

**Verdict**: **APPROVE**

---

## 5. Verification Method

To independently reproduce and verify all empirical findings:

```bash
# 1. Run the dedicated white-box adversarial stress test suite
pytest -v -s /home/sonev/amr_omni/tests/stress/test_m5_2_whitebox_adversarial_hardening.py

# 2. Run the complete test suite (233 tests across all tiers and stress suites)
pytest /home/sonev/amr_omni/tests/

# 3. Verify URDF tree acyclicity and root link base_link
source /home/sonev/amr_omni/install/setup.bash
xacro /home/sonev/amr_omni/src/omni_description/urdf/omni.urdf.xacro > /tmp/omni_test.urdf
python3 -c "
import xml.etree.ElementTree as ET
root = ET.parse('/tmp/omni_test.urdf').getroot()
links = set(e.attrib['name'] for e in root.findall('link'))
joints = root.findall('joint')
parent_map = {j.find('child').attrib['link']: j.find('parent').attrib['link'] for j in joints}
roots = [l for l in links if l not in parent_map]
assert roots == ['base_link'], f'Roots: {roots}'
assert parent_map['base_footprint'] == 'base_link'
print('URDF strictly acyclic, root=base_link, zero duplicate parents!')
"
```

**Invalidation Conditions**:
- Any kinematics roundtrip error exceeding $10^{-5}$ for unscaled speeds.
- Any zero diagonal element or non-positive eigenvalue in `process_noise_covariance` or `initial_estimate_covariance`.
- Any secondary node setting `publish_tf: true` on `odom -> base_link`.
- Any URDF joint making `base_link` a child of `base_footprint`.
- Any point inside $[-0.135, 0.135]^2$ escaping laser filter masking or outside point falsely masked.
- Any FileNotFoundError or 0-byte file read during concurrent atomic YAML writes.
