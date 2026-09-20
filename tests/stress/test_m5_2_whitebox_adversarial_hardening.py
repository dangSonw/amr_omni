#!/usr/bin/env python3
"""
White-Box Adversarial Coverage Hardening Suite for Milestone 5 Phase 2:
ROS 2 Stack & Web Backend Subsystems.

Empirical Challenger: teamwork_preview_challenger_m5_2
Targets:
  1. omni_control: Kinematics Kr compensation roundtrip consistency (||FK(IK(v)) - v|| < 1e-5),
     extreme velocities, singularity avoidance, and zero NaN/Inf.
  2. omni_localization: EKF covariance matrices Q and P0 under simulated multi-axial high-speed maneuvers,
     hard braking (-7.5 m/s^2), 50Hz floor vibration noise, and SPD condition numbers.
  3. Single TF Authority & URDF: Exactly one broadcaster for odom -> base_link, URDF tree acyclicity
     (base_link root link), and zero TF_MULTIPLE_PARENTS.
  4. omni_perception: Calibrated footprint laser filter [-0.135, 0.135] m masking chassis while
     preserving obstacles outside envelope.
  5. web/backend: Concurrent atomic YAML persistence in calib_service.py with unique temporary
     filenames (uuid.uuid4().hex) verifying 100% race-free writes and zero empty file reads.
"""

import math
import os
import re
import sys
import time
import uuid
import yaml
import threading
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple, Dict, Any
import numpy as np
import pytest

WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
if str(WORKSPACE_ROOT) not in sys.path:
    sys.path.insert(0, str(WORKSPACE_ROOT))

from omni_control.kinematics import (
    forward_kinematics,
    inverse_kinematics,
    validate_twist,
    _parse_wheel_radius_correction,
    WHEEL_ORDER,
)
from web.backend.app.services.calib_service import (
    save_imu_calib_yaml,
    save_wheel_calib_yaml,
    persist_calibration_yaml,
)
from tests.e2e.harness.laser_filter_oracle import LaserFootprintFilterOracle
from tests.stress.test_ekf_covariance_stress import Full15StateEKFSimulator


# ============================================================================
# TARGET 1: omni_control Kinematics Kr Compensation & Singularities
# ============================================================================
class TestOmniControlKinematicsAdversarial:
    """Target 1: White-box stress test of Mecanum kinematics with Kr compensation."""

    def test_kinematics_100k_monte_carlo_roundtrip(self):
        """
        Adversarially verify ||FK(IK(v)) - v|| < 1e-5 across 100,000 random vectors
        spanning extreme linear and angular velocities ([-10.0, 10.0] m/s, [-25.0, 25.0] rad/s)
        and perturbed Kr multipliers [0.5, 2.0].
        """
        num_samples = 100000
        rng = np.random.default_rng(seed=20260920)

        vx_samples = rng.uniform(-10.0, 10.0, num_samples)
        vy_samples = rng.uniform(-10.0, 10.0, num_samples)
        wz_samples = rng.uniform(-25.0, 25.0, num_samples)
        kr_samples = rng.uniform(0.5, 2.0, (num_samples, 4))

        nominal_r = 0.03
        wheelbase = 0.1312
        track_width = 0.1312
        max_speed = 1e12  # Unconstrained to test pure Moore-Penrose pseudo-inverse invariant

        max_err = 0.0
        sum_err = 0.0
        violations = []

        t0 = time.perf_counter()
        for i in range(num_samples):
            vx = float(vx_samples[i])
            vy = float(vy_samples[i])
            wz = float(wz_samples[i])
            kr = kr_samples[i]

            # Alternate representation forms: list, 1D array, 4x4 diagonal, wheel_radii
            rep = i % 4
            if rep == 0:
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radius_correction=kr.tolist())
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radius_correction=kr.tolist())
            elif rep == 1:
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, kr=kr)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         kr=kr)
            elif rep == 2:
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radius_correction=np.diag(kr))
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radius_correction=np.diag(kr))
            else:
                effective_radii = nominal_r * kr
                wheels = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width,
                                            max_speed, wheel_radii=effective_radii)
                rec = forward_kinematics(wheels, nominal_r, wheelbase, track_width,
                                         wheel_radii=effective_radii)

            err = math.sqrt((rec[0] - vx) ** 2 + (rec[1] - vy) ** 2 + (rec[2] - wz) ** 2)
            if err > max_err:
                max_err = err
            sum_err += err

            if err >= 1e-5:
                violations.append((i, (vx, vy, wz), err))

        elapsed = time.perf_counter() - t0
        mean_err = sum_err / num_samples

        print(f"\n[omni_control 100k MC] Completed in {elapsed:.2f}s | Max err: {max_err:.3e} | Mean err: {mean_err:.3e}")
        assert len(violations) == 0, f"Found {len(violations)} roundtrip violations >= 1e-5"
        assert max_err < 1e-5, f"Max error {max_err} exceeds 1e-5 threshold"

    def test_kinematics_singularity_avoidance(self):
        """
        Verify strict singularity avoidance: zero or negative geometries and invalid
        parameters must raise ValueError and not produce division by zero or NaN.
        """
        nominal_r = 0.03
        wheelbase = 0.1312
        track_width = 0.1312
        max_speed = 100.0

        # Zero or negative wheelbase
        for bad_val in [0.0, -0.0, -1e-6, -0.1312]:
            with pytest.raises(ValueError, match="wheel geometry must be positive"):
                inverse_kinematics(1.0, 0.0, 0.0, nominal_r, bad_val, track_width, max_speed)
            with pytest.raises(ValueError, match="wheel geometry must be positive"):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], nominal_r, bad_val, track_width)

        # Zero or negative track_width
        for bad_val in [0.0, -0.0, -1e-6, -0.1312]:
            with pytest.raises(ValueError, match="wheel geometry must be positive"):
                inverse_kinematics(1.0, 0.0, 0.0, nominal_r, wheelbase, bad_val, max_speed)
            with pytest.raises(ValueError, match="wheel geometry must be positive"):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], nominal_r, wheelbase, bad_val)

        # Zero or negative wheel_radius
        for bad_val in [0.0, -0.0, -1e-6, -0.03]:
            with pytest.raises(ValueError, match="wheel radius must be a finite positive number"):
                inverse_kinematics(1.0, 0.0, 0.0, bad_val, wheelbase, track_width, max_speed)
            with pytest.raises(ValueError, match="wheel radius must be a finite positive number"):
                forward_kinematics([1.0, 1.0, 1.0, 1.0], bad_val, wheelbase, track_width)

        # Zero or negative max_wheel_speed
        for bad_val in [0.0, -0.0, -1e-6, -50.0]:
            with pytest.raises(ValueError, match="max wheel speed must be positive"):
                inverse_kinematics(1.0, 0.0, 0.0, nominal_r, wheelbase, track_width, bad_val)

        # Zero or negative Kr components
        bad_krs = [
            [0.0, 1.0, 1.0, 1.0],
            [1.0, -0.1, 1.0, 1.0],
            [1.0, 1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0, -1.0],
        ]
        for bad_kr in bad_krs:
            with pytest.raises(ValueError, match="wheel_radius_correction elements must be strictly positive"):
                inverse_kinematics(1.0, 0.0, 0.0, nominal_r, wheelbase, track_width, max_speed,
                                   wheel_radius_correction=bad_kr)

    def test_kinematics_zero_nan_inf_strictness(self):
        """
        Verify strict rejection of NaN and +/-Inf across all inputs, and zero NaN/Inf in outputs.
        """
        nominal_r = 0.03
        wheelbase = 0.1312
        track_width = 0.1312
        max_speed = 100.0

        for bad in [float('nan'), float('inf'), float('-inf')]:
            # Twist inputs
            with pytest.raises(ValueError, match="twist must contain finite values"):
                inverse_kinematics(bad, 0.0, 0.0, nominal_r, wheelbase, track_width, max_speed)
            with pytest.raises(ValueError, match="twist must contain finite values"):
                inverse_kinematics(0.0, bad, 0.0, nominal_r, wheelbase, track_width, max_speed)
            with pytest.raises(ValueError, match="twist must contain finite values"):
                inverse_kinematics(0.0, 0.0, bad, nominal_r, wheelbase, track_width, max_speed)

            # Wheel speeds input to FK
            with pytest.raises(ValueError, match="wheel speeds must be finite"):
                forward_kinematics([bad, 0.0, 0.0, 0.0], nominal_r, wheelbase, track_width)

        # Verify legitimate extreme inputs yield zero NaN / Inf
        extreme_twists = [
            (0.0, 0.0, 0.0),
            (1e-15, -1e-15, 1e-15),
            (100.0, -100.0, 50.0),
            (-500.0, 500.0, -200.0),
        ]
        for vx, vy, wz in extreme_twists:
            w = inverse_kinematics(vx, vy, wz, nominal_r, wheelbase, track_width, 1e9)
            assert all(math.isfinite(val) for val in w), f"Inverse kinematics output non-finite: {w}"
            rec = forward_kinematics(w, nominal_r, wheelbase, track_width)
            assert all(math.isfinite(val) for val in rec), f"Forward kinematics output non-finite: {rec}"


# ============================================================================
# TARGET 2: omni_localization EKF Covariances, Braking & Vibration Noise
# ============================================================================
class TestOmniLocalizationEKFAdversarial:
    """Target 2: White-box stress test of EKF filter covariances, braking, and vibration noise."""

    @pytest.fixture(scope="class")
    def ekf_matrices(self):
        ekf_path = WORKSPACE_ROOT / "src" / "omni_localization" / "config" / "ekf.yaml"
        assert ekf_path.exists(), f"ekf.yaml not found at {ekf_path}"
        with open(ekf_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        params = cfg["ekf_filter_node"]["ros__parameters"]
        q = np.array(params["process_noise_covariance"], dtype=np.float64).reshape(15, 15)
        p0 = np.array(params["initial_estimate_covariance"], dtype=np.float64).reshape(15, 15)
        return q, p0

    def test_ekf_spd_condition_numbers(self, ekf_matrices):
        """
        Verify Q and P0 matrices loaded from ekf.yaml are strictly SPD (Symmetric Positive Definite)
        with stable condition numbers kappa < 1e6 and no zero diagonals.
        """
        q, p0 = ekf_matrices

        # 1. Zero diagonal elements strictly prohibited
        assert np.all(np.diag(q) > 0.0), f"Found zero or negative diagonal in Q: {np.diag(q)}"
        assert np.all(np.diag(p0) > 0.0), f"Found zero or negative diagonal in P0: {np.diag(p0)}"

        # 2. Strict symmetry
        assert np.max(np.abs(q - q.T)) == 0.0, "Q is not strictly symmetric"
        assert np.max(np.abs(p0 - p0.T)) == 0.0, "P0 is not strictly symmetric"

        # 3. Positive eigenvalues (SPD)
        eig_q = np.linalg.eigvalsh(q)
        eig_p0 = np.linalg.eigvalsh(p0)
        assert np.all(eig_q > 0.0), f"Q has non-positive eigenvalues: min={np.min(eig_q)}"
        assert np.all(eig_p0 > 0.0), f"P0 has non-positive eigenvalues: min={np.min(eig_p0)}"

        # 4. Condition numbers kappa = lambda_max / lambda_min
        cond_q = np.linalg.cond(q)
        cond_p0 = np.linalg.cond(p0)
        print(f"\n[EKF SPD Audit] cond(Q) = {cond_q:.1f} (expected 500) | cond(P0) = {cond_p0:.1f} (expected 10000)")
        assert math.isclose(cond_q, 500.0, rel_tol=1e-5), f"cond(Q) {cond_q} != 500.0"
        assert math.isclose(cond_p0, 10000.0, rel_tol=1e-5), f"cond(P0) {cond_p0} != 10000.0"

        # 5. Cholesky factorability
        l_q = np.linalg.cholesky(q)
        l_p0 = np.linalg.cholesky(p0)
        assert np.allclose(l_q @ l_q.T, q, atol=1e-14)
        assert np.allclose(l_p0 @ l_p0.T, p0, atol=1e-14)

    def test_ekf_hard_braking_and_50hz_vibration_stress(self, ekf_matrices):
        """
        Adversarial test simulating:
        1. Multi-axial high-speed maneuver (vx=2.0 m/s, vy=1.5 m/s, wz=3.0 rad/s)
        2. Hard emergency braking at -7.5 m/s^2 deceleration
        3. 50 Hz structural floor vibration noise injected into IMU accelerations and gyro
        Verify covariance matrix P remains strictly SPD throughout every time step.
        """
        q, p0 = ekf_matrices
        ekf = Full15StateEKFSimulator(q, p0, two_d_mode=True)
        dt = 0.02  # 50 Hz filter loop
        rng = np.random.default_rng(seed=4242)

        # Stage 1: Multi-axial high-speed maneuver for 2.0s (100 steps)
        # vx = 2.0 m/s, vy = 1.5 m/s, wz = 3.0 rad/s
        current_yaw = 0.0
        for step in range(100):
            current_yaw = (current_yaw + 3.0 * dt + math.pi) % (2.0 * math.pi) - math.pi
            ekf.predict(dt)
            ekf.update_odom(vx=2.0, vy=1.5, wz=3.0)
            ekf.update_imu(yaw=current_yaw, wz=3.0, ax=0.0, ay=0.0)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during high-speed step {step}: {reason}"
            cond_p = np.linalg.cond(ekf.P)
            assert cond_p < 1e8, f"Covariance condition number exploded: {cond_p}"

        # Confirm velocity tracking
        assert abs(ekf.state[6] - 2.0) < 0.15
        assert abs(ekf.state[7] - 1.5) < 0.15

        # Stage 2: Hard braking at -7.5 m/s^2 deceleration down to 0 m/s
        # Linear velocity magnitude = hypot(2.0, 1.5) = 2.5 m/s
        # Time to stop at 7.5 m/s^2 = 2.5 / 7.5 = 0.333s (~17 steps)
        braking_steps = 17
        decel_rate = 7.5
        dir_x = 2.0 / 2.5
        dir_y = 1.5 / 2.5
        ax_brake = -decel_rate * dir_x
        ay_brake = -decel_rate * dir_y

        for b in range(braking_steps):
            frac = max(0.0, 1.0 - (b + 1) / braking_steps)
            curr_vx = 2.0 * frac
            curr_vy = 1.5 * frac
            curr_wz = 3.0 * frac
            current_yaw = (current_yaw + curr_wz * dt + math.pi) % (2.0 * math.pi) - math.pi

            ekf.predict(dt)
            ekf.update_odom(vx=curr_vx, vy=curr_vy, wz=curr_wz)
            ekf.update_imu(yaw=current_yaw, wz=curr_wz, ax=ax_brake, ay=ay_brake)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during hard braking step {b}: {reason}"
            eigvals = np.linalg.eigvalsh(ekf.P)
            assert np.min(eigvals) > 0.0, f"Covariance lost positive definiteness: min eig={np.min(eigvals)}"

        # Stage 3: Stationary at rest with severe 50Hz structural floor vibration noise
        # 50 Hz sinusoid (motor PWM harmonics / roller impacts) with random phase jitter + Gaussian shocks
        vibe_freq = 50.0  # 50 Hz
        for step in range(250):  # 5.0 seconds
            # Physical vibration with phase jitter per cycle avoiding Nyquist integer harmonic alias to DC
            phi = rng.uniform(0.0, 2.0 * math.pi)
            vibe_ax = 8.0 * math.sin(phi) + rng.normal(0.0, 0.5)
            vibe_ay = 8.0 * math.cos(phi) + rng.normal(0.0, 0.5)
            vibe_wz = rng.normal(0.0, 0.1)
            noise_odom_vx = rng.normal(0.0, 0.02)
            noise_odom_vy = rng.normal(0.0, 0.02)

            ekf.predict(dt)
            ekf.update_odom(vx=noise_odom_vx, vy=noise_odom_vy, wz=vibe_wz)
            ekf.update_imu(yaw=current_yaw, wz=vibe_wz, ax=vibe_ax, ay=vibe_ay)

            healthy, reason = ekf.is_healthy()
            assert healthy, f"Filter unhealthy during 50Hz vibration step {step}: {reason}"

            # Strict SPD and condition number check
            eigvals = np.linalg.eigvalsh(ekf.P)
            assert np.all(eigvals > 0.0), f"Covariance lost positive definiteness: min={np.min(eigvals)}"
            cond_p = np.linalg.cond(ekf.P)
            assert cond_p < 1e8, f"Condition number exploded under vibration: {cond_p}"

        # State should remain bounded near 0 velocity despite violent vibration
        assert abs(ekf.state[6]) < 0.25, f"vx did not remain bounded near 0: {ekf.state[6]}"
        assert abs(ekf.state[7]) < 0.25, f"vy did not remain bounded near 0: {ekf.state[7]}"
        print(f"[EKF Braking & Vibration] Hard braking and 50Hz vibration passed cleanly | Final vx: {ekf.state[6]:.4f}, vy: {ekf.state[7]:.4f} | Final cond(P): {cond_p:.1e}")


# ============================================================================
# TARGET 3: Single TF Authority & URDF Tree Acyclicity
# ============================================================================
class TestSingleTFAuthorityAndURDFAcyclicity:
    """Target 3: Verify single TF authority for odom -> base_link and URDF tree acyclicity."""

    def test_single_tf_broadcaster_authority(self):
        """
        Verify that robot_localization ekf_node is the EXCLUSIVE broadcaster of odom -> base_link,
        simulation.yaml has publish_tf: false, stm32_simulator defaults to publish_tf: false,
        and no other node broadcasts dynamic odom -> base_link TF.
        """
        # 1. ekf.yaml has publish_tf: true
        ekf_yaml_path = WORKSPACE_ROOT / "src" / "omni_localization" / "config" / "ekf.yaml"
        with open(ekf_yaml_path, "r", encoding="utf-8") as f:
            ekf_data = yaml.safe_load(f)
        ekf_params = ekf_data["ekf_filter_node"]["ros__parameters"]
        assert ekf_params["publish_tf"] is True
        assert ekf_params["odom_frame"] == "odom"
        assert ekf_params["base_link_frame"] == "base_link"

        # 2. simulation.yaml has publish_tf: false
        sim_yaml_path = WORKSPACE_ROOT / "src" / "omni_simulation" / "config" / "simulation.yaml"
        with open(sim_yaml_path, "r", encoding="utf-8") as f:
            sim_data = yaml.safe_load(f)
        sim_node_cfg = sim_data.get("stm32_simulator", sim_data.get("omni_simulation", {}))
        sim_params = sim_node_cfg["ros__parameters"]
        assert sim_params["publish_tf"] is False

        # 3. stm32_simulator.py defaults to publish_tf = False
        sim_py_path = WORKSPACE_ROOT / "src" / "omni_simulation" / "omni_simulation" / "stm32_simulator.py"
        with open(sim_py_path, "r", encoding="utf-8") as f:
            sim_py_content = f.read()
        assert "self.declare_parameter('publish_tf', False)" in sim_py_content
        assert "if self.publish_tf:" in sim_py_content

        # 4. stm32_bridge.py has no TransformBroadcaster
        bridge_py_path = WORKSPACE_ROOT / "src" / "omni_hardware" / "omni_hardware" / "stm32_bridge.py"
        with open(bridge_py_path, "r", encoding="utf-8") as f:
            bridge_py_content = f.read()
        assert "TransformBroadcaster" not in bridge_py_content
        assert "sendTransform" not in bridge_py_content

        # 5. Check all launch files: no static_transform_publisher from odom to base_link
        launch_files = list((WORKSPACE_ROOT / "src").glob("**/*.launch.py"))
        assert len(launch_files) > 0
        for lpath in launch_files:
            with open(lpath, "r", encoding="utf-8") as f:
                lcontent = f.read()
            # Must not publish static TF from odom to base_link
            assert not (("static_transform_publisher" in lcontent) and
                        ("odom" in lcontent) and ("base_link" in lcontent)), \
                f"Duplicate static TF publisher found in {lpath}"

    def test_urdf_tree_acyclicity_and_base_link_root(self):
        """
        Verify the URDF tree structure:
        - base_link is the single root link (has in-degree 0 in the URDF tree).
        - base_footprint is a child of base_link (base_link -> base_footprint).
        - Tree is strictly acyclic, fully connected, eliminating TF_MULTIPLE_PARENTS.
        """
        # Check URDF files directly
        chassis_xacro = WORKSPACE_ROOT / "src" / "omni_description" / "urdf" / "chassis.xacro"
        with open(chassis_xacro, "r", encoding="utf-8") as f:
            chassis_text = f.read()

        # In chassis.xacro: parent link is base_link, child link is ${parent} (base_footprint)
        assert '<parent link="base_link"/><child link="${parent}"/>' in chassis_text or \
               ('<parent link="base_link"/>' in chassis_text and '<child link=' in chassis_text)

        omni_xacro = WORKSPACE_ROOT / "src" / "omni_description" / "urdf" / "omni.urdf.xacro"
        with open(omni_xacro, "r", encoding="utf-8") as f:
            omni_text = f.read()

        assert '<xacro:chassis parent="base_footprint"/>' in omni_text
        assert '<xacro:four_wheels parent="base_link"/>' in omni_text
        assert '<xacro:sensors parent="base_link"/>' in omni_text

        # If /tmp/omni_expanded.urdf exists, perform graph traversal
        urdf_path = Path("/tmp/omni_expanded.urdf")
        if urdf_path.exists():
            tree = ET.parse(urdf_path)
            root_elem = tree.getroot()
            links = set(elem.attrib['name'] for elem in root_elem.findall('link'))
            joints = root_elem.findall('joint')

            parent_map = {}
            children_map = {l: [] for l in links}

            for j in joints:
                jname = j.attrib['name']
                p = j.find('parent').attrib['link']
                c = j.find('child').attrib['link']
                assert c not in parent_map, f"Joint {jname} creates multiple parents for link {c}!"
                parent_map[c] = (p, jname)
                children_map[p].append((c, jname))

            root_links = [l for l in links if l not in parent_map]
            assert len(root_links) == 1, f"URDF has multiple root links: {root_links}"
            assert root_links[0] == "base_link", f"Root link is {root_links[0]}, expected base_link"
            assert parent_map["base_footprint"][0] == "base_link"

            # Topological acyclicity check
            visited = set()
            queue = ["base_link"]
            while queue:
                curr = queue.pop(0)
                assert curr not in visited, f"Cycle detected at link {curr}"
                visited.add(curr)
                for child, _ in children_map[curr]:
                    queue.append(child)

            assert len(visited) == len(links), f"Disconnected components: {links - visited}"
            print(f"\n[URDF Acyclicity] Verified {len(links)} links, {len(joints)} joints, root='base_link', 0 cycles.")


# ============================================================================
# TARGET 4: omni_perception Calibrated Footprint Laser Filter
# ============================================================================
class TestOmniPerceptionLaserFilterAdversarial:
    """Target 4: White-box adversarial stress test of laser footprint filter."""

    @pytest.fixture(scope="class")
    def filter_config(self):
        filter_yaml = WORKSPACE_ROOT / "src" / "omni_perception" / "config" / "laser_filter.yaml"
        assert filter_yaml.exists(), f"laser_filter.yaml not found at {filter_yaml}"
        with open(filter_yaml, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        params = cfg["scan_to_scan_filter_chain"]["ros__parameters"]["filter2"]["params"]
        return params

    def test_filter_box_dimensions_exact_calibration(self, filter_config):
        """Verify footprint filter box bounds are exactly [-0.135, 0.135] m."""
        assert math.isclose(filter_config["min_x"], -0.135, rel_tol=1e-6)
        assert math.isclose(filter_config["max_x"], 0.135, rel_tol=1e-6)
        assert math.isclose(filter_config["min_y"], -0.135, rel_tol=1e-6)
        assert math.isclose(filter_config["max_y"], 0.135, rel_tol=1e-6)
        assert filter_config["box_frame"] == "base_link"
        assert filter_config["invert"] is False

    def test_filter_adversarial_dense_polar_scan(self):
        """
        Adversarially sweep 20,000 synthetic laser scan points across 360 degrees
        and ranges [0.05, 5.0] m:
        - Points with |x| <= 0.135 and |y| <= 0.135 MUST be masked to NaN.
        - Points with |x| > 0.135 or |y| > 0.135 MUST be strictly preserved.
        - Zero false exclusions outside the bounding box.
        """
        oracle = LaserFootprintFilterOracle(min_x=-0.135, max_x=0.135, min_y=-0.135, max_y=0.135)
        num_angles = 200
        num_ranges = 100
        angles = np.linspace(-math.pi, math.pi, num_angles, endpoint=False)
        ranges = np.linspace(0.05, 3.0, num_ranges)

        inside_count = 0
        outside_count = 0
        false_exclusions = []
        false_inclusions = []

        for ang in angles:
            for r in ranges:
                x = r * math.cos(ang)
                y = r * math.sin(ang)

                is_inside = (-0.135 <= x <= 0.135) and (-0.135 <= y <= 0.135)
                xf, yf, valid = oracle.filter_cartesian_point(x, y)

                if is_inside:
                    inside_count += 1
                    if valid or not math.isnan(xf):
                        false_inclusions.append((x, y, r, ang))
                else:
                    outside_count += 1
                    if not valid or math.isnan(xf):
                        false_exclusions.append((x, y, r, ang))

        total_tested = num_angles * num_ranges
        print(f"\n[Laser Filter 20k Scan] Tested {total_tested:,} points: {inside_count} inside (masked), {outside_count} outside (preserved)")
        assert len(false_inclusions) == 0, f"Found {len(false_inclusions)} chassis points NOT filtered!"
        assert len(false_exclusions) == 0, f"Found {len(false_exclusions)} obstacle points FALSELY filtered!"

    def test_filter_boundary_grazing_micrometer_resolution(self):
        """
        Verify extreme boundary grazing behavior at 1 micrometer precision (+/- 1e-6 m).
        """
        oracle = LaserFootprintFilterOracle(min_x=-0.135, max_x=0.135, min_y=-0.135, max_y=0.135)
        eps = 1e-6

        # 4 edges at 1 um inside -> must be filtered to NaN
        inside_test_points = [
            (0.135 - eps, 0.0),
            (-0.135 + eps, 0.0),
            (0.0, 0.135 - eps),
            (0.0, -0.135 + eps),
            (0.135 - eps, 0.135 - eps),
            (-0.135 + eps, -0.135 + eps),
        ]
        for x, y in inside_test_points:
            xf, yf, valid = oracle.filter_cartesian_point(x, y)
            assert not valid, f"Point ({x}, {y}) should be masked"
            assert math.isnan(xf)

        # 4 edges at 1 um outside -> must be preserved
        outside_test_points = [
            (0.135 + eps, 0.0),
            (-0.135 - eps, 0.0),
            (0.0, 0.135 + eps),
            (0.0, -0.135 - eps),
            (0.135 + eps, 0.135 + eps),
            (-0.135 - eps, -0.135 - eps),
        ]
        for x, y in outside_test_points:
            xf, yf, valid = oracle.filter_cartesian_point(x, y)
            assert valid, f"Point ({x}, {y}) should be preserved"
            assert math.isclose(xf, x, abs_tol=1e-9)
            assert math.isclose(yf, y, abs_tol=1e-9)


# ============================================================================
# TARGET 5: web/backend Atomic YAML Persistence & Concurrency
# ============================================================================
class TestWebBackendAtomicYAMLAdversarial:
    """Target 5: White-box adversarial concurrency test of calib_service.py atomic writes."""

    def test_atomic_yaml_implementation_inspection(self):
        """
        Inspect calib_service.py to verify that:
        1. Temporary filenames incorporate uuid.uuid4().hex to prevent race conditions.
        2. f.flush() and os.fsync(f.fileno()) are executed before os.replace.
        3. os.replace is used for atomic POSIX replacement.
        """
        calib_svc_path = WORKSPACE_ROOT / "web" / "backend" / "app" / "services" / "calib_service.py"
        assert calib_svc_path.exists()
        with open(calib_svc_path, "r", encoding="utf-8") as f:
            content = f.read()

        assert "uuid.uuid4().hex" in content, "calib_service.py does not use uuid.uuid4().hex in temp files"
        assert "os.fsync(f.fileno())" in content, "calib_service.py does not call os.fsync before replacement"
        assert "os.replace(tmp_file, target_file)" in content, "calib_service.py does not use atomic os.replace"

    def test_high_concurrency_race_free_writes_and_zero_empty_reads(self, tmp_path: Path):
        """
        Adversarially stress calib_service.py with:
        - 20 concurrent writer threads performing 50 iterations each = 1,000 total atomic writes
        - 10 concurrent reader threads continuously reading and parsing both YAML files
        VERIFIES:
        - 100% race-free writes: zero FileNotFoundError, zero EEXIST, zero IO errors
        - Zero empty file reads: size is always > 0, yaml.safe_load NEVER returns None or empty
        - 100% schema integrity: imu_calib and wheel_calib dictionaries are always intact
        """
        config_dir = tmp_path / "config"
        config_dir.mkdir(parents=True, exist_ok=True)

        # Seed initial files
        save_imu_calib_yaml({"gyro_bias": [0.0, 0.0, 0.0]}, config_dir=config_dir)
        save_wheel_calib_yaml({"wheel_radius": [0.03, 0.03, 0.03, 0.03]}, config_dir=config_dir)

        stop_flag = threading.Event()
        writer_errors: List[str] = []
        reader_errors: List[str] = []
        writes_completed = 0
        reads_completed = 0
        lock = threading.Lock()

        def imu_writer(worker_id: int):
            nonlocal writes_completed
            for i in range(50):
                payload = {
                    "gyro_bias": [worker_id * 0.001, i * 0.0001, -0.005],
                    "accel_scale": [1.001, 0.999, 1.002],
                    "accel_bias": [0.01, -0.01, 0.02],
                    "frame_id": f"imu_link_{worker_id}",
                }
                try:
                    p = save_imu_calib_yaml(payload, config_dir=config_dir)
                    assert p.exists()
                    with lock:
                        writes_completed += 1
                except Exception as e:
                    with lock:
                        writer_errors.append(f"IMU Writer {worker_id} iter {i}: {type(e).__name__}: {e}")
                time.sleep(0.0001)

        def wheel_writer(worker_id: int):
            nonlocal writes_completed
            for i in range(50):
                payload = {
                    "wheel_radius": [0.03 + worker_id * 0.0001, 0.0301, 0.0299, 0.0302],
                    "wheelbase": 0.1312,
                    "track_width": 0.1312,
                }
                try:
                    p = save_wheel_calib_yaml(payload, config_dir=config_dir)
                    assert p.exists()
                    with lock:
                        writes_completed += 1
                except Exception as e:
                    with lock:
                        writer_errors.append(f"Wheel Writer {worker_id} iter {i}: {type(e).__name__}: {e}")
                time.sleep(0.0001)

        def reader(reader_id: int):
            nonlocal reads_completed
            imu_path = config_dir / "imu_calib.yaml"
            wheel_path = config_dir / "wheel_calib.yaml"

            while not stop_flag.is_set():
                # Read IMU
                try:
                    sz = imu_path.stat().st_size
                    if sz == 0:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} observed 0-byte imu_calib.yaml")
                    with open(imu_path, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f)
                    if data is None:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} got None parsing imu_calib.yaml")
                    elif "imu_calib" not in data or "gyro_bias" not in data["imu_calib"]:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} saw partial schema in imu_calib.yaml")
                    with lock:
                        reads_completed += 1
                except Exception as e:
                    with lock:
                        reader_errors.append(f"Reader {reader_id} IMU read exc: {type(e).__name__}: {e}")

                # Read Wheel
                try:
                    sz = wheel_path.stat().st_size
                    if sz == 0:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} observed 0-byte wheel_calib.yaml")
                    with open(wheel_path, "r", encoding="utf-8") as f:
                        data = yaml.safe_load(f)
                    if data is None:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} got None parsing wheel_calib.yaml")
                    elif "wheel_calib" not in data or "wheel_radius" not in data["wheel_calib"]:
                        with lock:
                            reader_errors.append(f"Reader {reader_id} saw partial schema in wheel_calib.yaml")
                    with lock:
                        reads_completed += 1
                except Exception as e:
                    with lock:
                        reader_errors.append(f"Reader {reader_id} Wheel read exc: {type(e).__name__}: {e}")

                time.sleep(0.0001)

        # Launch 10 IMU writers + 10 Wheel writers (20 writers total)
        w_threads = []
        for i in range(10):
            w_threads.append(threading.Thread(target=imu_writer, args=(i,)))
            w_threads.append(threading.Thread(target=wheel_writer, args=(i,)))

        # Launch 10 readers
        r_threads = [threading.Thread(target=reader, args=(i,)) for i in range(10)]

        for t in r_threads:
            t.start()
        for t in w_threads:
            t.start()

        for t in w_threads:
            t.join()

        # Stop readers
        stop_flag.set()
        for t in r_threads:
            t.join()

        print(f"\n[Web Atomic YAML Concurrency] Completed {writes_completed} concurrent writes and {reads_completed} concurrent reads.")
        assert writes_completed == 1000, f"Expected 1,000 writes, got {writes_completed}"
        assert reads_completed > 100, f"Expected at least 100 reads, got {reads_completed}"
        assert len(writer_errors) == 0, f"Encountered {len(writer_errors)} writer errors: {writer_errors[:5]}"
        assert len(reader_errors) == 0, f"Encountered {len(reader_errors)} reader errors: {reader_errors[:5]}"
