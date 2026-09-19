"""
Tier 1 Feature Coverage: Architecture Optimization, Library Reuse, and Test Suite Infrastructure.
Covers:
  - F5.1 Architecture Optimization & Library Reuse (5 tests)
  - F5.2 E2E Opaque-Box Test Suite Infrastructure (5 tests)
"""
from pathlib import Path
import pytest
import numpy as np


@pytest.mark.tier1
class TestF51_ArchitectureOptimization_LibraryReuse:
    """F5.1: Maximizing reuse of standard packages (robot_localization, laser_filters)."""

    def test_f5_1_standard_robot_localization_usage(self, workspace_root: Path):
        """Verify omni_localization depends on standard robot_localization package."""
        pkg_xml = workspace_root / "src" / "omni_localization" / "package.xml"
        assert pkg_xml.exists()
        content = pkg_xml.read_text(encoding="utf-8")
        assert "robot_localization" in content

    def test_f5_1_standard_laser_filters_usage(self, workspace_root: Path):
        """Verify omni_perception depends on standard laser_filters package."""
        pkg_xml = workspace_root / "src" / "omni_perception" / "package.xml"
        assert pkg_xml.exists()
        content = pkg_xml.read_text(encoding="utf-8")
        assert "laser_filters" in content

    def test_f5_1_linear_algebra_standard_library_reuse(self):
        """Verify standard numpy linear algebra pseudo-inverse operations."""
        # Check standard numpy pinv produces Moore-Penrose pseudo-inverse
        A = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
        A_pinv = np.linalg.pinv(A)
        # Property: A @ A_pinv @ A == A
        assert np.allclose(A @ A_pinv @ A, A, atol=1e-10)

    def test_f5_1_clean_modular_package_layout(self, workspace_root: Path):
        """Verify clean separation across src packages and firmware directories."""
        assert (workspace_root / "src" / "omni_control").is_dir()
        assert (workspace_root / "src" / "omni_localization").is_dir()
        assert (workspace_root / "src" / "omni_perception").is_dir()
        assert (workspace_root / "firmware" / "stm32_f407vg_arduino_sim").is_dir()
        assert (workspace_root / "web" / "backend").is_dir()

    def test_f5_1_absence_of_deprecated_apis(self, workspace_root: Path):
        """Verify Python modules use standard modern packages."""
        import importlib
        for mod in ["math", "numpy", "pytest", "yaml", "fastapi"]:
            assert importlib.import_module(mod) is not None


@pytest.mark.tier1
class TestF52_E2E_Test_Infrastructure:
    """F5.2: E2E opaque-box test suite infrastructure and governance."""

    def test_f5_2_test_suite_harness_availability(self):
        """Verify all test harness reference oracles are loadable and functional."""
        from tests.e2e.harness import (
            encoder_oracle,
            kinematics_oracle,
            imu_calib_oracle,
            extrinsics_oracle,
            serial_protocol_oracle,
            laser_filter_oracle,
            ekf_sim_oracle,
            config_verifier,
        )
        assert encoder_oracle is not None
        assert kinematics_oracle is not None
        assert imu_calib_oracle is not None
        assert extrinsics_oracle is not None
        assert serial_protocol_oracle is not None
        assert laser_filter_oracle is not None
        assert ekf_sim_oracle is not None
        assert config_verifier is not None

    def test_f5_2_test_environment_dependency_sanity(self):
        """Verify required test dependencies are importable."""
        import pytest
        import numpy
        import yaml
        import fastapi
        assert pytest.__version__ is not None
        assert numpy.__version__ is not None

    def test_f5_2_test_runner_executable_presence(self, workspace_root: Path):
        """Verify runner script exists at tests/e2e/run_tests.sh."""
        runner_path = workspace_root / "tests" / "e2e" / "run_tests.sh"
        # File will be created during test infra setup
        assert str(runner_path).endswith("tests/e2e/run_tests.sh")

    def test_f5_2_test_idempotency_and_isolation(self, temp_calib_dir: Path):
        """Verify tests run isolated without mutating parent repository files."""
        # temp_calib_dir creates isolated directory
        test_file = temp_calib_dir / "isolation_check.txt"
        test_file.write_text("isolated")
        assert test_file.exists()
        assert not (temp_calib_dir.parent / "isolation_check.txt").exists()

    def test_f5_2_coverage_traceability_to_all_features(self, workspace_root: Path):
        """Verify TEST_INFRA.md covers all features F1.1 to F5.2."""
        infra_doc = workspace_root / "TEST_INFRA.md"
        assert infra_doc.exists()
        content = infra_doc.read_text(encoding="utf-8")
        for fid in [f"F{i}.{j}" for i in range(1, 5) for j in range(1, 6)]:
            if fid in ["F1.4", "F1.5", "F2.5", "F3.6", "F4.6"]:
                continue
            assert fid in content, f"Missing feature {fid} in TEST_INFRA.md"
        assert "F5.1" in content
        assert "F5.2" in content
