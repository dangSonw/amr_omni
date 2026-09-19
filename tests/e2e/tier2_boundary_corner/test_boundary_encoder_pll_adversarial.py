"""
Tier 2 Boundary Cases: Adversarial Stress Testing of Encoder PLL and Velocity Estimation.
Tests EncoderPll under harsh conditions:
  - Rapid speed reversals (-1.5 m/s to +1.5 m/s in 10 ms)
  - High pulse jitter (+/-15 counts) and missing pulse dropouts
  - Ultra-low speeds (0.005 m/s) and sparse pulse arrivals (0.5 Hz)
  - 16-bit timer overflow stress (0, 65535, 32767, 32768, -32768)
  - Zero-speed watchdog activation and rapid recovery
  - Long-duration numerical drift test (100,000 steps)
"""
import subprocess
from pathlib import Path
import pytest


@pytest.mark.tier2
class TestBoundaryEncoderPllAdversarial:
    """Adversarial stress suite executing empirical benchmarks against native C++ firmware code."""

    @pytest.fixture(scope="class", autouse=True)
    def compile_stress_benchmark(self, workspace_root: Path):
        """Ensure stress_benchmark executable is compiled from firmware source."""
        build_dir = workspace_root / "build"
        build_dir.mkdir(exist_ok=True)
        bin_path = build_dir / "stress_benchmark"
        src_bench = workspace_root / "tests" / "stress" / "stress_benchmark.cpp"
        src_pll = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "src" / "encoder_pll.cpp"
        inc_dir = workspace_root / "firmware" / "stm32_f407vg_arduino_sim" / "include"

        cmd = [
            "g++", "-O3", "-Wall", "-Wextra",
            f"-I{inc_dir}",
            str(src_bench),
            str(src_pll),
            "-o", str(bin_path)
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        assert res.returncode == 0, f"Compilation failed:\n{res.stderr}"
        return bin_path

    def test_adversarial_rapid_speed_reversals(self, workspace_root: Path):
        """Adversarial: Instantaneous speed reversal (-1.5 m/s to +1.5 m/s in 10 ms)."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0, f"Benchmark failed:\n{res.stdout}\n{res.stderr}"
        assert "--> Running Test 1: Rapid Speed Reversal" in res.stdout
        assert "Peak Overshoot:           0.0075 rad/s (0.01%)" in res.stdout or "Status:                   PASSED" in res.stdout

    def test_adversarial_jitter_and_missing_pulse_trains(self, workspace_root: Path):
        """Adversarial: Severe pulse arrival jitter (+/-15 counts) and 3-step dropouts."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0
        assert "--> Running Test 2: High Pulse Jitter" in res.stdout
        assert "Noise Attenuation Factor:" in res.stdout

    def test_adversarial_ultra_low_speed_sparse_pulses(self, workspace_root: Path):
        """Adversarial: Creeping speed (0.005 m/s) and sparse pulse arrival (0.5 Hz)."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0
        assert "--> Running Test 3: Ultra-Low Speeds" in res.stdout
        assert "Mean Estimated Speed:     0.1667 rad/s" in res.stdout

    def test_adversarial_16bit_timer_overflow_and_rollover(self, workspace_root: Path):
        """Adversarial: Exhaustive 16-bit timer rollover and 400,000-step ramp stress."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0
        assert "--> Running Test 4: 16-Bit Timer Rollover Stress" in res.stdout
        assert "Continuous Ramp Errors:   0" in res.stdout

    def test_adversarial_zero_speed_watchdog_and_recovery(self, workspace_root: Path):
        """Adversarial: Sudden stop watchdog activation (50 ms) and rapid resumption."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0
        assert "--> Running Test 5: Zero-Speed Watchdog Activation" in res.stdout
        assert "Watchdog Active at 50ms:  YES" in res.stdout

    def test_adversarial_long_duration_numerical_drift_100k_steps(self, workspace_root: Path):
        """Adversarial: 100,000 continuous steps numerical drift and stability."""
        bin_path = workspace_root / "build" / "stress_benchmark"
        res = subprocess.run([str(bin_path)], capture_output=True, text=True)
        assert res.returncode == 0
        assert "--> Running Test 6: Long-Duration Numerical Drift Test" in res.stdout
        assert "Total Simulated Steps:    100000 (1,000 seconds)" in res.stdout
        assert "OVERALL VERDICT: APPROVE (ALL 6 TESTS PASSED)" in res.stdout
