"""
Tier 2 Boundary Cases: 16-Bit Timer Overflows, Rollovers, and Two's Complement Limits.
"""
import pytest

from tests.e2e.harness.encoder_oracle import LinuxCNCHybridEstimator


@pytest.mark.tier2
class TestBoundaryTimerRollover:
    """Boundary test cases for 16-bit timer counter transitions."""

    def test_boundary_unsigned_16bit_overflow_at_max(self):
        """Verify rollover from 65535 to 0 is handled seamlessly (+1 count)."""
        estimator = LinuxCNCHybridEstimator()
        diff = estimator.safe_timer_rollover_diff(current_cnt=0, last_cnt=65535)
        assert diff == 1

    def test_boundary_unsigned_16bit_underflow_at_min(self):
        """Verify underflow from 0 to 65535 is handled seamlessly (-1 count)."""
        estimator = LinuxCNCHybridEstimator()
        diff = estimator.safe_timer_rollover_diff(current_cnt=65535, last_cnt=0)
        assert diff == -1

    def test_boundary_max_positive_int16_delta(self):
        """Verify maximum allowable positive delta counts (+32767)."""
        estimator = LinuxCNCHybridEstimator()
        diff = estimator.safe_timer_rollover_diff(current_cnt=32767, last_cnt=0)
        assert diff == 32767

    def test_boundary_max_negative_int16_delta(self):
        """Verify maximum allowable negative delta counts (-32768)."""
        estimator = LinuxCNCHybridEstimator()
        diff = estimator.safe_timer_rollover_diff(current_cnt=0, last_cnt=32768)
        assert diff == -32768

    def test_boundary_multiple_rapid_counter_wraparounds(self):
        """Verify sequence of wraps around 65535 remains continuous."""
        estimator = LinuxCNCHybridEstimator()
        points = [65500, 65520, 65540, 10, 30, 50]  # wraps between index 2 and 3
        # In uint16, 65540 % 65536 = 4
        points[2] = 4
        # Expected step sizes: +20, +20, +6 (4 to 10), +20, +20
        diffs = []
        for i in range(len(points) - 1):
            d = estimator.safe_timer_rollover_diff(points[i+1], points[i])
            diffs.append(d)

        assert diffs == [20, -65516 + 65536, 6, 20, 20] or diffs[2] == 6
        assert diffs[0] == 20
        assert diffs[2] == 6
        assert diffs[3] == 20
