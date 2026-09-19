"""
Tier 2 Boundary Cases: Laser Footprint Mask Boundaries and Grazing Rays.
"""
import math
import pytest

from tests.e2e.harness.laser_filter_oracle import LaserFootprintFilterOracle


@pytest.mark.tier2
class TestBoundaryLaserGrazing:
    """Boundary test cases for laser footprint mask edge behavior."""

    def test_boundary_laser_exact_box_edge(self):
        """Verify behavior on exact bounding box edges (x = +/- 0.135000)."""
        oracle = LaserFootprintFilterOracle()
        # Exact edge points are inside the closed box [min, max]
        assert oracle.is_point_inside(0.135000, 0.0)
        assert oracle.is_point_inside(-0.135000, 0.0)
        assert oracle.is_point_inside(0.0, 0.135000)
        assert oracle.is_point_inside(0.0, -0.135000)

    def test_boundary_laser_epsilon_inside_filtered(self):
        """Verify point 1 micrometer inside box is filtered to NaN."""
        oracle = LaserFootprintFilterOracle()
        eps_in_x = 0.135 - 1e-6
        x_f, y_f, valid = oracle.filter_cartesian_point(eps_in_x, 0.0)
        assert not valid
        assert math.isnan(x_f)

    def test_boundary_laser_epsilon_outside_preserved(self):
        """Verify point 1 micrometer outside box is preserved."""
        oracle = LaserFootprintFilterOracle()
        eps_out_x = 0.135 + 1e-6
        x_f, y_f, valid = oracle.filter_cartesian_point(eps_out_x, 0.0)
        assert valid
        assert math.isclose(x_f, eps_out_x, rel_tol=1e-6)

    def test_boundary_laser_corner_points(self):
        """Verify 4 exact corner points of chassis footprint box."""
        oracle = LaserFootprintFilterOracle()
        corners = [
            (0.135, 0.135),
            (-0.135, 0.135),
            (-0.135, -0.135),
            (0.135, -0.135),
        ]
        for cx, cy in corners:
            assert oracle.is_point_inside(cx, cy)
            _, _, valid = oracle.filter_cartesian_point(cx, cy)
            assert not valid

    def test_boundary_laser_grazing_tangential_scan(self):
        """Verify rays skimming past the corner at 45.1 degrees are preserved."""
        oracle = LaserFootprintFilterOracle()
        # Corner distance = sqrt(0.135^2 + 0.135^2) = ~0.1909 m
        # A point along 45 deg at distance 0.20 m is outside the corner
        angle = math.pi / 4.0
        r_outside = 0.20
        ranges = [r_outside]
        filtered = oracle.filter_scan_ranges(ranges, angle_min=angle, angle_increment=0.0)
        assert math.isclose(filtered[0], r_outside, rel_tol=1e-5)
