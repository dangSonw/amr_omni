import math
import pytest
from app.services.grid_planner import GridMapPlanner, OmniMpcController


def test_grid_map_planner_costmap_inflation():
    planner = GridMapPlanner(cell_size=0.12, inflation_radius=0.55, inscribed_radius=0.24)
    # Add an obstacle cell at (1.0, 1.0)
    ox, oy = planner.world_to_grid(1.0, 1.0)
    planner.occupied_cells.add((ox, oy))
    planner._rebuild_inflated_cells()

    assert (ox, oy) in planner.occupied_cells
    assert planner.get_cell_cost(ox, oy) == 254

    # Check cell within inscribed radius (e.g. 1 cell away = 0.12m < 0.24m)
    inscribed_cell = (ox + 1, oy)
    assert inscribed_cell in planner.inscribed_cells
    assert planner.get_cell_cost(inscribed_cell[0], inscribed_cell[1]) == 253
    assert not planner.is_free(inscribed_cell[0], inscribed_cell[1])

    # Check cell in inflation gradient (e.g. 3 cells away = 0.36m, between 0.24 and 0.55)
    grad_cell = (ox + 3, oy)
    cost = planner.get_cell_cost(grad_cell[0], grad_cell[1])
    assert 1 <= cost < 253
    assert planner.is_free(grad_cell[0], grad_cell[1])  # Traversable but penalized


def test_cost_weighted_a_star_wide_detour():
    planner = GridMapPlanner(cell_size=0.12, inflation_radius=0.55, inscribed_radius=0.24)
    # Obstacle blocking direct path between (0.0, 0.0) and (2.0, 2.0)
    # Place obstacle at (1.0, 1.0)
    ox, oy = planner.world_to_grid(1.0, 1.0)
    planner.occupied_cells.add((ox, oy))
    planner._rebuild_inflated_cells()

    path = planner.plan_path(0.0, 0.0, 2.0, 2.0)
    assert len(path) >= 2

    # Verify that the path maintains safe clearance from the obstacle center
    min_dist_to_obstacle = min(math.hypot(p[0] - 1.0, p[1] - 1.0) for p in path)
    assert min_dist_to_obstacle >= 0.24, f"Path came too close to obstacle: {min_dist_to_obstacle}m"


def test_a_star_escape_from_inscribed_zone():
    planner = GridMapPlanner(cell_size=0.12, inflation_radius=0.55, inscribed_radius=0.24)
    ox, oy = planner.world_to_grid(1.0, 1.0)
    planner.occupied_cells.add((ox, oy))
    planner._rebuild_inflated_cells()

    # Robot starts inside the inscribed zone (e.g. 0.15m from obstacle)
    start_x, start_y = 1.0 + 0.15, 1.0
    path = planner.plan_path(start_x, start_y, 2.5, 2.5)

    assert len(path) >= 2, "A* must not fail when starting in inscribed zone"
    assert path[0] == [round(start_x, 3), round(start_y, 3)]
    assert path[-1] == [2.5, 2.5]


def test_omni_mpc_controller_lateral_strafe_evasion():
    controller = OmniMpcController(max_vx=0.35, max_vy=0.35, max_wz=1.5, robot_radius=0.22)

    # Detour path curving around an obstacle in front
    target_path = [
        [0.0, 0.0],
        [0.1, 0.3],
        [0.3, 0.6],
        [1.0, 1.5],
    ]

    # LiDAR detects obstacle 0.40m ahead
    lidar_points = []
    for deg in range(-20, 20, 4):
        r = 0.40
        th = math.radians(deg)
        lidar_points.append([r * math.cos(th), r * math.sin(th)])

    vx, vy, wz, dist, traj = controller.compute(
        current_x=0.0,
        current_y=0.0,
        current_theta=0.0,
        target_path=target_path,
        goal_x=1.0,
        goal_y=1.5,
        lidar_points=lidar_points,
    )

    # Robot must NOT drive straight into obstacle, and must NOT oscillate in full reverse
    assert vx >= 0.0, f"Robot should not back up unnecessarily: vx={vx}"
    # Holonomic lateral velocity should be active to strafe around obstacle
    assert abs(vy) > 0.05 or wz > 0.1, f"Robot should maneuver laterally: vy={vy}, wz={wz}"


def test_omni_mpc_controller_terminal_arrival():
    controller = OmniMpcController(robot_radius=0.22)
    target_path = [[0.95, 1.0], [1.0, 1.0]]

    vx, vy, wz, dist, traj = controller.compute(
        current_x=0.98,
        current_y=1.0,
        current_theta=0.0,
        target_path=target_path,
        goal_x=1.0,
        goal_y=1.0,
        lidar_points=None,
    )

    # Within 0.05m of goal -> stops completely
    assert dist < 0.05
    assert vx == 0.0
    assert vy == 0.0
    assert wz == 0.0


def test_grid_planner_scan_matcher():
    planner = GridMapPlanner(cell_size=0.12)
    # Pre-populate square room wall
    square_scan = []
    for a in range(0, 360, 2):
        th = math.radians(a)
        d = 3.0 / max(abs(math.cos(th)), abs(math.sin(th)))
        square_scan.append([d * math.cos(th), d * math.sin(th)])

    # Add initial scans to confirm walls
    for _ in range(2):
        planner.add_scan(0.0, 0.0, 0.0, square_scan)

    assert len(planner.occupied_cells) > 30

    # Test scan matching when input pose has drift (+6cm, +4cm, +0.02 rad)
    drifted_x = 0.06
    drifted_y = 0.04
    drifted_th = 0.02
    corr_x, corr_y, corr_th = planner.match_scan(drifted_x, drifted_y, drifted_th, square_scan)

    # CSM must correct the drift back towards (0.0, 0.0, 0.0)
    assert abs(corr_x) < abs(drifted_x) or math.isclose(corr_x, 0.0, abs_tol=0.04)
    assert abs(corr_y) < abs(drifted_y) or math.isclose(corr_y, 0.0, abs_tol=0.04)
    assert abs(corr_th) < abs(drifted_th) or math.isclose(corr_th, 0.0, abs_tol=0.02)


def test_grid_planner_probabilistic_outlier_filtering():
    planner = GridMapPlanner(cell_size=0.12)

    # An isolated single outlier point (no contiguous neighbor)
    single_noise = [[2.0, 2.0]]
    new_cells = planner.add_scan(0.0, 0.0, 0.0, single_noise)
    assert new_cells == 0, "Single isolated outlier ray must not be confirmed immediately"
    assert len(planner.occupied_cells) == 0

    # A contiguous surface (wall)
    wall = [[2.0, y * 0.1] for y in range(-5, 6)]
    new_wall_cells = planner.add_scan(0.0, 0.0, 0.0, wall)
    assert new_wall_cells > 0, "Contiguous surface must be confirmed as obstacles"
    assert len(planner.occupied_cells) > 0


