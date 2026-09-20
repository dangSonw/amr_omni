"""Mô-đun bản đồ lưới và lập lộ trình tự hành A* né vật cản thời gian thực.
Tích lũy chùm tia LiDAR để tạo bộ nhớ bản đồ (Persistent Occupancy Grid)
và sinh quỹ đạo không xuyên tường.
"""

from __future__ import annotations
import math
import heapq
import logging
from typing import List, Tuple, Set, Optional, Dict

logger = logging.getLogger(__name__)


class GridMapPlanner:
    def __init__(
        self,
        cell_size: float = 0.12,
        inflation_radius: float = 0.28,
        max_scan_range: float = 12.0,
        min_scan_range: float = 0.15,
        max_obstacle_memory: int = 15000,
    ):
        self.cell_size = cell_size
        self.inflation_radius = inflation_radius
        self.max_scan_range = max_scan_range
        self.min_scan_range = min_scan_range
        self.max_obstacle_memory = max_obstacle_memory

        # Tập hợp các ô có vật cản thực tế (world frame)
        self.occupied_cells: Set[Tuple[int, int]] = set()
        # Bộ đếm điểm tin cậy vật cản (Probabilistic occupancy hits)
        self.cell_hits: Dict[Tuple[int, int], int] = {}
        # Tập hợp các ô bị giãn nở (inflation) để bảo đảm bán kính an toàn cho thân robot
        self.inflated_cells: Set[Tuple[int, int]] = set()

        # Bán kính ô giãn nở (tính theo số ô)
        self.inflation_cell_radius = max(1, int(math.ceil(self.inflation_radius / self.cell_size)))

        # Bảng offset các ô nằm trong bán kính giãn nở
        self._inflation_offsets: List[Tuple[int, int]] = []
        r2 = (self.inflation_radius / self.cell_size) ** 2
        for dx in range(-self.inflation_cell_radius, self.inflation_cell_radius + 1):
            for dy in range(-self.inflation_cell_radius, self.inflation_cell_radius + 1):
                if dx * dx + dy * dy <= r2:
                    self._inflation_offsets.append((dx, dy))

    @staticmethod
    def _bresenham_cells(x0: int, y0: int, x1: int, y1: int):
        """Thuật toán đường thẳng Bresenham sinh các ô nằm giữa ray (không gồm điểm chạm cuối)."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        x, y = x0, y0
        while x != x1 or y != y1:
            yield (x, y)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        return int(round(x / self.cell_size)), int(round(y / self.cell_size))

    def grid_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return round(cx * self.cell_size, 3), round(cy * self.cell_size, 3)

    def add_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        lidar_points: List[List[float]],
    ) -> int:
        """Tích lũy các điểm LiDAR vào bộ nhớ bản đồ lưới toàn cục và dọn sạch bóng ma bằng ray clearing."""
        if not lidar_points:
            return 0

        rcx, rcy = self.world_to_grid(robot_x, robot_y)
        cos_th = math.cos(robot_theta)
        sin_th = math.sin(robot_theta)
        new_count = 0
        cleared_any = False
        has_existing = bool(self.cell_hits)

        # Nếu bộ nhớ quá lớn (quét bản đồ quá lâu), dọn bớt ô quá xa (> 18m)
        if len(self.occupied_cells) > self.max_obstacle_memory:
            keep_dist2 = (18.0 / self.cell_size) ** 2
            to_remove = [
                cell for cell in self.occupied_cells
                if (cell[0] - rcx) ** 2 + (cell[1] - rcy) ** 2 >= keep_dist2
            ]
            for cell in to_remove:
                self.occupied_cells.discard(cell)
                self.cell_hits.pop(cell, None)
            self._rebuild_inflated_cells()

        stride = 2 if len(lidar_points) > 180 else 1

        for idx in range(0, len(lidar_points), stride):
            pt = lidar_points[idx]
            lx, ly = pt[0], pt[1]
            dist = math.hypot(lx, ly)
            if dist < self.min_scan_range or dist > self.max_scan_range:
                continue

            # Chuyển đổi sang hệ tọa độ thế giới (World Frame)
            wx = robot_x + (cos_th * lx - sin_th * ly)
            wy = robot_y + (sin_th * lx + cos_th * ly)
            ocx, ocy = self.world_to_grid(wx, wy)

            # 1. Ray Clearing: Tia LiDAR nhìn xuyên qua khoảng trống làm sạch các ô bóng ma
            if has_existing:
                for fcx, fcy in self._bresenham_cells(rcx, rcy, ocx, ocy):
                    if (fcx, fcy) in self.cell_hits:
                        self.cell_hits[(fcx, fcy)] -= 1
                        if self.cell_hits[(fcx, fcy)] <= 0:
                            del self.cell_hits[(fcx, fcy)]
                            if (fcx, fcy) in self.occupied_cells:
                                self.occupied_cells.remove((fcx, fcy))
                                cleared_any = True

            # 2. Ghi nhận vật cản tại điểm chạm
            hits = self.cell_hits.get((ocx, ocy), 0)
            self.cell_hits[(ocx, ocy)] = min(10, hits + 2)
            if (ocx, ocy) not in self.occupied_cells:
                self.occupied_cells.add((ocx, ocy))
                new_count += 1
                for ox, oy in self._inflation_offsets:
                    self.inflated_cells.add((ocx + ox, ocy + oy))

        if cleared_any:
            self._rebuild_inflated_cells()

        return new_count

    def _rebuild_inflated_cells(self):
        self.inflated_cells.clear()
        for cell in self.occupied_cells:
            for ox, oy in self._inflation_offsets:
                self.inflated_cells.add((cell[0] + ox, cell[1] + oy))

    def clear(self):
        """Xóa toàn bộ bộ nhớ bản đồ."""
        self.occupied_cells.clear()
        self.cell_hits.clear()
        self.inflated_cells.clear()
        logger.info("Đã xóa bộ nhớ bản đồ lưới vật cản.")

    def get_obstacle_points(self, max_points: int = 1200) -> List[List[float]]:
        """Trả về danh sách tọa độ thế giới các vật cản để gửi lên Web hiển thị."""
        total = len(self.occupied_cells)
        if total == 0:
            return []
        step = max(1, total // max_points)
        pts = []
        for i, (cx, cy) in enumerate(self.occupied_cells):
            if i % step == 0:
                pts.append([round(cx * self.cell_size, 2), round(cy * self.cell_size, 2)])
        return pts

    def is_free(self, cx: int, cy: int) -> bool:
        return (cx, cy) not in self.inflated_cells

    def _find_nearest_free_cell(self, cx: int, cy: int, max_search_radius: int = 8) -> Tuple[int, int]:
        """Nếu điểm đích tình cờ rơi vào ô có tường hoặc vùng giãn nở, tìm ô an toàn gần nhất."""
        if self.is_free(cx, cy):
            return cx, cy

        best_cell = (cx, cy)
        best_dist = float("inf")
        for r in range(1, max_search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    cand = (cx + dx, cy + dy)
                    if self.is_free(cand[0], cand[1]):
                        d2 = dx * dx + dy * dy
                        if d2 < best_dist:
                            best_dist = d2
                            best_cell = cand
            if best_dist < float("inf"):
                break
        return best_cell

    def _line_of_sight(self, c0: Tuple[int, int], c1: Tuple[int, int]) -> bool:
        """Kiểm tra xem giữa 2 ô có đường ngắm thẳng không bị che khuất bởi vật cản (Bresenham raycasting)."""
        x0, y0 = c0
        x1, y1 = c1
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            if not self.is_free(x, y):
                return False
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return True

    def _prune_path(self, grid_path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Rút gọn đường đi A* bằng line-of-sight để có các góc rẽ mượt mà, không zíc-zắc."""
        if len(grid_path) <= 2:
            return grid_path

        pruned = [grid_path[0]]
        current_idx = 0
        n = len(grid_path)

        while current_idx < n - 1:
            next_idx = current_idx + 1
            # Thử nối trực tiếp tới điểm xa nhất có thể mà không đụng vật cản
            for test_idx in range(n - 1, current_idx, -1):
                if self._line_of_sight(grid_path[current_idx], grid_path[test_idx]):
                    next_idx = test_idx
                    break
            pruned.append(grid_path[next_idx])
            current_idx = next_idx

        return pruned

    def plan_path(
        self,
        start_x: float,
        start_y: float,
        goal_x: float,
        goal_y: float,
        max_expansions: int = 4000,
    ) -> List[List[float]]:
        """Lập lộ trình di chuyển A* né toàn bộ tường và vật cản đã lưu trong bộ nhớ bản đồ."""
        start_cell = self.world_to_grid(start_x, start_y)
        goal_cell = self.world_to_grid(goal_x, goal_y)

        # Nếu chưa có vật cản nào được quét hoặc robot đã rất gần đích
        if not self.occupied_cells:
            # Trả về đường nối thẳng phân đoạn
            steps = 15
            return [
                [
                    round(start_x + (goal_x - start_x) * (i / steps), 3),
                    round(start_y + (goal_y - start_y) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        # Kiểm tra nếu đường thẳng từ Start tới Goal hoàn toàn không có vật cản thì đi thẳng
        if self._line_of_sight(start_cell, goal_cell):
            steps = 15
            return [
                [
                    round(start_x + (goal_x - start_x) * (i / steps), 3),
                    round(start_y + (goal_y - start_y) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        # Nếu đích rơi vào tường, dời nhẹ sang ô trống gần nhất
        effective_goal = self._find_nearest_free_cell(goal_cell[0], goal_cell[1])

        # 8 hướng di chuyển liền kề và chi phí tương ứng
        neighbors = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

        def heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
            return math.hypot(a[0] - b[0], a[1] - b[1])

        # Hàng đợi ưu tiên A*: (f_score, g_score, cell)
        open_set: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (heuristic(start_cell, effective_goal), 0.0, start_cell))

        came_from = {}
        g_scores = {start_cell: 0.0}
        expansions = 0
        found = False

        while open_set and expansions < max_expansions:
            expansions += 1
            f, current_g, current = heapq.heappop(open_set)

            if current == effective_goal or heuristic(current, effective_goal) <= 1.0:
                came_from[effective_goal] = current if current != effective_goal else came_from.get(current, current)
                found = True
                break

            if current_g > g_scores.get(current, float("inf")):
                continue

            for dx, dy, cost in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                # Ô hàng xóm không được nằm trong vùng vật cản
                if not self.is_free(neighbor[0], neighbor[1]):
                    continue

                # Với bước chéo, kiểm tra tránh cắt góc xuyên tường
                if dx != 0 and dy != 0:
                    if not self.is_free(current[0] + dx, current[1]) or not self.is_free(current[0], current[1] + dy):
                        continue

                tentative_g = current_g + cost
                if tentative_g < g_scores.get(neighbor, float("inf")):
                    g_scores[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, effective_goal) * 1.1  # Weighted A*
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        if not found:
            # Fallback nếu bị cô lập: trả về đường nối thẳng phân đoạn
            logger.warning(f"A* không tìm được đường qua vật cản sau {expansions} lần duyệt, fallback đi thẳng.")
            steps = 15
            return [
                [
                    round(start_x + (goal_x - start_x) * (i / steps), 3),
                    round(start_y + (goal_y - start_y) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        # Tái tạo đường đi từ đích về nguồn
        curr = effective_goal
        raw_path = [curr]
        while curr in came_from and curr != start_cell:
            curr = came_from[curr]
            raw_path.append(curr)
        raw_path.reverse()

        # Rút gọn các bước dư thừa
        pruned_path = self._prune_path(raw_path)

        # Chuyển đổi tọa độ ô thành tọa độ thế giới (mét)
        result: List[List[float]] = [[round(start_x, 3), round(start_y, 3)]]
        for cell in pruned_path[1:-1]:
            wx, wy = self.grid_to_world(cell[0], cell[1])
            result.append([wx, wy])
        result.append([round(goal_x, 3), round(goal_y, 3)])

        # Làm mịn thêm: nội suy thêm các điểm giữa các waypoint nếu khoảng cách xa (> 0.5m)
        dense_result: List[List[float]] = []
        for i in range(len(result) - 1):
            p0 = result[i]
            p1 = result[i + 1]
            seg_dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            sub_steps = max(1, int(math.ceil(seg_dist / 0.35)))
            for s in range(sub_steps):
                t = s / sub_steps
                dense_result.append([
                    round(p0[0] + (p1[0] - p0[0]) * t, 3),
                    round(p0[1] + (p1[1] - p0[1]) * t, 3),
                ])
        dense_result.append([round(goal_x, 3), round(goal_y, 3)])

        logger.info(
            f"A* quy hoạch thành công: {len(dense_result)} điểm, né tránh {len(self.occupied_cells)} vật cản đã lưu."
        )
        return dense_result


class OmniMpcController:
    """Holonomic Model Predictive Controller (Omni MPC) for Mecanum Robot.

    Predicts state rollouts over horizon N using the kinematics:
        x_{k+1} = x_k + (vx * cos(th) - vy * sin(th)) * dt
        y_{k+1} = y_k + (vx * sin(th) + vy * cos(th)) * dt
        th_{k+1} = th_k + wz * dt

    Optimizes trajectory cost:
        - Reference path tracking error
        - Heading alignment
        - Obstacle repulsion penalty from 2D LiDAR points
        - Control effort & acceleration smoothness penalty
        - S-curve terminal deceleration in deadband
    """

    def __init__(
        self,
        horizon_steps: int = 10,
        dt: float = 0.08,
        max_vx: float = 0.40,
        max_vy: float = 0.35,
        max_wz: float = 2.0,
        robot_radius: float = 0.22,
    ):
        self.N = horizon_steps
        self.dt = dt
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz
        self.robot_radius = robot_radius
        self.last_cmd = [0.0, 0.0, 0.0]

    def reset(self):
        self.last_cmd = [0.0, 0.0, 0.0]

    def compute(
        self,
        current_x: float,
        current_y: float,
        current_theta: float,
        target_path: List[List[float]],
        goal_x: float,
        goal_y: float,
        lidar_points: Optional[List[List[float]]] = None,
    ) -> Tuple[float, float, float, float, List[List[float]]]:
        """Compute optimal (vx, vy, wz), distance to goal, and predicted trajectory points."""
        dist_to_final = math.hypot(goal_x - current_x, goal_y - current_y)

        # 1. Terminal Arrival Deadband (< 0.18m)
        if dist_to_final < 0.18:
            dx = goal_x - current_x
            dy = goal_y - current_y
            cos_th = math.cos(current_theta)
            sin_th = math.sin(current_theta)
            rx = cos_th * dx + sin_th * dy
            ry = -sin_th * dx + cos_th * dy
            r_norm = math.hypot(rx, ry)

            if dist_to_final < 0.04 or r_norm < 1e-3:
                self.last_cmd = [0.0, 0.0, 0.0]
                return 0.0, 0.0, 0.0, dist_to_final, [[round(current_x, 3), round(current_y, 3)]]

            crawl_speed = max(0.02, min(0.12, dist_to_final * 0.6))
            vr_x = crawl_speed * (rx / r_norm)
            vr_y = crawl_speed * (ry / r_norm) * 0.5
            wz = 0.0  # Zero angular velocity in terminal deadband
            self.last_cmd = [vr_x, vr_y, wz]
            pred_path = [
                [round(current_x, 3), round(current_y, 3)],
                [round(goal_x, 3), round(goal_y, 3)],
            ]
            return vr_x, vr_y, wz, dist_to_final, pred_path

        # 2. Extract local lookahead reference segment from target_path
        if not target_path:
            target_path = [[current_x, current_y], [goal_x, goal_y]]

        # Find closest waypoint index
        closest_idx = 0
        min_d = float("inf")
        for i, pt in enumerate(target_path):
            d = math.hypot(pt[0] - current_x, pt[1] - current_y)
            if d < min_d:
                min_d = d
                closest_idx = i

        # Lookahead reference point (e.g. 0.4 - 0.7m ahead)
        lookahead_idx = min(len(target_path) - 1, closest_idx + 2)
        ref_pt = target_path[lookahead_idx]
        ref_dx = ref_pt[0] - current_x
        ref_dy = ref_pt[1] - current_y
        ref_dist = math.hypot(ref_dx, ref_dy)
        nominal_heading = math.atan2(ref_dy, ref_dx) if ref_dist > 1e-3 else current_theta

        # 3. Dynamic candidate velocity samples
        base_speed = min(self.max_vx, max(0.08, dist_to_final * 0.6))
        cos_th = math.cos(current_theta)
        sin_th = math.sin(current_theta)

        target_rx = cos_th * ref_dx + sin_th * ref_dy
        target_ry = -sin_th * ref_dx + cos_th * ref_dy
        target_norm = math.hypot(target_rx, target_ry)
        if target_norm > 1e-4:
            u_nom_x = base_speed * (target_rx / target_norm)
            u_nom_y = base_speed * (target_ry / target_norm) * 0.3
        else:
            u_nom_x, u_nom_y = 0.0, 0.0

        angle_err = math.atan2(math.sin(nominal_heading - current_theta), math.cos(nominal_heading - current_theta))
        wz_nom = float(max(-self.max_wz, min(self.max_wz, 1.8 * angle_err)))

        # Candidate control actions
        vx_samples = [u_nom_x * 0.6, u_nom_x, min(self.max_vx, u_nom_x * 1.2)]
        vy_samples = [u_nom_y - 0.05, u_nom_y, u_nom_y + 0.05]
        wz_samples = [wz_nom - 0.3, wz_nom, wz_nom + 0.3]

        best_cost = float("inf")
        best_cmd = (u_nom_x, u_nom_y, wz_nom)
        best_trajectory = []

        # Near obstacles (within 1.0m of robot in robot frame)
        near_obstacles = []
        if lidar_points:
            for pt in lidar_points:
                px, py = pt[0], pt[1]
                if 0.05 < math.hypot(px, py) < 1.0:
                    wx = current_x + cos_th * px - sin_th * py
                    wy = current_y + sin_th * px + cos_th * py
                    near_obstacles.append((wx, wy))

        # 4. Evaluate MPC Trajectory Rollouts
        for vx_c in vx_samples:
            for vy_c in vy_samples:
                for wz_c in wz_samples:
                    cost = 0.0
                    rollout_pts = []
                    sim_x, sim_y, sim_th = current_x, current_y, current_theta
                    collision = False

                    for step in range(1, self.N + 1):
                        c_th = math.cos(sim_th)
                        s_th = math.sin(sim_th)
                        sim_x += (vx_c * c_th - vy_c * s_th) * self.dt
                        sim_y += (vx_c * s_th + vy_c * c_th) * self.dt
                        sim_th += wz_c * self.dt
                        rollout_pts.append([round(sim_x, 3), round(sim_y, 3)])

                        # Path tracking error: distance to reference point
                        cost += 3.0 * ((sim_x - ref_pt[0]) ** 2 + (sim_y - ref_pt[1]) ** 2)

                        # Obstacle distance penalty
                        for ox, oy in near_obstacles:
                            obs_dist = math.hypot(sim_x - ox, sim_y - oy)
                            if obs_dist < self.robot_radius:
                                collision = True
                                cost += 1000.0
                                break
                            elif obs_dist < self.robot_radius + 0.25:
                                cost += 15.0 / max(0.01, (obs_dist - self.robot_radius) ** 2)

                        if collision:
                            break

                    # Heading alignment cost
                    cost += 1.5 * (1.0 - math.cos(sim_th - nominal_heading))
                    # Control rate smoothness penalty
                    cost += 0.8 * ((vx_c - self.last_cmd[0]) ** 2 + (vy_c - self.last_cmd[1]) ** 2 + 0.3 * (wz_c - self.last_cmd[2]) ** 2)
                    # Goal distance cost
                    cost += 2.0 * ((sim_x - goal_x) ** 2 + (sim_y - goal_y) ** 2)

                    if cost < best_cost:
                        best_cost = cost
                        best_cmd = (vx_c, vy_c, wz_c)
                        best_trajectory = rollout_pts

        opt_vx, opt_vy, opt_wz = best_cmd
        self.last_cmd = [opt_vx, opt_vy, opt_wz]

        # S-curve speed limit when close to goal
        if dist_to_final < 0.4:
            scale = max(0.15, dist_to_final / 0.4)
            opt_vx *= scale
            opt_vy *= scale

        return opt_vx, opt_vy, opt_wz, dist_to_final, best_trajectory

