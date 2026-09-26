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
        inflation_radius: float = 0.55,
        max_scan_range: float = 12.0,
        min_scan_range: float = 0.15,
        max_obstacle_memory: int = 15000,
        inscribed_radius: float = 0.24,
    ):
        self.cell_size = cell_size
        self.inscribed_radius = inscribed_radius
        self.inflation_radius = inflation_radius
        self.max_scan_range = max_scan_range
        self.min_scan_range = min_scan_range
        self.max_obstacle_memory = max_obstacle_memory

        # Tập hợp các ô có vật cản thực tế (Lethal Obstacles - Cost 254)
        self.occupied_cells: Set[Tuple[int, int]] = set()
        # Bộ đếm điểm tin cậy vật cản (Probabilistic occupancy hits)
        self.cell_hits: Dict[Tuple[int, int], int] = {}
        # Vùng chạm thân xe (Inscribed Footprint Zone - Cost 253, robot_center cannot enter)
        self.inscribed_cells: Set[Tuple[int, int]] = set()
        # Bản đồ chi phí giãn nở mềm (Costmap Gradient: 1 - 253)
        self.cell_costs: Dict[Tuple[int, int], int] = {}

        # Tiền tính toán bảng offset 2 tầng: Inscribed & Inflation Gradient
        self._inflation_offsets: List[Tuple[int, int, int, bool]] = []
        max_r_cells = int(math.ceil(self.inflation_radius / self.cell_size))
        for dx in range(-max_r_cells, max_r_cells + 1):
            for dy in range(-max_r_cells, max_r_cells + 1):
                d_m = math.hypot(dx, dy) * self.cell_size
                if d_m <= self.inflation_radius:
                    if d_m <= self.inscribed_radius:
                        cost = 253
                        is_insc = True
                    else:
                        alpha = (d_m - self.inscribed_radius) / (self.inflation_radius - self.inscribed_radius)
                        cost = max(1, int(round(120.0 * ((1.0 - alpha) ** 2))))
                        is_insc = False
                    self._inflation_offsets.append((dx, dy, cost, is_insc))

        # Bộ lọc xác suất xác nhận vật cản & Correlative Scan Matcher
        self.min_confirm_hits = 2
        self.scan_count = 0
        self.last_matched_pose: Optional[Tuple[float, float, float]] = None

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

    def get_cell_cost(self, cx: int, cy: int) -> int:
        """Trả về chi phí của ô: 254 (Lethal), 253 (Inscribed), 1-252 (Gradient), 0 (Trống)."""
        if (cx, cy) in self.occupied_cells:
            return 254
        return self.cell_costs.get((cx, cy), 0)

    def is_free(self, cx: int, cy: int) -> bool:
        """Kiểm tra ô có an toàn cho tâm xe không (không phải lethal và không nằm trong inscribed)."""
        return (cx, cy) not in self.occupied_cells and (cx, cy) not in self.inscribed_cells

    def match_scan(
        self,
        rx: float,
        ry: float,
        rth: float,
        valid_points: List[List[float]],
    ) -> Tuple[float, float, float]:
        """Correlative Scan Matcher (CSM): Tự động so khớp chùm tia LiDAR với các bờ tường đã quét

        để triệt tiêu hoàn toàn sai số trượt bánh xe (wheel slip) và trôi odometry.
        """
        if len(self.occupied_cells) < 25 or not valid_points:
            return rx, ry, rth

        sub_step = max(1, len(valid_points) // 32)
        sub_pts = valid_points[::sub_step][:32]
        max_possible_score = len(sub_pts) * 4

        best_score = -1
        best_dx, best_dy, best_dth = 0.0, 0.0, 0.0

        dth_candidates = [-0.04, -0.02, 0.0, 0.02, 0.04]
        dxy_candidates = [-0.12, -0.06, 0.0, 0.06, 0.12]

        perfect_match = False
        for dth in dth_candidates:
            if perfect_match:
                break
            c_m = math.cos(rth + dth)
            s_m = math.sin(rth + dth)
            for dx in dxy_candidates:
                if perfect_match:
                    break
                cand_x = rx + dx
                for dy in dxy_candidates:
                    cand_y = ry + dy
                    score = 0
                    for lx, ly in sub_pts:
                        wx = cand_x + (c_m * lx - s_m * ly)
                        wy = cand_y + (s_m * lx + c_m * ly)
                        cx = int(round(wx / self.cell_size))
                        cy = int(round(wy / self.cell_size))
                        if (cx, cy) in self.occupied_cells:
                            score += 4
                        elif (
                            (cx + 1, cy) in self.occupied_cells
                            or (cx - 1, cy) in self.occupied_cells
                            or (cx, cy + 1) in self.occupied_cells
                            or (cx, cy - 1) in self.occupied_cells
                        ):
                            score += 1

                    if score > best_score:
                        best_score = score
                        best_dx, best_dy, best_dth = dx, dy, dth
                        if best_score >= max_possible_score:
                            perfect_match = True
                            break

        return rx + best_dx, ry + best_dy, rth + best_dth

    def add_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_theta: float,
        lidar_points: List[List[float]],
    ) -> int:
        """Tích lũy các điểm LiDAR với bộ so khớp Scan Matcher và bộ lọc xác suất (Probabilistic Filter)."""
        if not lidar_points:
            return 0

        # 1. Lọc bớt nhiễu khoảng cách không tin cậy (loại bỏ phản xạ sát thân xe và nhiễu cực xa)
        valid_points: List[List[float]] = []
        for pt in lidar_points:
            d = math.hypot(pt[0], pt[1])
            if max(0.18, self.min_scan_range) <= d <= min(7.5, self.max_scan_range):
                valid_points.append(pt)

        if not valid_points:
            return 0

        # 2. Correlative Scan Matching (CSM): Tự động khử trôi odometry & khóa khít bờ tường
        corr_x, corr_y, corr_th = self.match_scan(robot_x, robot_y, robot_theta, valid_points)
        self.last_matched_pose = (round(corr_x, 3), round(corr_y, 3), round(corr_th, 4))

        rcx, rcy = self.world_to_grid(corr_x, corr_y)
        cos_th = math.cos(corr_th)
        sin_th = math.sin(corr_th)
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

        stride = 2 if len(valid_points) > 180 else 1
        current_hit_cells: Set[Tuple[int, int]] = set()

        for idx in range(0, len(valid_points), stride):
            pt = valid_points[idx]
            lx, ly = pt[0], pt[1]

            # Kiểm tra tính liên tục của bề mặt (loại bỏ tia bụi/nhiễu đơn độc)
            is_contiguous = False
            if idx > 0:
                p_prev = valid_points[idx - 1]
                if math.hypot(lx - p_prev[0], ly - p_prev[1]) < 0.28:
                    is_contiguous = True
            if idx < len(valid_points) - 1:
                p_next = valid_points[idx + 1]
                if math.hypot(lx - p_next[0], ly - p_next[1]) < 0.28:
                    is_contiguous = True

            # Chuyển đổi sang hệ tọa độ thế giới bằng tọa độ đã hiệu chỉnh Scan Matcher
            wx = corr_x + (cos_th * lx - sin_th * ly)
            wy = corr_y + (sin_th * lx + cos_th * ly)
            ocx, ocy = self.world_to_grid(wx, wy)
            current_hit_cells.add((ocx, ocy))

            # Ray Clearing: Tia LiDAR nhìn xuyên qua khoảng trống làm sạch các bóng ma
            if has_existing:
                for fcx, fcy in self._bresenham_cells(rcx, rcy, ocx, ocy):
                    if (fcx, fcy) in self.cell_hits:
                        self.cell_hits[(fcx, fcy)] -= 1
                        if self.cell_hits[(fcx, fcy)] <= 0:
                            del self.cell_hits[(fcx, fcy)]
                            if (fcx, fcy) in self.occupied_cells:
                                self.occupied_cells.remove((fcx, fcy))
                                cleared_any = True

            # Ghi nhận điểm tin cậy vật cản (Probabilistic Confirmation)
            increment = 2 if is_contiguous else 1
            hits = min(15, self.cell_hits.get((ocx, ocy), 0) + increment)
            self.cell_hits[(ocx, ocy)] = hits

            # Chỉ xác nhận là ô vật cản khi đạt ngưỡng tin cậy >= min_confirm_hits
            if hits >= self.min_confirm_hits and (ocx, ocy) not in self.occupied_cells:
                self.occupied_cells.add((ocx, ocy))
                new_count += 1
                for ox, oy, cost, is_insc in self._inflation_offsets:
                    nbr = (ocx + ox, ocy + oy)
                    if is_insc:
                        self.inscribed_cells.add(nbr)
                    if cost > self.cell_costs.get(nbr, 0):
                        self.cell_costs[nbr] = cost

        # Tiêu biến các tia nhiễu lẻ loi theo chu kỳ (Temporal Decay)
        self.scan_count += 1
        if self.scan_count % 15 == 0:
            stale = [
                cell for cell, h in self.cell_hits.items()
                if h < self.min_confirm_hits and cell not in current_hit_cells
            ]
            for cell in stale:
                self.cell_hits[cell] -= 1
                if self.cell_hits[cell] <= 0:
                    del self.cell_hits[cell]

        if cleared_any:
            self._rebuild_inflated_cells()

        return new_count

    def _rebuild_inflated_cells(self):
        self.inscribed_cells.clear()
        self.cell_costs.clear()
        for cell in self.occupied_cells:
            for ox, oy, cost, is_insc in self._inflation_offsets:
                nbr = (cell[0] + ox, cell[1] + oy)
                if is_insc:
                    self.inscribed_cells.add(nbr)
                if cost > self.cell_costs.get(nbr, 0):
                    self.cell_costs[nbr] = cost

    @property
    def inflated_cells(self) -> Set[Tuple[int, int]]:
        """Tương thích ngược: trả về vùng inscribed_cells."""
        return self.inscribed_cells

    def clear(self):
        """Xóa toàn bộ bộ nhớ bản đồ."""
        self.occupied_cells.clear()
        self.cell_hits.clear()
        self.inscribed_cells.clear()
        self.cell_costs.clear()
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

    def _find_nearest_free_cell(self, cx: int, cy: int, max_search_radius: int = 16) -> Tuple[int, int]:
        """Nếu điểm đích tình cờ rơi vào ô có tường hoặc vùng giãn nở, tìm ô an toàn gần nhất."""
        if self.is_free(cx, cy):
            return cx, cy

        best_cell = (cx, cy)
        best_cost = float("inf")
        best_dist = float("inf")
        for r in range(1, max_search_radius + 1):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    cand = (cx + dx, cy + dy)
                    if cand not in self.occupied_cells:
                        cost = self.get_cell_cost(cand[0], cand[1])
                        d2 = dx * dx + dy * dy
                        if cost < best_cost or (cost == best_cost and d2 < best_dist):
                            best_cost = cost
                            best_dist = d2
                            best_cell = cand
            if best_cost < 253:
                break
        return best_cell

    def _line_of_sight(self, c0: Tuple[int, int], c1: Tuple[int, int], max_allowed_cost: int = 25) -> bool:
        """Kiểm tra đường ngắm thẳng có an toàn không, ngăn cắt góc sát mép vật cản."""
        x0, y0 = c0
        x1, y1 = c1
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            # Nếu chạm ô lethal, inscribed hoặc ô có chi phí cao vượt ngưỡng -> không cho phép nối thẳng
            cost = self.get_cell_cost(x, y)
            if cost > max_allowed_cost:
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
        """Rút gọn đường đi A* bằng line-of-sight an toàn để có góc cua mượt mà, không cắt góc sát vật cản."""
        if len(grid_path) <= 2:
            return grid_path

        pruned = [grid_path[0]]
        current_idx = 0
        n = len(grid_path)

        while current_idx < n - 1:
            next_idx = current_idx + 1
            # Thử nối trực tiếp tới điểm xa nhất có thể mà vẫn giữ khoảng cách an toàn (cost <= 25)
            for test_idx in range(n - 1, current_idx, -1):
                if self._line_of_sight(grid_path[current_idx], grid_path[test_idx], max_allowed_cost=25):
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
        max_expansions: int = 5000,
    ) -> List[List[float]]:
        """Lập lộ trình A* dựa trên bản đồ chi phí (Costmap-Weighted A*), luôn tìm đường né rộng và không bị kẹt."""
        start_cell = self.world_to_grid(start_x, start_y)
        goal_cell = self.world_to_grid(goal_x, goal_y)

        # Nếu chưa có vật cản nào được quét
        if not self.occupied_cells:
            steps = 15
            return [
                [
                    round(start_x + (goal_x - start_x) * (i / steps), 3),
                    round(start_y + (goal_y - start_y) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        # Nếu đích rơi vào tường, dời nhẹ sang ô trống an toàn gần nhất
        effective_goal = self._find_nearest_free_cell(goal_cell[0], goal_cell[1])

        # Kiểm tra nếu đường thẳng từ Start tới effective_goal hoàn toàn an toàn (xa vật cản) thì đi thẳng
        if self._line_of_sight(start_cell, effective_goal, max_allowed_cost=20):
            ex_w, ey_w = self.grid_to_world(effective_goal[0], effective_goal[1])
            steps = 15
            return [
                [
                    round(start_x + (ex_w - start_x) * (i / steps), 3),
                    round(start_y + (ey_w - start_y) * (i / steps), 3),
                ]
                for i in range(steps + 1)
            ]

        start_in_inscribed = (start_cell in self.inscribed_cells or start_cell in self.occupied_cells)

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

        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        g_scores: Dict[Tuple[int, int], float] = {start_cell: 0.0}
        expansions = 0
        found = False

        while open_set and expansions < max_expansions:
            expansions += 1
            f, current_g, current = heapq.heappop(open_set)

            if current == effective_goal or heuristic(current, effective_goal) <= 1.0:
                if current != effective_goal:
                    came_from[effective_goal] = current
                found = True
                break

            if current_g > g_scores.get(current, float("inf")):
                continue

            for dx, dy, step_dist in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                c = self.get_cell_cost(neighbor[0], neighbor[1])

                # Tuyệt đối không đi vào ô vật cản thực tế (Lethal)
                if c >= 254:
                    continue

                # Nếu ô nằm trong vùng chạm thân xe (Inscribed):
                # Chỉ cho phép khi robot đang bắt đầu từ trong vùng đó để thoát ra ngoài
                if c >= 253:
                    if not start_in_inscribed or current not in self.inscribed_cells:
                        continue

                # Với bước chéo, kiểm tra tránh cắt góc qua ô lethal
                if dx != 0 and dy != 0:
                    if self.get_cell_cost(current[0] + dx, current[1]) >= 254 or self.get_cell_cost(current[0], current[1] + dy) >= 254:
                        continue

                # Costmap Traversal Penalty: ưu tiên lộ trình xa vật cản
                if c >= 253:
                    penalty = 40.0
                else:
                    penalty = (c / 25.0) * 2.5

                step_cost = step_dist * (1.0 + penalty)
                tentative_g = current_g + step_cost

                if tentative_g < g_scores.get(neighbor, float("inf")):
                    g_scores[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, effective_goal) * 1.15
                    came_from[neighbor] = current
                    heapq.heappush(open_set, (f_score, tentative_g, neighbor))

        if not found:
            logger.warning(f"A* không tìm được đường tới đích sau {expansions} lần duyệt, dẫn tới điểm an toàn gần nhất.")
            best_candidate = start_cell
            best_d = float("inf")
            for node in g_scores:
                d = heuristic(node, effective_goal)
                if d < best_d and node not in self.occupied_cells:
                    best_d = d
                    best_candidate = node
            effective_goal = best_candidate
            if effective_goal == start_cell:
                # Fallback path đảm bảo luôn có ít nhất 2 điểm để cập nhật lộ trình
                steps = 10
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

        # Rút gọn các bước zíc-zắc an toàn
        pruned_path = self._prune_path(raw_path)

        # Chuyển đổi tọa độ ô thành tọa độ thế giới (mét)
        result: List[List[float]] = [[round(start_x, 3), round(start_y, 3)]]
        for cell in pruned_path[1:-1]:
            wx, wy = self.grid_to_world(cell[0], cell[1])
            result.append([wx, wy])
        result.append([round(goal_x, 3), round(goal_y, 3)])

        # Nội suy đều các điểm waypoint với mật độ ~0.25m để MPC bám mượt mà
        dense_result: List[List[float]] = []
        for i in range(len(result) - 1):
            p0 = result[i]
            p1 = result[i + 1]
            seg_dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            sub_steps = max(1, int(math.ceil(seg_dist / 0.25)))
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
        max_vx: float = 0.35,
        max_vy: float = 0.35,
        max_wz: float = 1.5,
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
        """Tính toán vận tốc tối ưu (vx, vy, wz), khoảng cách tới đích và quỹ đạo dự đoán MPC."""
        dist_to_final = math.hypot(goal_x - current_x, goal_y - current_y)
        cos_th = math.cos(current_theta)
        sin_th = math.sin(current_theta)

        # 1. Phân tích chướng ngại vật trong hệ tọa độ thân xe từ LiDAR
        min_front_dist = float("inf")
        min_obs_dist = float("inf")
        left_clearance = 0
        right_clearance = 0
        raw_obstacles: List[Tuple[float, float, float]] = []

        if lidar_points:
            for pt in lidar_points:
                px, py = pt[0], pt[1]
                d = math.hypot(px, py)
                if not (0.05 < d < 1.8):
                    continue
                if d < min_obs_dist:
                    min_obs_dist = d

                # Nón phía trước thân xe (-45° đến +45°)
                if px > 0 and abs(py) < max(0.18, px * 0.9):
                    if d < min_front_dist:
                        min_front_dist = d

                if py > 0:
                    left_clearance += 1
                else:
                    right_clearance += 1

                # Chuyển đổi sang hệ thế giới cho đánh giá rollout MPC
                wx = current_x + cos_th * px - sin_th * py
                wy = current_y + sin_th * px + cos_th * py
                raw_obstacles.append((d, wx, wy))

        if len(raw_obstacles) > 24:
            raw_obstacles.sort(key=lambda item: item[0])
            near_obstacles = [(wx, wy) for _, wx, wy in raw_obstacles[:24]]
        else:
            near_obstacles = [(wx, wy) for _, wx, wy in raw_obstacles]

        # 2. Vùng đệm cập bến (Terminal Arrival Deadband < 0.18m)
        if dist_to_final < 0.18:
            if dist_to_final < 0.05 or min_front_dist < (self.robot_radius + 0.05) or min_obs_dist < (self.robot_radius + 0.02):
                self.last_cmd = [0.0, 0.0, 0.0]
                return 0.0, 0.0, 0.0, dist_to_final, [[round(current_x, 3), round(current_y, 3)]]

            dx = goal_x - current_x
            dy = goal_y - current_y
            rx = cos_th * dx + sin_th * dy
            ry = -sin_th * dx + cos_th * dy
            r_norm = math.hypot(rx, ry)
            if r_norm < 1e-3:
                self.last_cmd = [0.0, 0.0, 0.0]
                return 0.0, 0.0, 0.0, dist_to_final, [[round(current_x, 3), round(current_y, 3)]]

            crawl_speed = max(0.02, min(0.10, dist_to_final * 0.5))
            vr_x = crawl_speed * (rx / r_norm)
            vr_y = crawl_speed * (ry / r_norm)
            if min_front_dist < (self.robot_radius + 0.08) and vr_x > 0:
                vr_x = 0.0
            self.last_cmd = [vr_x, vr_y, 0.0]
            pred_path = [
                [round(current_x, 3), round(current_y, 3)],
                [round(goal_x, 3), round(goal_y, 3)],
            ]
            return vr_x, vr_y, 0.0, dist_to_final, pred_path

        # 3. Trích xuất waypoint tham chiếu lookahead từ target_path
        if not target_path or len(target_path) < 2:
            target_path = [[current_x, current_y], [goal_x, goal_y]]

        closest_idx = 0
        min_d = float("inf")
        for i, pt in enumerate(target_path):
            d = math.hypot(pt[0] - current_x, pt[1] - current_y)
            if d < min_d:
                min_d = d
                closest_idx = i

        # Điểm lookahead cách ~0.35m - 0.50m dọc theo lộ trình né vật cản
        lookahead_idx = min(len(target_path) - 1, closest_idx + 2)
        ref_pt = target_path[lookahead_idx]
        ref_dx = ref_pt[0] - current_x
        ref_dy = ref_pt[1] - current_y
        ref_dist = math.hypot(ref_dx, ref_dy)
        nominal_heading = math.atan2(ref_dy, ref_dx) if ref_dist > 1e-3 else current_theta

        # 4. Tính toán vector vận tốc danh định theo hệ tọa độ thân xe
        base_speed = min(self.max_vx, max(0.10, dist_to_final * 0.5))
        target_rx = cos_th * ref_dx + sin_th * ref_dy
        target_ry = -sin_th * ref_dx + cos_th * ref_dy
        target_norm = math.hypot(target_rx, target_ry)
        if target_norm > 1e-4:
            u_nom_x = base_speed * (target_rx / target_norm)
            u_nom_y = base_speed * (target_ry / target_norm)
        else:
            u_nom_x, u_nom_y = 0.0, 0.0

        # Nếu phía trước có vật cản gần (< robot_radius + 0.12 = 0.34m):
        # Chặn vận tốc tiến u_nom_x (tránh đâm), nhưng ưu tiên lách ngang u_nom_y qua khoảng trống
        if min_front_dist < (self.robot_radius + 0.12):
            u_nom_x = min(0.0, u_nom_x)
            # Nếu waypoint chưa kịp lệch ngang, chủ động tạo xung lách sang phía thoáng hơn
            if abs(u_nom_y) < 0.08:
                strafe_dir = 1.0 if left_clearance >= right_clearance else -1.0
                u_nom_y = 0.22 * strafe_dir

        angle_err = math.atan2(math.sin(nominal_heading - current_theta), math.cos(nominal_heading - current_theta))
        wz_nom = float(max(-self.max_wz, min(self.max_wz, 1.4 * angle_err)))

        # 5. Sinh tập hợp candidate control actions đa hướng (Holonomic Mecanum)
        cand_vx = {0.0}
        # Nếu phía trước an toàn (>= 0.30m), cho phép tiến
        if min_front_dist >= (self.robot_radius + 0.08):
            if u_nom_x > 0:
                cand_vx.add(min(self.max_vx, u_nom_x))
                cand_vx.add(min(self.max_vx, u_nom_x * 0.6))
            else:
                cand_vx.add(0.12)
                cand_vx.add(self.max_vx)
        # Chỉ lùi khi robot bị kẹt vật lý ở cự ly nguy hiểm (< 16cm)
        if min_obs_dist < (self.robot_radius - 0.06):
            cand_vx.add(-0.10)

        cand_vy = {
            0.0,
            max(-self.max_vy, min(self.max_vy, u_nom_y)),
            0.22,
            -0.22,
        }
        if abs(u_nom_y) > 0.05:
            cand_vy.add(max(-self.max_vy, min(self.max_vy, u_nom_y * 1.25)))

        cand_wz = {0.0, wz_nom, wz_nom * 0.5}

        best_cost = float("inf")
        best_cmd = (0.0, 0.0, 0.0)
        best_trajectory: List[List[float]] = []

        # 6. Đánh giá quỹ đạo mô phỏng (MPC Horizon Rollout)
        r_col_sq = (self.robot_radius + 0.03) ** 2
        r_rep_sq = (self.robot_radius + 0.30) ** 2

        for vx_c in cand_vx:
            # Ngăn cản tuyệt đối vận tốc tiến nếu vật cản ngay trước mũi xe
            if min_front_dist < (self.robot_radius + 0.08) and vx_c > 0.0:
                continue

            for vy_c in cand_vy:
                for wz_c in cand_wz:
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

                        # Kiểm tra va chạm với các vật cản gần
                        for ox, oy in near_obstacles:
                            dx_o = sim_x - ox
                            dy_o = sim_y - oy
                            d_sq = dx_o * dx_o + dy_o * dy_o
                            if d_sq < r_col_sq:
                                collision = True
                                cost += 50000.0
                                break
                            elif d_sq < r_rep_sq:
                                obs_dist = math.sqrt(d_sq)
                                cost += 15.0 / max(0.01, (obs_dist - self.robot_radius) ** 2)

                        if collision:
                            break

                    if collision:
                        continue

                    # Chi phí bám đường tham chiếu (bám các waypoint né của A*)
                    cost += 4.0 * ((sim_x - ref_pt[0]) ** 2 + (sim_y - ref_pt[1]) ** 2)
                    # Chi phí xoay hướng đầu xe
                    cost += 1.0 * (1.0 - math.cos(sim_th - nominal_heading))
                    # Độ êm dịu điều khiển
                    cost += 0.3 * ((vx_c - self.last_cmd[0]) ** 2 + (vy_c - self.last_cmd[1]) ** 2 + 0.1 * (wz_c - self.last_cmd[2]) ** 2)
                    # Tiến dần tới đích
                    cost += 2.0 * ((sim_x - goal_x) ** 2 + (sim_y - goal_y) ** 2)

                    if cost < best_cost:
                        best_cost = cost
                        best_cmd = (vx_c, vy_c, wz_c)
                        best_trajectory = rollout_pts

        opt_vx, opt_vy, opt_wz = best_cmd
        self.last_cmd = [opt_vx, opt_vy, opt_wz]

        # Giảm tốc S-curve khi vào gần đích
        if dist_to_final < 0.35:
            scale = max(0.15, dist_to_final / 0.35)
            opt_vx *= scale
            opt_vy *= scale

        return opt_vx, opt_vy, opt_wz, dist_to_final, best_trajectory

