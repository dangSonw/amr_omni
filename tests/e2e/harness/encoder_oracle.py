"""
Authoritative Reference Oracle for Encoder Estimation (ODrive PLL and LinuxCNC M/T).
Implements equations from PROJECT.md (F1.1, F1.2) and Encoder.docx.
"""
import math


class ODrivePLLObserver:
    """
    Discrete-time 2nd-order PLL tracking observer for wheel position and velocity.
    Matches ODrive algorithm:
      kp = 2 * omega_pll
      ki = omega_pll^2
      delta_pos = pos_measurement - pos_estimate
      pos_estimate += dt * (vel_estimate + kp * delta_pos)
      vel_estimate += dt * (ki * delta_pos)
    """

    def __init__(self, omega_pll: float = 100.0, initial_pos: float = 0.0, initial_vel: float = 0.0):
        if omega_pll <= 0:
            raise ValueError("omega_pll must be positive")
        self.omega_pll = float(omega_pll)
        self.kp = 2.0 * self.omega_pll
        self.ki = self.omega_pll * self.omega_pll
        self.pos_estimate = float(initial_pos)
        self.vel_estimate = float(initial_vel)

    def set_bandwidth(self, omega_pll: float):
        if omega_pll <= 0:
            raise ValueError("omega_pll must be positive")
        self.omega_pll = float(omega_pll)
        self.kp = 2.0 * self.omega_pll
        self.ki = self.omega_pll * self.omega_pll

    def update(self, pos_measurement: float, dt: float) -> tuple[float, float]:
        if dt <= 0:
            raise ValueError("dt must be positive")
        if not math.isfinite(pos_measurement):
            raise ValueError("pos_measurement must be finite")

        delta_pos = pos_measurement - self.pos_estimate
        self.pos_estimate += dt * (self.vel_estimate + self.kp * delta_pos)
        self.vel_estimate += dt * (self.ki * delta_pos)
        return self.pos_estimate, self.vel_estimate

    def reset(self, pos: float = 0.0, vel: float = 0.0):
        self.pos_estimate = float(pos)
        self.vel_estimate = float(vel)


class LinuxCNCHybridEstimator:
    """
    LinuxCNC M/T Hybrid Velocity Estimator with 16-bit timer rollover handling
    and zero-speed watchdog timeout.
    Equation:
      omega = (2 * pi * delta_m) / (CPR * (N_timer / f_clk))
    """

    def __init__(self, cpr: int = 4000, f_clk: float = 84_000_000.0, watchdog_timeout_s: float = 0.05):
        if cpr <= 0:
            raise ValueError("cpr must be positive")
        if f_clk <= 0:
            raise ValueError("f_clk must be positive")
        if watchdog_timeout_s <= 0:
            raise ValueError("watchdog_timeout_s must be positive")

        self.cpr = int(cpr)
        self.f_clk = float(f_clk)
        self.watchdog_timeout_s = float(watchdog_timeout_s)

        self.last_timer_cnt = 0
        self.last_edge_timestamp = 0.0
        self.last_velocity = 0.0

    @staticmethod
    def safe_timer_rollover_diff(current_cnt: int, last_cnt: int) -> int:
        """
        Two's complement safe difference for 16-bit timer counter:
        int16_t delta_counts = (int16_t)(current_timer_cnt - last_timer_cnt);
        """
        diff = (int(current_cnt) - int(last_cnt)) & 0xFFFF
        if diff >= 0x8000:
            diff -= 0x10000
        return diff

    def calculate_velocity(self, delta_m: int, n_timer_ticks: int) -> float:
        """Calculate angular velocity in rad/s from delta counts and timer ticks."""
        if n_timer_ticks <= 0:
            return 0.0
        dt = float(n_timer_ticks) / self.f_clk
        if dt <= 0:
            return 0.0
        return (2.0 * math.pi * float(delta_m)) / (float(self.cpr) * dt)

    def update_sample(self, current_pulse_cnt: int, last_pulse_cnt: int,
                      current_timer_cnt: int, last_timer_cnt: int,
                      dt_sample: float) -> float:
        """
        Hybrid M/T sample step with rollover and watchdog logic.
        """
        delta_m = self.safe_timer_rollover_diff(current_pulse_cnt, last_pulse_cnt)
        n_ticks = (int(current_timer_cnt) - int(last_timer_cnt)) & 0xFFFFFFFF

        if delta_m == 0:
            if dt_sample >= self.watchdog_timeout_s:
                self.last_velocity = 0.0
                return 0.0
            return self.last_velocity

        vel = self.calculate_velocity(delta_m, n_ticks)
        self.last_velocity = vel
        return vel
