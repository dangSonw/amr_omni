import math
import numpy as np


WHEEL_ORDER = (
    'omni_wheel_joint_1', 'omni_wheel_joint_2',
    'omni_wheel_joint_3', 'omni_wheel_joint_4',
)


def validate_twist(vx_mps, vy_mps, wz_rad_s, max_linear_speed_mps=None,
                   max_angular_speed_rad_s=None):
    """Validate twist values for finiteness and optional physical velocity limits."""
    values = (vx_mps, vy_mps, wz_rad_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('twist must contain finite values')
    if max_linear_speed_mps is not None:
        if abs(vx_mps) > max_linear_speed_mps or abs(vy_mps) > max_linear_speed_mps:
            raise ValueError('linear velocity exceeds maximum allowed speed')
    if max_angular_speed_rad_s is not None:
        if abs(wz_rad_s) > max_angular_speed_rad_s:
            raise ValueError('angular velocity exceeds maximum allowed speed')


def inverse_kinematics(vx_mps, vy_mps, wz_rad_s, wheel_radius_m,
                       wheelbase_m, track_width_m, max_wheel_speed_rad_s):
    validate_twist(vx_mps, vy_mps, wz_rad_s)
    if wheel_radius_m <= 0 or wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')
    if max_wheel_speed_rad_s <= 0:
        raise ValueError('max wheel speed must be positive')

    d = np.sqrt(0.5)
    radius = 0.5 * np.hypot(wheelbase_m, track_width_m)
    # Wheel 1 (FR: -45 deg), Wheel 2 (FL: +45 deg), Wheel 3 (RL: +135 deg), Wheel 4 (RR: -135 deg)
    matrix = np.array([
        [ d,  d, radius],
        [-d,  d, radius],
        [-d, -d, radius],
        [ d, -d, radius],
    ])
    wheel_speeds = (matrix @ np.array([vx_mps, vy_mps, wz_rad_s])) / wheel_radius_m
    scale = max(1.0, float(np.max(np.abs(wheel_speeds))) / max_wheel_speed_rad_s)
    return tuple((wheel_speeds / scale).tolist())


def forward_kinematics(wheel_speeds_rad_s, wheel_radius_m, wheelbase_m,
                       track_width_m):
    if len(wheel_speeds_rad_s) != 4:
        raise ValueError('four wheel speeds are required')
    speeds = np.array(wheel_speeds_rad_s, dtype=float)
    if not np.all(np.isfinite(speeds)):
        raise ValueError('wheel speeds must be finite')
    if wheel_radius_m <= 0 or wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')

    d = np.sqrt(0.5)
    radius = 0.5 * np.hypot(wheelbase_m, track_width_m)
    pinv_matrix = 0.25 * np.array([
        [ 1.0 / d, -1.0 / d, -1.0 / d,  1.0 / d],
        [ 1.0 / d,  1.0 / d, -1.0 / d, -1.0 / d],
        [ 1.0 / radius, 1.0 / radius, 1.0 / radius, 1.0 / radius],
    ])
    twist = (pinv_matrix @ speeds) * wheel_radius_m
    return float(twist[0]), float(twist[1]), float(twist[2])
