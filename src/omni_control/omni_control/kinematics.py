import math


WHEEL_ORDER = (
    'omni_wheel_joint_1', 'omni_wheel_joint_2',
    'omni_wheel_joint_3', 'omni_wheel_joint_4',
)


def validate_twist(vx_mps, vy_mps, wz_rad_s):
    values = (vx_mps, vy_mps, wz_rad_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('twist must contain finite values')


def inverse_kinematics(vx_mps, vy_mps, wz_rad_s, wheel_radius_m,
                       wheelbase_m, track_width_m, max_wheel_speed_rad_s):
    validate_twist(vx_mps, vy_mps, wz_rad_s)
    if wheel_radius_m <= 0 or wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')
    if max_wheel_speed_rad_s <= 0:
        raise ValueError('max wheel speed must be positive')

    # Exact upstream 4w order:
    # Wheel 1: Front-Right (FR) at (x=+L/2, y=-W/2), angle -45 deg
    # Wheel 2: Front-Left  (FL) at (x=+L/2, y=+W/2), angle +45 deg
    # Wheel 3: Rear-Left   (RL) at (x=-L/2, y=+W/2), angle +135 deg
    # Wheel 4: Rear-Right  (RR) at (x=-L/2, y=-W/2), angle +225 (-135) deg
    # The physical distance from center of robot to each 45-deg omni wheel center is:
    # R = sqrt((L/2)^2 + (W/2)^2) = 0.5 * hypot(wheelbase, track_width).
    radius = 0.5 * math.hypot(wheelbase_m, track_width_m)
    diagonal = math.sqrt(0.5)
    velocities = (
        diagonal * vx_mps + diagonal * vy_mps + radius * wz_rad_s,
        -diagonal * vx_mps + diagonal * vy_mps + radius * wz_rad_s,
        -diagonal * vx_mps - diagonal * vy_mps + radius * wz_rad_s,
        diagonal * vx_mps - diagonal * vy_mps + radius * wz_rad_s,
    )
    speeds = tuple(value / wheel_radius_m for value in velocities)
    scale = max(1.0, max(abs(value) for value in speeds) /
                max_wheel_speed_rad_s)
    return tuple(value / scale for value in speeds)


def forward_kinematics(wheel_speeds_rad_s, wheel_radius_m, wheelbase_m,
                       track_width_m):
    if len(wheel_speeds_rad_s) != 4:
        raise ValueError('four wheel speeds are required')
    values = tuple(float(value) for value in wheel_speeds_rad_s)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('wheel speeds must be finite')
    if wheel_radius_m <= 0 or wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')

    radius = 0.5 * math.hypot(wheelbase_m, track_width_m)
    diagonal = math.sqrt(0.5)
    # Pseudoinverse of the tangent-wheel matrix above.  The radius term is the
    # distance from the body centre to a wheel contact direction.
    vx = wheel_radius_m * (values[0] - values[1]
                           - values[2] + values[3]) / (4.0 * diagonal)
    vy = wheel_radius_m * (values[0] + values[1]
                           - values[2] - values[3]) / (4.0 * diagonal)
    wz = wheel_radius_m * sum(values) / (4.0 * radius)
    return vx, vy, wz
