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
                       wheelbase_m, track_width_m, max_wheel_speed_rad_s,
                       wheel_radius_correction=None, kr=None,
                       wheel_radii=None):
    validate_twist(vx_mps, vy_mps, wz_rad_s)
    if not (math.isfinite(wheelbase_m) and math.isfinite(track_width_m)):
        raise ValueError('wheelbase and track width must be finite numbers')
    if wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')
    if not math.isfinite(max_wheel_speed_rad_s) or max_wheel_speed_rad_s <= 0:
        raise ValueError('max wheel speed must be positive')

    effective_radii = _parse_wheel_radius_correction(
        wheel_radius_m,
        wheel_radius_correction=wheel_radius_correction,
        kr=kr,
        wheel_radii=wheel_radii,
    )

    d = np.sqrt(0.5)
    radius = 0.5 * np.hypot(wheelbase_m, track_width_m)
    # Wheel 1 (FR: -45 deg), Wheel 2 (FL: +45 deg), Wheel 3 (RL: +135 deg), Wheel 4 (RR: -135 deg)
    matrix = np.array([
        [ d,  d, radius],
        [-d,  d, radius],
        [-d, -d, radius],
        [ d, -d, radius],
    ])
    wheel_speeds = (matrix @ np.array([vx_mps, vy_mps, wz_rad_s])) / effective_radii
    scale = max(1.0, float(np.max(np.abs(wheel_speeds))) / max_wheel_speed_rad_s)
    return tuple((wheel_speeds / scale).tolist())


def _parse_wheel_radius_correction(wheel_radius_m, wheel_radius_correction=None,
                                   kr=None, wheel_radii=None):
    if not math.isfinite(wheel_radius_m) or wheel_radius_m <= 0.0:
        raise ValueError('wheel radius must be a finite positive number')

    if wheel_radii is not None:
        arr = np.asarray(wheel_radii, dtype=float)
        if arr.shape != (4,):
            raise ValueError('wheel_radii must be a sequence of 4 elements')
        if not np.all(np.isfinite(arr)):
            raise ValueError('wheel_radii must contain finite values')
        if np.any(arr <= 0.0):
            raise ValueError('wheel_radii elements must be strictly positive')
        return arr

    correction = wheel_radius_correction if wheel_radius_correction is not None else kr
    if correction is None:
        return np.full(4, float(wheel_radius_m), dtype=float)

    arr = np.asarray(correction, dtype=float)
    if arr.ndim == 2:
        if arr.shape != (4, 4):
            raise ValueError('wheel_radius_correction matrix must have shape (4, 4)')
        diag_arr = np.diag(arr)
        if not np.allclose(arr, np.diag(diag_arr)):
            raise ValueError('wheel_radius_correction matrix must be diagonal')
        arr = diag_arr
    elif arr.ndim == 1:
        if arr.shape != (4,):
            raise ValueError('wheel_radius_correction sequence must have length 4')
    else:
        raise ValueError('wheel_radius_correction must be a 4-element sequence or (4, 4) matrix')

    if not np.all(np.isfinite(arr)):
        raise ValueError('wheel_radius_correction must contain finite values')
    if np.any(arr <= 0.0):
        raise ValueError('wheel_radius_correction elements must be strictly positive')

    return float(wheel_radius_m) * arr


def forward_kinematics(wheel_speeds_rad_s, wheel_radius_m, wheelbase_m,
                       track_width_m, wheel_radius_correction=None,
                       kr=None, wheel_radii=None):
    if len(wheel_speeds_rad_s) != 4:
        raise ValueError('four wheel speeds are required')
    speeds = np.array(wheel_speeds_rad_s, dtype=float)
    if not np.all(np.isfinite(speeds)):
        raise ValueError('wheel speeds must be finite')
    if not (math.isfinite(wheelbase_m) and math.isfinite(track_width_m)):
        raise ValueError('wheelbase and track width must be finite numbers')
    if wheelbase_m <= 0 or track_width_m <= 0:
        raise ValueError('wheel geometry must be positive')

    effective_radii = _parse_wheel_radius_correction(
        wheel_radius_m,
        wheel_radius_correction=wheel_radius_correction,
        kr=kr,
        wheel_radii=wheel_radii,
    )
    rim_speeds = speeds * effective_radii

    d = np.sqrt(0.5)
    radius = 0.5 * np.hypot(wheelbase_m, track_width_m)
    pinv_matrix = 0.25 * np.array([
        [ 1.0 / d, -1.0 / d, -1.0 / d,  1.0 / d],
        [ 1.0 / d,  1.0 / d, -1.0 / d, -1.0 / d],
        [ 1.0 / radius, 1.0 / radius, 1.0 / radius, 1.0 / radius],
    ])
    twist = pinv_matrix @ rim_speeds
    return float(twist[0]), float(twist[1]), float(twist[2])


compute_wheel_speeds = inverse_kinematics
compute_body_twist = forward_kinematics
