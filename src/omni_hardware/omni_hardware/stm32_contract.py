import math

try:
    from omni_control.kinematics import validate_twist as _kinematics_validate_twist
except ImportError:
    _kinematics_validate_twist = None


TOPICS = {
    'input_cmd_vel': 'cmd_vel',
    'safe_cmd_vel': 'safe_cmd_vel',
    'stm32_cmd_vel': 'stm32_cmd_vel',
    'estop': 'estop',
    'wheel_odom': 'wheel/odom',
    'imu': 'imu/data',
    'odom': 'wheel/odom',
    'status': 'status',
}


def validate_twist(values, max_linear_speed_mps, max_angular_speed_rad_s):
    if len(values) != 3:
        return False
    try:
        numeric_values = tuple(float(value) for value in values)
        if _kinematics_validate_twist is not None:
            _kinematics_validate_twist(
                numeric_values[0], numeric_values[1], numeric_values[2],
                max_linear_speed_mps=max_linear_speed_mps,
                max_angular_speed_rad_s=max_angular_speed_rad_s,
            )
            return True
        return (all(math.isfinite(value) for value in numeric_values) and
                abs(numeric_values[0]) <= max_linear_speed_mps and
                abs(numeric_values[1]) <= max_linear_speed_mps and
                abs(numeric_values[2]) <= max_angular_speed_rad_s)
    except (TypeError, ValueError):
        return False