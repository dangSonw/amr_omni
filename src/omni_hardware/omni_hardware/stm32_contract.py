import math


TOPICS = {
    'input_cmd_vel': 'cmd_vel',
    'safe_cmd_vel': 'safe_cmd_vel',
    'stm32_cmd_vel': 'stm32_cmd_vel',
    'estop': 'estop',
    'odom': 'odom',
    'imu': 'imu/data_raw',
    'wheel_state': 'wheel_state',
    'encoder_counts': 'encoder_counts',
    'diagnostics': 'diagnostics',
    'status': 'status',
}


def validate_twist(values, max_linear_speed_mps, max_angular_speed_rad_s):
    if len(values) != 3:
        return False
    try:
        numeric_values = tuple(float(value) for value in values)
    except (TypeError, ValueError):
        return False
    return (all(math.isfinite(value) for value in numeric_values) and
            abs(numeric_values[0]) <= max_linear_speed_mps and
            abs(numeric_values[1]) <= max_linear_speed_mps and
            abs(numeric_values[2]) <= max_angular_speed_rad_s)