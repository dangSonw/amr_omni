import math


TOPICS = {
    'input_cmd_vel': 'cmd_vel',
    'safe_cmd_vel': 'safe_cmd_vel',
    'stm32_cmd_vel': 'stm32_cmd_vel',
    'estop': 'estop',
    'wheel_odom': 'wheel/odom',
    'imu': 'imu/data',
    'debug_data': 'debug/data',
}


def validate_twist(values, max_linear_speed_mps, max_angular_speed_rad_s):
    if len(values) != 3:
        return False
    try:
        numeric_values = tuple(float(value) for value in values)
        if not all(math.isfinite(v) for v in numeric_values):
            return False
        return (abs(numeric_values[0]) <= max_linear_speed_mps and
                abs(numeric_values[1]) <= max_linear_speed_mps and
                abs(numeric_values[2]) <= max_angular_speed_rad_s)
    except (TypeError, ValueError):
        return False