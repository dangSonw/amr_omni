#include "robot_state.h"

#include <string.h>

void initialize_robot_state(RobotState &state) {
    memset(&state, 0, sizeof(state));
    state.settings.wheel_radius_m = kWheelRadiusM;
    state.settings.wheelbase_m = kWheelbaseM;
    state.settings.track_width_m = kTrackWidthM;
    state.settings.max_wheel_speed_rad_s = kMaxWheelSpeedRadS;
    state.settings.command_timeout_ms = kCommandTimeoutMs;
    state.settings.motor_kp = kDefaultMotorKp;
    state.settings.motor_ki = kDefaultMotorKi;
    state.settings.motor_kd = kDefaultMotorKd;
    state.imu.valid = false;
    state.imu_fault = true;
    state.motor_fault = false;
}