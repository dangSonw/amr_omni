#include "robot_state.h"

#include <string.h>

void initialize_robot_state(RobotState &state) {
    memset(&state, 0, sizeof(state));
    state.settings.wheel_radius_m = kWheelRadiusM;
    state.settings.wheelbase_m = kWheelbaseM;
    state.settings.track_width_m = kTrackWidthM;
    state.settings.max_wheel_speed_rad_s = kMaxWheelSpeedRadS;
    state.settings.command_timeout_ms = kCommandTimeoutMs;
    for (uint8_t i = 0U; i < kWheelCount; ++i) {
        state.settings.motor_kp[i] = kDefaultMotorKp;
        state.settings.motor_ki[i] = kDefaultMotorKi;
        state.settings.motor_kd[i] = kDefaultMotorKd;
        state.settings.kalman_q[i] = kDefaultKalmanQ;
        state.settings.kalman_r[i] = kDefaultKalmanR;
    }
    state.imu.valid = false;
    state.imu_fault = true;
    state.motor_fault = false;
}