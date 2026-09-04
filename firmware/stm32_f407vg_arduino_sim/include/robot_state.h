#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdint.h>

#include "firmware_config.h"

struct RobotState {
    FirmwareSettings settings;
    float target_wheel_speed_rad_s[kWheelCount];
    float measured_wheel_speed_rad_s[kWheelCount];
    float motor_output[kWheelCount];
    int32_t encoder_counts[kWheelCount];
    TwistCommand command_twist;
    PoseState pose;
    ImuSample imu;
    uint32_t last_command_ms;
    uint32_t last_encoder_ms;
    uint32_t last_imu_ms;
    uint32_t last_telemetry_ms;
    bool estop_active;
    bool command_valid;
    bool imu_fault;
};

void initialize_robot_state(RobotState &state);

#endif