#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

#include <stdint.h>

static const uint8_t kWheelCount = 4U;
static const float kWheelRadiusM = 0.03F;
static const float kWheelbaseM = 0.1312F;
static const float kTrackWidthM = 0.1312F;
static const float kMaxWheelSpeedRadS = 18.0F;
static const float kMaxLinearSpeedMps = 0.54F;
static const float kMaxAngularSpeedRadS = 3.0F;
static const uint32_t kCommandTimeoutMs = 250U;
static const uint32_t kEncoderPeriodMs = 10U;
static const uint32_t kControlPeriodMs = 10U;
static const uint32_t kImuPeriodMs = 20U;
static const uint32_t kOdometryPeriodMs = 10U;
static const uint32_t kTelemetryPeriodMs = 50U;
static const uint32_t kEncoderCountsPerRevolution = 2048U;
static const float kDefaultMotorKp = 1.0F;
static const float kDefaultMotorKi = 0.0F;
static const float kDefaultMotorKd = 0.0F;
static const float kDefaultKalmanQ = 0.5F;
static const float kDefaultKalmanR = 0.04F;

struct FirmwareSettings {
    float wheel_radius_m;
    float wheelbase_m;
    float track_width_m;
    float max_wheel_speed_rad_s;
    uint32_t command_timeout_ms;
    float motor_kp[kWheelCount];
    float motor_ki[kWheelCount];
    float motor_kd[kWheelCount];
    float kalman_q[kWheelCount];
    float kalman_r[kWheelCount];
};

struct TwistCommand {
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
};

struct ImuSample {
    float linear_accel_mps2[3];
    float gyro_rad_s[3];
    float mag_uT[3];
    float quaternion_xyzw[4];
    float accuracy_rad;
    bool valid;
};

struct PoseState {
    float x_m;
    float y_m;
    float yaw_rad;
    float vx_mps;
    float vy_mps;
    float wz_rad_s;
};

#endif