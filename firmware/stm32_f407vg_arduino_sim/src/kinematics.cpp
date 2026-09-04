#include "kinematics.h"

#include <math.h>

namespace {

const float kDiagonal = 0.70710678118F;
const float kPi = 3.14159265359F;

bool is_finite(float value) {
    return isfinite(value) != 0;
}

float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

}  // namespace

bool compute_wheel_speeds(const TwistCommand &twist,
                          const FirmwareSettings &settings,
                          float wheel_speeds[kWheelCount]) {
    if (settings.wheel_radius_m <= 0.0F ||
        settings.wheelbase_m <= 0.0F ||
        settings.track_width_m <= 0.0F ||
        settings.max_wheel_speed_rad_s <= 0.0F) {
        return false;
    }
    if (!is_finite(twist.vx_mps) || !is_finite(twist.vy_mps) ||
        !is_finite(twist.wz_rad_s)) {
        return false;
    }

    const float radius = 0.5F *
        sqrtf(settings.wheelbase_m * settings.wheelbase_m +
              settings.track_width_m * settings.track_width_m);
    const float values[kWheelCount] = {
        kDiagonal * twist.vx_mps + kDiagonal * twist.vy_mps +
            radius * twist.wz_rad_s,
        -kDiagonal * twist.vx_mps + kDiagonal * twist.vy_mps +
            radius * twist.wz_rad_s,
        -kDiagonal * twist.vx_mps - kDiagonal * twist.vy_mps +
            radius * twist.wz_rad_s,
        kDiagonal * twist.vx_mps - kDiagonal * twist.vy_mps +
            radius * twist.wz_rad_s,
    };
    float scale = 1.0F;
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        wheel_speeds[index] = values[index] / settings.wheel_radius_m;
        const float magnitude = fabsf(wheel_speeds[index]);
        if (magnitude > settings.max_wheel_speed_rad_s) {
            scale = fmaxf(scale, magnitude / settings.max_wheel_speed_rad_s);
        }
    }
    if (scale > 1.0F) {
        for (uint8_t index = 0U; index < kWheelCount; ++index) {
            wheel_speeds[index] /= scale;
        }
    }
    return true;
}

bool compute_body_twist(const float wheel_speeds[kWheelCount],
                        const FirmwareSettings &settings,
                        TwistCommand &twist) {
    if (settings.wheel_radius_m <= 0.0F ||
        settings.wheelbase_m <= 0.0F ||
        settings.track_width_m <= 0.0F) {
        return false;
    }
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        if (!is_finite(wheel_speeds[index])) {
            return false;
        }
    }
    const float radius = 0.5F *
        sqrtf(settings.wheelbase_m * settings.wheelbase_m +
              settings.track_width_m * settings.track_width_m);
    twist.vx_mps = settings.wheel_radius_m *
        (wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] +
         wheel_speeds[3]) / (4.0F * kDiagonal);
    twist.vy_mps = settings.wheel_radius_m *
        (wheel_speeds[0] + wheel_speeds[1] - wheel_speeds[2] -
         wheel_speeds[3]) / (4.0F * kDiagonal);
    twist.wz_rad_s = settings.wheel_radius_m *
        (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] +
         wheel_speeds[3]) / (4.0F * radius);
    return true;
}

float wrap_angle(float angle_rad) {
    while (angle_rad > kPi) {
        angle_rad -= 2.0F * kPi;
    }
    while (angle_rad < -kPi) {
        angle_rad += 2.0F * kPi;
    }
    return angle_rad;
}