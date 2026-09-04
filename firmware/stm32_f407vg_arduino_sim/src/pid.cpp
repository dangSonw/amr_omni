#include "pid.h"

#include <math.h>

namespace {

float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
}

}  // namespace

WheelSpeedPid::WheelSpeedPid(float kp, float ki, float kd, float output_limit)
    : kp_(0.0F),
      ki_(0.0F),
      kd_(0.0F),
      output_limit_(1.0F),
      integral_limit_(1.0F),
      integral_(0.0F),
      previous_error_(0.0F),
      initialized_(false) {
    configure(kp, ki, kd, output_limit);
    reset();
}

void WheelSpeedPid::configure(float kp, float ki, float kd, float output_limit) {
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
        !isfinite(output_limit) || kp < 0.0F || ki < 0.0F || kd < 0.0F ||
        output_limit <= 0.0F) {
        return;
    }
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    output_limit_ = output_limit;
    integral_limit_ = output_limit;
}

void WheelSpeedPid::reset() {
    integral_ = 0.0F;
    previous_error_ = 0.0F;
    initialized_ = false;
}

float WheelSpeedPid::update(float setpoint, float measurement, float delta_sec) {
    if (!isfinite(setpoint) || !isfinite(measurement) ||
        !isfinite(delta_sec)) {
        reset();
        return 0.0F;
    }
    const float dt = delta_sec > 0.0F ? delta_sec : 0.001F;
    const float error = setpoint - measurement;
    if (!initialized_) {
        previous_error_ = error;
        initialized_ = true;
    }
    const float derivative = (error - previous_error_) / dt;
    const float candidate_integral = clamp(
        integral_ + error * dt, -integral_limit_, integral_limit_);
    const float unclamped = kp_ * error + ki_ * candidate_integral +
        kd_ * derivative;
    const float output = clamp(unclamped, -output_limit_, output_limit_);
    if (fabsf(output) < output_limit_ || output * error < 0.0F) {
        integral_ = candidate_integral;
    }
    previous_error_ = error;
    return output;
}