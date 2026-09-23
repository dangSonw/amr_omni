#include "kalman.h"
#include <math.h>

ScalarKalman::ScalarKalman(float process_noise, float measurement_noise)
    : process_noise_(process_noise),
      measurement_noise_(measurement_noise),
      estimate_(0.0F),
      covariance_(1.0F),
      initialized_(false) {}

void ScalarKalman::reset(float estimate, float covariance) {
    estimate_ = estimate;
    covariance_ = covariance > 0.0F ? covariance : 1.0F;
    initialized_ = true;
}

float ScalarKalman::update(float measurement, float delta_sec) {
    if (!isfinite(measurement)) {
        return estimate_;
    }

    if (!initialized_) {
        reset(measurement, 1.0F);
        return estimate_;
    }

    const float dt = delta_sec > 0.0F ? delta_sec : 0.001F;
    covariance_ += process_noise_ * dt;
    const float denominator = covariance_ + measurement_noise_;
    const float gain = denominator > 0.0F ? covariance_ / denominator : 0.0F;
    estimate_ += gain * (measurement - estimate_);
    covariance_ *= 1.0F - gain;
    return estimate_;
}

float ScalarKalman::estimate() const {
    return estimate_;
}

void ScalarKalman::set_noise(float process_noise, float measurement_noise) {
    if (process_noise >= 0.0F && measurement_noise > 0.0F) {
        process_noise_ = process_noise;
        measurement_noise_ = measurement_noise;
    }
}

void ScalarKalman::get_noise(float &process_noise, float &measurement_noise) const {
    process_noise = process_noise_;
    measurement_noise = measurement_noise_;
}