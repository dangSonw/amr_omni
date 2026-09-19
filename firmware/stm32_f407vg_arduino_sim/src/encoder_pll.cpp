#include "encoder_pll.h"
#include <math.h>

namespace {
const float kTwoPi = 6.283185307179586F;
}  // namespace

EncoderPll::EncoderPll(float bandwidth_rad_s, uint32_t counts_per_revolution)
    : counts_per_rev_(counts_per_revolution > 0U ? counts_per_revolution : 2048U),
      pos_estimate_rad_(0.0F),
      vel_estimate_rad_s_(0.0F),
      pos_error_rad_(0.0F),
      time_since_last_pulse_sec_(0.0F),
      prev_timer_count_(0U),
      timer_initialized_(false) {
    rad_per_count_ = kTwoPi / static_cast<float>(counts_per_rev_);
    set_bandwidth(bandwidth_rad_s);
}

void EncoderPll::set_bandwidth(float bandwidth_rad_s) {
    pll_bandwidth_ = bandwidth_rad_s > 0.0F ? bandwidth_rad_s : 20.0F;
    kp_ = 2.0F * pll_bandwidth_;
    ki_ = 0.25F * (kp_ * kp_);  // ki = omega_pll^2 (zeta = 1.0 critical damping)
}

void EncoderPll::reset(float initial_pos_rad, float initial_vel_rad_s) {
    pos_estimate_rad_ = initial_pos_rad;
    vel_estimate_rad_s_ = initial_vel_rad_s;
    pos_error_rad_ = 0.0F;
    time_since_last_pulse_sec_ = 0.0F;
    timer_initialized_ = false;
}

float EncoderPll::update(int32_t delta_counts, float delta_sec) {
    const float dt = delta_sec > 0.0F ? delta_sec : 0.01F;

    // 1. Zero-speed watchdog tracking
    if (delta_counts == 0) {
        time_since_last_pulse_sec_ += dt;
        if (time_since_last_pulse_sec_ >= (kZeroSpeedTimeoutSec - 0.0001F)) {
            // Watchdog timeout (>= 50 ms): force zero velocity
            vel_estimate_rad_s_ = 0.0F;
            pos_error_rad_ = 0.0F;
            return 0.0F;
        }
    } else {
        time_since_last_pulse_sec_ = 0.0F;
    }

    // 2. Measured angular increment
    const float delta_theta_m = static_cast<float>(delta_counts) * rad_per_count_;

    // 3. Predicted angular increment
    const float delta_theta_pred = dt * vel_estimate_rad_s_;

    // 4. Innovation residual
    const float residual = pos_error_rad_ + delta_theta_m - delta_theta_pred;
    const float prior_vel = vel_estimate_rad_s_;

    // 5. State corrections (Second-Order tracking observer)
    vel_estimate_rad_s_ += dt * ki_ * residual;
    const float pos_correction = dt * kp_ * residual;
    pos_estimate_rad_ += delta_theta_pred + pos_correction;
    pos_error_rad_ = residual - pos_correction;

    // Zero-crossing prevention when no pulses arrive (monotonic decay)
    if (delta_counts == 0) {
        if ((prior_vel > 0.0F && vel_estimate_rad_s_ < 0.0F) ||
            (prior_vel < 0.0F && vel_estimate_rad_s_ > 0.0F)) {
            vel_estimate_rad_s_ = 0.0F;
            pos_error_rad_ = 0.0F;
        }
    }

    // 6. Numerical safeguards
    if (!isfinite(vel_estimate_rad_s_) || !isfinite(pos_estimate_rad_)) {
        reset(0.0F, 0.0F);
        return 0.0F;
    }

    // 7. LinuxCNC M/T decay envelope at ultra-low speeds
    if (delta_counts == 0 && time_since_last_pulse_sec_ > 0.0F) {
        const float max_possible_speed = rad_per_count_ / time_since_last_pulse_sec_;
        if (fabsf(vel_estimate_rad_s_) > max_possible_speed) {
            vel_estimate_rad_s_ = (vel_estimate_rad_s_ > 0.0F) ? max_possible_speed : -max_possible_speed;
        }
    }

    return vel_estimate_rad_s_;
}

float EncoderPll::update_raw(uint16_t current_raw_timer_count, float delta_sec) {
    if (!timer_initialized_) {
        prev_timer_count_ = current_raw_timer_count;
        timer_initialized_ = true;
        return vel_estimate_rad_s_;
    }
    const int16_t delta_counts = compute_timer_delta(current_raw_timer_count, prev_timer_count_);
    prev_timer_count_ = current_raw_timer_count;
    return update(static_cast<int32_t>(delta_counts), delta_sec);
}
