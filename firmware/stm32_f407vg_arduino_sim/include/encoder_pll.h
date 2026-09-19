#ifndef ENCODER_PLL_H
#define ENCODER_PLL_H

#include <stdint.h>

/**
 * @brief Second-Order Phase-Locked Loop (PLL) Tracking Observer & LinuxCNC M/T Hybrid Velocity Estimator.
 * 
 * Provides critically damped (zeta = 1.0) velocity estimation with zero steady-state phase lag,
 * 16-bit hardware timer rollover-safe difference arithmetic, and 50 ms zero-speed watchdog.
 */
class EncoderPll {
public:
    /**
     * @brief Construct an EncoderPll observer.
     * @param bandwidth_rad_s Observer bandwidth omega_pll in rad/s (default: 20.0 rad/s).
     * @param counts_per_revolution Encoder resolution CPR (default: 2048).
     */
    explicit EncoderPll(float bandwidth_rad_s = 20.0F,
                        uint32_t counts_per_revolution = 2048U);

    /**
     * @brief Reset observer states to specified position and velocity.
     */
    void reset(float initial_pos_rad = 0.0F, float initial_vel_rad_s = 0.0F);

    /**
     * @brief Configure PLL bandwidth and update gains with critical damping (zeta = 1.0).
     * kp = 2.0 * omega_pll
     * ki = omega_pll^2 = 0.25 * kp^2
     */
    void set_bandwidth(float bandwidth_rad_s);

    /**
     * @brief Update observer using signed count delta.
     * @param delta_counts Signed encoder counts since last update.
     * @param delta_sec Time interval in seconds since last update.
     * @return Estimated angular velocity in rad/s.
     */
    float update(int32_t delta_counts, float delta_sec);

    /**
     * @brief Update observer using raw 16-bit hardware timer counter register.
     * Safe across 16-bit rollover: (int16_t)(curr - prev).
     * @param current_raw_timer_count Current TIMx->CNT value (0..65535).
     * @param delta_sec Time interval in seconds.
     * @return Estimated angular velocity in rad/s.
     */
    float update_raw(uint16_t current_raw_timer_count, float delta_sec);

    float velocity() const { return vel_estimate_rad_s_; }
    float position() const { return pos_estimate_rad_; }
    float position_error() const { return pos_error_rad_; }
    float bandwidth() const { return pll_bandwidth_; }
    float kp() const { return kp_; }
    float ki() const { return ki_; }
    uint32_t counts_per_revolution() const { return counts_per_rev_; }

    /**
     * @brief Two's complement 16-bit rollover safe subtraction helper.
     */
    static int16_t compute_timer_delta(uint16_t current_count, uint16_t previous_count) {
        return static_cast<int16_t>(current_count - previous_count);
    }

private:
    float pll_bandwidth_;
    float kp_;
    float ki_;
    uint32_t counts_per_rev_;
    float rad_per_count_;

    float pos_estimate_rad_;
    float vel_estimate_rad_s_;
    float pos_error_rad_;

    float time_since_last_pulse_sec_;
    static constexpr float kZeroSpeedTimeoutSec = 0.050F;  // 50 ms timeout

    uint16_t prev_timer_count_;
    bool timer_initialized_;
};

#endif  // ENCODER_PLL_H
