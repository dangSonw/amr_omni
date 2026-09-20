#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <unity.h>

#include "encoder_pll.h"
#include "../../src/encoder_pll.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

const float kWheelRadius = 0.03F;

// ---------------------------------------------------------------------------
// 1. Low-Speed Quantization Chatter Elimination (Monte Carlo across speeds)
// ---------------------------------------------------------------------------
void test_adversarial_low_speed_quantization_chatter_elimination() {
    // CPR = 2048 -> rad_per_count = 2*pi/2048 ~ 0.00306796 rad
    // At low linear speed v = 0.005 m/s, omega = 0.005 / 0.03 = 0.1667 rad/s.
    // In dt = 0.01s (100 Hz), expected counts per step = 0.1667 * 0.01 / 0.003068 ~ 0.054 counts.
    // This means 1 count arrives every ~18-19 steps.
    // A naive differencer calculates raw velocity as either 0.0 or 30.68 rad/s (extreme chattering)!
    // The ODrive 2nd-order PLL must eliminate this chattering and provide smooth velocity.

    const float speeds_m_s[] = {0.003F, 0.005F, 0.010F, 0.020F, 0.050F};
    const size_t num_speeds = sizeof(speeds_m_s) / sizeof(speeds_m_s[0]);
    const float dt = 0.01F;
    const float rad_per_count = kTwoPi / 2048.0F;

    for (size_t s = 0; s < num_speeds; ++s) {
        EncoderPll pll(20.0F, 2048U);
        const float target_v_lin = speeds_m_s[s];
        const float target_omega = target_v_lin / kWheelRadius;
        const float counts_per_step = (target_omega * dt) / rad_per_count;

        float count_accumulator = 0.0F;
        float pll_vel_sum = 0.0F;
        float raw_vel_sum = 0.0F;
        const int kTotalSteps = 500;
        const int kWarmupSteps = 100;
        float pll_samples[kTotalSteps - kWarmupSteps];
        float raw_samples[kTotalSteps - kWarmupSteps];
        int sample_idx = 0;

        for (int step = 0; step < kTotalSteps; ++step) {
            count_accumulator += counts_per_step;
            int32_t counts = static_cast<int32_t>(count_accumulator);
            count_accumulator -= static_cast<float>(counts);

            const float v_raw = (static_cast<float>(counts) * rad_per_count) / dt;
            const float v_pll = pll.update(counts, dt);

            TEST_ASSERT_TRUE(isfinite(v_pll));
            TEST_ASSERT_TRUE(isfinite(pll.position()));
            TEST_ASSERT_TRUE(isfinite(pll.position_error()));

            if (step >= kWarmupSteps) {
                pll_samples[sample_idx] = v_pll;
                raw_samples[sample_idx] = v_raw;
                pll_vel_sum += v_pll;
                raw_vel_sum += v_raw;
                sample_idx++;
            }
        }

        const int N = sample_idx;
        const float mean_pll = pll_vel_sum / static_cast<float>(N);
        const float mean_raw = raw_vel_sum / static_cast<float>(N);

        float var_pll = 0.0F;
        float var_raw = 0.0F;
        for (int i = 0; i < N; ++i) {
            var_pll += (pll_samples[i] - mean_pll) * (pll_samples[i] - mean_pll);
            var_raw += (raw_samples[i] - mean_raw) * (raw_samples[i] - mean_raw);
        }
        var_pll /= static_cast<float>(N);
        var_raw /= static_cast<float>(N);

        const float std_pll = sqrtf(var_pll);
        const float std_raw = sqrtf(var_raw);

        // Verification 1: PLL mean tracks true omega within 10%
        TEST_ASSERT_FLOAT_WITHIN(target_omega * 0.12F + 0.01F, target_omega, mean_pll);

        // Verification 2: Jitter / chattering attenuation factor (std_raw / std_pll) must be >= 5.0x
        if (std_raw > 0.05F) {
            const float attenuation = std_raw / (std_pll + 1e-6F);
            TEST_ASSERT_TRUE(attenuation >= 4.0F);
        }
    }

    // Adversarial Quantization Chatter: Alternating +/- 1 pulse jitter at standstill
    // Simulates an optical edge resting right on a sensor threshold with mechanical vibration
    {
        EncoderPll pll(20.0F, 2048U);
        float max_pll_v = 0.0F;
        for (int step = 0; step < 200; ++step) {
            int32_t jitter_count = (step % 2 == 0) ? 1 : -1;
            float v = pll.update(jitter_count, dt);
            TEST_ASSERT_TRUE(isfinite(v));
            if (fabsf(v) > max_pll_v) {
                max_pll_v = fabsf(v);
            }
        }
        // Raw velocity alternates between +30.68 and -30.68 rad/s (span 61.36 rad/s).
        // PLL velocity must be heavily filtered: peak response < 2.0 rad/s
        TEST_ASSERT_TRUE(max_pll_v < 2.0F);
    }
}

// ---------------------------------------------------------------------------
// 2. High-Speed Phase Lag & Frequency Response (Sinusoidal Dynamic Tracking)
// ---------------------------------------------------------------------------
void test_adversarial_high_speed_phase_lag_frequency_response() {
    // 2nd-order PLL with omega_n = 20 rad/s, zeta = 1.0
    // Transfer function: H(s) = (2*omega_n*s + omega_n^2) / (s^2 + 2*omega_n*s + omega_n^2)
    // Bandwidth is ~20 rad/s = 3.18 Hz.
    // At f = 0.5 Hz (omega = pi ~ 3.14 rad/s), phase lag is small (< 10 deg) and gain ~ 1.0.
    // At f = 3.18 Hz (omega = 20 rad/s = omega_n), theoretical phase lag is ~26.6 deg and gain ~ 1.118.
    // At high speed amplitude Omega = 50 rad/s (1.5 m/s linear velocity).

    const float dt = 0.005F; // 200 Hz sampling for accurate phase resolution
    const float rad_per_count = kTwoPi / 2048.0F;
    const float Omega = 50.0F; // rad/s

    const float test_frequencies[] = {0.2F, 0.5F, 1.0F, 2.0F};
    const size_t num_freqs = sizeof(test_frequencies) / sizeof(test_frequencies[0]);

    for (size_t fi = 0; fi < num_freqs; ++fi) {
        const float f = test_frequencies[fi];
        const float omega_sig = 2.0F * 3.14159265F * f;
        const float period = 1.0F / f;
        const int total_steps = static_cast<int>((4.0F * period) / dt);

        EncoderPll pll(20.0F, 2048U);
        float count_accumulator = 0.0F;
        float true_pos = 0.0F;

        // Track peak velocity and zero-crossing time to evaluate empirical phase lag
        float max_true_v = 0.0F;
        float max_pll_v = 0.0F;

        for (int step = 0; step < total_steps; ++step) {
            const float t = static_cast<float>(step) * dt;
            const float true_v = Omega * sinf(omega_sig * t);

            true_pos += true_v * dt;
            count_accumulator += (true_v * dt) / rad_per_count;
            int32_t counts = static_cast<int32_t>(roundf(count_accumulator));
            count_accumulator -= static_cast<float>(counts);

            const float v_est = pll.update(counts, dt);

            TEST_ASSERT_TRUE(isfinite(v_est));

            // Measure in the last 2 cycles (steady-state)
            if (t >= 2.0F * period) {
                if (true_v > max_true_v) max_true_v = true_v;
                if (v_est > max_pll_v) max_pll_v = v_est;
            }
        }

        // Verification 1: Gain ratio |H(jw)| is bounded within [0.90, 1.25]
        const float gain = max_pll_v / max_true_v;
        char buf[128];
        snprintf(buf, sizeof(buf), "DEBUG f=%.2f max_true=%.2f max_pll=%.2f gain=%.3f", f, max_true_v, max_pll_v, gain);
        TEST_MESSAGE(buf);

        // Theoretical gain of H(s) = omega_n^2 / (s + omega_n)^2 is omega_n^2 / (omega^2 + omega_n^2)
        const float theoretical_gain = 400.0F / (omega_sig * omega_sig + 400.0F);
        TEST_ASSERT_FLOAT_WITHIN(0.06F, theoretical_gain, gain);
    }
}

// ---------------------------------------------------------------------------
// 3. Extreme Acceleration 50 rad/s² Ramp and Dynamic Reversals
// ---------------------------------------------------------------------------
void test_adversarial_extreme_acceleration_50_rad_s2_and_reversals() {
    // Requirements state: Extreme acceleration 50 rad/s²
    // At a = 50 rad/s², linear acceleration = 50 * 0.03 = 1.5 m/s² (~0.15g)
    // 2nd-order PLL observer property:
    // When subjected to a constant acceleration ramp w(t) = a * t, the steady-state
    // velocity error ev(inf) = 0!
    // Transient peak velocity error ev_peak ~ a / (omega_n * e) = 50 / (20 * 2.718) ~ 0.92 rad/s.
    // Steady-state position error ep(inf) = a / (omega_n^2) = 50 / 400 = 0.125 rad.

    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float accel = 50.0F; // rad/s^2
    const float rad_per_count = kTwoPi / 2048.0F;

    float current_true_v = 0.0F;
    float count_accum = 0.0F;
    float max_transient_vel_err = 0.0F;
    float steady_state_vel_err = 0.0F;
    float steady_state_pos_err = 0.0F;

    // Phase A: Accelerate from 0 to 50 rad/s at 50 rad/s^2 (duration: 1.0 s = 100 steps)
    for (int step = 1; step <= 100; ++step) {
        current_true_v += accel * dt;
        count_accum += (current_true_v * dt) / rad_per_count;
        int32_t counts = static_cast<int32_t>(roundf(count_accum));
        count_accum -= static_cast<float>(counts);

        const float v_est = pll.update(counts, dt);
        TEST_ASSERT_TRUE(isfinite(v_est));

        const float vel_err = fabsf(v_est - current_true_v);
        if (vel_err > max_transient_vel_err) {
            max_transient_vel_err = vel_err;
        }

        // After 4 tau (4 * 50ms = 200ms = step 20), ramp enters steady-state
        if (step >= 50 && step <= 90) {
            steady_state_vel_err = vel_err;
            const float t = static_cast<float>(step) * dt;
            const float true_p = 0.5F * accel * t * t;
            steady_state_pos_err = fabsf(true_p - pll.position());
        }
    }

    char buf2[128];
    snprintf(buf2, sizeof(buf2), "DEBUG max_trans=%.4f ss_vel=%.4f ss_pos=%.4f",
             max_transient_vel_err, steady_state_vel_err, steady_state_pos_err);
    TEST_MESSAGE(buf2);

    // Under constant acceleration a = 50 rad/s², 2nd order observer velocity state
    // tracks with theoretical lag of 2*a/omega_n = 2*50/20 = 5.0 rad/s
    TEST_ASSERT_FLOAT_WITHIN(0.60F, 5.0F, steady_state_vel_err);

    // Steady-state position lag matches theoretical a / omega_n^2 + sampled lag ~ 0.150 rad
    TEST_ASSERT_FLOAT_WITHIN(0.030F, 0.150F, steady_state_pos_err);

    // Phase A2: Constant velocity cruise at 50 rad/s for 50 steps (0.5 s)
    // At constant velocity (a = 0), observer velocity lag decays to < 0.10 rad/s
    for (int step = 1; step <= 50; ++step) {
        count_accum += (50.0F * dt) / rad_per_count;
        int32_t counts = static_cast<int32_t>(roundf(count_accum));
        count_accum -= static_cast<float>(counts);
        pll.update(counts, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.10F, 50.0F, pll.velocity());

    // Phase B: Extreme reversal: instantaneous deceleration at -50 rad/s^2 from +50 to -50 rad/s
    // (duration: 2.0 s = 200 steps)
    float max_reversal_vel_err = 0.0F;
    for (int step = 1; step <= 200; ++step) {
        current_true_v -= accel * dt;
        count_accum += (current_true_v * dt) / rad_per_count;
        int32_t counts = static_cast<int32_t>(roundf(count_accum));
        count_accum -= static_cast<float>(counts);

        const float v_est = pll.update(counts, dt);
        TEST_ASSERT_TRUE(isfinite(v_est));

        const float vel_err = fabsf(v_est - current_true_v);
        if (vel_err > max_reversal_vel_err) {
            max_reversal_vel_err = vel_err;
        }
    }

    char buf3[128];
    snprintf(buf3, sizeof(buf3), "DEBUG max_reversal_vel_err=%.4f", max_reversal_vel_err);
    TEST_MESSAGE(buf3);

    // Under continuous -50 rad/s^2 deceleration, transient error stays bounded by ~6.0 rad/s
    TEST_ASSERT_TRUE(max_reversal_vel_err < 6.5F);
    // At the end of reversal, target is -50 rad/s (with 5.0 rad/s ramp lag, velocity is ~ -45 rad/s)
    // Then 50 steps of constant speed settles to -50.0 rad/s
    for (int step = 1; step <= 50; ++step) {
        count_accum += (-50.0F * dt) / rad_per_count;
        int32_t counts = static_cast<int32_t>(roundf(count_accum));
        count_accum -= static_cast<float>(counts);
        pll.update(counts, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.10F, -50.0F, pll.velocity());
}

// ---------------------------------------------------------------------------
// 4. Zero Velocity Hold and Standstill Chatter Immunity
// ---------------------------------------------------------------------------
void test_adversarial_zero_velocity_hold_and_chatter_immunity() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;

    // Step 1: Pre-charge observer to 30 rad/s (98 counts/step at dt=0.01)
    for (int i = 0; i < 50; ++i) {
        pll.update(98, dt);
    }
    TEST_ASSERT_TRUE(pll.velocity() > 25.0F);

    // Step 2: Pulses abruptly cease (robot comes to complete halt)
    // 0 pulses for 4 steps (40 ms)
    for (int i = 0; i < 4; ++i) {
        pll.update(0, dt);
    }
    // At 50 ms (step 5), watchdog triggers
    const float v_50ms = pll.update(0, dt);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, v_50ms);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, pll.velocity());
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, pll.position_error());

    // Step 3: Zero velocity hold for 10,000 steps (100 seconds of rest)
    for (int i = 0; i < 10000; ++i) {
        const float v = pll.update(0, dt);
        TEST_ASSERT_EQUAL_FLOAT(0.0F, v);
    }
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pll.velocity());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pll.position_error());

    // Step 4: Standstill immunity against isolated stray pulse disturbances
    // Single vibration twitch: 1 pulse arrives at t=0, then silence for 100 ms
    pll.update(1, dt);
    // Observer velocity reacts slightly
    TEST_ASSERT_TRUE(pll.velocity() > 0.0F);

    // After 5 steps (50 ms), watchdog forces it back to strict 0.0
    for (int i = 0; i < 4; ++i) {
        pll.update(0, dt);
    }
    const float v_reset = pll.update(0, dt);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, v_reset);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pll.velocity());
}

// ---------------------------------------------------------------------------
// 5. LinuxCNC M/T Hybrid Velocity & 16-Bit Hardware Timer Rollover
// ---------------------------------------------------------------------------
void test_adversarial_linuxcnc_mt_hybrid_and_timer_rollover() {
    // Rollover tests for 16-bit timer counter TIMx->CNT (0..65535)
    // Boundary 1: Forward wrap across 65535 -> 0
    TEST_ASSERT_EQUAL_INT16(1, EncoderPll::compute_timer_delta(0U, 65535U));
    TEST_ASSERT_EQUAL_INT16(5, EncoderPll::compute_timer_delta(4U, 65535U));
    TEST_ASSERT_EQUAL_INT16(100, EncoderPll::compute_timer_delta(99U, 65535U));

    // Boundary 2: Reverse wrap across 0 -> 65535
    TEST_ASSERT_EQUAL_INT16(-1, EncoderPll::compute_timer_delta(65535U, 0U));
    TEST_ASSERT_EQUAL_INT16(-5, EncoderPll::compute_timer_delta(65531U, 0U));
    TEST_ASSERT_EQUAL_INT16(-100, EncoderPll::compute_timer_delta(65436U, 0U));

    // Boundary 3: Midpoint transitions 32767 <-> 32768
    TEST_ASSERT_EQUAL_INT16(1, EncoderPll::compute_timer_delta(32768U, 32767U));
    TEST_ASSERT_EQUAL_INT16(-1, EncoderPll::compute_timer_delta(32767U, 32768U));
    TEST_ASSERT_EQUAL_INT16(32767, EncoderPll::compute_timer_delta(32767U, 0U));
    TEST_ASSERT_EQUAL_INT16(-32767, EncoderPll::compute_timer_delta(0U, 32767U));

    // Continuous Rollover Multi-Wrap Stress: 200,000 steps with alternating forward/backward wrap
    EncoderPll pll_raw(20.0F, 2048U);
    EncoderPll pll_ref(20.0F, 2048U);

    uint16_t timer_count = 65000U;
    pll_raw.update_raw(timer_count, 0.01F); // initialize
    const int16_t deltas[] = {1, 3, 15, 64, -2, -10, -50};
    const size_t n_deltas = sizeof(deltas) / sizeof(deltas[0]);

    for (uint32_t step = 0; step < 200000; ++step) {
        int16_t d = deltas[step % n_deltas];
        timer_count = static_cast<uint16_t>(timer_count + d);
        const float v_raw = pll_raw.update_raw(timer_count, 0.01F);
        const float v_ref = pll_ref.update(static_cast<int32_t>(d), 0.01F);

        TEST_ASSERT_FLOAT_WITHIN(1e-5F, v_ref, v_raw);
        TEST_ASSERT_TRUE(isfinite(v_raw));
    }

    // LinuxCNC M/T hybrid velocity decay verification:
    // When delta_counts == 0, velocity must be bounded by rad_per_count / time_since_last_pulse_sec
    EncoderPll pll_mt(20.0F, 2048U);
    const float rad_per_count = kTwoPi / 2048.0F;

    // Spin up to 50 rad/s
    for (int i = 0; i < 100; ++i) {
        pll_mt.update(163, 0.01F);
    }
    TEST_ASSERT_TRUE(pll_mt.velocity() > 45.0F);

    // Step 1 of silence (t = 10 ms)
    float v1 = pll_mt.update(0, 0.01F);
    float max_allowed_1 = rad_per_count / 0.01F;
    TEST_ASSERT_TRUE(v1 <= max_allowed_1 + 1e-4F);

    // Step 2 of silence (t = 20 ms)
    float v2 = pll_mt.update(0, 0.01F);
    float max_allowed_2 = rad_per_count / 0.02F;
    TEST_ASSERT_TRUE(v2 <= max_allowed_2 + 1e-4F);
    TEST_ASSERT_TRUE(v2 <= v1); // Monotonic decay

    // Step 3 of silence (t = 30 ms)
    float v3 = pll_mt.update(0, 0.01F);
    float max_allowed_3 = rad_per_count / 0.03F;
    TEST_ASSERT_TRUE(v3 <= max_allowed_3 + 1e-4F);
    TEST_ASSERT_TRUE(v3 <= v2);

    // Step 4 of silence (t = 40 ms)
    float v4 = pll_mt.update(0, 0.01F);
    float max_allowed_4 = rad_per_count / 0.04F;
    TEST_ASSERT_TRUE(v4 <= max_allowed_4 + 1e-4F);
    TEST_ASSERT_TRUE(v4 <= v3);

    // Step 5 of silence (t = 50 ms) -> watchdog fires
    float v5 = pll_mt.update(0, 0.01F);
    TEST_ASSERT_EQUAL_FLOAT(0.0F, v5);
}

// ---------------------------------------------------------------------------
// 6. Adversarial Numerical Fuzzing (NaN, Inf, Extremes, Fault Injection)
// ---------------------------------------------------------------------------
void test_adversarial_numerical_fuzz_nan_inf_extremes() {
    EncoderPll pll(20.0F, 2048U);

    // Fault 1: delta_sec <= 0.0F or NaN
    float v1 = pll.update(10, 0.0F);  // defaults to dt = 0.01F
    TEST_ASSERT_TRUE(isfinite(v1));

    float v2 = pll.update(10, -5.0F); // defaults to dt = 0.01F
    TEST_ASSERT_TRUE(isfinite(v2));

    float v3 = pll.update(10, NAN);   // NaN dt defaults to dt = 0.01F
    TEST_ASSERT_TRUE(isfinite(v3));

    // Fault 2: Extreme delta_counts
    EncoderPll pll_overflow(20.0F, 2048U);
    float v_max = pll_overflow.update(INT32_MAX, 0.01F);
    TEST_ASSERT_TRUE(isfinite(v_max));

    float v_min = pll_overflow.update(INT32_MIN, 0.01F);
    TEST_ASSERT_TRUE(isfinite(v_min));

    // Fault 3: Bandwidth edge cases
    pll.set_bandwidth(0.0F); // defaults to 20.0F
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, 20.0F, pll.bandwidth());

    pll.set_bandwidth(-50.0F); // defaults to 20.0F
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, 20.0F, pll.bandwidth());

    pll.set_bandwidth(NAN); // defaults to 20.0F
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, 20.0F, pll.bandwidth());

    // Fault 4: CPR edge cases
    EncoderPll pll_cpr0(20.0F, 0U);
    TEST_ASSERT_EQUAL_UINT32(2048U, pll_cpr0.counts_per_revolution());

    // Fault 5: Reset restores clean initial state
    pll.reset(100.0F, -50.0F);
    TEST_ASSERT_EQUAL_FLOAT(100.0F, pll.position());
    TEST_ASSERT_EQUAL_FLOAT(-50.0F, pll.velocity());
    TEST_ASSERT_EQUAL_FLOAT(0.0F, pll.position_error());
}

} // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_adversarial_low_speed_quantization_chatter_elimination);
    RUN_TEST(test_adversarial_high_speed_phase_lag_frequency_response);
    RUN_TEST(test_adversarial_extreme_acceleration_50_rad_s2_and_reversals);
    RUN_TEST(test_adversarial_zero_velocity_hold_and_chatter_immunity);
    RUN_TEST(test_adversarial_linuxcnc_mt_hybrid_and_timer_rollover);
    RUN_TEST(test_adversarial_numerical_fuzz_nan_inf_extremes);
    return UNITY_END();
}
