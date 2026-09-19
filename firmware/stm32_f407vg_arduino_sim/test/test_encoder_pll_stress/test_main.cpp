#include <math.h>
#include <unity.h>
#include <stdint.h>
#include <stdlib.h>

#include "encoder_pll.h"
#include "../../src/encoder_pll.cpp"

namespace {

// ---------------------------------------------------------------------------
// 1. Rapid Speed Reversals (-1.5 m/s to +1.5 m/s in 10 ms)
// ---------------------------------------------------------------------------
void test_stress_rapid_speed_reversal_dynamics() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float wheel_radius = 0.03F;
    const float target_speed = 1.5F / wheel_radius;  // 50.0 rad/s
    const float rad_per_count = 6.283185307F / 2048.0F;
    const float counts_target = (target_speed * dt) / rad_per_count;  // ~162.97 counts

    // Step A: Pre-condition at steady-state negative velocity (-50 rad/s)
    float acc_counts = 0.0F;
    for (int step = 0; step < 200; ++step) {
        acc_counts -= counts_target;
        int32_t c = static_cast<int32_t>(roundf(acc_counts));
        acc_counts -= static_cast<float>(c);
        pll.update(c, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0F, -target_speed, pll.velocity());

    // Step B: Instantaneous reversal in a single 10 ms step to +50 rad/s
    float max_tracking_err = 0.0F;
    float peak_overshoot = 0.0F;
    float settling_time_5pct_ms = -1.0F;
    float settling_time_2pct_ms = -1.0F;
    bool settled_5 = false;
    bool settled_2 = false;

    acc_counts = 0.0F;
    for (int step = 1; step <= 200; ++step) {
        const float elapsed_ms = static_cast<float>(step) * dt * 1000.0F;
        acc_counts += counts_target;
        int32_t c = static_cast<int32_t>(roundf(acc_counts));
        acc_counts -= static_cast<float>(c);

        const float v = pll.update(c, dt);
        TEST_ASSERT_TRUE(isfinite(v));
        TEST_ASSERT_TRUE(isfinite(pll.position()));
        TEST_ASSERT_TRUE(isfinite(pll.position_error()));

        const float err = fabsf(v - target_speed);
        if (err > max_tracking_err) {
            max_tracking_err = err;
        }
        if (v > target_speed && (v - target_speed) > peak_overshoot) {
            peak_overshoot = v - target_speed;
        }

        // Settling time 5% (|v - 50| <= 2.5 rad/s)
        if (err <= 0.05F * target_speed) {
            if (!settled_5) {
                settling_time_5pct_ms = elapsed_ms;
                settled_5 = true;
            }
        } else {
            settled_5 = false;
        }

        // Settling time 2% (|v - 50| <= 1.0 rad/s)
        if (err <= 0.02F * target_speed) {
            if (!settled_2) {
                settling_time_2pct_ms = elapsed_ms;
                settled_2 = true;
            }
        } else {
            settled_2 = false;
        }
    }

    // Critical damping verification: peak overshoot must be < 1% (ideal continuous = 0%)
    const float peak_overshoot_pct = (peak_overshoot / target_speed) * 100.0F;
    TEST_ASSERT_TRUE(peak_overshoot_pct < 1.0F);

    // Settling time must be within 350 ms for 20 rad/s bandwidth (tau = 1/20 = 50 ms)
    TEST_ASSERT_TRUE(settling_time_5pct_ms > 0.0F && settling_time_5pct_ms <= 350.0F);
    TEST_ASSERT_TRUE(settling_time_2pct_ms > 0.0F && settling_time_2pct_ms <= 450.0F);

    // Final steady-state tracking error < 0.5 rad/s (1%)
    TEST_ASSERT_FLOAT_WITHIN(0.5F, target_speed, pll.velocity());
}

// ---------------------------------------------------------------------------
// 2. High Pulse Jitter and Missing Pulse Trains
// ---------------------------------------------------------------------------
void test_stress_pulse_jitter_and_dropouts() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float target_speed = 30.0F;  // rad/s
    const float rad_per_count = 6.283185307F / 2048.0F;
    const float nom_counts = (target_speed * dt) / rad_per_count;  // ~97.78 counts

    srand(12345);
    const int N = 300;
    float raw_sum = 0.0F, pll_sum = 0.0F;
    float raw_arr[N], pll_arr[N];

    for (int i = 0; i < N; ++i) {
        const int jitter = (rand() % 31) - 15;  // +/- 15 counts (15% jitter)
        const int32_t counts = static_cast<int32_t>(roundf(nom_counts)) + jitter;
        const float v_raw = static_cast<float>(counts) * rad_per_count / dt;
        const float v_pll = pll.update(counts, dt);

        TEST_ASSERT_TRUE(isfinite(v_pll));
        raw_arr[i] = v_raw;
        pll_arr[i] = v_pll;
        if (i >= 50) {
            raw_sum += v_raw;
            pll_sum += v_pll;
        }
    }

    const int count_ss = N - 50;
    const float raw_mean = raw_sum / count_ss;
    const float pll_mean = pll_sum / count_ss;
    float raw_var = 0.0F, pll_var = 0.0F;
    for (int i = 50; i < N; ++i) {
        raw_var += (raw_arr[i] - raw_mean) * (raw_arr[i] - raw_mean);
        pll_var += (pll_arr[i] - pll_mean) * (pll_arr[i] - pll_mean);
    }
    const float raw_std = sqrtf(raw_var / count_ss);
    const float pll_std = sqrtf(pll_var / count_ss);

    // Jitter attenuation factor must be at least 3.0x
    const float attenuation = raw_std / (pll_std + 1e-6F);
    TEST_ASSERT_TRUE(attenuation >= 3.0F);

    // Test missing pulse burst: 3 dropped steps (30 ms), then 4x burst
    // At 10 ms (first drop), velocity is bounded by M/T decay envelope (> 0)
    const float v_drop1 = pll.update(0, dt);
    TEST_ASSERT_TRUE(v_drop1 > 0.0F);
    TEST_ASSERT_TRUE(v_drop1 <= (rad_per_count / dt) + 0.001F);

    // After 2nd and 3rd drop, velocity decays monotonically to 0
    pll.update(0, dt);
    const float v_drop3 = pll.update(0, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, v_drop3);

    // Buffer flushes: 4x counts arrive
    const int32_t burst_counts = static_cast<int32_t>(roundf(nom_counts * 4.0F));
    const float v_burst = pll.update(burst_counts, dt);
    TEST_ASSERT_TRUE(isfinite(v_burst));

    // Recovery within 250 ms
    float rec_time_ms = -1.0F;
    for (int step = 1; step <= 30; ++step) {
        const float v = pll.update(static_cast<int32_t>(roundf(nom_counts)), dt);
        if (fabsf(v - target_speed) <= 0.05F * target_speed && rec_time_ms < 0.0F) {
            rec_time_ms = step * dt * 1000.0F;
        }
    }
    TEST_ASSERT_TRUE(rec_time_ms > 0.0F && rec_time_ms <= 250.0F);
}

// ---------------------------------------------------------------------------
// 3. Ultra-Low Speeds (0.005 m/s) and Sparse Pulse Arrival
// ---------------------------------------------------------------------------
void test_stress_ultra_low_speed_and_sparse_pulses() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float wheel_radius = 0.03F;
    const float low_v = 0.005F;  // 5 mm/s
    const float target_omega = low_v / wheel_radius;  // ~0.16667 rad/s
    const float rad_per_count = 6.283185307F / 2048.0F;

    float acc_counts = 0.0F;
    float sum_pll = 0.0F;
    float sum_raw = 0.0F;
    const int N = 400;

    for (int step = 0; step < N; ++step) {
        acc_counts += (target_omega * dt) / rad_per_count;
        int32_t c = static_cast<int32_t>(acc_counts);
        acc_counts -= static_cast<float>(c);

        const float v_raw = static_cast<float>(c) * rad_per_count / dt;
        const float v_pll = pll.update(c, dt);

        TEST_ASSERT_TRUE(isfinite(v_pll));
        if (step >= 100) {
            sum_pll += v_pll;
            sum_raw += v_raw;
        }
    }

    const float mean_pll = sum_pll / (N - 100);
    // Estimated speed must be within 10% of true speed without quantization chattering
    TEST_ASSERT_FLOAT_WITHIN(0.02F, target_omega, mean_pll);

    // Test sparse arrival: 1 pulse every 2.0 seconds (200 steps at dt=0.01)
    EncoderPll pll_sparse(20.0F, 2048U);
    pll_sparse.update(1, dt);
    // After 5 steps (50 ms), watchdog must clamp to 0.0
    for (int step = 1; step < 5; ++step) {
        pll_sparse.update(0, dt);
    }
    const float v_50ms = pll_sparse.update(0, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, v_50ms);

    // Stays zero until next sparse pulse at step 200
    for (int step = 6; step < 200; ++step) {
        const float v = pll_sparse.update(0, dt);
        TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, v);
    }

    // Step 200: sparse pulse arrives
    const float v_next = pll_sparse.update(1, dt);
    TEST_ASSERT_TRUE(isfinite(v_next));
    TEST_ASSERT_TRUE(v_next > 0.0F);
}

// ---------------------------------------------------------------------------
// 4. 16-Bit Timer Overflow Stress (Boundaries and Continuous Rollover Ramp)
// ---------------------------------------------------------------------------
void test_stress_16bit_timer_rollover_exhaustive() {
    // 17 Boundary Vectors
    TEST_ASSERT_EQUAL_INT16(1, EncoderPll::compute_timer_delta(0U, 65535U));
    TEST_ASSERT_EQUAL_INT16(-1, EncoderPll::compute_timer_delta(65535U, 0U));
    TEST_ASSERT_EQUAL_INT16(16, EncoderPll::compute_timer_delta(10U, 65530U));
    TEST_ASSERT_EQUAL_INT16(-16, EncoderPll::compute_timer_delta(65530U, 10U));
    TEST_ASSERT_EQUAL_INT16(1, EncoderPll::compute_timer_delta(32767U, 32766U));
    TEST_ASSERT_EQUAL_INT16(1, EncoderPll::compute_timer_delta(32768U, 32767U));
    TEST_ASSERT_EQUAL_INT16(-1, EncoderPll::compute_timer_delta(32767U, 32768U));
    TEST_ASSERT_EQUAL_INT16(2, EncoderPll::compute_timer_delta(32769U, 32767U));
    TEST_ASSERT_EQUAL_INT16(-2, EncoderPll::compute_timer_delta(32767U, 32769U));
    TEST_ASSERT_EQUAL_INT16(0, EncoderPll::compute_timer_delta(0U, 0U));
    TEST_ASSERT_EQUAL_INT16(0, EncoderPll::compute_timer_delta(65535U, 65535U));
    TEST_ASSERT_EQUAL_INT16(0, EncoderPll::compute_timer_delta(32767U, 32767U));
    TEST_ASSERT_EQUAL_INT16(0, EncoderPll::compute_timer_delta(32768U, 32768U));
    TEST_ASSERT_EQUAL_INT16(32767, EncoderPll::compute_timer_delta(32767U, 0U));
    TEST_ASSERT_EQUAL_INT16(-32767, EncoderPll::compute_timer_delta(0U, 32767U));
    TEST_ASSERT_EQUAL_INT16(-32768, EncoderPll::compute_timer_delta(32768U, 0U));
    TEST_ASSERT_EQUAL_INT16(-32768, EncoderPll::compute_timer_delta(0U, 32768U));

    // Continuous Rollover Stress: 100,000 steps cycling through all 16-bit counts
    EncoderPll pll_raw(20.0F, 2048U);
    EncoderPll pll_ref(20.0F, 2048U);

    uint16_t timer_val = 65500U;
    pll_raw.update_raw(timer_val, 0.01F);
    const int16_t step_fwd = 37;

    for (uint32_t step = 0; step < 100000; ++step) {
        timer_val = static_cast<uint16_t>(timer_val + step_fwd);
        const float v_raw = pll_raw.update_raw(timer_val, 0.01F);
        const float v_ref = pll_ref.update(step_fwd, 0.01F);
        TEST_ASSERT_FLOAT_WITHIN(0.0001F, v_ref, v_raw);
    }
}

// ---------------------------------------------------------------------------
// 5. Zero-Speed Watchdog Activation and Rapid Recovery
// ---------------------------------------------------------------------------
void test_stress_zero_speed_watchdog_and_rapid_recovery() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float target_speed = 50.0F;  // rad/s
    const float rad_per_count = 6.283185307F / 2048.0F;
    const int32_t counts_50 = static_cast<int32_t>(roundf((target_speed * dt) / rad_per_count));

    // Spin up to 50 rad/s
    for (int i = 0; i < 100; ++i) {
        pll.update(counts_50, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0F, target_speed, pll.velocity());

    // Pulses stop abruptly: test LinuxCNC M/T decay envelope at 10, 20, 30, 40 ms
    const float v10 = pll.update(0, dt);
    TEST_ASSERT_TRUE(v10 <= (rad_per_count / 0.01F) + 0.001F);

    const float v20 = pll.update(0, dt);
    TEST_ASSERT_TRUE(v20 <= (rad_per_count / 0.02F) + 0.001F);

    pll.update(0, dt);  // 30 ms
    pll.update(0, dt);  // 40 ms

    // 50 ms: watchdog triggers and forces velocity to strictly 0.0
    const float v50 = pll.update(0, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.00001F, 0.0F, v50);
    TEST_ASSERT_FLOAT_WITHIN(0.00001F, 0.0F, pll.velocity());
    TEST_ASSERT_FLOAT_WITHIN(0.00001F, 0.0F, pll.position_error());

    // Rapid recovery: pulses resume at 50 rad/s
    float rec_time_90_ms = -1.0F;
    float rec_time_95_ms = -1.0F;
    float peak_v = 0.0F;

    for (int step = 1; step <= 50; ++step) {
        const float elapsed_ms = step * dt * 1000.0F;
        const float v = pll.update(counts_50, dt);

        if (v > peak_v) peak_v = v;
        if (v >= 0.90F * target_speed && rec_time_90_ms < 0.0F) {
            rec_time_90_ms = elapsed_ms;
        }
        if (v >= 0.95F * target_speed && rec_time_95_ms < 0.0F) {
            rec_time_95_ms = elapsed_ms;
        }
    }

    TEST_ASSERT_TRUE(rec_time_90_ms > 0.0F && rec_time_90_ms <= 220.0F);
    TEST_ASSERT_TRUE(rec_time_95_ms > 0.0F && rec_time_95_ms <= 270.0F);

    // Recovery overshoot < 0.5%
    const float overshoot_pct = (peak_v > target_speed) ? ((peak_v - target_speed) / target_speed * 100.0F) : 0.0F;
    TEST_ASSERT_TRUE(overshoot_pct < 0.5F);
}

// ---------------------------------------------------------------------------
// 6. Long-Duration Numerical Drift Test (100,000 steps)
// ---------------------------------------------------------------------------
void test_stress_long_duration_numerical_drift() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float target_speed = 50.0F;  // rad/s
    const float rad_per_count = 6.283185307F / 2048.0F;
    const float counts_step = (target_speed * dt) / rad_per_count;

    float frac_counts = 0.0F;
    float max_v_err = 0.0F;
    float max_pos_err = 0.0F;

    for (uint32_t step = 0; step < 100000; ++step) {
        frac_counts += counts_step;
        int32_t c = static_cast<int32_t>(frac_counts);
        frac_counts -= static_cast<float>(c);

        const float v = pll.update(c, dt);

        TEST_ASSERT_TRUE(isfinite(v));
        TEST_ASSERT_TRUE(isfinite(pll.position()));
        TEST_ASSERT_TRUE(isfinite(pll.position_error()));

        if (step >= 500) {
            const float err_v = fabsf(v - target_speed);
            if (err_v > max_v_err) max_v_err = err_v;

            const float err_pos = fabsf(pll.position_error());
            if (err_pos > max_pos_err) max_pos_err = err_pos;
        }
    }

    // Over 100,000 steps (16.67 minutes):
    // Max steady-state velocity error must remain < 0.05 rad/s (0.1%)
    TEST_ASSERT_TRUE(max_v_err < 0.05F);
    // Position error remains bounded within 0.01 rad (sub-pulse)
    TEST_ASSERT_TRUE(max_pos_err < 0.01F);
    // Final velocity is within 0.01 rad/s of target
    TEST_ASSERT_FLOAT_WITHIN(0.01F, target_speed, pll.velocity());
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stress_rapid_speed_reversal_dynamics);
    RUN_TEST(test_stress_pulse_jitter_and_dropouts);
    RUN_TEST(test_stress_ultra_low_speed_and_sparse_pulses);
    RUN_TEST(test_stress_16bit_timer_rollover_exhaustive);
    RUN_TEST(test_stress_zero_speed_watchdog_and_rapid_recovery);
    RUN_TEST(test_stress_long_duration_numerical_drift);
    return UNITY_END();
}
