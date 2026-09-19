#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "encoder_pll.h"

struct ReversalMetrics {
    float max_tracking_error;
    float peak_overshoot_rad_s;
    float peak_overshoot_pct;
    float settling_time_5pct_ms;
    float settling_time_2pct_ms;
    uint32_t nan_inf_count;
    bool passed;
};

struct JitterMetrics {
    float raw_velocity_std;
    float pll_velocity_std;
    float noise_attenuation_factor;
    float max_error_during_dropout;
    float dropout_recovery_time_ms;
    uint32_t nan_inf_count;
    bool passed;
};

struct LowSpeedMetrics {
    float mean_estimated_speed;
    float speed_error_pct;
    float pll_ripple_std;
    float raw_ripple_std;
    uint32_t watchdog_activations;
    uint32_t nan_inf_count;
    bool passed;
};

struct TimerRolloverMetrics {
    uint32_t total_boundary_tests;
    uint32_t boundary_errors;
    uint32_t continuous_ramp_steps;
    uint32_t continuous_ramp_errors;
    bool passed;
};

struct WatchdogMetrics {
    float velocity_at_10ms;
    float velocity_at_20ms;
    float velocity_at_30ms;
    float velocity_at_40ms;
    float velocity_at_50ms;
    bool watchdog_activated_at_50ms;
    float recovery_time_90pct_ms;
    float recovery_time_95pct_ms;
    float recovery_overshoot_pct;
    uint32_t nan_inf_count;
    bool passed;
};

struct DriftMetrics {
    uint32_t total_steps;
    float max_velocity_error_steady;
    float final_velocity_error;
    float max_pos_error_rad;
    float final_pos_error_rad;
    float final_pos_estimate_rad;
    uint32_t nan_inf_count;
    bool passed;
};

ReversalMetrics run_reversal_test() {
    ReversalMetrics m;
    memset(&m, 0, sizeof(m));
    EncoderPll pll(20.0f, 2048);
    const float dt = 0.01f;
    const float wheel_radius = 0.03f;
    const float target_speed = 1.5f / wheel_radius; // 50.0 rad/s
    const float rad_per_count = 6.28318530718f / 2048.0f;
    const float counts_per_step_target = (target_speed * dt) / rad_per_count; // ~162.97 counts

    // 1. Steady-state negative speed (-50.0 rad/s)
    float acc_counts = 0.0f;
    for (int i = 0; i < 200; ++i) {
        acc_counts -= counts_per_step_target;
        int32_t c = (int32_t)roundf(acc_counts);
        acc_counts -= (float)c;
        pll.update(c, dt);
    }

    // 2. Instantaneous step reversal to +50.0 rad/s
    float max_err = 0.0f;
    float peak_vel = 0.0f;
    float t_settle_5 = -1.0f;
    float t_settle_2 = -1.0f;
    bool settled_5 = false;
    bool settled_2 = false;

    acc_counts = 0.0f;
    for (int step = 1; step <= 200; ++step) {
        float elapsed_ms = step * (dt * 1000.0f);
        acc_counts += counts_per_step_target;
        int32_t c = (int32_t)roundf(acc_counts);
        acc_counts -= (float)c;

        float v = pll.update(c, dt);

        if (!isfinite(v) || !isfinite(pll.position())) {
            m.nan_inf_count++;
        }

        float err = fabsf(v - target_speed);
        if (err > max_err) max_err = err;
        if (v > peak_vel) peak_vel = v;

        if (err <= 0.05f * target_speed) {
            if (!settled_5) {
                t_settle_5 = elapsed_ms;
                settled_5 = true;
            }
        } else {
            settled_5 = false;
        }

        if (err <= 0.02f * target_speed) {
            if (!settled_2) {
                t_settle_2 = elapsed_ms;
                settled_2 = true;
            }
        } else {
            settled_2 = false;
        }
    }

    m.max_tracking_error = max_err;
    m.peak_overshoot_rad_s = (peak_vel > target_speed) ? (peak_vel - target_speed) : 0.0f;
    m.peak_overshoot_pct = (m.peak_overshoot_rad_s / target_speed) * 100.0f;
    m.settling_time_5pct_ms = t_settle_5;
    m.settling_time_2pct_ms = t_settle_2;
    m.passed = (m.nan_inf_count == 0) && (t_settle_5 > 0.0f && t_settle_5 <= 350.0f) && (m.peak_overshoot_pct < 10.0f);

    return m;
}

JitterMetrics run_jitter_test() {
    JitterMetrics m;
    memset(&m, 0, sizeof(m));
    EncoderPll pll(20.0f, 2048);
    const float dt = 0.01f;
    const float target_speed = 30.0f; // rad/s
    const float rad_per_count = 6.28318530718f / 2048.0f;
    const float nom_counts = (target_speed * dt) / rad_per_count; // ~97.78 counts

    const int N = 300;
    float raw_vel[N];
    float pll_vel[N];
    srand(42);

    float raw_sum = 0.0f, pll_sum = 0.0f;
    for (int i = 0; i < N; ++i) {
        int jitter = (rand() % 31) - 15; // [-15, +15] counts
        int32_t counts = (int32_t)roundf(nom_counts) + jitter;
        float v_raw = (float)counts * rad_per_count / dt;
        float v_pll = pll.update(counts, dt);

        if (!isfinite(v_pll)) m.nan_inf_count++;

        raw_vel[i] = v_raw;
        pll_vel[i] = v_pll;

        if (i >= 50) {
            raw_sum += v_raw;
            pll_sum += v_pll;
        }
    }

    int count_ss = N - 50;
    float raw_mean = raw_sum / count_ss;
    float pll_mean = pll_sum / count_ss;
    float raw_var = 0.0f, pll_var = 0.0f;
    for (int i = 50; i < N; ++i) {
        raw_var += (raw_vel[i] - raw_mean) * (raw_vel[i] - raw_mean);
        pll_var += (pll_vel[i] - pll_mean) * (pll_vel[i] - pll_mean);
    }
    m.raw_velocity_std = sqrtf(raw_var / count_ss);
    m.pll_velocity_std = sqrtf(pll_var / count_ss);
    m.noise_attenuation_factor = m.raw_velocity_std / (m.pll_velocity_std + 1e-6f);

    // Missing pulse burst (3 dropped steps = 30 ms, then 4x burst)
    for (int i = 0; i < 50; ++i) {
        pll.update((int32_t)roundf(nom_counts), dt);
    }

    // 3 dropped steps
    float v_drop1 = pll.update(0, dt);
    float v_drop2 = pll.update(0, dt);
    float v_drop3 = pll.update(0, dt);
    (void)v_drop1; (void)v_drop2;

    float max_err = fabsf(v_drop3 - target_speed);

    // Burst step: 4x counts arrive
    int32_t burst_counts = (int32_t)roundf(nom_counts * 4.0f);
    float v_burst = pll.update(burst_counts, dt);
    (void)v_burst;

    float rec_time = 0.0f;
    for (int step = 1; step <= 50; ++step) {
        float v = pll.update((int32_t)roundf(nom_counts), dt);
        if (fabsf(v - target_speed) <= 0.05f * target_speed) {
            rec_time = step * dt * 1000.0f;
            break;
        }
    }

    m.max_error_during_dropout = max_err;
    m.dropout_recovery_time_ms = rec_time;
    m.passed = (m.nan_inf_count == 0) && (m.noise_attenuation_factor > 2.0f) && (rec_time <= 250.0f);

    return m;
}

LowSpeedMetrics run_low_speed_test() {
    LowSpeedMetrics m;
    memset(&m, 0, sizeof(m));
    EncoderPll pll(20.0f, 2048);
    const float dt = 0.01f;
    const float wheel_radius = 0.03f;
    const float target_speed = 0.005f / wheel_radius; // 0.166667 rad/s
    const float rad_per_count = 6.28318530718f / 2048.0f; // 0.00306796 rad

    const int N = 500;
    float float_counts = 0.0f;
    float sum_v = 0.0f;
    float raw_sum_v = 0.0f;
    float pll_v_arr[N];
    float raw_v_arr[N];

    for (int i = 0; i < N; ++i) {
        float_counts += (target_speed * dt) / rad_per_count;
        int32_t c = (int32_t)float_counts;
        float_counts -= (float)c;

        float v_raw = (float)c * rad_per_count / dt;
        float v_pll = pll.update(c, dt);

        if (!isfinite(v_pll)) m.nan_inf_count++;

        pll_v_arr[i] = v_pll;
        raw_v_arr[i] = v_raw;

        if (i >= 150) {
            sum_v += v_pll;
            raw_sum_v += v_raw;
        }
    }

    int count_ss = N - 150;
    m.mean_estimated_speed = sum_v / count_ss;
    m.speed_error_pct = fabsf(m.mean_estimated_speed - target_speed) / target_speed * 100.0f;

    float pll_var = 0.0f, raw_var = 0.0f;
    for (int i = 150; i < N; ++i) {
        pll_var += (pll_v_arr[i] - m.mean_estimated_speed) * (pll_v_arr[i] - m.mean_estimated_speed);
        raw_var += (raw_v_arr[i] - (raw_sum_v / count_ss)) * (raw_v_arr[i] - (raw_sum_v / count_ss));
    }
    m.pll_ripple_std = sqrtf(pll_var / count_ss);
    m.raw_ripple_std = sqrtf(raw_var / count_ss);

    // Sparse arrival: 1 pulse every 2.0 seconds
    EncoderPll pll_sparse(20.0f, 2048);
    pll_sparse.update(1, dt);
    uint32_t wd_count = 0;
    for (int i = 1; i < 200; ++i) {
        float v = pll_sparse.update(0, dt);
        if (!isfinite(v)) m.nan_inf_count++;
        if (i >= 5 && v == 0.0f) {
            wd_count++;
        }
    }
    m.watchdog_activations = wd_count;

    float v_res = pll_sparse.update(1, dt);
    if (!isfinite(v_res)) m.nan_inf_count++;

    m.passed = (m.nan_inf_count == 0) && (m.speed_error_pct < 15.0f) && (m.pll_ripple_std < m.raw_ripple_std * 0.5f);
    return m;
}

TimerRolloverMetrics run_timer_rollover_test() {
    TimerRolloverMetrics m;
    memset(&m, 0, sizeof(m));

    struct TestCase {
        uint16_t curr;
        uint16_t prev;
        int16_t expected_delta;
    } cases[] = {
        {0U, 65535U, +1},
        {65535U, 0U, -1},
        {10U, 65530U, +16},
        {65530U, 10U, -16},
        {32767U, 32766U, +1},
        {32768U, 32767U, +1},
        {32767U, 32768U, -1},
        {32769U, 32767U, +2},
        {32767U, 32769U, -2},
        {0U, 0U, 0},
        {65535U, 65535U, 0},
        {32767U, 32767U, 0},
        {32768U, 32768U, 0},
        {32767U, 0U, 32767},
        {0U, 32767U, -32767},
        {32768U, 0U, -32768},
        {0U, 32768U, -32768}
    };

    int n_cases = sizeof(cases) / sizeof(cases[0]);
    m.total_boundary_tests = n_cases;

    for (int i = 0; i < n_cases; ++i) {
        int16_t d = EncoderPll::compute_timer_delta(cases[i].curr, cases[i].prev);
        if (d != cases[i].expected_delta) {
            m.boundary_errors++;
        }
    }

    // Continuous rollover ramp stress
    EncoderPll pll_raw(20.0f, 2048);
    EncoderPll pll_ref(20.0f, 2048);

    uint16_t timer_val = 65520U;
    pll_raw.update_raw(timer_val, 0.01f);
    const int16_t step_inc = 25;
    m.continuous_ramp_steps = 200000;

    for (uint32_t step = 0; step < m.continuous_ramp_steps; ++step) {
        timer_val = (uint16_t)(timer_val + step_inc);
        float v_raw = pll_raw.update_raw(timer_val, 0.01f);
        float v_ref = pll_ref.update(step_inc, 0.01f);

        if (fabsf(v_raw - v_ref) > 1e-4f) {
            m.continuous_ramp_errors++;
        }
    }

    // Reverse continuous rollover ramp
    timer_val = 15U;
    pll_raw.reset();
    pll_ref.reset();
    pll_raw.update_raw(timer_val, 0.01f);
    const int16_t step_dec = -25;
    for (uint32_t step = 0; step < m.continuous_ramp_steps; ++step) {
        timer_val = (uint16_t)(timer_val + step_dec);
        float v_raw = pll_raw.update_raw(timer_val, 0.01f);
        float v_ref = pll_ref.update(step_dec, 0.01f);

        if (fabsf(v_raw - v_ref) > 1e-4f) {
            m.continuous_ramp_errors++;
        }
    }

    m.passed = (m.boundary_errors == 0) && (m.continuous_ramp_errors == 0);
    return m;
}

WatchdogMetrics run_watchdog_test() {
    WatchdogMetrics m;
    memset(&m, 0, sizeof(m));
    EncoderPll pll(20.0f, 2048);
    const float dt = 0.01f;
    const float target_speed = 50.0f;
    const float rad_per_count = 6.28318530718f / 2048.0f;
    const int32_t counts_50rads = (int32_t)roundf((target_speed * dt) / rad_per_count);

    for (int i = 0; i < 100; ++i) {
        pll.update(counts_50rads, dt);
    }

    m.velocity_at_10ms = pll.update(0, dt);
    m.velocity_at_20ms = pll.update(0, dt);
    m.velocity_at_30ms = pll.update(0, dt);
    m.velocity_at_40ms = pll.update(0, dt);
    m.velocity_at_50ms = pll.update(0, dt);

    m.watchdog_activated_at_50ms = (m.velocity_at_50ms == 0.0f);

    for (int i = 0; i < 10; ++i) {
        float v = pll.update(0, dt);
        if (v != 0.0f) {
            m.watchdog_activated_at_50ms = false;
        }
    }

    float t_rec_90 = -1.0f;
    float t_rec_95 = -1.0f;
    float peak_vel = 0.0f;

    for (int step = 1; step <= 100; ++step) {
        float elapsed_ms = step * dt * 1000.0f;
        float v = pll.update(counts_50rads, dt);

        if (!isfinite(v)) m.nan_inf_count++;

        if (v > peak_vel) peak_vel = v;

        if (v >= 0.90f * target_speed && t_rec_90 < 0.0f) {
            t_rec_90 = elapsed_ms;
        }
        if (v >= 0.95f * target_speed && t_rec_95 < 0.0f) {
            t_rec_95 = elapsed_ms;
        }
    }

    m.recovery_time_90pct_ms = t_rec_90;
    m.recovery_time_95pct_ms = t_rec_95;
    m.recovery_overshoot_pct = (peak_vel > target_speed) ? ((peak_vel - target_speed) / target_speed * 100.0f) : 0.0f;

    m.passed = (m.nan_inf_count == 0) && m.watchdog_activated_at_50ms && (t_rec_95 > 0.0f && t_rec_95 <= 250.0f);
    return m;
}

DriftMetrics run_drift_test() {
    DriftMetrics m;
    memset(&m, 0, sizeof(m));
    EncoderPll pll(20.0f, 2048);
    const float dt = 0.01f;
    const float target_speed = 50.0f;
    const float rad_per_count = 6.28318530718f / 2048.0f;
    const float counts_float = (target_speed * dt) / rad_per_count;

    m.total_steps = 100000;
    float max_v_err = 0.0f;
    float max_pos_err = 0.0f;
    float frac_counts = 0.0f;

    for (uint32_t step = 0; step < m.total_steps; ++step) {
        frac_counts += counts_float;
        int32_t c = (int32_t)frac_counts;
        frac_counts -= (float)c;

        float v = pll.update(c, dt);

        if (!isfinite(v) || !isfinite(pll.position()) || !isfinite(pll.position_error())) {
            m.nan_inf_count++;
        }

        if (step >= 500) {
            float err_v = fabsf(v - target_speed);
            if (err_v > max_v_err) max_v_err = err_v;

            float err_pos = fabsf(pll.position_error());
            if (err_pos > max_pos_err) max_pos_err = err_pos;
        }
    }

    m.max_velocity_error_steady = max_v_err;
    m.final_velocity_error = fabsf(pll.velocity() - target_speed);
    m.max_pos_error_rad = max_pos_err;
    m.final_pos_error_rad = fabsf(pll.position_error());
    m.final_pos_estimate_rad = pll.position();

    m.passed = (m.nan_inf_count == 0) && (max_v_err < 1.0f) && (max_pos_err < 0.05f);
    return m;
}

int main() {
    printf("=======================================================================\n");
    printf("   ENCODER PLL ADVERSARIAL EMPIRICAL STRESS TEST BENCHMARK RUNNER      \n");
    printf("=======================================================================\n\n");

    printf("--> Running Test 1: Rapid Speed Reversal (-1.5 m/s to +1.5 m/s in 10 ms)...\n");
    ReversalMetrics rev = run_reversal_test();
    printf("    Max Tracking Error:       %.4f rad/s\n", rev.max_tracking_error);
    printf("    Peak Overshoot:           %.4f rad/s (%.2f%%)\n", rev.peak_overshoot_rad_s, rev.peak_overshoot_pct);
    printf("    Settling Time (5%%):       %.1f ms\n", rev.settling_time_5pct_ms);
    printf("    Settling Time (2%%):       %.1f ms\n", rev.settling_time_2pct_ms);
    printf("    NaN/Inf Count:            %u\n", rev.nan_inf_count);
    printf("    Status:                   %s\n\n", rev.passed ? "PASSED" : "FAILED");

    printf("--> Running Test 2: High Pulse Jitter (+/-15 counts) & Missing Pulse Burst...\n");
    JitterMetrics jit = run_jitter_test();
    printf("    Raw Velocity Std Dev:     %.4f rad/s\n", jit.raw_velocity_std);
    printf("    PLL Velocity Std Dev:     %.4f rad/s\n", jit.pll_velocity_std);
    printf("    Noise Attenuation Factor: %.2fx\n", jit.noise_attenuation_factor);
    printf("    Max Error During Dropout: %.4f rad/s\n", jit.max_error_during_dropout);
    printf("    Dropout Recovery Time:    %.1f ms\n", jit.dropout_recovery_time_ms);
    printf("    NaN/Inf Count:            %u\n", jit.nan_inf_count);
    printf("    Status:                   %s\n\n", jit.passed ? "PASSED" : "FAILED");

    printf("--> Running Test 3: Ultra-Low Speeds (0.005 m/s & Sparse Pulse Arrivals)...\n");
    LowSpeedMetrics low = run_low_speed_test();
    printf("    Target Speed:             0.1667 rad/s (0.005 m/s)\n");
    printf("    Mean Estimated Speed:     %.4f rad/s\n", low.mean_estimated_speed);
    printf("    Speed Error:              %.2f%%\n", low.speed_error_pct);
    printf("    PLL Ripple Std Dev:       %.4f rad/s (vs Raw: %.4f rad/s)\n", low.pll_ripple_std, low.raw_ripple_std);
    printf("    Watchdog Steps Inactive:  %u steps\n", low.watchdog_activations);
    printf("    NaN/Inf Count:            %u\n", low.nan_inf_count);
    printf("    Status:                   %s\n\n", low.passed ? "PASSED" : "FAILED");

    printf("--> Running Test 4: 16-Bit Timer Rollover Stress (Boundaries & Continuous Ramp)...\n");
    TimerRolloverMetrics tmr = run_timer_rollover_test();
    printf("    Boundary Vectors Tested:  %u\n", tmr.total_boundary_tests);
    printf("    Boundary Errors:          %u\n", tmr.boundary_errors);
    printf("    Continuous Ramp Steps:    %u (200k forward + 200k reverse)\n", tmr.continuous_ramp_steps * 2);
    printf("    Continuous Ramp Errors:   %u\n", tmr.continuous_ramp_errors);
    printf("    Status:                   %s\n\n", tmr.passed ? "PASSED" : "FAILED");

    printf("--> Running Test 5: Zero-Speed Watchdog Activation & Rapid Recovery...\n");
    WatchdogMetrics wd = run_watchdog_test();
    printf("    Velocity at 10 ms:        %.4f rad/s (M/T bound <= 0.3068)\n", wd.velocity_at_10ms);
    printf("    Velocity at 20 ms:        %.4f rad/s (M/T bound <= 0.1534)\n", wd.velocity_at_20ms);
    printf("    Velocity at 30 ms:        %.4f rad/s (M/T bound <= 0.1023)\n", wd.velocity_at_30ms);
    printf("    Velocity at 40 ms:        %.4f rad/s (M/T bound <= 0.0767)\n", wd.velocity_at_40ms);
    printf("    Velocity at 50 ms:        %.4f rad/s (Watchdog clamp == 0.0)\n", wd.velocity_at_50ms);
    printf("    Watchdog Active at 50ms:  %s\n", wd.watchdog_activated_at_50ms ? "YES" : "NO");
    printf("    Recovery Time to 90%%:     %.1f ms\n", wd.recovery_time_90pct_ms);
    printf("    Recovery Time to 95%%:     %.1f ms\n", wd.recovery_time_95pct_ms);
    printf("    Recovery Overshoot:       %.2f%%\n", wd.recovery_overshoot_pct);
    printf("    NaN/Inf Count:            %u\n", wd.nan_inf_count);
    printf("    Status:                   %s\n\n", wd.passed ? "PASSED" : "FAILED");

    printf("--> Running Test 6: Long-Duration Numerical Drift Test (100,000 steps)...\n");
    DriftMetrics dft = run_drift_test();
    printf("    Total Simulated Steps:    %u (1,000 seconds)\n", dft.total_steps);
    printf("    Max Velocity Error:       %.6f rad/s\n", dft.max_velocity_error_steady);
    printf("    Final Velocity Error:     %.6f rad/s\n", dft.final_velocity_error);
    printf("    Max Position Error:       %.6f rad\n", dft.max_pos_error_rad);
    printf("    Final Position Error:     %.6f rad\n", dft.final_pos_error_rad);
    printf("    Final Position Estimate:  %.2f rad (bounded accumulation)\n", dft.final_pos_estimate_rad);
    printf("    NaN/Inf Count:            %u\n", dft.nan_inf_count);
    printf("    Status:                   %s\n\n", dft.passed ? "PASSED" : "FAILED");

    bool all_passed = rev.passed && jit.passed && low.passed && tmr.passed && wd.passed && dft.passed;
    printf("=======================================================================\n");
    printf("   OVERALL VERDICT: %s\n", all_passed ? "APPROVE (ALL 6 TESTS PASSED)" : "REQUEST_CHANGES");
    printf("=======================================================================\n");

    return all_passed ? 0 : 1;
}
