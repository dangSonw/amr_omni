#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <unity.h>

#include "firmware_config.h"
#include "imu_calibration.h"

#include "../../src/imu_calibration.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

// Deterministic LCG random number generator for reproducible native embedded tests
uint32_t lcg_state = 123456789U;

float lcg_uniform(float min_val, float max_val) {
    lcg_state = lcg_state * 1664525U + 1013904223U;
    const float norm = static_cast<float>(lcg_state & 0x00FFFFFFU) / static_cast<float>(0x00FFFFFFU);
    return min_val + norm * (max_val - min_val);
}

float lcg_gaussian(float mean, float std_dev) {
    // Box-Muller transform
    const float u1 = lcg_uniform(1e-6F, 1.0F);
    const float u2 = lcg_uniform(0.0F, 6.2831853F);
    const float z0 = sqrtf(-2.0F * logf(u1)) * cosf(u2);
    return mean + z0 * std_dev;
}

// ---------------------------------------------------------------------------
// 1. Residual Static Drift < 0.05 deg/s across 1,000 Randomized Bias Vectors
// ---------------------------------------------------------------------------
void test_stress_1000_random_bias_vectors_residual_drift() {
    lcg_state = 424242U;
    const uint32_t num_trials = 1000U;
    uint32_t passed_trials = 0U;

    for (uint32_t trial = 0U; trial < num_trials; ++trial) {
        ImuCalibrator cal;
        TEST_ASSERT_TRUE(cal.start_gyro_calibration(1000U));

        const float true_bias[3] = {
            lcg_uniform(-0.3F, 0.3F),
            lcg_uniform(-0.3F, 0.3F),
            lcg_uniform(-0.3F, 0.3F)
        };

        for (uint32_t s = 0U; s < 1000U; ++s) {
            const float sample[3] = {
                true_bias[0] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[1] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[2] + lcg_gaussian(0.0F, 0.0012F)
            };
            cal.update_gyro_sample(sample);
        }

        float reported_drift = 0.0F;
        const bool finish_ok = cal.finish_gyro_calibration(reported_drift);
        TEST_ASSERT_TRUE(finish_ok);

        const ImuCalibrationParams &params = cal.get_params();
        TEST_ASSERT_TRUE(params.gyro_calibrated);

        const float ex = params.gyro_bias[0] - true_bias[0];
        const float ey = params.gyro_bias[1] - true_bias[1];
        const float ez = params.gyro_bias[2] - true_bias[2];
        const float error_norm = sqrtf(ex * ex + ey * ey + ez * ez);

        if (reported_drift < kMaxGyroDriftRadS && error_norm < kMaxGyroDriftRadS) {
            passed_trials++;
        }
    }

    TEST_ASSERT_EQUAL_UINT32(num_trials, passed_trials);
}

// ---------------------------------------------------------------------------
// 2. Long-Term 60-Second Heading Integration Drift < 3.0 Degrees
// ---------------------------------------------------------------------------
void test_stress_60s_yaw_integration_heading_stability() {
    lcg_state = 777777U;
    const uint32_t num_trials = 50U;
    const float dt = 0.01F; // 100 Hz
    const uint32_t sim_steps = 6000U; // 60s
    const float rad_to_deg = 180.0F / 3.14159265F;

    for (uint32_t trial = 0U; trial < num_trials; ++trial) {
        ImuCalibrator cal;
        cal.start_gyro_calibration(1000U);

        const float true_bias[3] = {
            lcg_uniform(-0.15F, 0.15F),
            lcg_uniform(-0.15F, 0.15F),
            lcg_uniform(-0.15F, 0.15F)
        };

        // Calibration phase
        for (uint32_t s = 0U; s < 1000U; ++s) {
            const float sample[3] = {
                true_bias[0] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[1] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[2] + lcg_gaussian(0.0F, 0.0012F)
            };
            cal.update_gyro_sample(sample);
        }

        float res_drift = 0.0F;
        TEST_ASSERT_TRUE(cal.finish_gyro_calibration(res_drift));

        // 60-second resting integration phase
        float cal_yaw = 0.0F;
        const float dummy_a[3] = {0.0F, 0.0F, kStandardGravityMps2};
        float out_a[3] = {0};
        float out_g[3] = {0};

        for (uint32_t step = 0U; step < sim_steps; ++step) {
            const float raw_g[3] = {
                true_bias[0] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[1] + lcg_gaussian(0.0F, 0.0012F),
                true_bias[2] + lcg_gaussian(0.0F, 0.0012F)
            };
            cal.apply(dummy_a, raw_g, out_a, out_g);
            cal_yaw += out_g[2] * dt;
        }

        const float heading_drift_deg = fabsf(cal_yaw) * rad_to_deg;
        TEST_ASSERT_TRUE(heading_drift_deg < 3.0F);
    }
}

// ---------------------------------------------------------------------------
// 3. Motion Disturbance Rejection Across Temporal Windows
// ---------------------------------------------------------------------------
void test_stress_motion_impulse_rejection_across_temporal_phases() {
    lcg_state = 999999U;
    const uint32_t num_trials = 300U;
    uint32_t rejected_count = 0U;

    for (uint32_t trial = 0U; trial < num_trials; ++trial) {
        ImuCalibrator cal;
        cal.start_gyro_calibration(1000U);

        const float base_bias[3] = {
            lcg_uniform(-0.1F, 0.1F),
            lcg_uniform(-0.1F, 0.1F),
            lcg_uniform(-0.1F, 0.1F)
        };

        // Temporal injection target: 0=early (10-40), 1=mid (100-300), 2=late (600-800)
        uint32_t start_idx = 0U;
        if (trial < 100U) {
            start_idx = static_cast<uint32_t>(lcg_uniform(10.0F, 40.0F));
        } else if (trial < 200U) {
            start_idx = static_cast<uint32_t>(lcg_uniform(100.0F, 300.0F));
        } else {
            start_idx = static_cast<uint32_t>(lcg_uniform(600.0F, 800.0F));
        }

        const uint32_t duration = static_cast<uint32_t>(lcg_uniform(15.0F, 40.0F));
        const float impulse_mag = lcg_uniform(0.08F, 2.0F); // w > 0.05 rad/s
        const int axis = trial % 3;

        bool rejected_during_stream = false;

        for (uint32_t s = 0U; s < 1000U; ++s) {
            float sample[3] = {
                base_bias[0] + lcg_gaussian(0.0F, 0.001F),
                base_bias[1] + lcg_gaussian(0.0F, 0.001F),
                base_bias[2] + lcg_gaussian(0.0F, 0.001F)
            };

            if (s >= start_idx && s < (start_idx + duration)) {
                sample[axis] += impulse_mag;
            }

            if (!cal.update_gyro_sample(sample)) {
                rejected_during_stream = true;
                break;
            }
        }

        float dummy_drift = 0.0F;
        const bool finish_res = cal.finish_gyro_calibration(dummy_drift);
        if (rejected_during_stream || (!finish_res) || (cal.get_state() == CALIB_FAILED_MOTION)) {
            rejected_count++;
        }
    }

    // Rejection success rate must be >= 99.0%
    const float rejection_rate = (static_cast<float>(rejected_count) / static_cast<float>(num_trials)) * 100.0F;
    TEST_ASSERT_TRUE(rejection_rate >= 99.0F);
}

// ---------------------------------------------------------------------------
// 4. REP-103 ENU Standard & Right-Hand Rule Compliance
// ---------------------------------------------------------------------------
void test_stress_rep103_enu_compliance_and_right_hand_rotations() {
    ImuCalibrator cal;
    const float g = kStandardGravityMps2;

    // Static horizontal rest: +Z acceleration is +9.80665 m/s^2
    float raw_a[3] = {0.0F, 0.0F, g};
    float raw_g[3] = {0.0F, 0.0F, 0.0F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    cal.apply(raw_a, raw_g, cal_a, cal_g);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, cal_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, cal_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, g, cal_a[2]);
    TEST_ASSERT_TRUE(cal_a[2] > 0.0F);

    // Right-hand rule rotation checks:
    // CCW yaw (+Z)
    float ccw_yaw[3] = {0.0F, 0.0F, 0.5F};
    cal.apply(raw_a, ccw_yaw, cal_a, cal_g);
    TEST_ASSERT_TRUE(cal_g[2] > 0.0F);

    // CCW pitch (+Y)
    float ccw_pitch[3] = {0.0F, 0.5F, 0.0F};
    cal.apply(raw_a, ccw_pitch, cal_a, cal_g);
    TEST_ASSERT_TRUE(cal_g[1] > 0.0F);

    // CCW roll (+X)
    float ccw_roll[3] = {0.5F, 0.0F, 0.0F};
    cal.apply(raw_a, ccw_roll, cal_a, cal_g);
    TEST_ASSERT_TRUE(cal_g[0] > 0.0F);
}

// ---------------------------------------------------------------------------
// 5. Numerical Edge Cases & Fuzzing
// ---------------------------------------------------------------------------
void test_stress_numerical_fuzz_nan_inf_extremes() {
    ImuCalibrator cal;

    // NaN input rejection
    cal.start_gyro_calibration(100U);
    float nan_sample[3] = {NAN, 0.0F, 0.0F};
    TEST_ASSERT_FALSE(cal.update_gyro_sample(nan_sample));

    // Inf input rejection
    cal.reset();
    cal.start_gyro_calibration(100U);
    float inf_sample[3] = {0.0F, -INFINITY, 0.0F};
    TEST_ASSERT_FALSE(cal.update_gyro_sample(inf_sample));

    // Premature finish attempt with insufficient samples
    cal.reset();
    cal.start_gyro_calibration(100U);
    float normal_sample[3] = {0.0F, 0.0F, 0.0F};
    for (int i = 0; i < 20; ++i) {
        cal.update_gyro_sample(normal_sample);
    }
    float drift = 0.0F;
    TEST_ASSERT_FALSE(cal.finish_gyro_calibration(drift));
    TEST_ASSERT_EQUAL(CALIB_FAILED_MATH, cal.get_state());

    // Extreme bias nulling (0.5 rad/s ~ 28.6 deg/s)
    cal.reset();
    cal.start_gyro_calibration(1000U);
    const float huge_bias[3] = {0.5F, -0.4F, 0.35F};
    for (int i = 0; i < 1000; ++i) {
        cal.update_gyro_sample(huge_bias);
    }
    TEST_ASSERT_TRUE(cal.finish_gyro_calibration(drift));
    const ImuCalibrationParams &params = cal.get_params();
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, huge_bias[0], params.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, huge_bias[1], params.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-4F, huge_bias[2], params.gyro_bias[2]);
}

} // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_stress_1000_random_bias_vectors_residual_drift);
    RUN_TEST(test_stress_60s_yaw_integration_heading_stability);
    RUN_TEST(test_stress_motion_impulse_rejection_across_temporal_phases);
    RUN_TEST(test_stress_rep103_enu_compliance_and_right_hand_rotations);
    RUN_TEST(test_stress_numerical_fuzz_nan_inf_extremes);
    return UNITY_END();
}
