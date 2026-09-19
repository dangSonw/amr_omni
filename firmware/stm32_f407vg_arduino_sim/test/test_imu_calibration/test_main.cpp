#include <math.h>
#include <stdint.h>
#include <unity.h>

#include "firmware_config.h"
#include "imu_calibration.h"

#include "../../src/imu_calibration.cpp"

void setUp(void) {}
void tearDown(void) {}

namespace {

void test_default_calibration_is_identity() {
    ImuCalibrator calibrator;
    const ImuCalibrationParams &params = calibrator.get_params();

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 1.0F, params.accel_scale[2]);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.accel_bias[2]);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, 0.0F, params.gyro_bias[2]);

    float raw_a[3] = {0.1F, -0.2F, 9.8F};
    float raw_g[3] = {0.01F, -0.02F, 0.05F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    calibrator.apply(raw_a, raw_g, cal_a, cal_g);

    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[0], cal_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[1], cal_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_a[2], cal_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[0], cal_g[0]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[1], cal_g[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-5F, raw_g[2], cal_g[2]);
}

void test_gyro_bias_nulling_and_drift_threshold() {
    ImuCalibrator calibrator;
    TEST_ASSERT_TRUE(calibrator.start_gyro_calibration(1000U));

    const float injected_bias[3] = {0.015F, -0.025F, 0.008F};

    // Feed 1000 samples with small stationary zero-mean noise
    for (uint32_t i = 0U; i < 1000U; ++i) {
        float noise = (static_cast<float>(i % 7) - 3.0F) * 1e-4F;
        float sample[3] = {
            injected_bias[0] + noise,
            injected_bias[1] - noise,
            injected_bias[2] + noise * 0.5F
        };
        TEST_ASSERT_TRUE(calibrator.update_gyro_sample(sample));
    }

    float residual_drift = 0.0F;
    TEST_ASSERT_TRUE(calibrator.finish_gyro_calibration(residual_drift));

    const ImuCalibrationParams &params = calibrator.get_params();
    TEST_ASSERT_TRUE(params.gyro_calibrated);

    // Verify calculated bias matches injected bias
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[0], params.gyro_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[1], params.gyro_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, injected_bias[2], params.gyro_bias[2]);

    // Verify static residual drift < 0.05 deg/s (8.7266e-4 rad/s)
    TEST_ASSERT_TRUE(residual_drift < kMaxGyroDriftRadS);

    // Verify calibrated output on resting data has zero rate
    float raw_a[3] = {0.0F, 0.0F, 9.80665F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};
    calibrator.apply(raw_a, injected_bias, cal_a, cal_g);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.0005F, 0.0F, cal_g[2]);
}

void test_gyro_motion_rejection() {
    ImuCalibrator calibrator;
    TEST_ASSERT_TRUE(calibrator.start_gyro_calibration(500U));

    // Feed noisy motion samples
    bool rejected = false;
    for (uint32_t i = 0U; i < 100U; ++i) {
        float sample[3] = {
            sinf(static_cast<float>(i) * 0.5F) * 0.5F,
            cosf(static_cast<float>(i) * 0.5F) * 0.5F,
            0.1F
        };
        if (!calibrator.update_gyro_sample(sample)) {
            rejected = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(rejected);
    TEST_ASSERT_EQUAL(CALIB_FAILED_MOTION, calibrator.get_state());
}

void test_accel_6_position_calibration_and_norm_error() {
    ImuCalibrator calibrator;

    const float g = kStandardGravityMps2;
    const float inj_scale[3] = {1.05F, 0.95F, 1.02F};
    const float inj_bias[3] = {0.12F, -0.08F, 0.15F};

    // Ground truth acceleration for 6 faces:
    // +X, -X, +Y, -Y, +Z, -Z
    const float true_faces[6][3] = {
        {+g, 0.0F, 0.0F},
        {-g, 0.0F, 0.0F},
        {0.0F, +g, 0.0F},
        {0.0F, -g, 0.0F},
        {0.0F, 0.0F, +g},
        {0.0F, 0.0F, -g}
    };

    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        TEST_ASSERT_TRUE(calibrator.start_accel_face(static_cast<AccelFace>(f), 200U));

        // Raw = scale * true + bias
        float raw[3];
        raw[0] = inj_scale[0] * true_faces[f][0] + inj_bias[0];
        raw[1] = inj_scale[1] * true_faces[f][1] + inj_bias[1];
        raw[2] = inj_scale[2] * true_faces[f][2] + inj_bias[2];

        for (uint32_t s = 0U; s < 200U; ++s) {
            TEST_ASSERT_TRUE(calibrator.update_accel_sample(raw));
        }
        TEST_ASSERT_TRUE(calibrator.finish_accel_face());
    }

    float max_norm_error = 0.0F;
    TEST_ASSERT_TRUE(calibrator.compute_accel_calibration(max_norm_error));

    const ImuCalibrationParams &params = calibrator.get_params();
    TEST_ASSERT_TRUE(params.accel_calibrated);

    // Verify calculated scales and biases match injected values
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[0], params.accel_scale[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[1], params.accel_scale[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, inj_scale[2], params.accel_scale[2]);

    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[0], params.accel_bias[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[1], params.accel_bias[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.005F, inj_bias[2], params.accel_bias[2]);

    // Verify residual norm error across all 6 faces is < 0.05 m/s^2 (< 0.5%)
    TEST_ASSERT_TRUE(max_norm_error < kMaxAccelNormErrorMps2);
}

void test_rep103_enu_coordinate_alignment() {
    ImuCalibrator calibrator;
    const float g = kStandardGravityMps2;

    // Simulate robot level on ground: raw reads approx +9.81 on Z
    float raw_a[3] = {0.0F, 0.0F, g};
    float raw_g[3] = {0.0F, 0.0F, 0.0F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    calibrator.apply(raw_a, raw_g, cal_a, cal_g);

    // REP-103 East-North-Up: X forward, Y left, Z up (+9.80665 m/s^2 at rest)
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_a[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_a[1]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, g, cal_a[2]);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, cal_g[2]);

    // Counter-clockwise rotation about +Z yields positive yaw rate
    float ccw_raw_g[3] = {0.0F, 0.0F, 0.5F};
    calibrator.apply(raw_a, ccw_raw_g, cal_a, cal_g);
    TEST_ASSERT_TRUE(cal_g[2] > 0.0F);
}

void test_allan_variance_covariance_inflation() {
    ImuCalibrator calibrator;
    float gyro_cov[9] = {0};
    float accel_cov[9] = {0};

    calibrator.compute_covariances(0.02F, gyro_cov, accel_cov);

    // Verify all diagonal elements are strictly non-zero
    TEST_ASSERT_TRUE(gyro_cov[0] > 0.0F);
    TEST_ASSERT_TRUE(gyro_cov[4] > 0.0F);
    TEST_ASSERT_TRUE(gyro_cov[8] > 0.0F);

    TEST_ASSERT_TRUE(accel_cov[0] > 0.0F);
    TEST_ASSERT_TRUE(accel_cov[4] > 0.0F);
    TEST_ASSERT_TRUE(accel_cov[8] > 0.0F);

    // Off-diagonals must be zero
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, gyro_cov[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, gyro_cov[3]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, accel_cov[1]);
    TEST_ASSERT_FLOAT_WITHIN(1e-6F, 0.0F, accel_cov[3]);

    // Verify inflation factor: alpha = 1.8 -> alpha^2 = 3.24
    // sigma_gyro^2 = max(3.24 * (1.4e-4)^2 / 0.02, 1e-4) = 1e-4 (clamped)
    TEST_ASSERT_TRUE(gyro_cov[8] >= 1.0e-4F);
    TEST_ASSERT_TRUE(accel_cov[0] >= 1.0e-2F);
}

void test_invalid_inputs_and_edge_cases() {
    ImuCalibrator calibrator;

    // NaN / Inf inputs rejected
    float invalid_sample[3] = {NAN, 0.0F, 0.0F};
    calibrator.start_gyro_calibration(100U);
    TEST_ASSERT_FALSE(calibrator.update_gyro_sample(invalid_sample));

    // Premature compute without all faces fails safely
    float max_err = 0.0F;
    TEST_ASSERT_FALSE(calibrator.compute_accel_calibration(max_err));
    TEST_ASSERT_EQUAL(CALIB_FAILED_MATH, calibrator.get_state());

    // Invalid face index
    TEST_ASSERT_FALSE(calibrator.start_accel_face(static_cast<AccelFace>(99U), 100U));

    // Premature finish gyro without samples
    ImuCalibrator empty_calibrator;
    float residual = 0.0F;
    TEST_ASSERT_FALSE(empty_calibrator.finish_gyro_calibration(residual));
    TEST_ASSERT_EQUAL(CALIB_FAILED_MATH, empty_calibrator.get_state());
}

} // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_default_calibration_is_identity);
    RUN_TEST(test_gyro_bias_nulling_and_drift_threshold);
    RUN_TEST(test_gyro_motion_rejection);
    RUN_TEST(test_accel_6_position_calibration_and_norm_error);
    RUN_TEST(test_rep103_enu_coordinate_alignment);
    RUN_TEST(test_allan_variance_covariance_inflation);
    RUN_TEST(test_invalid_inputs_and_edge_cases);
    return UNITY_END();
}
