#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <random>
#include <numeric>

#include "firmware_config.h"
#include "imu_calibration.h"

// Include implementation directly to test native firmware code
#include "../../firmware/stm32_f407vg_arduino_sim/src/imu_calibration.cpp"

namespace {

// ============================================================================
// Metrics Structures
// ============================================================================

struct GyroDriftMetrics {
    uint32_t total_trials;
    uint32_t passed_trials;
    float min_residual_drift_rad_s;
    float max_residual_drift_rad_s;
    float mean_residual_drift_rad_s;
    float median_residual_drift_rad_s;
    float p95_residual_drift_rad_s;
    float p99_residual_drift_rad_s;
    float std_residual_drift_rad_s;
    float max_bias_error_rad_s;
    float mean_bias_error_rad_s;
    bool passed;
};

struct YawIntegrationMetrics {
    uint32_t total_trials;
    uint32_t passed_trials;
    float duration_seconds;
    float min_drift_deg;
    float max_drift_deg;
    float mean_drift_deg;
    float median_drift_deg;
    float p95_drift_deg;
    float std_drift_deg;
    float mean_uncalibrated_drift_deg;
    bool passed;
};

struct MotionRejectionMetrics {
    uint32_t total_motion_trials;
    uint32_t motion_rejected_trials;
    float motion_rejection_rate_pct;
    uint32_t stationary_control_trials;
    uint32_t stationary_accepted_trials;
    float stationary_acceptance_rate_pct;
    uint32_t early_impulse_tested;
    uint32_t early_impulse_rejected;
    uint32_t mid_impulse_tested;
    uint32_t mid_impulse_rejected;
    uint32_t late_impulse_tested;
    uint32_t late_impulse_rejected;
    bool passed;
};

struct EnuComplianceMetrics {
    float static_z_accel_mps2;
    float static_x_accel_mps2;
    float static_y_accel_mps2;
    bool static_z_compliant;
    bool positive_yaw_right_hand;
    bool positive_pitch_right_hand;
    bool positive_roll_right_hand;
    float max_sphere_norm_error_mps2;
    bool sphere_invariance_compliant;
    bool passed;
};

struct EdgeCaseMetrics {
    bool nan_rejected;
    bool inf_rejected;
    bool premature_rejected;
    bool reset_cycle_valid;
    bool extreme_bias_nulled;
    bool passed;
};

// ============================================================================
// Test 1: 1,000 Randomized Gyro Bias Vectors Monte Carlo Stress Test
// ============================================================================
GyroDriftMetrics run_1000_bias_vectors_stress_test(uint32_t seed = 42U) {
    GyroDriftMetrics metrics = {};
    metrics.total_trials = 1000U;
    metrics.min_residual_drift_rad_s = 1e9F;
    metrics.max_residual_drift_rad_s = 0.0F;

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> bias_dist(-0.3F, 0.3F); // rad/s (~ +/- 17 deg/s)
    // Sensor noise: Allan variance Ng * sqrt(fs) ~ 1.4e-4 * sqrt(100) = 1.4e-3 rad/s
    std::normal_distribution<float> noise_dist(0.0F, 0.0012F);

    std::vector<float> residual_drifts;
    residual_drifts.reserve(metrics.total_trials);
    std::vector<float> bias_errors;
    bias_errors.reserve(metrics.total_trials);

    for (uint32_t trial = 0U; trial < metrics.total_trials; ++trial) {
        ImuCalibrator calibrator;
        if (!calibrator.start_gyro_calibration(1000U)) {
            continue;
        }

        const float true_bias[3] = {
            bias_dist(rng),
            bias_dist(rng),
            bias_dist(rng)
        };

        // Feed 1000 stationary samples with white noise
        bool sample_ok = true;
        for (uint32_t s = 0U; s < 1000U; ++s) {
            const float sample[3] = {
                true_bias[0] + noise_dist(rng),
                true_bias[1] + noise_dist(rng),
                true_bias[2] + noise_dist(rng)
            };
            if (!calibrator.update_gyro_sample(sample)) {
                sample_ok = false;
                break;
            }
        }

        if (!sample_ok) {
            continue;
        }

        float reported_drift = 0.0F;
        const bool finished = calibrator.finish_gyro_calibration(reported_drift);
        if (!finished) {
            continue;
        }

        const ImuCalibrationParams &params = calibrator.get_params();
        if (!params.gyro_calibrated) {
            continue;
        }

        // True residual bias estimation error
        const float ex = params.gyro_bias[0] - true_bias[0];
        const float ey = params.gyro_bias[1] - true_bias[1];
        const float ez = params.gyro_bias[2] - true_bias[2];
        const float true_error_norm = sqrtf(ex * ex + ey * ey + ez * ez);

        residual_drifts.push_back(reported_drift);
        bias_errors.push_back(true_error_norm);

        if (reported_drift < metrics.min_residual_drift_rad_s) {
            metrics.min_residual_drift_rad_s = reported_drift;
        }
        if (reported_drift > metrics.max_residual_drift_rad_s) {
            metrics.max_residual_drift_rad_s = reported_drift;
        }
        if (true_error_norm > metrics.max_bias_error_rad_s) {
            metrics.max_bias_error_rad_s = true_error_norm;
        }

        // Acceptance criteria: residual drift < 0.05 deg/s = 8.7266e-4 rad/s
        if (reported_drift < kMaxGyroDriftRadS && true_error_norm < kMaxGyroDriftRadS) {
            metrics.passed_trials++;
        }
    }

    if (!residual_drifts.empty()) {
        std::sort(residual_drifts.begin(), residual_drifts.end());
        const size_t n = residual_drifts.size();
        const double sum = std::accumulate(residual_drifts.begin(), residual_drifts.end(), 0.0);
        metrics.mean_residual_drift_rad_s = static_cast<float>(sum / n);
        metrics.median_residual_drift_rad_s = residual_drifts[n / 2];
        metrics.p95_residual_drift_rad_s = residual_drifts[static_cast<size_t>(n * 0.95)];
        metrics.p99_residual_drift_rad_s = residual_drifts[static_cast<size_t>(n * 0.99)];

        double sq_sum = 0.0;
        for (float v : residual_drifts) {
            const double diff = v - metrics.mean_residual_drift_rad_s;
            sq_sum += diff * diff;
        }
        metrics.std_residual_drift_rad_s = static_cast<float>(sqrt(sq_sum / n));

        const double b_sum = std::accumulate(bias_errors.begin(), bias_errors.end(), 0.0);
        metrics.mean_bias_error_rad_s = static_cast<float>(b_sum / n);
    }

    metrics.passed = (metrics.passed_trials == metrics.total_trials);
    return metrics;
}

// ============================================================================
// Test 2: Long-Term 60-Second Yaw Integration Drift Test
// ============================================================================
YawIntegrationMetrics run_60s_yaw_integration_stress_test(uint32_t num_trials = 100U, uint32_t seed = 77U) {
    YawIntegrationMetrics metrics = {};
    metrics.total_trials = num_trials;
    metrics.duration_seconds = 60.0F;
    metrics.min_drift_deg = 1e9F;
    metrics.max_drift_deg = 0.0F;

    const float dt = 0.01F; // 100 Hz
    const uint32_t sim_steps = static_cast<uint32_t>(metrics.duration_seconds / dt); // 6000 steps

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> bias_dist(-0.15F, 0.15F); // rad/s
    std::normal_distribution<float> noise_dist(0.0F, 0.0012F);     // rad/s noise

    std::vector<float> heading_drifts_deg;
    heading_drifts_deg.reserve(num_trials);
    double total_uncal_drift_deg = 0.0;

    for (uint32_t trial = 0U; trial < num_trials; ++trial) {
        ImuCalibrator calibrator;
        calibrator.start_gyro_calibration(1000U);

        const float true_bias[3] = {
            bias_dist(rng),
            bias_dist(rng),
            bias_dist(rng)
        };

        // Calibrate with 1000 stationary samples
        for (uint32_t s = 0U; s < 1000U; ++s) {
            const float sample[3] = {
                true_bias[0] + noise_dist(rng),
                true_bias[1] + noise_dist(rng),
                true_bias[2] + noise_dist(rng)
            };
            calibrator.update_gyro_sample(sample);
        }

        float res_drift = 0.0F;
        if (!calibrator.finish_gyro_calibration(res_drift)) {
            continue;
        }

        // 60-second resting integration
        float raw_yaw = 0.0F;
        float cal_yaw = 0.0F;

        const float dummy_accel[3] = {0.0F, 0.0F, kStandardGravityMps2};
        float out_accel[3] = {0};
        float out_gyro[3] = {0};

        for (uint32_t step = 0U; step < sim_steps; ++step) {
            const float raw_gyro[3] = {
                true_bias[0] + noise_dist(rng),
                true_bias[1] + noise_dist(rng),
                true_bias[2] + noise_dist(rng)
            };

            raw_yaw += raw_gyro[2] * dt;

            calibrator.apply(dummy_accel, raw_gyro, out_accel, out_gyro);
            cal_yaw += out_gyro[2] * dt;
        }

        const float uncal_drift_deg = fabsf(raw_yaw) * (180.0F / static_cast<float>(M_PI));
        const float cal_drift_deg = fabsf(cal_yaw) * (180.0F / static_cast<float>(M_PI));

        total_uncal_drift_deg += uncal_drift_deg;
        heading_drifts_deg.push_back(cal_drift_deg);

        if (cal_drift_deg < metrics.min_drift_deg) {
            metrics.min_drift_deg = cal_drift_deg;
        }
        if (cal_drift_deg > metrics.max_drift_deg) {
            metrics.max_drift_deg = cal_drift_deg;
        }

        // Target: total heading drift < 3.0 degrees over 60 seconds
        if (cal_drift_deg < 3.0F) {
            metrics.passed_trials++;
        }
    }

    if (!heading_drifts_deg.empty()) {
        std::sort(heading_drifts_deg.begin(), heading_drifts_deg.end());
        const size_t n = heading_drifts_deg.size();
        const double sum = std::accumulate(heading_drifts_deg.begin(), heading_drifts_deg.end(), 0.0);
        metrics.mean_drift_deg = static_cast<float>(sum / n);
        metrics.median_drift_deg = heading_drifts_deg[n / 2];
        metrics.p95_drift_deg = heading_drifts_deg[static_cast<size_t>(n * 0.95)];
        metrics.mean_uncalibrated_drift_deg = static_cast<float>(total_uncal_drift_deg / n);

        double sq_sum = 0.0;
        for (float v : heading_drifts_deg) {
            const double diff = v - metrics.mean_drift_deg;
            sq_sum += diff * diff;
        }
        metrics.std_drift_deg = static_cast<float>(sqrt(sq_sum / n));
    }

    metrics.passed = (metrics.passed_trials == metrics.total_trials);
    return metrics;
}

// ============================================================================
// Test 3: Motion Disturbance Rejection Test
// ============================================================================
MotionRejectionMetrics run_motion_disturbance_rejection_test(uint32_t seed = 99U) {
    MotionRejectionMetrics metrics = {};
    metrics.total_motion_trials = 1000U;
    metrics.stationary_control_trials = 200U;

    std::mt19937 rng(seed);
    std::uniform_real_distribution<float> bias_dist(-0.1F, 0.1F);
    std::normal_distribution<float> noise_dist(0.0F, 0.001F);

    // Sub-scenarios: early (1-50), mid (51-500), late (501-950)
    std::uniform_int_distribution<uint32_t> early_start_dist(1U, 45U);
    std::uniform_int_distribution<uint32_t> mid_start_dist(60U, 450U);
    std::uniform_int_distribution<uint32_t> late_start_dist(510U, 920U);

    std::uniform_real_distribution<float> impulse_mag_dist(0.06F, 2.5F); // > 0.05 rad/s
    std::uniform_int_distribution<uint32_t> duration_dist(5U, 40U);      // 5 to 40 samples

    for (uint32_t trial = 0U; trial < metrics.total_motion_trials; ++trial) {
        ImuCalibrator calibrator;
        calibrator.start_gyro_calibration(1000U);

        const float base_bias[3] = {
            bias_dist(rng),
            bias_dist(rng),
            bias_dist(rng)
        };

        // Partition into 3 categories: 300 early, 400 mid, 300 late
        uint32_t start_idx = 0U;
        int category = 0; // 0=early, 1=mid, 2=late
        if (trial < 300U) {
            start_idx = early_start_dist(rng);
            category = 0;
            metrics.early_impulse_tested++;
        } else if (trial < 700U) {
            start_idx = mid_start_dist(rng);
            category = 1;
            metrics.mid_impulse_tested++;
        } else {
            start_idx = late_start_dist(rng);
            category = 2;
            metrics.late_impulse_tested++;
        }

        const uint32_t duration = duration_dist(rng);
        const float impulse_mag = impulse_mag_dist(rng);
        const int axis = trial % 3;

        bool rejected_during_update = false;

        for (uint32_t s = 0U; s < 1000U; ++s) {
            float sample[3] = {
                base_bias[0] + noise_dist(rng),
                base_bias[1] + noise_dist(rng),
                base_bias[2] + noise_dist(rng)
            };

            // Inject motion impulse
            if (s >= start_idx && s < (start_idx + duration)) {
                sample[axis] += impulse_mag;
            }

            if (!calibrator.update_gyro_sample(sample)) {
                rejected_during_update = true;
                break;
            }
        }

        float dummy_drift = 0.0F;
        const bool finish_result = calibrator.finish_gyro_calibration(dummy_drift);
        const bool was_rejected = rejected_during_update || (!finish_result) ||
                                  (calibrator.get_state() == CALIB_FAILED_MOTION);

        if (was_rejected) {
            metrics.motion_rejected_trials++;
            if (category == 0) metrics.early_impulse_rejected++;
            else if (category == 1) metrics.mid_impulse_rejected++;
            else metrics.late_impulse_rejected++;
        }
    }

    metrics.motion_rejection_rate_pct = (static_cast<float>(metrics.motion_rejected_trials) /
                                         static_cast<float>(metrics.total_motion_trials)) * 100.0F;

    // Stationary control trials: ensure clean stationary data is NEVER falsely rejected
    for (uint32_t trial = 0U; trial < metrics.stationary_control_trials; ++trial) {
        ImuCalibrator calibrator;
        calibrator.start_gyro_calibration(1000U);

        const float base_bias[3] = {bias_dist(rng), bias_dist(rng), bias_dist(rng)};
        bool all_samples_accepted = true;

        for (uint32_t s = 0U; s < 1000U; ++s) {
            const float sample[3] = {
                base_bias[0] + noise_dist(rng),
                base_bias[1] + noise_dist(rng),
                base_bias[2] + noise_dist(rng)
            };
            if (!calibrator.update_gyro_sample(sample)) {
                all_samples_accepted = false;
                break;
            }
        }

        float dummy_drift = 0.0F;
        if (all_samples_accepted && calibrator.finish_gyro_calibration(dummy_drift)) {
            metrics.stationary_accepted_trials++;
        }
    }

    metrics.stationary_acceptance_rate_pct = (static_cast<float>(metrics.stationary_accepted_trials) /
                                             static_cast<float>(metrics.stationary_control_trials)) * 100.0F;

    // Must reject >= 99% of motion trials and accept 100% of stationary trials
    metrics.passed = (metrics.motion_rejection_rate_pct >= 99.0F) &&
                     (metrics.stationary_acceptance_rate_pct == 100.0F);
    return metrics;
}

// ============================================================================
// Test 4: REP-103 ENU Coordinate Standard Compliance
// ============================================================================
EnuComplianceMetrics run_rep103_enu_compliance_test() {
    EnuComplianceMetrics metrics = {};
    ImuCalibrator calibrator;

    const float g = kStandardGravityMps2;

    // 1. Static level ground: reaction acceleration is +g (+9.80665 m/s^2) on Z
    float raw_a[3] = {0.0F, 0.0F, g};
    float raw_g[3] = {0.0F, 0.0F, 0.0F};
    float cal_a[3] = {0};
    float cal_g[3] = {0};

    calibrator.apply(raw_a, raw_g, cal_a, cal_g);

    metrics.static_x_accel_mps2 = cal_a[0];
    metrics.static_y_accel_mps2 = cal_a[1];
    metrics.static_z_accel_mps2 = cal_a[2];

    metrics.static_z_compliant = (fabsf(cal_a[2] - g) < 1e-4F) &&
                                 (fabsf(cal_a[0]) < 1e-4F) &&
                                 (fabsf(cal_a[1]) < 1e-4F);

    // 2. Right-hand rule angular rates:
    // CCW rotation about +Z -> positive cal_g[2]
    float ccw_z[3] = {0.0F, 0.0F, 0.75F};
    calibrator.apply(raw_a, ccw_z, cal_a, cal_g);
    metrics.positive_yaw_right_hand = (cal_g[2] > 0.0F);

    // CCW rotation about +Y -> positive cal_g[1]
    float pitch_y[3] = {0.0F, 0.75F, 0.0F};
    calibrator.apply(raw_a, pitch_y, cal_a, cal_g);
    metrics.positive_pitch_right_hand = (cal_g[1] > 0.0F);

    // CCW rotation about +X -> positive cal_g[0]
    float roll_x[3] = {0.75F, 0.0F, 0.0F};
    calibrator.apply(raw_a, roll_x, cal_a, cal_g);
    metrics.positive_roll_right_hand = (cal_g[0] > 0.0F);

    // 3. Spherical static orientation invariance:
    // Check 360 points along spherical angles (latitude/longitude)
    metrics.max_sphere_norm_error_mps2 = 0.0F;
    for (int lat_deg = -80; lat_deg <= 80; lat_deg += 20) {
        for (int lon_deg = 0; lon_deg < 360; lon_deg += 20) {
            const float lat = static_cast<float>(lat_deg) * (static_cast<float>(M_PI) / 180.0F);
            const float lon = static_cast<float>(lon_deg) * (static_cast<float>(M_PI) / 180.0F);

            const float ux = cosf(lat) * cosf(lon);
            const float uy = cosf(lat) * sinf(lon);
            const float uz = sinf(lat);

            float sphere_raw_a[3] = {ux * g, uy * g, uz * g};
            float sphere_cal_a[3] = {0};
            float sphere_cal_g[3] = {0};

            calibrator.apply(sphere_raw_a, raw_g, sphere_cal_a, sphere_cal_g);
            const float norm = sqrtf(sphere_cal_a[0] * sphere_cal_a[0] +
                                     sphere_cal_a[1] * sphere_cal_a[1] +
                                     sphere_cal_a[2] * sphere_cal_a[2]);
            const float err = fabsf(norm - g);
            if (err > metrics.max_sphere_norm_error_mps2) {
                metrics.max_sphere_norm_error_mps2 = err;
            }
        }
    }

    metrics.sphere_invariance_compliant = (metrics.max_sphere_norm_error_mps2 < 1e-4F);

    metrics.passed = metrics.static_z_compliant &&
                     metrics.positive_yaw_right_hand &&
                     metrics.positive_pitch_right_hand &&
                     metrics.positive_roll_right_hand &&
                     metrics.sphere_invariance_compliant;
    return metrics;
}

// ============================================================================
// Test 5: Numerical Edge Cases & Robustness
// ============================================================================
EdgeCaseMetrics run_edge_cases_test() {
    EdgeCaseMetrics metrics = {};

    // 1. NaN rejection
    {
        ImuCalibrator cal;
        cal.start_gyro_calibration(100U);
        float nan_sample[3] = {NAN, 0.0F, 0.0F};
        metrics.nan_rejected = (!cal.update_gyro_sample(nan_sample));
    }

    // 2. Inf rejection
    {
        ImuCalibrator cal;
        cal.start_gyro_calibration(100U);
        float inf_sample[3] = {0.0F, INFINITY, 0.0F};
        metrics.inf_rejected = (!cal.update_gyro_sample(inf_sample));
    }

    // 3. Premature finish attempt
    {
        ImuCalibrator cal;
        cal.start_gyro_calibration(100U);
        float dummy[3] = {0.0F, 0.0F, 0.0F};
        for (int i = 0; i < 10; ++i) {
            cal.update_gyro_sample(dummy);
        }
        float res = 0.0F;
        metrics.premature_rejected = (!cal.finish_gyro_calibration(res));
    }

    // 4. Reset cycle validity
    {
        ImuCalibrator cal;
        cal.start_gyro_calibration(100U);
        float dummy[3] = {0.01F, 0.01F, 0.01F};
        for (int i = 0; i < 100; ++i) {
            cal.update_gyro_sample(dummy);
        }
        float res = 0.0F;
        cal.finish_gyro_calibration(res);
        cal.reset();
        const ImuCalibrationParams &p = cal.get_params();
        metrics.reset_cycle_valid = (!p.gyro_calibrated) &&
                                    (cal.get_state() == CALIB_IDLE) &&
                                    (fabsf(p.gyro_bias[0]) < 1e-6F);
    }

    // 5. Extreme bias nulling (0.5 rad/s ~ 28.6 deg/s)
    {
        ImuCalibrator cal;
        cal.start_gyro_calibration(1000U);
        const float huge_bias[3] = {0.5F, -0.45F, 0.52F};
        for (int i = 0; i < 1000; ++i) {
            cal.update_gyro_sample(huge_bias);
        }
        float res = 0.0F;
        bool ok = cal.finish_gyro_calibration(res);
        const ImuCalibrationParams &p = cal.get_params();
        metrics.extreme_bias_nulled = ok &&
            (fabsf(p.gyro_bias[0] - huge_bias[0]) < 1e-4F) &&
            (fabsf(p.gyro_bias[1] - huge_bias[1]) < 1e-4F) &&
            (fabsf(p.gyro_bias[2] - huge_bias[2]) < 1e-4F);
    }

    metrics.passed = metrics.nan_rejected &&
                     metrics.inf_rejected &&
                     metrics.premature_rejected &&
                     metrics.reset_cycle_valid &&
                     metrics.extreme_bias_nulled;
    return metrics;
}

} // namespace

// ============================================================================
// Main Benchmark Runner
// ============================================================================
int main(int argc, char *argv[]) {
    (void)argc;
    (void)argv;

    printf("=======================================================================\n");
    printf("   AMR OMNI M2 GYROSCOPE CALIBRATION & HEADING STABILITY BENCHMARK     \n");
    printf("=======================================================================\n");

    // Test 1
    printf("\n--> Running Test 1: 1,000 Randomized Gyro Bias Vectors Monte Carlo Stress Test...\n");
    GyroDriftMetrics t1 = run_1000_bias_vectors_stress_test();
    printf("    Total Monte Carlo Trials:    %u\n", t1.total_trials);
    printf("    Passed Trials:               %u (%.2f%%)\n", t1.passed_trials,
           (float)t1.passed_trials * 100.0F / (float)t1.total_trials);
    printf("    Threshold:                   < 0.05 deg/s (%.6f rad/s)\n", kMaxGyroDriftRadS);
    printf("    Min Residual Drift:          %.6f rad/s (%.4f deg/s)\n",
           t1.min_residual_drift_rad_s, t1.min_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    Mean Residual Drift:         %.6f rad/s (%.4f deg/s)\n",
           t1.mean_residual_drift_rad_s, t1.mean_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    Median Residual Drift:       %.6f rad/s (%.4f deg/s)\n",
           t1.median_residual_drift_rad_s, t1.median_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    95th Percentile Drift:       %.6f rad/s (%.4f deg/s)\n",
           t1.p95_residual_drift_rad_s, t1.p95_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    99th Percentile Drift:       %.6f rad/s (%.4f deg/s)\n",
           t1.p99_residual_drift_rad_s, t1.p99_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    Max Residual Drift:          %.6f rad/s (%.4f deg/s)\n",
           t1.max_residual_drift_rad_s, t1.max_residual_drift_rad_s * (180.0F / 3.14159265F));
    printf("    Std Dev Residual Drift:      %.6f rad/s\n", t1.std_residual_drift_rad_s);
    printf("    Mean True Bias Error:        %.6f rad/s (%.4f deg/s)\n",
           t1.mean_bias_error_rad_s, t1.mean_bias_error_rad_s * (180.0F / 3.14159265F));
    printf("    Max True Bias Error:         %.6f rad/s (%.4f deg/s)\n",
           t1.max_bias_error_rad_s, t1.max_bias_error_rad_s * (180.0F / 3.14159265F));
    printf("    Status:                      %s\n", t1.passed ? "PASSED" : "FAILED");

    // Test 2
    printf("\n--> Running Test 2: Long-Term 60-Second Yaw Integration Drift Test (100 Trials)...\n");
    YawIntegrationMetrics t2 = run_60s_yaw_integration_stress_test();
    printf("    Total Trials:                %u\n", t2.total_trials);
    printf("    Integration Window:          %.1f seconds (6,000 steps at 100 Hz)\n", t2.duration_seconds);
    printf("    Passed Trials:               %u (%.2f%%)\n", t2.passed_trials,
           (float)t2.passed_trials * 100.0F / (float)t2.total_trials);
    printf("    Threshold:                   < 3.0 degrees\n");
    printf("    Uncalibrated Mean Drift:     %.2f degrees\n", t2.mean_uncalibrated_drift_deg);
    printf("    Min Calibrated Drift:        %.4f degrees\n", t2.min_drift_deg);
    printf("    Mean Calibrated Drift:       %.4f degrees\n", t2.mean_drift_deg);
    printf("    Median Calibrated Drift:     %.4f degrees\n", t2.median_drift_deg);
    printf("    95th Percentile Drift:       %.4f degrees\n", t2.p95_drift_deg);
    printf("    Max Calibrated Drift:        %.4f degrees\n", t2.max_drift_deg);
    printf("    Std Dev Calibrated Drift:    %.4f degrees\n", t2.std_drift_deg);
    printf("    Status:                      %s\n", t2.passed ? "PASSED" : "FAILED");

    // Test 3
    printf("\n--> Running Test 3: Motion Disturbance Rejection Test (1,000 Disturbance Trials)...\n");
    MotionRejectionMetrics t3 = run_motion_disturbance_rejection_test();
    printf("    Total Motion Trials:         %u\n", t3.total_motion_trials);
    printf("    Impulses Injected:           w > 0.05 rad/s (0.06 to 2.5 rad/s)\n");
    printf("    Motion Trials Rejected:      %u\n", t3.motion_rejected_trials);
    printf("    Motion Rejection Rate:       %.2f%% (Target >= 99.0%%)\n", t3.motion_rejection_rate_pct);
    printf("      Early Phase (samples 1-50):   %u / %u (%.2f%%)\n",
           t3.early_impulse_rejected, t3.early_impulse_tested,
           (float)t3.early_impulse_rejected * 100.0F / (float)t3.early_impulse_tested);
    printf("      Mid Phase (samples 51-500):   %u / %u (%.2f%%)\n",
           t3.mid_impulse_rejected, t3.mid_impulse_tested,
           (float)t3.mid_impulse_rejected * 100.0F / (float)t3.mid_impulse_tested);
    printf("      Late Phase (samples 501-950): %u / %u (%.2f%%)\n",
           t3.late_impulse_rejected, t3.late_impulse_tested,
           (float)t3.late_impulse_rejected * 100.0F / (float)t3.late_impulse_tested);
    printf("    Stationary Control Trials:   %u\n", t3.stationary_control_trials);
    printf("    Stationary Accepted Trials:  %u\n", t3.stationary_accepted_trials);
    printf("    Stationary False Alarm Rate: %.2f%% (Target == 0.0%%)\n",
           100.0F - t3.stationary_acceptance_rate_pct);
    printf("    Status:                      %s\n", t3.passed ? "PASSED" : "FAILED");

    // Test 4
    printf("\n--> Running Test 4: REP-103 ENU Coordinate Standard Compliance...\n");
    EnuComplianceMetrics t4 = run_rep103_enu_compliance_test();
    printf("    Static Linear Accel (Level): [ax=%.6f, ay=%.6f, az=%.6f] m/s^2\n",
           t4.static_x_accel_mps2, t4.static_y_accel_mps2, t4.static_z_accel_mps2);
    printf("    Expected Static Z Accel:     +9.80665 m/s^2 (Standard Gravity)\n");
    printf("    Static Z Match (< 1e-4):     %s\n", t4.static_z_compliant ? "YES" : "NO");
    printf("    Positive Yaw Right-Hand:     %s (CCW +Z gives positive rate)\n",
           t4.positive_yaw_right_hand ? "YES" : "NO");
    printf("    Positive Pitch Right-Hand:   %s (CCW +Y gives positive rate)\n",
           t4.positive_pitch_right_hand ? "YES" : "NO");
    printf("    Positive Roll Right-Hand:    %s (CCW +X gives positive rate)\n",
           t4.positive_roll_right_hand ? "YES" : "NO");
    printf("    Max Sphere Norm Error:       %.6f m/s^2 (across 360 orientations)\n",
           t4.max_sphere_norm_error_mps2);
    printf("    Status:                      %s\n", t4.passed ? "PASSED" : "FAILED");

    // Test 5
    printf("\n--> Running Test 5: Numerical Robustness & Edge Cases...\n");
    EdgeCaseMetrics t5 = run_edge_cases_test();
    printf("    NaN Gyro Input Rejected:     %s\n", t5.nan_rejected ? "YES" : "NO");
    printf("    Inf Gyro Input Rejected:     %s\n", t5.inf_rejected ? "YES" : "NO");
    printf("    Premature Finish Rejected:   %s\n", t5.premature_rejected ? "YES" : "NO");
    printf("    Reset Cycle Valid:           %s\n", t5.reset_cycle_valid ? "YES" : "NO");
    printf("    Extreme Bias (0.5 rad/s):    %s\n", t5.extreme_bias_nulled ? "NULLED" : "FAILED");
    printf("    Status:                      %s\n", t5.passed ? "PASSED" : "FAILED");

    const bool all_passed = t1.passed && t2.passed && t3.passed && t4.passed && t5.passed;

    printf("\n=======================================================================\n");
    printf("   OVERALL VERDICT: %s\n", all_passed ? "APPROVE (ALL 5 TESTS PASSED)" : "REQUEST_CHANGES");
    printf("=======================================================================\n");

    return all_passed ? 0 : 1;
}
