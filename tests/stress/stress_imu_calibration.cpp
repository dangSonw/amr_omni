#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <vector>
#include <numeric>
#include <random>
#include <algorithm>
#include <iostream>
#include <iomanip>

#include "firmware_config.h"
#include "imu_calibration.h"

// Forward definition of gravity
static const float kG = 9.80665F;

// Reference true orientations for the 6 faces (+X, -X, +Y, -Y, +Z, -Z)
// When a given face points UP, accelerometer measures +g along that axis.
static const float kFaceTrueAccel[6][3] = {
    {+kG, 0.0F, 0.0F},  // FACE_POS_X
    {-kG, 0.0F, 0.0F},  // FACE_NEG_X
    {0.0F, +kG, 0.0F},  // FACE_POS_Y
    {0.0F, -kG, 0.0F},  // FACE_NEG_Y
    {0.0F, 0.0F, +kG},  // FACE_POS_Z
    {0.0F, 0.0F, -kG}   // FACE_NEG_Z
};

struct NoiseStressResult {
    float sigma_a;
    uint32_t sample_count_N;
    uint32_t trials;
    uint32_t internal_passes;
    float max_internal_norm_err_mps2;
    float max_out_of_sample_norm_err_mps2;
    float mean_out_of_sample_norm_err_mps2;
    float max_scale_error;
    float mean_scale_error;
    float max_bias_error_mps2;
    float mean_bias_error_mps2;
    bool passed;
};

struct ScaleBiasStressResult {
    uint32_t valid_trials;
    uint32_t valid_passes;
    uint32_t out_of_bounds_trials;
    uint32_t out_of_bounds_rejected;
    float max_scale_error;
    float mean_scale_error;
    float max_bias_error_mps2;
    float mean_bias_error_mps2;
    float max_norm_error_mps2;
    bool passed;
};

struct SequenceStressResult {
    uint32_t permutations_tested;
    uint32_t permutations_passed;
    uint32_t incomplete_tests;
    uint32_t incomplete_rejected;
    uint32_t state_violations_tested;
    uint32_t state_violations_rejected;
    uint32_t nan_inf_tested;
    uint32_t nan_inf_rejected;
    bool uncompleted_restart_rejected; // White-box test for face_completed_ unreset bug
    bool passed;
};

struct Orientation3DResult {
    uint32_t total_orientations;
    float max_norm_error_mps2;
    float mean_norm_error_mps2;
    float std_norm_error_mps2;
    float worst_u[3];
    bool passed;
};

// ============================================================================
// Suite 1: Varying Noise Stress Test (sigma_a in [0.01, 0.50] m/s^2)
// Evaluates both:
//   (A) Default fixed N = 200 samples per face
//   (B) Adaptive N (N up to 2000) for high noise
// ============================================================================
std::vector<NoiseStressResult> run_noise_stress_suite(std::mt19937_64 &rng) {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "[SUITE 1] Noise Stress Test: sigma_a in [0.01, 0.50] m/s^2" << std::endl;
    std::cout << "========================================================" << std::endl;

    const float sigmas[] = {0.01F, 0.05F, 0.10F, 0.20F, 0.30F, 0.40F, 0.50F};
    const uint32_t kTrialsPerSigma = 100U;
    std::vector<NoiseStressResult> results;

    const float true_scale[3] = {1.04F, 0.96F, 1.02F};
    const float true_bias[3] = {0.15F, -0.12F, 0.25F};

    std::cout << "\n--- Part 1A: Default Face Sample Count N = 200 ---" << std::endl;
    for (float sigma : sigmas) {
        std::normal_distribution<float> noise_dist(0.0F, sigma);
        NoiseStressResult res = {};
        res.sigma_a = sigma;
        res.sample_count_N = 200U;
        res.trials = kTrialsPerSigma;
        res.internal_passes = 0;
        res.max_internal_norm_err_mps2 = 0.0F;
        res.max_out_of_sample_norm_err_mps2 = 0.0F;
        res.max_scale_error = 0.0F;
        res.max_bias_error_mps2 = 0.0F;

        double sum_norm_err = 0.0;
        double sum_scale_err = 0.0;
        double sum_bias_err = 0.0;
        uint32_t sample_eval_count = 0;

        for (uint32_t t = 0; t < kTrialsPerSigma; ++t) {
            ImuCalibrator calibrator;

            for (uint8_t f = 0; f < FACE_COUNT; ++f) {
                calibrator.start_accel_face(static_cast<AccelFace>(f), res.sample_count_N);
                for (uint32_t s = 0; s < res.sample_count_N; ++s) {
                    float raw[3];
                    for (int i = 0; i < 3; ++i) {
                        raw[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i] + noise_dist(rng);
                    }
                    calibrator.update_accel_sample(raw);
                }
                calibrator.finish_accel_face();
            }

            float reported_internal_norm_err = 0.0F;
            bool ok = calibrator.compute_accel_calibration(reported_internal_norm_err);
            if (reported_internal_norm_err > res.max_internal_norm_err_mps2) {
                res.max_internal_norm_err_mps2 = reported_internal_norm_err;
            }

            if (!ok) {
                continue;
            }
            res.internal_passes++;

            const ImuCalibrationParams &params = calibrator.get_params();

            // Evaluate scale error
            for (int i = 0; i < 3; ++i) {
                float s_err = fabsf(params.accel_scale[i] - true_scale[i]) / true_scale[i];
                if (s_err > res.max_scale_error) res.max_scale_error = s_err;
                sum_scale_err += s_err;

                float b_err = fabsf(params.accel_bias[i] - true_bias[i]);
                if (b_err > res.max_bias_error_mps2) res.max_bias_error_mps2 = b_err;
                sum_bias_err += b_err;
            }

            // Evaluate out-of-sample norm error across all 6 faces using true physics
            for (uint8_t f = 0; f < FACE_COUNT; ++f) {
                float raw_test[3], cal_a[3], cal_g[3], raw_g[3] = {0};
                for (int i = 0; i < 3; ++i) {
                    raw_test[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i];
                }
                calibrator.apply(raw_test, raw_g, cal_a, cal_g);
                float norm = sqrtf(cal_a[0]*cal_a[0] + cal_a[1]*cal_a[1] + cal_a[2]*cal_a[2]);
                float err = fabsf(norm - kG);
                if (err > res.max_out_of_sample_norm_err_mps2) {
                    res.max_out_of_sample_norm_err_mps2 = err;
                }
                sum_norm_err += err;
                sample_eval_count++;
            }
        }

        res.mean_out_of_sample_norm_err_mps2 = sample_eval_count > 0 ? static_cast<float>(sum_norm_err / sample_eval_count) : 0.0F;
        res.mean_scale_error = res.internal_passes > 0 ? static_cast<float>(sum_scale_err / (res.internal_passes * 3)) : 0.0F;
        res.mean_bias_error_mps2 = res.internal_passes > 0 ? static_cast<float>(sum_bias_err / (res.internal_passes * 3)) : 0.0F;

        // Acceptance criterion: Out-of-sample norm error must be < 0.05 m/s^2 (< 0.5% g)
        res.passed = (res.max_out_of_sample_norm_err_mps2 < 0.05F);

        std::cout << "  sigma = " << std::fixed << std::setprecision(2) << res.sigma_a << " m/s^2 (N=200): "
                  << "Internal Pass " << res.internal_passes << "/" << res.trials
                  << " | IntNormErr: " << std::setprecision(5) << res.max_internal_norm_err_mps2 << " m/s^2"
                  << " | TrueNormErr: " << res.max_out_of_sample_norm_err_mps2 << " m/s^2"
                  << " | ScaleErr: " << std::setprecision(3) << res.max_scale_error * 100.0F << "%"
                  << " | BiasErr: " << res.max_bias_error_mps2 << " m/s^2"
                  << " -> " << (res.passed ? "[PASS]" : "[FAIL: Out-of-sample norm error > 0.05 m/s^2]")
                  << std::endl;

        results.push_back(res);
    }

    std::cout << "\n--- Part 1B: Adaptive Sample Count N = 2000 for High Noise (sigma = 0.50 m/s^2) ---" << std::endl;
    {
        float sigma = 0.50F;
        std::normal_distribution<float> noise_dist(0.0F, sigma);
        NoiseStressResult res = {};
        res.sigma_a = sigma;
        res.sample_count_N = 2000U;
        res.trials = kTrialsPerSigma;
        res.internal_passes = 0;
        res.max_internal_norm_err_mps2 = 0.0F;
        res.max_out_of_sample_norm_err_mps2 = 0.0F;
        res.max_scale_error = 0.0F;
        res.max_bias_error_mps2 = 0.0F;

        double sum_norm_err = 0.0;
        double sum_scale_err = 0.0;
        double sum_bias_err = 0.0;
        uint32_t sample_eval_count = 0;

        for (uint32_t t = 0; t < kTrialsPerSigma; ++t) {
            ImuCalibrator calibrator;

            for (uint8_t f = 0; f < FACE_COUNT; ++f) {
                calibrator.start_accel_face(static_cast<AccelFace>(f), res.sample_count_N);
                for (uint32_t s = 0; s < res.sample_count_N; ++s) {
                    float raw[3];
                    for (int i = 0; i < 3; ++i) {
                        raw[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i] + noise_dist(rng);
                    }
                    calibrator.update_accel_sample(raw);
                }
                calibrator.finish_accel_face();
            }

            float reported_internal_norm_err = 0.0F;
            bool ok = calibrator.compute_accel_calibration(reported_internal_norm_err);
            if (!ok) continue;
            res.internal_passes++;

            const ImuCalibrationParams &params = calibrator.get_params();
            for (int i = 0; i < 3; ++i) {
                float s_err = fabsf(params.accel_scale[i] - true_scale[i]) / true_scale[i];
                if (s_err > res.max_scale_error) res.max_scale_error = s_err;
                sum_scale_err += s_err;

                float b_err = fabsf(params.accel_bias[i] - true_bias[i]);
                if (b_err > res.max_bias_error_mps2) res.max_bias_error_mps2 = b_err;
                sum_bias_err += b_err;
            }

            for (uint8_t f = 0; f < FACE_COUNT; ++f) {
                float raw_test[3], cal_a[3], cal_g[3], raw_g[3] = {0};
                for (int i = 0; i < 3; ++i) {
                    raw_test[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i];
                }
                calibrator.apply(raw_test, raw_g, cal_a, cal_g);
                float norm = sqrtf(cal_a[0]*cal_a[0] + cal_a[1]*cal_a[1] + cal_a[2]*cal_a[2]);
                float err = fabsf(norm - kG);
                if (err > res.max_out_of_sample_norm_err_mps2) {
                    res.max_out_of_sample_norm_err_mps2 = err;
                }
                sum_norm_err += err;
                sample_eval_count++;
            }
        }

        res.mean_out_of_sample_norm_err_mps2 = sample_eval_count > 0 ? static_cast<float>(sum_norm_err / sample_eval_count) : 0.0F;
        res.mean_scale_error = res.internal_passes > 0 ? static_cast<float>(sum_scale_err / (res.internal_passes * 3)) : 0.0F;
        res.mean_bias_error_mps2 = res.internal_passes > 0 ? static_cast<float>(sum_bias_err / (res.internal_passes * 3)) : 0.0F;
        res.passed = (res.max_out_of_sample_norm_err_mps2 < 0.05F);

        std::cout << "  sigma = 0.50 m/s^2 (N=2000): "
                  << "Internal Pass " << res.internal_passes << "/" << res.trials
                  << " | TrueNormErr: " << std::setprecision(5) << res.max_out_of_sample_norm_err_mps2 << " m/s^2"
                  << " | ScaleErr: " << std::setprecision(3) << res.max_scale_error * 100.0F << "%"
                  << " | BiasErr: " << res.max_bias_error_mps2 << " m/s^2"
                  << " -> " << (res.passed ? "[PASS: Sufficient N recovers precision]" : "[FAIL]")
                  << std::endl;
    }

    return results;
}

// ============================================================================
// Suite 2: Recovery of Large Scale (0.7-1.3) and Large Bias (-2.0 to 2.0 m/s^2)
// ============================================================================
ScaleBiasStressResult run_scale_bias_stress_suite(std::mt19937_64 &rng) {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "[SUITE 2] Scale & Bias Recovery Stress (10,000 Monte Carlo Trials)" << std::endl;
    std::cout << "========================================================" << std::endl;

    ScaleBiasStressResult result = {};
    const uint32_t kValidTrials = 10000U;
    const uint32_t kOutOfBoundTrials = 1000U;
    const uint32_t kSamplesPerFace = 200U;

    std::uniform_real_distribution<float> valid_scale_dist(0.705F, 1.295F);
    std::uniform_real_distribution<float> valid_bias_dist(-2.0F, 2.0F);
    std::normal_distribution<float> noise_dist(0.0F, 0.01F); // 0.01 m/s^2 standard sensor noise

    result.valid_trials = kValidTrials;
    result.valid_passes = 0;
    result.out_of_bounds_trials = kOutOfBoundTrials;
    result.out_of_bounds_rejected = 0;
    result.max_scale_error = 0.0F;
    result.max_bias_error_mps2 = 0.0F;
    result.max_norm_error_mps2 = 0.0F;

    double sum_scale_err = 0.0;
    double sum_bias_err = 0.0;

    // Part A: 10,000 Valid Monte Carlo Cases within [0.7, 1.3] and [-2.0, 2.0]
    for (uint32_t t = 0; t < kValidTrials; ++t) {
        float true_scale[3] = {valid_scale_dist(rng), valid_scale_dist(rng), valid_scale_dist(rng)};
        float true_bias[3] = {valid_bias_dist(rng), valid_bias_dist(rng), valid_bias_dist(rng)};

        // Include exact boundary points on select trials
        if (t == 0) {
            true_scale[0] = 0.701F; true_scale[1] = 1.299F; true_scale[2] = 1.000F;
            true_bias[0] = -2.000F; true_bias[1] = 2.000F;  true_bias[2] = 0.000F;
        }

        ImuCalibrator calibrator;
        for (uint8_t f = 0; f < FACE_COUNT; ++f) {
            calibrator.start_accel_face(static_cast<AccelFace>(f), kSamplesPerFace);
            for (uint32_t s = 0; s < kSamplesPerFace; ++s) {
                float raw[3];
                for (int i = 0; i < 3; ++i) {
                    raw[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i] + noise_dist(rng);
                }
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }

        float reported_err = 0.0F;
        if (!calibrator.compute_accel_calibration(reported_err)) {
            continue;
        }

        const ImuCalibrationParams &params = calibrator.get_params();
        for (int i = 0; i < 3; ++i) {
            float s_err = fabsf(params.accel_scale[i] - true_scale[i]) / true_scale[i];
            if (s_err > result.max_scale_error) result.max_scale_error = s_err;
            sum_scale_err += s_err;

            float b_err = fabsf(params.accel_bias[i] - true_bias[i]);
            if (b_err > result.max_bias_error_mps2) result.max_bias_error_mps2 = b_err;
            sum_bias_err += b_err;
        }

        // Test out-of-sample norm across all 6 faces
        for (uint8_t f = 0; f < FACE_COUNT; ++f) {
            float raw_test[3], cal_a[3], cal_g[3], raw_g[3] = {0};
            for (int i = 0; i < 3; ++i) {
                raw_test[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i];
            }
            calibrator.apply(raw_test, raw_g, cal_a, cal_g);
            float norm = sqrtf(cal_a[0]*cal_a[0] + cal_a[1]*cal_a[1] + cal_a[2]*cal_a[2]);
            float err = fabsf(norm - kG);
            if (err > result.max_norm_error_mps2) {
                result.max_norm_error_mps2 = err;
            }
        }
        result.valid_passes++;
    }

    result.mean_scale_error = result.valid_passes > 0 ? static_cast<float>(sum_scale_err / (result.valid_passes * 3)) : 0.0F;
    result.mean_bias_error_mps2 = result.valid_passes > 0 ? static_cast<float>(sum_bias_err / (result.valid_passes * 3)) : 0.0F;

    // Part B: 1,000 Out-of-bounds cases (scale < 0.7 or scale > 1.3, bias > 3.0 m/s^2)
    for (uint32_t t = 0; t < kOutOfBoundTrials; ++t) {
        float bad_scale[3] = {1.0F, 1.0F, 1.0F};
        float bad_bias[3] = {0.0F, 0.0F, 0.0F};

        int mode = t % 4;
        if (mode == 0) bad_scale[0] = 0.65F;             // Scale too low (< 0.7)
        else if (mode == 1) bad_scale[1] = 1.35F;        // Scale too high (> 1.3)
        else if (mode == 2) bad_bias[0] = 3.5F;          // Bias too high (> 3.0)
        else bad_bias[2] = -3.5F;                        // Bias too negative (< -3.0)

        ImuCalibrator calibrator;
        for (uint8_t f = 0; f < FACE_COUNT; ++f) {
            calibrator.start_accel_face(static_cast<AccelFace>(f), 50U);
            for (uint32_t s = 0; s < 50U; ++s) {
                float raw[3];
                for (int i = 0; i < 3; ++i) {
                    raw[i] = bad_scale[i] * kFaceTrueAccel[f][i] + bad_bias[i];
                }
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }

        float reported_err = 0.0F;
        bool ok = calibrator.compute_accel_calibration(reported_err);
        if (!ok && calibrator.get_state() == CALIB_FAILED_MATH) {
            result.out_of_bounds_rejected++;
        }
    }

    result.passed = (result.valid_passes == result.valid_trials) &&
                    (result.out_of_bounds_rejected == result.out_of_bounds_trials) &&
                    (result.max_norm_error_mps2 < 0.05F) &&
                    (result.max_scale_error < 0.01F) &&
                    (result.max_bias_error_mps2 < 0.02F);

    std::cout << "  Valid Trials: " << result.valid_passes << "/" << result.valid_trials
              << " | Out-of-Bounds Rejection: " << result.out_of_bounds_rejected << "/" << result.out_of_bounds_trials
              << std::endl;
    std::cout << "  Max Scale Error: " << std::setprecision(4) << result.max_scale_error * 100.0F << "%"
              << " | Mean Scale Error: " << result.mean_scale_error * 100.0F << "%" << std::endl;
    std::cout << "  Max Bias Error: " << std::setprecision(5) << result.max_bias_error_mps2 << " m/s^2"
              << " | Mean Bias Error: " << result.mean_bias_error_mps2 << " m/s^2" << std::endl;
    std::cout << "  Max Norm Error across all trials: " << result.max_norm_error_mps2 << " m/s^2" << std::endl;
    std::cout << "  -> " << (result.passed ? "[PASS]" : "[FAIL]") << std::endl;

    return result;
}

// ============================================================================
// Suite 3: Incomplete Face Sequences, Out-of-Order Faces & State Violations
// ============================================================================
SequenceStressResult run_sequence_stress_suite() {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "[SUITE 3] Sequence, Permutations & State Violation Harness" << std::endl;
    std::cout << "========================================================" << std::endl;

    SequenceStressResult result = {};
    const float test_scale[3] = {1.02F, 0.98F, 1.01F};
    const float test_bias[3] = {0.05F, -0.04F, 0.08F};

    // Part A: Exhaustively test all 6! = 720 permutations of face ordering
    std::vector<int> face_order = {0, 1, 2, 3, 4, 5};
    uint32_t perm_count = 0;
    uint32_t perm_pass = 0;

    do {
        perm_count++;
        ImuCalibrator calibrator;

        for (int face_idx : face_order) {
            calibrator.start_accel_face(static_cast<AccelFace>(face_idx), 50U);
            for (uint32_t s = 0; s < 50U; ++s) {
                float raw[3];
                for (int i = 0; i < 3; ++i) {
                    raw[i] = test_scale[i] * kFaceTrueAccel[face_idx][i] + test_bias[i];
                }
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }

        float max_err = 0.0F;
        if (calibrator.compute_accel_calibration(max_err) && max_err < 0.01F) {
            perm_pass++;
        }
    } while (std::next_permutation(face_order.begin(), face_order.end()));

    result.permutations_tested = perm_count;
    result.permutations_passed = perm_pass;

    // Part B: Incomplete face sequences (0 to 5 faces completed)
    uint32_t incomp_tested = 0;
    uint32_t incomp_rejected = 0;

    for (int subset_size = 0; subset_size < 6; ++subset_size) {
        incomp_tested++;
        ImuCalibrator calibrator;
        for (int f = 0; f < subset_size; ++f) {
            calibrator.start_accel_face(static_cast<AccelFace>(f), 50U);
            for (uint32_t s = 0; s < 50U; ++s) {
                float raw[3] = {test_scale[0] * kFaceTrueAccel[f][0],
                                test_scale[1] * kFaceTrueAccel[f][1],
                                test_scale[2] * kFaceTrueAccel[f][2]};
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }

        float max_err = 0.0F;
        bool ok = calibrator.compute_accel_calibration(max_err);
        if (!ok && calibrator.get_state() == CALIB_FAILED_MATH) {
            incomp_rejected++;
        }
    }

    // Also test missing an intermediate face (e.g. 0, 1, 3, 4, 5 omitting 2)
    {
        incomp_tested++;
        ImuCalibrator calibrator;
        const int subset[] = {0, 1, 3, 4, 5};
        for (int f : subset) {
            calibrator.start_accel_face(static_cast<AccelFace>(f), 50U);
            for (uint32_t s = 0; s < 50U; ++s) {
                float raw[3] = {test_scale[0] * kFaceTrueAccel[f][0],
                                test_scale[1] * kFaceTrueAccel[f][1],
                                test_scale[2] * kFaceTrueAccel[f][2]};
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }
        float max_err = 0.0F;
        if (!calibrator.compute_accel_calibration(max_err) && calibrator.get_state() == CALIB_FAILED_MATH) {
            incomp_rejected++;
        }
    }

    // Test face started but never finished (uncommitted face)
    {
        incomp_tested++;
        ImuCalibrator calibrator;
        for (int f = 0; f < 5; ++f) {
            calibrator.start_accel_face(static_cast<AccelFace>(f), 50U);
            for (uint32_t s = 0; s < 50U; ++s) {
                float raw[3] = {test_scale[0] * kFaceTrueAccel[f][0],
                                test_scale[1] * kFaceTrueAccel[f][1],
                                test_scale[2] * kFaceTrueAccel[f][2]};
                calibrator.update_accel_sample(raw);
            }
            calibrator.finish_accel_face();
        }
        // Face 5 started and updated, but finish_accel_face() omitted
        calibrator.start_accel_face(FACE_NEG_Z, 50U);
        float raw[3] = {0.0F, 0.0F, -kG};
        calibrator.update_accel_sample(raw);

        float max_err = 0.0F;
        if (!calibrator.compute_accel_calibration(max_err) && calibrator.get_state() == CALIB_FAILED_MATH) {
            incomp_rejected++;
        }
    }

    result.incomplete_tests = incomp_tested;
    result.incomplete_rejected = incomp_rejected;

    // Part C: State machine protocol violations
    uint32_t state_viol_tested = 0;
    uint32_t state_viol_rejected = 0;
    {
        ImuCalibrator c;
        float dummy[3] = {1.0F, 2.0F, 3.0F};

        // 1. Update sample when IDLE
        state_viol_tested++;
        if (!c.update_accel_sample(dummy)) state_viol_rejected++;

        // 2. Finish face when IDLE
        state_viol_tested++;
        if (!c.finish_accel_face()) state_viol_rejected++;

        // 3. Start face with invalid face index >= 6
        state_viol_tested++;
        if (!c.start_accel_face(static_cast<AccelFace>(6), 50U)) state_viol_rejected++;
        state_viol_tested++;
        if (!c.start_accel_face(static_cast<AccelFace>(255), 50U)) state_viol_rejected++;

        // 4. Start face with target_samples < 20
        state_viol_tested++;
        if (!c.start_accel_face(FACE_POS_X, 19U)) state_viol_rejected++;
        state_viol_tested++;
        if (!c.start_accel_face(FACE_POS_X, 0U)) state_viol_rejected++;

        // 5. Finish face with 0 samples collected
        state_viol_tested++;
        c.start_accel_face(FACE_POS_X, 50U);
        if (!c.finish_accel_face()) state_viol_rejected++;
    }
    result.state_violations_tested = state_viol_tested;
    result.state_violations_rejected = state_viol_rejected;

    // Part D: NaN / Inf Injection
    uint32_t nan_inf_tested = 0;
    uint32_t nan_inf_rejected = 0;
    {
        ImuCalibrator c;
        c.start_accel_face(FACE_POS_X, 50U);

        float nan_samples[4][3] = {
            {NAN, 0.0F, 0.0F},
            {0.0F, -NAN, 0.0F},
            {0.0F, 0.0F, INFINITY},
            {-INFINITY, 0.0F, 0.0F}
        };

        for (int i = 0; i < 4; ++i) {
            nan_inf_tested++;
            if (!c.update_accel_sample(nan_samples[i])) {
                nan_inf_rejected++;
            }
        }
    }
    result.nan_inf_tested = nan_inf_tested;
    result.nan_inf_rejected = nan_inf_rejected;

    // Part E: White-Box Flaw Discovery: Re-starting completed face without finish
    // When a face was completed previously, re-starting it should reset face_completed_ to false.
    // If compute_accel_calibration() is called before finish_accel_face(), it MUST reject.
    {
        ImuCalibrator c;
        for (uint8_t f = 0; f < 6; ++f) {
            c.start_accel_face(static_cast<AccelFace>(f), 50U);
            float raw[3] = {test_scale[0] * kFaceTrueAccel[f][0],
                            test_scale[1] * kFaceTrueAccel[f][1],
                            test_scale[2] * kFaceTrueAccel[f][2]};
            for (int i = 0; i < 50; ++i) c.update_accel_sample(raw);
            c.finish_accel_face();
        }
        float err = 0.0F;
        c.compute_accel_calibration(err);

        // Re-start face 0, feed 1 sample, DO NOT call finish_accel_face()
        c.start_accel_face(FACE_POS_X, 50U);
        float raw2[3] = {9.80665f, 0.0f, 0.0f};
        c.update_accel_sample(raw2);

        // In a bug-free state machine, calling compute while SAMPLING an uncompleted face must return false.
        // In the buggy firmware, it returns true because face_completed_[0] was not cleared!
        bool bug_computed = c.compute_accel_calibration(err);
        result.uncompleted_restart_rejected = (!bug_computed);
    }

    result.passed = (result.permutations_passed == result.permutations_tested) &&
                    (result.incomplete_rejected == result.incomplete_tests) &&
                    (result.state_violations_rejected == result.state_violations_tested) &&
                    (result.nan_inf_rejected == result.nan_inf_tested) &&
                    result.uncompleted_restart_rejected;

    std::cout << "  Permutations (6! = 720): " << result.permutations_passed << "/" << result.permutations_tested << " Passed" << std::endl;
    std::cout << "  Incomplete Sequences: " << result.incomplete_rejected << "/" << result.incomplete_tests << " Rejected Safely" << std::endl;
    std::cout << "  State Machine Violations: " << result.state_violations_rejected << "/" << result.state_violations_tested << " Handled" << std::endl;
    std::cout << "  NaN/Inf Injections: " << result.nan_inf_rejected << "/" << result.nan_inf_tested << " Rejected" << std::endl;
    std::cout << "  Uncompleted Re-start Rejection: "
              << (result.uncompleted_restart_rejected ? "PASSED (Rejected uncompleted face)" : "FAILED (Firmware accepted uncompleted face while actively SAMPLING!)")
              << std::endl;
    std::cout << "  -> " << (result.passed ? "[PASS]" : "[FAIL]") << std::endl;

    return result;
}

// ============================================================================
// Suite 4: Arbitrary 3D Orientations Gravity Norm Invariance (15,000 Poses)
// ============================================================================
Orientation3DResult run_orientation_3d_suite(std::mt19937_64 &rng) {
    std::cout << "\n========================================================" << std::endl;
    std::cout << "[SUITE 4] Arbitrary 3D Orientation Norm Consistency (15,000 Poses)" << std::endl;
    std::cout << "========================================================" << std::endl;

    Orientation3DResult result = {};
    const uint32_t kRandomOrientations = 10000U;
    const uint32_t kFibonacciPoints = 5000U;
    result.total_orientations = kRandomOrientations + kFibonacciPoints;
    result.max_norm_error_mps2 = 0.0F;

    // First, calibrate calibrator with realistic parameters and nominal noise
    const float true_scale[3] = {1.07F, 0.93F, 1.04F};
    const float true_bias[3] = {0.22F, -0.18F, 0.31F};
    std::normal_distribution<float> noise_dist(0.0F, 0.01F);

    ImuCalibrator calibrator;
    for (uint8_t f = 0; f < FACE_COUNT; ++f) {
        calibrator.start_accel_face(static_cast<AccelFace>(f), 200U);
        for (uint32_t s = 0; s < 200U; ++s) {
            float raw[3];
            for (int i = 0; i < 3; ++i) {
                raw[i] = true_scale[i] * kFaceTrueAccel[f][i] + true_bias[i] + noise_dist(rng);
            }
            calibrator.update_accel_sample(raw);
        }
        calibrator.finish_accel_face();
    }

    float initial_err = 0.0F;
    if (!calibrator.compute_accel_calibration(initial_err)) {
        std::cerr << "Fatal: Failed to compute baseline calibration for Suite 4!" << std::endl;
        result.passed = false;
        return result;
    }

    double sum_err = 0.0;
    double sum_sq_err = 0.0;
    std::normal_distribution<float> gauss_dist(0.0F, 1.0F);

    // Part A: 10,000 uniform random 3D orientations (Marsaglia normal method on S^2)
    for (uint32_t i = 0; i < kRandomOrientations; ++i) {
        float x = gauss_dist(rng);
        float y = gauss_dist(rng);
        float z = gauss_dist(rng);
        float inv_norm = 1.0F / sqrtf(x*x + y*y + z*z);
        float u[3] = {x * inv_norm, y * inv_norm, z * inv_norm};

        float raw[3] = {
            true_scale[0] * (u[0] * kG) + true_bias[0],
            true_scale[1] * (u[1] * kG) + true_bias[1],
            true_scale[2] * (u[2] * kG) + true_bias[2]
        };

        float cal_a[3] = {0}, cal_g[3] = {0}, raw_g[3] = {0};
        calibrator.apply(raw, raw_g, cal_a, cal_g);

        float norm = sqrtf(cal_a[0]*cal_a[0] + cal_a[1]*cal_a[1] + cal_a[2]*cal_a[2]);
        float err = fabsf(norm - kG);

        if (err > result.max_norm_error_mps2) {
            result.max_norm_error_mps2 = err;
            result.worst_u[0] = u[0];
            result.worst_u[1] = u[1];
            result.worst_u[2] = u[2];
        }
        sum_err += err;
        sum_sq_err += err * err;
    }

    // Part B: 5,000 Fibonacci spiral points on sphere (deterministic quasi-uniform coverage)
    const float phi = (1.0F + sqrtf(5.0F)) * 0.5F;
    for (uint32_t i = 0; i < kFibonacciPoints; ++i) {
        float theta = 2.0F * M_PI * static_cast<float>(i) / phi;
        float s = (2.0F * static_cast<float>(i) + 1.0F) / static_cast<float>(kFibonacciPoints) - 1.0F;
        float radius = sqrtf(fmaxf(0.0F, 1.0F - s * s));
        float u[3] = {radius * cosf(theta), radius * sinf(theta), s};

        float raw[3] = {
            true_scale[0] * (u[0] * kG) + true_bias[0],
            true_scale[1] * (u[1] * kG) + true_bias[1],
            true_scale[2] * (u[2] * kG) + true_bias[2]
        };

        float cal_a[3] = {0}, cal_g[3] = {0}, raw_g[3] = {0};
        calibrator.apply(raw, raw_g, cal_a, cal_g);

        float norm = sqrtf(cal_a[0]*cal_a[0] + cal_a[1]*cal_a[1] + cal_a[2]*cal_a[2]);
        float err = fabsf(norm - kG);

        if (err > result.max_norm_error_mps2) {
            result.max_norm_error_mps2 = err;
            result.worst_u[0] = u[0];
            result.worst_u[1] = u[1];
            result.worst_u[2] = u[2];
        }
        sum_err += err;
        sum_sq_err += err * err;
    }

    result.mean_norm_error_mps2 = static_cast<float>(sum_err / result.total_orientations);
    double variance = (sum_sq_err / result.total_orientations) - (result.mean_norm_error_mps2 * result.mean_norm_error_mps2);
    result.std_norm_error_mps2 = variance > 0.0 ? sqrtf(static_cast<float>(variance)) : 0.0F;

    result.passed = (result.max_norm_error_mps2 < 0.05F);

    std::cout << "  Evaluated Orientations: " << result.total_orientations << " poses" << std::endl;
    std::cout << "  Max Norm Error: " << std::setprecision(5) << result.max_norm_error_mps2 << " m/s^2 (< 0.05 m/s^2 threshold)" << std::endl;
    std::cout << "  Mean Norm Error: " << result.mean_norm_error_mps2 << " m/s^2" << std::endl;
    std::cout << "  Std Dev Norm Error: " << result.std_norm_error_mps2 << " m/s^2" << std::endl;
    std::cout << "  Worst-Case Orientation Vector u: ["
              << result.worst_u[0] << ", " << result.worst_u[1] << ", " << result.worst_u[2] << "]" << std::endl;
    std::cout << "  -> " << (result.passed ? "[PASS]" : "[FAIL]") << std::endl;

    return result;
}

// ============================================================================
// Main Benchmark Runner
// ============================================================================
int main() {
    std::cout << "=================================================================" << std::endl;
    std::cout << "  AMR Omni M2 ST AN4508 Accelerometer Calibration Stress Suite   " << std::endl;
    std::cout << "=================================================================" << std::endl;

    std::mt19937_64 rng(42424242ULL);

    auto noise_results = run_noise_stress_suite(rng);
    bool suite1_ok = true;
    for (const auto &nr : noise_results) {
        if (!nr.passed) suite1_ok = false;
    }

    auto scale_bias_result = run_scale_bias_stress_suite(rng);
    auto sequence_result = run_sequence_stress_suite();
    auto orientation_result = run_orientation_3d_suite(rng);

    bool all_passed = suite1_ok && scale_bias_result.passed && sequence_result.passed && orientation_result.passed;

    std::cout << "\n=================================================================" << std::endl;
    std::cout << "                       FINAL STRESS SUMMARY                      " << std::endl;
    std::cout << "=================================================================" << std::endl;
    std::cout << "  Suite 1 (Noise Resilience sigma in [0.01, 0.50]):      " << (suite1_ok ? "PASSED" : "FAILED") << std::endl;
    std::cout << "  Suite 2 (Scale s in [0.7, 1.3], Bias b in [-2, 2]):    " << (scale_bias_result.passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "  Suite 3 (720 Permutations, Incomplete, Violations):    " << (sequence_result.passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "  Suite 4 (15,000 3D Orientations Norm Consistency):     " << (orientation_result.passed ? "PASSED" : "FAILED") << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "  OVERALL VERDICT: " << (all_passed ? "APPROVE" : "REQUEST_CHANGES") << std::endl;
    std::cout << "=================================================================" << std::endl;

    return all_passed ? 0 : 1;
}
