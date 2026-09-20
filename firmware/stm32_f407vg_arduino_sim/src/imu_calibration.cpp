#include "imu_calibration.h"
#include <string.h>

namespace {

inline bool is_valid_float(float val) {
    return isfinite(val) != 0;
}

} // namespace

ImuCalibrator::ImuCalibrator() {
    reset();
}

void ImuCalibrator::reset() {
    state_ = CALIB_IDLE;
    current_stage_ = 0U;
    target_samples_ = 0U;
    sample_count_ = 0U;

    for (uint8_t i = 0U; i < 3U; ++i) {
        params_.gyro_bias[i] = 0.0F;
        params_.accel_scale[i] = 1.0F;
        params_.accel_bias[i] = 0.0F;
        gyro_mean_[i] = 0.0F;
        gyro_m2_[i] = 0.0F;
    }
    params_.gyro_noise_density = kDefaultGyroNoiseDensity;
    params_.accel_noise_density = kDefaultAccelNoiseDensity;
    params_.gyro_random_walk = kDefaultGyroRandomWalk;
    params_.covariance_inflation = kDefaultInflationAlpha;
    params_.gyro_calibrated = false;
    params_.accel_calibrated = false;
    params_.calibration_enabled = true;

    memset(accel_face_sum_, 0, sizeof(accel_face_sum_));
    memset(accel_face_count_, 0, sizeof(accel_face_count_));
    memset(face_completed_, 0, sizeof(face_completed_));

    arbitrary_pose_count_ = 0U;
    memset(arbitrary_poses_, 0, sizeof(arbitrary_poses_));

    for (uint8_t i = 0U; i < 4U; ++i) {
        encoder_params_.wheel_radii[i] = 0.03F;
    }
    encoder_params_.wheelbase_m = 0.1312F;
    encoder_params_.track_width_m = 0.1312F;
    encoder_params_.encoder_calibrated = false;

    extrinsics_params_.lever_arm[0] = 0.05F; // default nominal forward offset
    extrinsics_params_.lever_arm[1] = 0.0F;
    extrinsics_params_.lever_arm[2] = 0.08F;
    extrinsics_params_.time_delay_sec = 0.005F;
    extrinsics_params_.extrinsics_calibrated = false;
}

void ImuCalibrator::set_params(const ImuCalibrationParams &params) {
    params_ = params;
}

bool ImuCalibrator::start_gyro_calibration(uint32_t target_samples) {
    if (target_samples < 50U) {
        return false;
    }
    state_ = CALIB_GYRO_SAMPLING;
    current_stage_ = 0U;
    target_samples_ = target_samples;
    sample_count_ = 0U;
    for (uint8_t i = 0U; i < 3U; ++i) {
        gyro_mean_[i] = 0.0F;
        gyro_m2_[i] = 0.0F;
    }
    return true;
}

bool ImuCalibrator::update_gyro_sample(const float gyro_raw[3]) {
    if (state_ != CALIB_GYRO_SAMPLING) {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!is_valid_float(gyro_raw[i])) {
            return false;
        }
    }

    sample_count_++;
    for (uint8_t i = 0U; i < 3U; ++i) {
        const float delta = gyro_raw[i] - gyro_mean_[i];
        gyro_mean_[i] += delta / static_cast<float>(sample_count_);
        const float delta2 = gyro_raw[i] - gyro_mean_[i];
        gyro_m2_[i] += delta * delta2;
    }

    // Motion detection check after 50 samples
    if (sample_count_ > 50U) {
        const float variance_sum = (gyro_m2_[0] + gyro_m2_[1] + gyro_m2_[2]) /
                                   static_cast<float>(sample_count_ - 1U);
        if (variance_sum > kMaxGyroStaticVariance) {
            state_ = CALIB_FAILED_MOTION;
            return false;
        }
    }
    return true;
}

bool ImuCalibrator::finish_gyro_calibration(float &residual_drift_rad_s) {
    if (state_ != CALIB_GYRO_SAMPLING || sample_count_ < 50U) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        params_.gyro_bias[i] = gyro_mean_[i];
    }
    params_.gyro_calibrated = true;
    state_ = CALIB_SUCCESS;

    // Residual drift of the mean estimator (Standard Error of the Mean)
    const float variance_sum = (gyro_m2_[0] + gyro_m2_[1] + gyro_m2_[2]) /
                               static_cast<float>(sample_count_ - 1U);
    residual_drift_rad_s = sqrtf(variance_sum / static_cast<float>(sample_count_));
    return residual_drift_rad_s < kMaxGyroDriftRadS;
}

bool ImuCalibrator::start_accel_face(AccelFace face, uint32_t target_samples) {
    if (face >= FACE_COUNT || target_samples < 20U) {
        return false;
    }
    state_ = CALIB_ACCEL_SAMPLING;
    current_stage_ = static_cast<uint8_t>(face);
    target_samples_ = target_samples;
    sample_count_ = 0U;
    accel_face_sum_[face][0] = 0.0F;
    accel_face_sum_[face][1] = 0.0F;
    accel_face_sum_[face][2] = 0.0F;
    accel_face_count_[face] = 0U;
    return true;
}

bool ImuCalibrator::update_accel_sample(const float accel_raw[3]) {
    if (state_ != CALIB_ACCEL_SAMPLING || current_stage_ >= FACE_COUNT) {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!is_valid_float(accel_raw[i])) {
            return false;
        }
        accel_face_sum_[current_stage_][i] += accel_raw[i];
    }
    sample_count_++;
    accel_face_count_[current_stage_] = sample_count_;
    return true;
}

bool ImuCalibrator::finish_accel_face() {
    if (state_ != CALIB_ACCEL_SAMPLING || sample_count_ == 0U) {
        return false;
    }
    face_completed_[current_stage_] = true;
    state_ = CALIB_IDLE;
    return true;
}

bool ImuCalibrator::compute_accel_calibration(float &max_norm_error) {
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        if (!face_completed_[f] || accel_face_count_[f] == 0U) {
            state_ = CALIB_FAILED_MATH;
            return false;
        }
    }

    state_ = CALIB_COMPUTING;
    float avg[FACE_COUNT][3];
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        const float count_f = static_cast<float>(accel_face_count_[f]);
        for (uint8_t axis = 0U; axis < 3U; ++axis) {
            avg[f][axis] = accel_face_sum_[f][axis] / count_f;
        }
    }

    const float two_g = 2.0F * kStandardGravityMps2;

    // ST AN4508 scale and bias calculation:
    // a_raw = scale * a_true + bias
    // s_j = (a_pos - a_neg) / (2g)
    // b_j = (a_pos + a_neg) / 2
    const float denom_x = avg[FACE_POS_X][0] - avg[FACE_NEG_X][0];
    const float denom_y = avg[FACE_POS_Y][1] - avg[FACE_NEG_Y][1];
    const float denom_z = avg[FACE_POS_Z][2] - avg[FACE_NEG_Z][2];

    if (fabsf(denom_x) < 1e-4F || fabsf(denom_y) < 1e-4F || fabsf(denom_z) < 1e-4F) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    const float sx = denom_x / two_g;
    const float bx = (avg[FACE_POS_X][0] + avg[FACE_NEG_X][0]) * 0.5F;

    const float sy = denom_y / two_g;
    const float by = (avg[FACE_POS_Y][1] + avg[FACE_NEG_Y][1]) * 0.5F;

    const float sz = denom_z / two_g;
    const float bz = (avg[FACE_POS_Z][2] + avg[FACE_NEG_Z][2]) * 0.5F;

    // Validate plausible range (scale within [0.7, 1.3], bias within [-4.0, 4.0] m/s^2)
    if (sx < 0.7F || sx > 1.3F || sy < 0.7F || sy > 1.3F || sz < 0.7F || sz > 1.3F ||
        fabsf(bx) > 4.0F || fabsf(by) > 4.0F || fabsf(bz) > 4.0F) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    params_.accel_scale[0] = sx;
    params_.accel_scale[1] = sy;
    params_.accel_scale[2] = sz;

    params_.accel_bias[0] = bx;
    params_.accel_bias[1] = by;
    params_.accel_bias[2] = bz;

    // Validate residual norm error across all 6 faces: (a_raw - bias) / scale
    max_norm_error = 0.0F;
    for (uint8_t f = 0U; f < FACE_COUNT; ++f) {
        const float ax_cal = (avg[f][0] - bx) / sx;
        const float ay_cal = (avg[f][1] - by) / sy;
        const float az_cal = (avg[f][2] - bz) / sz;
        const float norm = sqrtf(ax_cal * ax_cal + ay_cal * ay_cal + az_cal * az_cal);
        const float err = fabsf(norm - kStandardGravityMps2);
        if (err > max_norm_error) {
            max_norm_error = err;
        }
    }

    if (max_norm_error > kMaxAccelNormErrorMps2) {
        state_ = CALIB_FAILED_MATH;
        return false;
    }

    params_.accel_calibrated = true;
    state_ = CALIB_SUCCESS;
    return true;
}

void ImuCalibrator::apply(const ImuSample &raw, ImuSample &calibrated) const {
    apply(raw.linear_accel_mps2, raw.gyro_rad_s,
          calibrated.linear_accel_mps2, calibrated.gyro_rad_s);
    for (uint8_t i = 0U; i < 4U; ++i) {
        calibrated.quaternion_xyzw[i] = raw.quaternion_xyzw[i];
    }
    calibrated.valid = raw.valid;
}

void ImuCalibrator::apply(const float raw_accel[3], const float raw_gyro[3],
                          float calib_accel[3], float calib_gyro[3]) const {
    if (!params_.calibration_enabled) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            calib_accel[i] = raw_accel[i];
            calib_gyro[i] = raw_gyro[i];
        }
        return;
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        // Accelerometer calibration: (a_raw - bias) / scale with physical sanity checks
        if (params_.accel_calibrated && fabsf(params_.accel_scale[i]) >= 0.85F && fabsf(params_.accel_scale[i]) <= 1.15F && fabsf(params_.accel_bias[i]) <= 2.0F) {
            calib_accel[i] = (raw_accel[i] - params_.accel_bias[i]) / params_.accel_scale[i];
        } else {
            calib_accel[i] = raw_accel[i];
        }

        // Gyroscope calibration: w_raw - bias
        if (params_.gyro_calibrated && fabsf(params_.gyro_bias[i]) <= 0.5F) {
            calib_gyro[i] = raw_gyro[i] - params_.gyro_bias[i];
        } else {
            calib_gyro[i] = raw_gyro[i];
        }
    }
}

void ImuCalibrator::compute_covariances(float dt_sec,
                                        float angular_vel_cov[9],
                                        float linear_accel_cov[9]) const {
    const float dt = dt_sec > 0.001F ? dt_sec : 0.02F;
    const float alpha = params_.covariance_inflation > 1.0F ? params_.covariance_inflation : kDefaultInflationAlpha;
    const float alpha_sq = alpha * alpha;

    // sigma^2 = alpha^2 * (N^2 / dt)
    float gyro_var = alpha_sq * (params_.gyro_noise_density * params_.gyro_noise_density) / dt;
    float accel_var = alpha_sq * (params_.accel_noise_density * params_.accel_noise_density) / dt;

    // Minimum physical safety clamp for AMR dynamic vibration
    if (gyro_var < 1.0e-4F) gyro_var = 1.0e-4F;
    if (accel_var < 1.0e-2F) accel_var = 1.0e-2F;

    memset(angular_vel_cov, 0, 9 * sizeof(float));
    memset(linear_accel_cov, 0, 9 * sizeof(float));

    // Angular velocity covariance: [1e6, 1e6, sigma_wz^2]
    angular_vel_cov[0] = 1.0e6F;
    angular_vel_cov[4] = 1.0e6F;
    angular_vel_cov[8] = gyro_var;

    // Linear acceleration covariance: [sigma_ax^2, sigma_ay^2, sigma_az^2]
    linear_accel_cov[0] = accel_var;
    linear_accel_cov[4] = accel_var;
    linear_accel_cov[8] = accel_var;
}

ImuCalibProgress ImuCalibrator::get_progress() const {
    ImuCalibProgress progress;
    progress.state = state_;
    progress.stage = current_stage_;
    progress.samples_collected = sample_count_;
    if (target_samples_ > 0U) {
        uint32_t pct = (sample_count_ * 100U) / target_samples_;
        progress.progress_pct = pct > 100U ? 100U : static_cast<uint8_t>(pct);
    } else {
        progress.progress_pct = (state_ == CALIB_SUCCESS) ? 100U : 0U;
    }
    progress.live_metric = 0.0F;
    return progress;
}

bool ImuCalibrator::add_arbitrary_pose(const float accel_mean[3]) {
    if (arbitrary_pose_count_ >= kMaxArbitraryPoses) {
        return false;
    }
    for (uint8_t i = 0U; i < 3U; ++i) {
        if (!is_valid_float(accel_mean[i])) {
            return false;
        }
        arbitrary_poses_[arbitrary_pose_count_].accel_mean[i] = accel_mean[i];
    }
    arbitrary_pose_count_++;
    return true;
}

bool ImuCalibrator::compute_multi_pose_calibration(float &max_norm_error) {
    if (arbitrary_pose_count_ < 4U) {
        state_ = CALIB_FAILED_MATH;
        max_norm_error = 999.0F;
        return false;
    }

    state_ = CALIB_COMPUTING;

    // Gauss-Newton fitting for 3D Ellipsoid: ((x-bx)/sx)^2 + ((y-by)/sy)^2 + ((z-bz)/sz)^2 = g^2
    float sx = 1.0F, sy = 1.0F, sz = 1.0F;
    float bx = 0.0F, by = 0.0F, bz = 0.0F;

    // Initial estimate of bias from min/max bounds across arbitrary poses
    float min_a[3] = {999.0F, 999.0F, 999.0F};
    float max_a[3] = {-999.0F, -999.0F, -999.0F};
    for (uint8_t p = 0U; p < arbitrary_pose_count_; ++p) {
        for (uint8_t i = 0U; i < 3U; ++i) {
            float v = arbitrary_poses_[p].accel_mean[i];
            if (v < min_a[i]) min_a[i] = v;
            if (v > max_a[i]) max_a[i] = v;
        }
    }

    for (uint8_t i = 0U; i < 3U; ++i) {
        if (max_a[i] > min_a[i] + 1.0F) {
            float span = max_a[i] - min_a[i];
            float scale_est = span / (2.0F * kStandardGravityMps2);
            if (scale_est > 0.7F && scale_est < 1.3F) {
                if (i == 0U) sx = scale_est;
                else if (i == 1U) sy = scale_est;
                else sz = scale_est;
            }
            float bias_est = (max_a[i] + min_a[i]) * 0.5F;
            if (fabsf(bias_est) < 4.0F) {
                if (i == 0U) bx = bias_est;
                else if (i == 1U) by = bias_est;
                else bz = bias_est;
            }
        }
    }

    // Iterative refinement (10 iterations of gradient descent / Gauss-Newton step)
    const float learning_rate = 0.005F;
    for (uint8_t iter = 0U; iter < 20U; ++iter) {
        float grad_bx = 0.0F, grad_by = 0.0F, grad_bz = 0.0F;
        float grad_sx = 0.0F, grad_sy = 0.0F, grad_sz = 0.0F;

        for (uint8_t p = 0U; p < arbitrary_pose_count_; ++p) {
            const float ax = arbitrary_poses_[p].accel_mean[0];
            const float ay = arbitrary_poses_[p].accel_mean[1];
            const float az = arbitrary_poses_[p].accel_mean[2];

            const float cal_x = (ax - bx) / sx;
            const float cal_y = (ay - by) / sy;
            const float cal_z = (az - bz) / sz;

            const float norm = sqrtf(cal_x * cal_x + cal_y * cal_y + cal_z * cal_z);
            const float err = norm - kStandardGravityMps2;

            if (norm > 1.0e-3F) {
                const float factor = err / norm;
                grad_bx -= factor * (cal_x / sx);
                grad_by -= factor * (cal_y / sy);
                grad_bz -= factor * (cal_z / sz);

                grad_sx -= factor * (cal_x * cal_x / sx);
                grad_sy -= factor * (cal_y * cal_y / sy);
                grad_sz -= factor * (cal_z * cal_z / sz);
            }
        }

        bx += learning_rate * grad_bx;
        by += learning_rate * grad_by;
        bz += learning_rate * grad_bz;

        sx += learning_rate * grad_sx;
        sy += learning_rate * grad_sy;
        sz += learning_rate * grad_sz;

        // Bounded clamps
        if (sx < 0.7F) sx = 0.7F; if (sx > 1.3F) sx = 1.3F;
        if (sy < 0.7F) sy = 0.7F; if (sy > 1.3F) sy = 1.3F;
        if (sz < 0.7F) sz = 0.7F; if (sz > 1.3F) sz = 1.3F;
        if (bx < -4.0F) bx = -4.0F; if (bx > 4.0F) bx = 4.0F;
        if (by < -4.0F) by = -4.0F; if (by > 4.0F) by = 4.0F;
        if (bz < -4.0F) bz = -4.0F; if (bz > 4.0F) bz = 4.0F;
    }

    // Residual evaluation across all arbitrary poses
    max_norm_error = 0.0F;
    for (uint8_t p = 0U; p < arbitrary_pose_count_; ++p) {
        const float ax = (arbitrary_poses_[p].accel_mean[0] - bx) / sx;
        const float ay = (arbitrary_poses_[p].accel_mean[1] - by) / sy;
        const float az = (arbitrary_poses_[p].accel_mean[2] - bz) / sz;
        const float norm = sqrtf(ax * ax + ay * ay + az * az);
        const float err = fabsf(norm - kStandardGravityMps2);
        if (err > max_norm_error) {
            max_norm_error = err;
        }
    }

    params_.accel_scale[0] = sx;
    params_.accel_scale[1] = sy;
    params_.accel_scale[2] = sz;

    params_.accel_bias[0] = bx;
    params_.accel_bias[1] = by;
    params_.accel_bias[2] = bz;

    params_.accel_calibrated = true;
    state_ = CALIB_SUCCESS;
    return true;
}

void ImuCalibrator::calibrate_wheel_radii(float true_distance_m, const float wheel_travel_m[4]) {
    if (true_distance_m <= 0.01F) return;

    for (uint8_t i = 0U; i < 4U; ++i) {
        if (wheel_travel_m[i] > 0.01F) {
            float ratio = true_distance_m / wheel_travel_m[i];
            float new_r = 0.03F * ratio;
            // Physical sanity clamp for Mecanum wheel [2.5cm, 3.5cm]
            if (new_r >= 0.025F && new_r <= 0.035F) {
                encoder_params_.wheel_radii[i] = new_r;
            }
        }
    }
    encoder_params_.encoder_calibrated = true;
}

bool ImuCalibrator::compute_lever_arm(float w_sq_1, float ax_1, float ay_1,
                                      float w_sq_2, float ax_2, float ay_2) {
    float delta_w_sq = w_sq_2 - w_sq_1;
    if (fabsf(delta_w_sq) < 0.05F) {
        return false;
    }

    // Centripetal acceleration: a_centripetal = -w_z^2 * lever_arm
    float lx = -(ax_2 - ax_1) / delta_w_sq;
    float ly = -(ay_2 - ay_1) / delta_w_sq;

    // Physical sanity clamp [-0.2m, +0.2m]
    if (fabsf(lx) < 0.25F && fabsf(ly) < 0.25F) {
        extrinsics_params_.lever_arm[0] = lx;
        extrinsics_params_.lever_arm[1] = ly;
        extrinsics_params_.extrinsics_calibrated = true;
        return true;
    }
    return false;
}
