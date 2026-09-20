#ifndef IMU_CALIBRATION_H
#define IMU_CALIBRATION_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "firmware_config.h"

// Standard gravitational acceleration (REP-103)
static const float kStandardGravityMps2 = 9.80665F;

// Acceptance criteria thresholds
static const float kMaxGyroDriftRadS = 8.7266e-4F;       // 0.05 deg/s in rad/s
static const float kMaxAccelNormErrorMps2 = 0.05F;       // < 0.5% of 1g
static const float kMaxGyroStaticVariance = 1.0e-4F;     // (rad/s)^2
static const uint32_t kMinGyroCalibrationSamples = 1000U; // >= 10s at 100Hz / 20s at 50Hz
static const uint32_t kMinAccelFaceSamples = 200U;       // >= 4s per face at 50Hz

// Default Allan Variance Noise Model Parameters
static const float kDefaultGyroNoiseDensity = 1.4e-4F;    // Ng: rad/s/sqrt(Hz)
static const float kDefaultAccelNoiseDensity = 1.9e-3F;   // Na: m/s^2/sqrt(Hz)
static const float kDefaultGyroRandomWalk = 1.5e-5F;      // Kg: rad/s^2/sqrt(Hz)
static const float kDefaultInflationAlpha = 1.8F;         // 1.5x - 2.0x

enum AccelFace : uint8_t {
    FACE_POS_X = 0U,
    FACE_NEG_X = 1U,
    FACE_POS_Y = 2U,
    FACE_NEG_Y = 3U,
    FACE_POS_Z = 4U,
    FACE_NEG_Z = 5U,
    FACE_COUNT = 6U
};

enum ImuCalibState : uint8_t {
    CALIB_IDLE = 0U,
    CALIB_GYRO_SAMPLING = 1U,
    CALIB_ACCEL_SAMPLING = 2U,
    CALIB_COMPUTING = 3U,
    CALIB_SUCCESS = 4U,
    CALIB_FAILED_MOTION = 5U,
    CALIB_FAILED_MATH = 6U
};

static const uint8_t kMaxArbitraryPoses = 24U;

struct ArbitraryPose {
    float accel_mean[3];
};

struct ExtrinsicsCalibrationParams {
    float lever_arm[3];         // [lx, ly, lz] in meters (relative to base_link)
    float time_delay_sec;       // temporal delay in seconds
    bool extrinsics_calibrated;
};

struct EncoderCalibrationParams {
    float wheel_radii[4];       // [r1, r2, r3, r4] in meters
    float wheelbase_m;          // L
    float track_width_m;        // W
    bool encoder_calibrated;
};

struct ImuCalibrationParams {
    float gyro_bias[3];            // [bx, by, bz] in rad/s
    float accel_scale[3];          // [sx, sy, sz] unitless (~1.0)
    float accel_bias[3];           // [bx, by, bz] in m/s^2
    float gyro_noise_density;      // Ng
    float accel_noise_density;     // Na
    float gyro_random_walk;        // Kg
    float covariance_inflation;    // alpha
    bool gyro_calibrated;
    bool accel_calibrated;
    bool calibration_enabled;
};

struct ImuCalibProgress {
    ImuCalibState state;
    uint8_t stage;                 // Face index or gyro stage
    uint8_t progress_pct;          // 0..100%
    uint32_t samples_collected;
    float live_metric;             // Live variance or residual error
};

class ImuCalibrator {
public:
    ImuCalibrator();

    void reset();
    void set_enabled(bool enabled) { params_.calibration_enabled = enabled; }
    bool is_enabled() const { return params_.calibration_enabled; }

    // Gyroscope Calibration
    bool start_gyro_calibration(uint32_t target_samples = kMinGyroCalibrationSamples);
    bool update_gyro_sample(const float gyro_raw[3]);
    bool finish_gyro_calibration(float &residual_drift_rad_s);

    // Accelerometer Calibration (ST AN4508 6-Position)
    bool start_accel_face(AccelFace face, uint32_t target_samples = kMinAccelFaceSamples);
    bool update_accel_sample(const float accel_raw[3]);
    bool finish_accel_face();
    bool compute_accel_calibration(float &max_norm_error);

    // Arbitrary Multi-Pose Static Calibration (Tedaldi et al. ICRA 2014 & imu_tk)
    bool add_arbitrary_pose(const float accel_mean[3]);
    bool compute_multi_pose_calibration(float &max_norm_error);
    uint8_t get_arbitrary_pose_count() const { return arbitrary_pose_count_; }
    void clear_arbitrary_poses() { arbitrary_pose_count_ = 0U; }
    const ArbitraryPose* get_arbitrary_poses() const { return arbitrary_poses_; }

    // Encoder Calibration (r1, r2, r3, r4 and L+W)
    void calibrate_wheel_radii(float true_distance_m, const float wheel_travel_m[4]);
    const EncoderCalibrationParams& get_encoder_params() const { return encoder_params_; }

    // Extrinsics Lever-Arm Calibration
    bool compute_lever_arm(float w_sq_1, float ax_1, float ay_1,
                           float w_sq_2, float ax_2, float ay_2);
    const ExtrinsicsCalibrationParams& get_extrinsics_params() const { return extrinsics_params_; }
    void set_extrinsics_params(const ExtrinsicsCalibrationParams& params) { extrinsics_params_ = params; }

    // Real-time correction & ENU transformation
    void apply(const ImuSample &raw, ImuSample &calibrated) const;
    void apply(const float raw_accel[3], const float raw_gyro[3],
               float calib_accel[3], float calib_gyro[3]) const;
    void calibrate_sample(const ImuSample &raw, ImuSample &calibrated) const {
        apply(raw, calibrated);
    }

    // Covariance matrix population with Allan variance & inflation
    void compute_covariances(float dt_sec,
                             float angular_vel_cov[9],
                             float linear_accel_cov[9]) const;

    // Getters / Setters
    const ImuCalibrationParams &get_params() const { return params_; }
    void set_params(const ImuCalibrationParams &params);
    ImuCalibProgress get_progress() const;
    ImuCalibState get_state() const { return state_; }

private:
    ImuCalibrationParams params_;
    EncoderCalibrationParams encoder_params_;
    ExtrinsicsCalibrationParams extrinsics_params_;
    ImuCalibState state_;
    uint8_t current_stage_;
    uint32_t target_samples_;
    uint32_t sample_count_;

    // Welford running statistics for Gyro
    float gyro_mean_[3];
    float gyro_m2_[3];

    // Accelerometer 6-face statistics
    float accel_face_sum_[FACE_COUNT][3];
    uint32_t accel_face_count_[FACE_COUNT];
    bool face_completed_[FACE_COUNT];

    // Arbitrary Multi-Pose storage
    ArbitraryPose arbitrary_poses_[kMaxArbitraryPoses];
    uint8_t arbitrary_pose_count_;
};

#endif // IMU_CALIBRATION_H

