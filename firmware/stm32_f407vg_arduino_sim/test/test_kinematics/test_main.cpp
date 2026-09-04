#include <math.h>

#include <unity.h>

#include "firmware_config.h"
#include "kinematics.h"

#include "../../src/kinematics.cpp"

namespace {

FirmwareSettings default_settings() {
    FirmwareSettings settings = {
        kWheelRadiusM,
        kWheelbaseM,
        kTrackWidthM,
        kMaxWheelSpeedRadS,
        kCommandTimeoutMs,
        kDefaultMotorKp,
        kDefaultMotorKi,
        kDefaultMotorKd,
    };
    return settings;
}

void test_inverse_forward_consistency() {
    const FirmwareSettings settings = default_settings();
    const TwistCommand expected = {0.2F, -0.1F, 0.3F};
    float wheel_speeds[kWheelCount];
    TwistCommand actual;
    TEST_ASSERT_TRUE(compute_wheel_speeds(expected, settings, wheel_speeds));
    TEST_ASSERT_TRUE(compute_body_twist(wheel_speeds, settings, actual));
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, expected.vx_mps, actual.vx_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, expected.vy_mps, actual.vy_mps);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, expected.wz_rad_s, actual.wz_rad_s);
}

void test_wheel_speed_is_saturated() {
    const FirmwareSettings settings = default_settings();
    const TwistCommand command = {10.0F, 10.0F, 10.0F};
    float wheel_speeds[kWheelCount];
    TEST_ASSERT_TRUE(compute_wheel_speeds(command, settings, wheel_speeds));
    for (uint8_t index = 0U; index < kWheelCount; ++index) {
        TEST_ASSERT_TRUE(fabsf(wheel_speeds[index]) <=
                         settings.max_wheel_speed_rad_s);
    }
}

void test_invalid_geometry_is_rejected() {
    FirmwareSettings settings = default_settings();
    settings.wheel_radius_m = 0.0F;
    const TwistCommand command = {0.0F, 0.0F, 0.0F};
    float wheel_speeds[kWheelCount];
    TEST_ASSERT_FALSE(compute_wheel_speeds(command, settings, wheel_speeds));
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_inverse_forward_consistency);
    RUN_TEST(test_wheel_speed_is_saturated);
    RUN_TEST(test_invalid_geometry_is_rejected);
    return UNITY_END();
}