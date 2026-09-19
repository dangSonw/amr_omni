#include <math.h>
#include <unity.h>

#include "encoder_pll.h"
#include "../../src/encoder_pll.cpp"

namespace {

void test_pll_initialization_and_gains() {
    EncoderPll pll(20.0F, 2048U);
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 20.0F, pll.bandwidth());
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 40.0F, pll.kp());
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 400.0F, pll.ki());
    // Verify critical damping: ki == 0.25 * kp^2
    TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.25F * pll.kp() * pll.kp(), pll.ki());
    TEST_ASSERT_EQUAL_UINT32(2048U, pll.counts_per_revolution());
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, pll.velocity());
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, pll.position());
}

void test_timer_rollover_forward() {
    // 65530 -> 10 across 16-bit boundary: delta = +16
    const uint16_t prev = 65530U;
    const uint16_t curr = 10U;
    const int16_t delta = EncoderPll::compute_timer_delta(curr, prev);
    TEST_ASSERT_EQUAL_INT16(16, delta);

    EncoderPll pll(20.0F, 2048U);
    pll.update_raw(prev, 0.01F);  // initializes prev_timer_count_
    const float vel = pll.update_raw(curr, 0.01F);
    TEST_ASSERT_TRUE(vel > 0.0F);
}

void test_timer_rollover_reverse() {
    // 10 -> 65530 across 16-bit boundary: delta = -16
    const uint16_t prev = 10U;
    const uint16_t curr = 65530U;
    const int16_t delta = EncoderPll::compute_timer_delta(curr, prev);
    TEST_ASSERT_EQUAL_INT16(-16, delta);

    EncoderPll pll(20.0F, 2048U);
    pll.update_raw(prev, 0.01F);
    const float vel = pll.update_raw(curr, 0.01F);
    TEST_ASSERT_TRUE(vel < 0.0F);
}

void test_zero_speed_watchdog_timeout() {
    EncoderPll pll(20.0F, 2048U);
    // Spin up to 10 rad/s
    const float dt = 0.01F;
    const int32_t counts_per_step = static_cast<int32_t>(10.0F * dt * 2048.0F / 6.2831853F);  // ~33 counts
    for (int i = 0; i < 50; ++i) {
        pll.update(counts_per_step, dt);
    }
    TEST_ASSERT_TRUE(pll.velocity() > 5.0F);

    // Motor stops: 0 counts arrival
    // 10 ms (dt=0.01)
    pll.update(0, dt);
    TEST_ASSERT_TRUE(pll.velocity() > 0.0F);

    // 20 ms
    pll.update(0, dt);
    // 30 ms
    pll.update(0, dt);
    // 40 ms
    pll.update(0, dt);
    // 50 ms -> Watchdog triggers at >= 50 ms!
    const float vel_at_50ms = pll.update(0, dt);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, vel_at_50ms);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F, pll.velocity());
}

void test_mt_hybrid_velocity_decay() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    // Fast initial velocity
    for (int i = 0; i < 50; ++i) {
        pll.update(200, dt);
    }
    const float initial_speed = pll.velocity();
    TEST_ASSERT_TRUE(initial_speed > 50.0F);

    // First cycle of zero pulses: max allowed speed is rad_per_count / dt
    const float rad_per_count = 6.2831853F / 2048.0F;
    const float max_speed_cycle1 = rad_per_count / dt;
    pll.update(0, dt);
    TEST_ASSERT_TRUE(pll.velocity() <= max_speed_cycle1 + 0.01F);

    // Second cycle: max allowed speed is rad_per_count / (2*dt)
    const float max_speed_cycle2 = rad_per_count / (2.0F * dt);
    pll.update(0, dt);
    TEST_ASSERT_TRUE(pll.velocity() <= max_speed_cycle2 + 0.01F);
}

void test_smooth_estimation_low_to_high_speed() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float rad_per_count = 6.283185307F / 2048.0F;

    // 1. Low speed: 0.01 m/s on 0.03 m wheel -> omega = 0.3333 rad/s
    const float low_omega = 0.01F / 0.03F;  // ~0.333 rad/s
    // At 0.333 rad/s and dt=0.01s: delta_theta = 0.00333 rad ~ 1.09 counts per step
    float float_counts = 0.0F;
    for (int step = 0; step < 100; ++step) {
        float_counts += (low_omega * dt) / rad_per_count;
        int32_t c = static_cast<int32_t>(float_counts);
        float_counts -= static_cast<float>(c);
        pll.update(c, dt);
    }
    // After settling (~1 sec), PLL velocity should be smooth and close to low_omega
    TEST_ASSERT_FLOAT_WITHIN(0.08F, low_omega, pll.velocity());

    // 2. High speed: 1.5 m/s on 0.03 m wheel -> omega = 50.0 rad/s
    const float high_omega = 1.5F / 0.03F;  // 50.0 rad/s
    pll.reset(0.0F, 0.0F);
    float_counts = 0.0F;
    for (int step = 0; step < 100; ++step) {
        float_counts += (high_omega * dt) / rad_per_count;
        int32_t c = static_cast<int32_t>(float_counts);
        float_counts -= static_cast<float>(c);
        pll.update(c, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(1.0F, high_omega, pll.velocity());
}

void test_phase_tracking_during_velocity_ramp() {
    EncoderPll pll(20.0F, 2048U);
    const float dt = 0.01F;
    const float rad_per_count = 6.283185307F / 2048.0F;
    const float accel = 5.0F;  // rad/s^2

    float current_omega = 0.0F;
    float float_counts = 0.0F;
    // Accelerate for 2.0 seconds (200 steps)
    for (int step = 0; step < 200; ++step) {
        current_omega += accel * dt;
        float_counts += (current_omega * dt) / rad_per_count;
        int32_t c = static_cast<int32_t>(float_counts);
        float_counts -= static_cast<float>(c);
        pll.update(c, dt);
    }
    // During constant acceleration ramp, 2nd order PLL tracks with minimal lag
    // At 2 seconds: omega = 10.0 rad/s.
    TEST_ASSERT_FLOAT_WITHIN(0.5F, current_omega, pll.velocity());
}

void test_nan_rejection() {
    EncoderPll pll(20.0F, 2048U);
    pll.reset(1.0F, 5.0F);
    // Calling update with dt <= 0 defaults safely
    const float vel = pll.update(0, -1.0F);
    TEST_ASSERT_TRUE(isfinite(vel));
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pll_initialization_and_gains);
    RUN_TEST(test_timer_rollover_forward);
    RUN_TEST(test_timer_rollover_reverse);
    RUN_TEST(test_zero_speed_watchdog_timeout);
    RUN_TEST(test_mt_hybrid_velocity_decay);
    RUN_TEST(test_smooth_estimation_low_to_high_speed);
    RUN_TEST(test_phase_tracking_during_velocity_ramp);
    RUN_TEST(test_nan_rejection);
    return UNITY_END();
}
