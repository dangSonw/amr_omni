#include <math.h>

#include <unity.h>

#include "pid.h"

#include "../../src/pid.cpp"

namespace {

void test_pid_output_is_bounded() {
    WheelSpeedPid controller(1.0F, 0.5F, 0.0F, 1.0F);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 1.0F,
                             controller.update(10.0F, 0.0F, 0.01F));
}

void test_pid_reduces_output_when_feedback_reaches_setpoint() {
    WheelSpeedPid controller(1.0F, 0.0F, 0.0F, 1.0F);
    controller.update(1.0F, 0.0F, 0.01F);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F,
                             controller.update(1.0F, 1.0F, 0.01F));
}

void test_pid_reset_clears_integrator() {
    WheelSpeedPid controller(0.0F, 1.0F, 0.0F, 1.0F);
    controller.update(1.0F, 0.0F, 0.01F);
    controller.reset();
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 0.0F,
                             controller.update(0.0F, 0.0F, 0.01F));
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_pid_output_is_bounded);
    RUN_TEST(test_pid_reduces_output_when_feedback_reaches_setpoint);
    RUN_TEST(test_pid_reset_clears_integrator);
    return UNITY_END();
}