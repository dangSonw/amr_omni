#include <unity.h>

#include "kalman.h"

#include "../../src/kalman.cpp"

namespace {

void test_first_measurement_initializes_estimate() {
    ScalarKalman filter(0.1F, 0.1F);
    TEST_ASSERT_FLOAT_WITHIN(0.0001F, 4.0F, filter.update(4.0F, 0.01F));
}

void test_filter_moves_toward_measurement() {
    ScalarKalman filter(0.1F, 0.1F);
    filter.reset(0.0F, 1.0F);
    const float estimate = filter.update(1.0F, 0.01F);
    TEST_ASSERT_TRUE(estimate > 0.0F);
    TEST_ASSERT_TRUE(estimate < 1.0F);
}

}  // namespace

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_first_measurement_initializes_estimate);
    RUN_TEST(test_filter_moves_toward_measurement);
    return UNITY_END();
}