/**
 * @file test_pwm_driver.cpp
 * @brief Unit tests for RP2040 PWM driver — boundary, ramp, safety limits.
 */

#include "../../src/pwm_driver.h"
#include <unity.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_pwm_initialization(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};

    /* Frequency below 20 kHz must be rejected */
    bool res_low = init_pwm_channels(pins, 10000);
    TEST_ASSERT_FALSE(res_low);

    /* Frequency >= 20 kHz accepted */
    bool res_ok = init_pwm_channels(pins, 25000);
    TEST_ASSERT_TRUE(res_ok);
    TEST_ASSERT_EQUAL_UINT32(25000, get_pwm_frequency());
}

void test_pwm_duty_boundary(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    /* Valid duty = 0 */
    TEST_ASSERT_TRUE(set_pwm_duty(0, 0));
    TEST_ASSERT_EQUAL_UINT16(0, get_pwm_duty(0));

    /* Valid duty = 300 (within MAX_DUTY_LIMIT) */
    TEST_ASSERT_TRUE(set_pwm_duty(0, 300));
    TEST_ASSERT_EQUAL_UINT16(300, get_pwm_duty(0));

    /* Duty at MAX_DUTY_LIMIT (460) should be accepted as-is */
    TEST_ASSERT_TRUE(set_pwm_duty(0, MAX_DUTY_LIMIT));
    TEST_ASSERT_EQUAL_UINT16(MAX_DUTY_LIMIT, get_pwm_duty(0));

    /* Duty above MAX_DUTY_LIMIT: clamped, NOT rejected. */
    TEST_ASSERT_TRUE(set_pwm_duty(0, 800));
    TEST_ASSERT_EQUAL_UINT16(MAX_DUTY_LIMIT, get_pwm_duty(0));

    TEST_ASSERT_TRUE(set_pwm_duty(0, 1023));
    TEST_ASSERT_EQUAL_UINT16(MAX_DUTY_LIMIT, get_pwm_duty(0));

    /* Out-of-bound channel (>= 5) should fail */
    TEST_ASSERT_FALSE(set_pwm_duty(5, 100));
}

void test_smooth_ramp_up(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    set_pwm_duty(0, 0);
    TEST_ASSERT_EQUAL_UINT16(0, get_pwm_duty(0));

    uint16_t d1 = ramp_pwm_duty(0, 100, 10);
    TEST_ASSERT_EQUAL_UINT16(10, d1);

    uint16_t d2 = ramp_pwm_duty(0, 100, 10);
    TEST_ASSERT_EQUAL_UINT16(20, d2);

    uint16_t d3 = ramp_pwm_duty(0, 25, 50);
    TEST_ASSERT_EQUAL_UINT16(25, d3);
}

void test_smooth_ramp_down(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    set_pwm_duty(1, 200);
    TEST_ASSERT_EQUAL_UINT16(200, get_pwm_duty(1));

    uint16_t d1 = ramp_pwm_duty(1, 100, 30);
    TEST_ASSERT_EQUAL_UINT16(170, d1);

    uint16_t d2 = ramp_pwm_duty(1, 160, 50);
    TEST_ASSERT_EQUAL_UINT16(160, d2);
}

void test_ramp_clamps_to_max_duty_limit(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    for (int i = 0; i < 200; i++) {
        ramp_pwm_duty(2, 1000, 10);
    }
    TEST_ASSERT_EQUAL_UINT16(MAX_DUTY_LIMIT, get_pwm_duty(2));
}

void test_kill_all_pwm(void) {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        set_pwm_duty(i, 200);
    }

    kill_all_pwm();

    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        TEST_ASSERT_EQUAL_UINT16(0, get_pwm_duty(i));
    }
}

void test_ramp_invalid_channel(void) {
    uint16_t result = ramp_pwm_duty(10, 100, 10);
    TEST_ASSERT_EQUAL_UINT16(0, result);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pwm_initialization);
    RUN_TEST(test_pwm_duty_boundary);
    RUN_TEST(test_smooth_ramp_up);
    RUN_TEST(test_smooth_ramp_down);
    RUN_TEST(test_ramp_clamps_to_max_duty_limit);
    RUN_TEST(test_kill_all_pwm);
    RUN_TEST(test_ramp_invalid_channel);
    return UNITY_END();
}
