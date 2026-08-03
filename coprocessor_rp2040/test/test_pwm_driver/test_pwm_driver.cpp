/**
 * @file test_pwm_driver.cpp
 * @brief Unit tests for RP2040 PWM driver — boundary, ramp, safety limits.
 */

#include "../../src/pwm_driver.h"
#include <assert.h>
#include <stdio.h>

void test_pwm_initialization() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};

    /* Frequency below 20 kHz must be rejected */
    bool res_low = init_pwm_channels(pins, 10000);
    assert(res_low == false);

    /* Frequency >= 20 kHz accepted */
    bool res_ok = init_pwm_channels(pins, 25000);
    assert(res_ok == true);
    assert(get_pwm_frequency() == 25000);

    printf("[PASS] test_pwm_initialization\n");
}

void test_pwm_duty_boundary() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    /* Valid duty = 0 */
    assert(set_pwm_duty(0, 0) == true);
    assert(get_pwm_duty(0) == 0);

    /* Valid duty = 512 (within MAX_DUTY_LIMIT) */
    assert(set_pwm_duty(0, 300) == true);
    assert(get_pwm_duty(0) == 300);

    /* Duty at MAX_DUTY_LIMIT (460) should be accepted as-is */
    assert(set_pwm_duty(0, MAX_DUTY_LIMIT) == true);
    assert(get_pwm_duty(0) == MAX_DUTY_LIMIT);

    /* Duty above MAX_DUTY_LIMIT: clamped, NOT rejected.
     * PID overshoot should be handled gracefully. */
    assert(set_pwm_duty(0, 800) == true);
    assert(get_pwm_duty(0) == MAX_DUTY_LIMIT);  /* Clamped to 460 */

    assert(set_pwm_duty(0, 1023) == true);
    assert(get_pwm_duty(0) == MAX_DUTY_LIMIT);  /* Still clamped */

    /* Out-of-bound channel (>= 5) should fail */
    assert(set_pwm_duty(5, 100) == false);

    printf("[PASS] test_pwm_duty_boundary\n");
}

void test_smooth_ramp_up() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    /* Start at 0 */
    set_pwm_duty(0, 0);
    assert(get_pwm_duty(0) == 0);

    /* Ramp toward target 100, step 10 */
    uint16_t d1 = ramp_pwm_duty(0, 100, 10);
    assert(d1 == 10);   /* 0 + 10 = 10 */

    uint16_t d2 = ramp_pwm_duty(0, 100, 10);
    assert(d2 == 20);   /* 10 + 10 = 20 */

    /* Ramp with large step overshooting target */
    uint16_t d3 = ramp_pwm_duty(0, 25, 50);
    assert(d3 == 25);   /* Clamped to target, not 70 */

    printf("[PASS] test_smooth_ramp_up\n");
}

void test_smooth_ramp_down() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    /* Set to 200 */
    set_pwm_duty(1, 200);
    assert(get_pwm_duty(1) == 200);

    /* Ramp down toward 100, step 30 */
    uint16_t d1 = ramp_pwm_duty(1, 100, 30);
    assert(d1 == 170);  /* 200 - 30 = 170 */

    /* Ramp down past target clamps */
    uint16_t d2 = ramp_pwm_duty(1, 160, 50);
    assert(d2 == 160);  /* 170 - 50 would be 120, but clamped to 160 */

    printf("[PASS] test_smooth_ramp_down\n");
}

void test_ramp_clamps_to_max_duty_limit() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    /* Ramp toward 1000 (above MAX_DUTY_LIMIT) */
    for (int i = 0; i < 200; i++) {
        ramp_pwm_duty(2, 1000, 10);
    }
    /* Should cap at MAX_DUTY_LIMIT, not 1000 */
    assert(get_pwm_duty(2) == MAX_DUTY_LIMIT);

    printf("[PASS] test_ramp_clamps_to_max_duty_limit\n");
}

void test_kill_all_pwm() {
    uint8_t pins[PWM_CHANNELS] = {0, 1, 2, 3, 4};
    init_pwm_channels(pins, 25000);

    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        set_pwm_duty(i, 200);
    }

    kill_all_pwm();

    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        assert(get_pwm_duty(i) == 0);
    }

    printf("[PASS] test_kill_all_pwm\n");
}

void test_ramp_invalid_channel() {
    uint16_t result = ramp_pwm_duty(10, 100, 10);
    assert(result == 0);
    printf("[PASS] test_ramp_invalid_channel\n");
}

int main() {
    printf("--- Running PWM Driver Unit Tests ---\n");
    test_pwm_initialization();
    test_pwm_duty_boundary();
    test_smooth_ramp_up();
    test_smooth_ramp_down();
    test_ramp_clamps_to_max_duty_limit();
    test_kill_all_pwm();
    test_ramp_invalid_channel();
    printf("All PWM Driver tests passed successfully.\n");
    return 0;
}
