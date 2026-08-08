/**
 * @file test_smart_coil.cpp
 * @brief Unit tests for Smart DC-Coil — software fuse, dead-zone, Qi interlock.
 */

#include "../../src/smart_coil.h"
#include <unity.h>
#include <stdio.h>
#include <math.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_software_fuse_protection(void) {
    SmartCoilState state = {0, 0, false, true};  /* qi_isolated = true */
    float roll  = 5.0f;
    float pitch = 0.0f;
    float gain  = 100.0f;

    /* Initial activation at t = 100 ms */
    uint16_t duty1 = update_smart_coil(roll, pitch, gain, 100, &state);
    TEST_ASSERT_TRUE(duty1 > 0);
    TEST_ASSERT_FALSE(state.fuse_tripped);

    /* Active for 300 ms (t = 400 ms) — within limit */
    uint16_t duty2 = update_smart_coil(roll, pitch, gain, 400, &state);
    TEST_ASSERT_TRUE(duty2 > 0);
    TEST_ASSERT_FALSE(state.fuse_tripped);

    /* Past 500 ms limit (t = 650 ms, 550 ms elapsed) — FUSE TRIPPED */
    uint16_t duty3 = update_smart_coil(roll, pitch, gain, 650, &state);
    TEST_ASSERT_EQUAL_UINT16(0, duty3);
    TEST_ASSERT_TRUE(state.fuse_tripped);

    /* Subsequent calls locked at 0 while fuse tripped */
    uint16_t duty4 = update_smart_coil(roll, pitch, gain, 700, &state);
    TEST_ASSERT_EQUAL_UINT16(0, duty4);

    /* Manual reset of software fuse */
    reset_software_fuse(&state);
    TEST_ASSERT_FALSE(state.fuse_tripped);
}

void test_zero_inclination_duty(void) {
    SmartCoilState state = {0, 0, false, true};
    float roll  = 0.1f;   /* Within 0.5° dead-zone */
    float pitch = 0.2f;
    float gain  = 100.0f;

    uint16_t duty = update_smart_coil(roll, pitch, gain, 100, &state);
    TEST_ASSERT_EQUAL_UINT16(0, duty);
}

void test_qi_interlock_blocks_coil(void) {
    /*
     * CRITICAL TEST: If Qi relay is NOT isolated (qi_isolated = false),
     * the coil must NOT fire — regardless of tilt.
     * Without this interlock, back-EMF destroys the Qi diode bridge.
     */
    SmartCoilState state = {0, 0, false, false};  /* qi_isolated = FALSE */
    float roll  = 10.0f;  /* Significant tilt */
    float pitch = 5.0f;
    float gain  = 100.0f;

    uint16_t duty = update_smart_coil(roll, pitch, gain, 100, &state);
    TEST_ASSERT_EQUAL_UINT16(0, duty);  /* Must be blocked! */
    TEST_ASSERT_EQUAL_UINT16(0, state.current_duty);
}

void test_qi_isolated_allows_coil(void) {
    SmartCoilState state = {0, 0, false, true};  /* qi_isolated = TRUE */
    float roll  = 10.0f;
    float pitch = 5.0f;
    float gain  = 100.0f;

    uint16_t duty = update_smart_coil(roll, pitch, gain, 100, &state);
    TEST_ASSERT_TRUE(duty > 0);  /* Should be allowed */
}

void test_compute_coil_duty_math(void) {
    /* Tilt = sqrt(3² + 4²) = 5.0°, dead-zone = 0.5°
     * Effective = 5.0 - 0.5 = 4.5, gain = 100 → duty = 450 */
    uint16_t duty = compute_coil_duty(3.0f, 4.0f, 100.0f);
    TEST_ASSERT_EQUAL_UINT16(450, duty);
}

void test_compute_coil_duty_clamping(void) {
    /* Very large tilt → clamped to MAX_COIL_DUTY (1023) */
    uint16_t duty = compute_coil_duty(45.0f, 0.0f, 100.0f);
    TEST_ASSERT_EQUAL_UINT16(MAX_COIL_DUTY, duty);
}

void test_null_state_returns_zero(void) {
    uint16_t duty = update_smart_coil(10.0f, 5.0f, 100.0f, 100, NULL);
    TEST_ASSERT_EQUAL_UINT16(0, duty);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_software_fuse_protection);
    RUN_TEST(test_zero_inclination_duty);
    RUN_TEST(test_qi_interlock_blocks_coil);
    RUN_TEST(test_qi_isolated_allows_coil);
    RUN_TEST(test_compute_coil_duty_math);
    RUN_TEST(test_compute_coil_duty_clamping);
    RUN_TEST(test_null_state_returns_zero);
    return UNITY_END();
}
