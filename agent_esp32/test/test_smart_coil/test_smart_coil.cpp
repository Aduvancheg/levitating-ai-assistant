/**
 * @file test_smart_coil.cpp
 * @brief Unit tests for Smart DC-Coil — software fuse, dead-zone, Qi interlock.
 */

#include "../../src/smart_coil.h"
#include "../../src/qi_relay_guard.h"
#include <unity.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

extern uint32_t mock_time_ms;

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

void test_f1_relay_never_switches_when_duty_is_zero(void) {
    /* Review F1: at rest (tilt within dead-zone → duty == 0),
     * the Qi relay must stay CONNECTED across 100 consecutive ticks (1 s)
     * without ever entering SETTLING or ISOLATED. */
    mock_time_ms = 0;
    qi_relay_init(QI_RELAY_PIN_DEFAULT);

    SmartCoilState state;
    memset(&state, 0, sizeof(state));

    for (uint32_t tick = 0; tick < 100; tick++) {
        mock_time_ms = tick * 10;
        CoilRelayStepResult res = smart_coil_step_with_relay(0.1f, 0.1f, 50.0f, mock_time_ms, &state);
        TEST_ASSERT_EQUAL(COIL_STEP_APPLIED, res);
        TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
        TEST_ASSERT_FALSE(state.qi_isolated);
        TEST_ASSERT_EQUAL_UINT16(0, state.current_duty);
    }
}

void test_f1_isolate_only_on_tilt_and_restore_with_hysteresis(void) {
    mock_time_ms = 0;
    qi_relay_init(QI_RELAY_PIN_DEFAULT);

    SmartCoilState state;
    memset(&state, 0, sizeof(state));

    /* 1. Tilt occurs → must request relay isolation BEFORE applying duty */
    mock_time_ms = 10;
    CoilRelayStepResult res = smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state);
    TEST_ASSERT_EQUAL(COIL_STEP_WAIT_ISOLATE, res);
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
    TEST_ASSERT_EQUAL_UINT16(0, state.current_duty);

    /* 2. Relay settles after 5 ms → caller marks qi_isolated = true */
    mock_time_ms = 16;
    TEST_ASSERT_TRUE(qi_relay_is_ready(mock_time_ms));
    state.qi_isolated = true;

    /* 3. Next coil update applies non-zero duty */
    res = smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state);
    TEST_ASSERT_EQUAL(COIL_STEP_APPLIED, res);
    TEST_ASSERT_TRUE(state.current_duty > 0);

    /* 4. Tilt returns to 0 → duty drops to 0, but relay stays isolated during hysteresis */
    mock_time_ms = 30;
    res = smart_coil_step_with_relay(0.0f, 0.0f, 50.0f, mock_time_ms, &state);
    TEST_ASSERT_EQUAL(COIL_STEP_APPLIED, res);
    TEST_ASSERT_EQUAL_UINT16(0, state.current_duty);
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());

    /* 50 ms later (< QI_RESTORE_HYSTERESIS_MS = 100 ms) → still isolated */
    mock_time_ms = 80;
    res = smart_coil_step_with_relay(0.0f, 0.0f, 50.0f, mock_time_ms, &state);
    TEST_ASSERT_EQUAL(COIL_STEP_APPLIED, res);
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());

    /* 100 ms after zero-crossing (t = 130 ms) → restore initiated */
    mock_time_ms = 130;
    res = smart_coil_step_with_relay(0.0f, 0.0f, 50.0f, mock_time_ms, &state);
    TEST_ASSERT_EQUAL(COIL_STEP_WAIT_RESTORE, res);
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
}

void test_f1_fuse_trip_restores_qi_relay_after_hysteresis(void) {
    /* If the sphere stays tilted > MAX_PULSE_DURATION_MS (500 ms), the software
     * fuse trips and cuts coil duty to 0. Even if the sphere remains tilted,
     * Qi charging must be restored after QI_RESTORE_HYSTERESIS_MS (100 ms) so
     * the battery does not drain with the relay stranded open. */
    mock_time_ms = 0;
    qi_relay_init(QI_RELAY_PIN_DEFAULT);

    SmartCoilState state;
    memset(&state, 0, sizeof(state));

    /* Isolate relay and start coil pulse at t = 16 ms */
    mock_time_ms = 10;
    TEST_ASSERT_EQUAL(COIL_STEP_WAIT_ISOLATE,
                      smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state));
    mock_time_ms = 16;
    TEST_ASSERT_TRUE(qi_relay_is_ready(mock_time_ms));
    state.qi_isolated = true;
    TEST_ASSERT_EQUAL(COIL_STEP_APPLIED,
                      smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state));
    TEST_ASSERT_TRUE(state.current_duty > 0);

    /* At t = 520 ms (> 500 ms pulse), software fuse trips → duty = 0, timer armed */
    mock_time_ms = 520;
    TEST_ASSERT_EQUAL(COIL_STEP_APPLIED,
                      smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state));
    TEST_ASSERT_TRUE(state.fuse_tripped);
    TEST_ASSERT_EQUAL_UINT16(0, state.current_duty);
    TEST_ASSERT_TRUE(state.restore_timer_armed);

    /* 100 ms later (t = 620 ms), Qi relay restore is initiated despite continued tilt */
    mock_time_ms = 620;
    TEST_ASSERT_EQUAL(COIL_STEP_WAIT_RESTORE,
                      smart_coil_step_with_relay(5.0f, 0.0f, 50.0f, mock_time_ms, &state));
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
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
    RUN_TEST(test_f1_relay_never_switches_when_duty_is_zero);
    RUN_TEST(test_f1_isolate_only_on_tilt_and_restore_with_hysteresis);
    RUN_TEST(test_f1_fuse_trip_restores_qi_relay_after_hysteresis);
    return UNITY_END();
}
