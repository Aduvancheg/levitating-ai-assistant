/**
 * @file test_smart_coil.cpp
 * @brief Unit tests for Smart DC-Coil — software fuse, dead-zone, Qi interlock.
 */

#include "../src/smart_coil.h"
#include "../src/qi_relay_guard.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

void test_software_fuse_protection() {
    SmartCoilState state = {0, 0, false, true};  /* qi_isolated = true */
    float roll  = 5.0f;
    float pitch = 0.0f;
    float gain  = 100.0f;

    /* Initial activation at t = 100 ms */
    uint16_t duty1 = update_smart_coil(roll, pitch, gain, 100, &state);
    assert(duty1 > 0);
    assert(state.fuse_tripped == false);

    /* Active for 300 ms (t = 400 ms) — within limit */
    uint16_t duty2 = update_smart_coil(roll, pitch, gain, 400, &state);
    assert(duty2 > 0);
    assert(state.fuse_tripped == false);

    /* Past 500 ms limit (t = 650 ms, 550 ms elapsed) — FUSE TRIPPED */
    uint16_t duty3 = update_smart_coil(roll, pitch, gain, 650, &state);
    assert(duty3 == 0);
    assert(state.fuse_tripped == true);

    /* Subsequent calls locked at 0 while fuse tripped */
    uint16_t duty4 = update_smart_coil(roll, pitch, gain, 700, &state);
    assert(duty4 == 0);

    /* Manual reset of software fuse */
    reset_software_fuse(&state);
    assert(state.fuse_tripped == false);

    printf("[PASS] test_software_fuse_protection\n");
}

void test_zero_inclination_duty() {
    SmartCoilState state = {0, 0, false, true};
    float roll  = 0.1f;   /* Within 0.5° dead-zone */
    float pitch = 0.2f;
    float gain  = 100.0f;

    uint16_t duty = update_smart_coil(roll, pitch, gain, 100, &state);
    assert(duty == 0);

    printf("[PASS] test_zero_inclination_duty\n");
}

void test_qi_interlock_blocks_coil() {
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
    assert(duty == 0);  /* Must be blocked! */
    assert(state.current_duty == 0);

    printf("[PASS] test_qi_interlock_blocks_coil\n");
}

void test_qi_isolated_allows_coil() {
    SmartCoilState state = {0, 0, false, true};  /* qi_isolated = TRUE */
    float roll  = 10.0f;
    float pitch = 5.0f;
    float gain  = 100.0f;

    uint16_t duty = update_smart_coil(roll, pitch, gain, 100, &state);
    assert(duty > 0);  /* Should be allowed */

    printf("[PASS] test_qi_isolated_allows_coil\n");
}

void test_compute_coil_duty_math() {
    /* Tilt = sqrt(3² + 4²) = 5.0°, dead-zone = 0.5°
     * Effective = 5.0 - 0.5 = 4.5, gain = 100 → duty = 450 */
    uint16_t duty = compute_coil_duty(3.0f, 4.0f, 100.0f);
    assert(duty == 450);

    printf("[PASS] test_compute_coil_duty_math\n");
}

void test_compute_coil_duty_clamping() {
    /* Very large tilt → clamped to MAX_COIL_DUTY (1023) */
    uint16_t duty = compute_coil_duty(45.0f, 0.0f, 100.0f);
    assert(duty == MAX_COIL_DUTY);

    printf("[PASS] test_compute_coil_duty_clamping\n");
}

void test_null_state_returns_zero() {
    uint16_t duty = update_smart_coil(10.0f, 5.0f, 100.0f, 100, NULL);
    assert(duty == 0);

    printf("[PASS] test_null_state_returns_zero\n");
}

int main() {
    printf("--- Running Smart DC-Coil Unit Tests ---\n");
    test_software_fuse_protection();
    test_zero_inclination_duty();
    test_qi_interlock_blocks_coil();
    test_qi_isolated_allows_coil();
    test_compute_coil_duty_math();
    test_compute_coil_duty_clamping();
    test_null_state_returns_zero();
    printf("All Smart DC-Coil tests passed successfully.\n");
    return 0;
}
