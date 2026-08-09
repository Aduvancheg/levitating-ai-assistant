/**
 * @file test_qi_relay.cpp
 * @brief Unit tests for Qi Relay Guard — non-blocking isolation, restoration,
 *        settling behaviour, idempotency, and coil-safety during transitions.
 *
 * The native (UNIT_TEST) build uses a mock clock (mock_time_ms) that
 * defaults to 0.  qi_relay_is_ready() is used to complete transitions
 * after the settle period (5 ms) has elapsed.
 */

#include "../src/qi_relay_guard.h"
#include <assert.h>
#include <stdio.h>

/* ---- Access mock clock from qi_relay_guard.cpp (native build) ------ */
/* In native builds the .cpp exposes `mock_time_ms` for test control.   */
extern uint32_t mock_time_ms;

/* ---- Helper: advance mock clock and complete settle ---------------- */
static void complete_settle(void) {
    mock_time_ms += 10;  /* Well past RELAY_SETTLE_MS (5 ms) */
    bool ready = qi_relay_is_ready(mock_time_ms);
    assert(ready == true);
}

/* ---- Tests --------------------------------------------------------- */

void test_initial_state_connected() {
    mock_time_ms = 0;
    qi_relay_init(13);
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);
    assert(qi_relay_is_coil_safe() == false);
    assert(qi_relay_is_ready(0) == true);  /* Not settling */

    printf("[PASS] test_initial_state_connected\n");
}

void test_isolate_enters_settling() {
    mock_time_ms = 100;
    qi_relay_init(13);

    bool iso = qi_relay_isolate();
    assert(iso == true);
    assert(qi_relay_get_state() == QI_STATE_SETTLING);
    assert(qi_relay_is_coil_safe() == false);  /* NOT safe during settle! */

    printf("[PASS] test_isolate_enters_settling\n");
}

void test_isolate_then_ready_completes() {
    mock_time_ms = 100;
    qi_relay_init(13);

    qi_relay_isolate();

    /* Not enough time elapsed */
    mock_time_ms = 103;
    assert(qi_relay_is_ready(mock_time_ms) == false);
    assert(qi_relay_get_state() == QI_STATE_SETTLING);

    /* Enough time elapsed (>= 5 ms) */
    mock_time_ms = 105;
    assert(qi_relay_is_ready(mock_time_ms) == true);
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);
    assert(qi_relay_is_coil_safe() == true);

    printf("[PASS] test_isolate_then_ready_completes\n");
}

void test_isolate_then_restore_full_cycle() {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* Phase 1: Isolate */
    qi_relay_isolate();
    complete_settle();
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);
    assert(qi_relay_is_coil_safe() == true);

    /* Phase 2: Restore */
    qi_relay_restore();
    assert(qi_relay_get_state() == QI_STATE_SETTLING);
    assert(qi_relay_is_coil_safe() == false);

    complete_settle();
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);
    assert(qi_relay_is_coil_safe() == false);

    printf("[PASS] test_isolate_then_restore_full_cycle\n");
}

void test_double_isolate_idempotent() {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    complete_settle();
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);

    /* Second isolate when already isolated — no-op, returns true */
    bool second = qi_relay_isolate();
    assert(second == true);
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);

    printf("[PASS] test_double_isolate_idempotent\n");
}

void test_double_isolate_during_settling() {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    assert(qi_relay_get_state() == QI_STATE_SETTLING);

    /* Second isolate while already settling to ISOLATED */
    bool second = qi_relay_isolate();
    assert(second == true);
    assert(qi_relay_get_state() == QI_STATE_SETTLING);

    printf("[PASS] test_double_isolate_during_settling\n");
}

void test_double_restore_idempotent() {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* Already connected — restore should be no-op */
    bool res = qi_relay_restore();
    assert(res == true);
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);

    printf("[PASS] test_double_restore_idempotent\n");
}

void test_coil_unsafe_during_settling() {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    /* During settling, coil must NOT be considered safe */
    assert(qi_relay_is_coil_safe() == false);

    /* Only after settle completes */
    complete_settle();
    assert(qi_relay_is_coil_safe() == true);

    printf("[PASS] test_coil_unsafe_during_settling\n");
}

void test_ready_when_not_settling() {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* When in a stable state (CONNECTED), is_ready always returns true */
    assert(qi_relay_is_ready(0) == true);
    assert(qi_relay_is_ready(1000) == true);

    printf("[PASS] test_ready_when_not_settling\n");
}

int main() {
    printf("--- Running Qi Relay Guard Unit Tests ---\n");
    test_initial_state_connected();
    test_isolate_enters_settling();
    test_isolate_then_ready_completes();
    test_isolate_then_restore_full_cycle();
    test_double_isolate_idempotent();
    test_double_isolate_during_settling();
    test_double_restore_idempotent();
    test_coil_unsafe_during_settling();
    test_ready_when_not_settling();
    printf("All Qi Relay Guard tests passed successfully.\n");
    return 0;
}

