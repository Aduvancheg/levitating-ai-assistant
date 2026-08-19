/**
 * @file test_qi_relay.cpp
 * @brief Unit tests for Qi Relay Guard — non-blocking isolation, restoration,
 *        settling behaviour, idempotency, and coil-safety during transitions.
 */

#include "../../src/qi_relay_guard.h"
#include <unity.h>

/* Mock clock from qi_relay_guard.cpp (native unit-test build) */
extern uint32_t mock_time_ms;

void setUp(void) {
    mock_time_ms = 0;
    qi_relay_init(13);
}

void tearDown(void) {
    // Empty
}

/* Helper to advance mock clock and complete settling period */
static void complete_settle(void) {
    mock_time_ms += 10;  /* Past RELAY_SETTLE_MS (5 ms) */
    bool ready = qi_relay_is_ready(mock_time_ms);
    TEST_ASSERT_TRUE(ready);
}

void test_initial_state_connected(void) {
    mock_time_ms = 0;
    qi_relay_init(13);
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());
    TEST_ASSERT_TRUE(qi_relay_is_ready(0));
}

void test_isolate_enters_settling(void) {
    mock_time_ms = 100;
    qi_relay_init(13);

    bool iso = qi_relay_isolate();
    TEST_ASSERT_TRUE(iso);
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());  /* NOT safe during settle! */
}

void test_isolate_then_ready_completes(void) {
    mock_time_ms = 100;
    qi_relay_init(13);

    qi_relay_isolate();

    /* 3 ms elapsed — not enough */
    mock_time_ms = 103;
    TEST_ASSERT_FALSE(qi_relay_is_ready(mock_time_ms));
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());

    /* 5 ms elapsed — settle complete */
    mock_time_ms = 105;
    TEST_ASSERT_TRUE(qi_relay_is_ready(mock_time_ms));
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());
    TEST_ASSERT_TRUE(qi_relay_is_coil_safe());
}

void test_isolate_then_restore_full_cycle(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* Phase 1: Isolate */
    qi_relay_isolate();
    complete_settle();
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());
    TEST_ASSERT_TRUE(qi_relay_is_coil_safe());

    /* Phase 2: Restore */
    qi_relay_restore();
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());

    complete_settle();
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());
}

void test_double_isolate_idempotent(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    complete_settle();
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());

    /* Second isolate when already isolated — no-op, returns true */
    bool second = qi_relay_isolate();
    TEST_ASSERT_TRUE(second);
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());
}

void test_double_isolate_during_settling(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());

    /* Second isolate while already settling to ISOLATED */
    bool second = qi_relay_isolate();
    TEST_ASSERT_TRUE(second);
    TEST_ASSERT_EQUAL(QI_STATE_SETTLING, qi_relay_get_state());
}

void test_double_restore_idempotent(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* Already connected — restore should be no-op */
    bool res = qi_relay_restore();
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
}

void test_coil_unsafe_during_settling(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    qi_relay_isolate();
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());

    complete_settle();
    TEST_ASSERT_TRUE(qi_relay_is_coil_safe());
}

void test_ready_when_not_settling(void) {
    mock_time_ms = 0;
    qi_relay_init(13);

    /* When in a stable state (CONNECTED), is_ready always returns true */
    TEST_ASSERT_TRUE(qi_relay_is_ready(0));
    TEST_ASSERT_TRUE(qi_relay_is_ready(1000));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_connected);
    RUN_TEST(test_isolate_enters_settling);
    RUN_TEST(test_isolate_then_ready_completes);
    RUN_TEST(test_isolate_then_restore_full_cycle);
    RUN_TEST(test_double_isolate_idempotent);
    RUN_TEST(test_double_isolate_during_settling);
    RUN_TEST(test_double_restore_idempotent);
    RUN_TEST(test_coil_unsafe_during_settling);
    RUN_TEST(test_ready_when_not_settling);
    return UNITY_END();
}
