/**
 * @file test_qi_relay.cpp
 * @brief Unit tests for Qi Relay Guard — isolation, restoration, idempotency.
 */

#include "../../src/qi_relay_guard.h"
#include <unity.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_initial_state_connected(void) {
    qi_relay_init(13);
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());
}

void test_isolate_then_restore(void) {
    qi_relay_init(13);

    bool iso = qi_relay_isolate();
    TEST_ASSERT_TRUE(iso);
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());
    TEST_ASSERT_TRUE(qi_relay_is_coil_safe());

    bool rest = qi_relay_restore();
    TEST_ASSERT_TRUE(rest);
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());
}

void test_double_isolate_idempotent(void) {
    qi_relay_init(13);

    qi_relay_isolate();
    bool second = qi_relay_isolate();
    TEST_ASSERT_TRUE(second);
    TEST_ASSERT_EQUAL(QI_STATE_ISOLATED, qi_relay_get_state());
}

void test_double_restore_idempotent(void) {
    qi_relay_init(13);

    qi_relay_restore();  /* Already connected — should be no-op */
    bool res = qi_relay_restore();
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_EQUAL(QI_STATE_CONNECTED, qi_relay_get_state());
}

void test_coil_safe_only_when_isolated(void) {
    qi_relay_init(13);

    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());

    qi_relay_isolate();
    TEST_ASSERT_TRUE(qi_relay_is_coil_safe());

    qi_relay_restore();
    TEST_ASSERT_FALSE(qi_relay_is_coil_safe());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_connected);
    RUN_TEST(test_isolate_then_restore);
    RUN_TEST(test_double_isolate_idempotent);
    RUN_TEST(test_double_restore_idempotent);
    RUN_TEST(test_coil_safe_only_when_isolated);
    return UNITY_END();
}
