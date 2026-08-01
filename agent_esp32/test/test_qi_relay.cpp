/**
 * @file test_qi_relay.cpp
 * @brief Unit tests for Qi Relay Guard — isolation, restoration, idempotency.
 */

#include "../src/qi_relay_guard.h"
#include <assert.h>
#include <stdio.h>

void test_initial_state_connected() {
    qi_relay_init(13);
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);
    assert(qi_relay_is_coil_safe() == false);

    printf("[PASS] test_initial_state_connected\n");
}

void test_isolate_then_restore() {
    qi_relay_init(13);

    bool iso = qi_relay_isolate();
    assert(iso == true);
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);
    assert(qi_relay_is_coil_safe() == true);

    bool rest = qi_relay_restore();
    assert(rest == true);
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);
    assert(qi_relay_is_coil_safe() == false);

    printf("[PASS] test_isolate_then_restore\n");
}

void test_double_isolate_idempotent() {
    qi_relay_init(13);

    qi_relay_isolate();
    bool second = qi_relay_isolate();
    assert(second == true);
    assert(qi_relay_get_state() == QI_STATE_ISOLATED);

    printf("[PASS] test_double_isolate_idempotent\n");
}

void test_double_restore_idempotent() {
    qi_relay_init(13);

    qi_relay_restore();  /* Already connected — should be no-op */
    bool res = qi_relay_restore();
    assert(res == true);
    assert(qi_relay_get_state() == QI_STATE_CONNECTED);

    printf("[PASS] test_double_restore_idempotent\n");
}

void test_coil_safe_only_when_isolated() {
    qi_relay_init(13);

    assert(qi_relay_is_coil_safe() == false);

    qi_relay_isolate();
    assert(qi_relay_is_coil_safe() == true);

    qi_relay_restore();
    assert(qi_relay_is_coil_safe() == false);

    printf("[PASS] test_coil_safe_only_when_isolated\n");
}

int main() {
    printf("--- Running Qi Relay Guard Unit Tests ---\n");
    test_initial_state_connected();
    test_isolate_then_restore();
    test_double_isolate_idempotent();
    test_double_restore_idempotent();
    test_coil_safe_only_when_isolated();
    printf("All Qi Relay Guard tests passed successfully.\n");
    return 0;
}
