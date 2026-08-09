/**
 * @file qi_relay_guard.cpp
 * @brief NC Relay Guard implementation — Qi receiver hardware fuse.
 *
 * Non-blocking design: relay transitions use a settle timer checked
 * via qi_relay_is_ready() instead of blocking delay().  This keeps
 * the main loop FSM running at full speed during relay switching.
 */

#include "qi_relay_guard.h"

/* ---- Static state ------------------------------------------------- */
static uint8_t      relay_pin   = QI_RELAY_PIN_DEFAULT;
static QiRelayState relay_state = QI_STATE_CONNECTED;

/* Settling time in ms for the relay to fully open / close.
 * Solid-state relays typically switch in < 1 ms, but we use 5 ms
 * as a conservative guard against mechanical relay variants.           */
static const uint32_t RELAY_SETTLE_MS = 5;

/* Non-blocking settle tracking */
static uint32_t     settle_start_ms   = 0;
static QiRelayState settle_target     = QI_STATE_CONNECTED;

/* ---- Platform-specific helpers ------------------------------------ */
#if defined(ARDUINO)
#include <Arduino.h>

static void drive_pin(bool high) {
    digitalWrite(relay_pin, high ? HIGH : LOW);
}

static uint32_t get_time_ms(void) {
    return millis();
}

#else
/* Host / unit-test stub — no real GPIO, no real clock */
uint32_t mock_time_ms = 0;  /* Non-static: accessible from tests via extern */
static void drive_pin(bool high) { (void)high; }
static uint32_t get_time_ms(void) { return mock_time_ms; }
#endif

/* ---- Public API --------------------------------------------------- */

void qi_relay_init(uint8_t pin) {
    relay_pin   = pin;
    relay_state = QI_STATE_CONNECTED;
    settle_start_ms = 0;
    settle_target   = QI_STATE_CONNECTED;

#if defined(ARDUINO)
    pinMode(relay_pin, OUTPUT);
    digitalWrite(relay_pin, LOW);   /* NC default = Qi connected */
#endif
}

bool qi_relay_isolate(void) {
    if (relay_state == QI_STATE_ISOLATED) {
        return true;   /* Already isolated — idempotent */
    }

    if (relay_state == QI_STATE_SETTLING && settle_target == QI_STATE_ISOLATED) {
        return true;   /* Already transitioning to isolated */
    }

    drive_pin(true);               /* HIGH = open NC relay = isolate Qi */
    settle_start_ms = get_time_ms();
    settle_target   = QI_STATE_ISOLATED;
    relay_state     = QI_STATE_SETTLING;
    return true;
}

bool qi_relay_restore(void) {
    if (relay_state == QI_STATE_CONNECTED) {
        return true;   /* Already connected — idempotent */
    }

    if (relay_state == QI_STATE_SETTLING && settle_target == QI_STATE_CONNECTED) {
        return true;   /* Already transitioning to connected */
    }

    drive_pin(false);              /* LOW = close NC relay = Qi active  */
    settle_start_ms = get_time_ms();
    settle_target   = QI_STATE_CONNECTED;
    relay_state     = QI_STATE_SETTLING;
    return true;
}

bool qi_relay_is_ready(uint32_t current_time_ms) {
    if (relay_state != QI_STATE_SETTLING) {
        return true;   /* Not settling — already in a stable state */
    }

    if ((current_time_ms - settle_start_ms) >= RELAY_SETTLE_MS) {
        /* Settle complete — promote to target state */
        relay_state = settle_target;
        return true;
    }

    return false;  /* Still settling */
}

QiRelayState qi_relay_get_state(void) {
    return relay_state;
}

bool qi_relay_is_coil_safe(void) {
    /* Coil may only fire when relay is FULLY isolated —
     * during SETTLING the relay contacts may still be bouncing,
     * and back-EMF could reach the Qi receiver circuit.               */
    return (relay_state == QI_STATE_ISOLATED);
}

