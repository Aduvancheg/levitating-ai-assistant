/**
 * @file qi_relay_guard.cpp
 * @brief NC Relay Guard implementation — Qi receiver hardware fuse.
 */

#include "qi_relay_guard.h"

/* ---- Static state ------------------------------------------------- */
static uint8_t      relay_pin   = QI_RELAY_PIN_DEFAULT;
static QiRelayState relay_state = QI_STATE_CONNECTED;

/* Settling time in ms for the relay to fully open / close.
 * Solid-state relays typically switch in < 1 ms, but we use 5 ms
 * as a conservative guard against mechanical relay variants.           */
static const uint32_t RELAY_SETTLE_MS = 5;

/* ---- Platform-specific helpers ------------------------------------ */
#if defined(ARDUINO)
#include <Arduino.h>

static void drive_pin(bool high) {
    digitalWrite(relay_pin, high ? HIGH : LOW);
}

static void settle_delay(void) {
    /* Non-blocking delay via millis() would be ideal, but the relay
     * transition is a one-shot event (not in the hot loop), so a
     * short blocking wait is acceptable here.                          */
    delay(RELAY_SETTLE_MS);
}

#else
/* Host / unit-test stub — no real GPIO */
static void drive_pin(bool high) { (void)high; }
static void settle_delay(void)   { /* no-op in test builds */ }
#endif

/* ---- Public API --------------------------------------------------- */

void qi_relay_init(uint8_t pin) {
    relay_pin   = pin;
    relay_state = QI_STATE_CONNECTED;

#if defined(ARDUINO)
    pinMode(relay_pin, OUTPUT);
    digitalWrite(relay_pin, LOW);   /* NC default = Qi connected */
#endif
}

bool qi_relay_isolate(void) {
    if (relay_state == QI_STATE_ISOLATED) {
        return true;   /* Already isolated — idempotent */
    }

    drive_pin(true);               /* HIGH = open NC relay = isolate Qi */
    settle_delay();
    relay_state = QI_STATE_ISOLATED;
    return true;
}

bool qi_relay_restore(void) {
    if (relay_state == QI_STATE_CONNECTED) {
        return true;   /* Already connected — idempotent */
    }

    drive_pin(false);              /* LOW = close NC relay = Qi active  */
    settle_delay();
    relay_state = QI_STATE_CONNECTED;
    return true;
}

QiRelayState qi_relay_get_state(void) {
    return relay_state;
}

bool qi_relay_is_coil_safe(void) {
    return (relay_state == QI_STATE_ISOLATED);
}
