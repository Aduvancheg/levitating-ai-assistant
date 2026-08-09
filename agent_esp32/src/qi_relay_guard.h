/**
 * @file qi_relay_guard.h
 * @brief NC Relay Guard — Hardware Fuse for Qi Wireless Charging Protection
 *
 * Architecture §4 mandate: The Normally-Closed (NC) solid-state relay isolates
 * the Qi receiver circuit. The Agent MUST programmatically open (isolate) the
 * relay BEFORE applying any PWM to the on-board 60mm coil via MOSFET D4184.
 * Failure to do so will cause a back-EMF pulse to instantly destroy the Qi
 * receiver's diode bridge.
 *
 * Safety invariant: qi_relay_isolate() → coil PWM ON → coil PWM OFF → qi_relay_restore()
 *
 * Non-blocking design (quality.md §2 compliance):
 *   All relay transitions are two-phase:
 *     1. qi_relay_isolate() / qi_relay_restore() — initiates transition instantly.
 *     2. qi_relay_is_ready() — polls whether the settling time has elapsed.
 *   The caller's state machine must check is_ready() before proceeding.
 *   No delay() or vTaskDelay() is used anywhere in this module.
 */

#ifndef QI_RELAY_GUARD_H
#define QI_RELAY_GUARD_H

#include <stdint.h>
#include <stdbool.h>

/* -------------------------------------------------------------------
 * Configuration — adapt the GPIO pin to your PCB layout.
 * The relay is Normally-Closed (NC): LOW = Qi circuit connected,
 *                                    HIGH = Qi circuit isolated.
 * ------------------------------------------------------------------- */
#define QI_RELAY_PIN_DEFAULT 13

typedef enum {
    QI_STATE_CONNECTED = 0,   /* NC relay closed — Qi charging active   */
    QI_STATE_ISOLATED  = 1,   /* NC relay open  — Qi circuit protected  */
    QI_STATE_SETTLING  = 2    /* Relay transitioning — wait for settle  */
} QiRelayState;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the relay GPIO pin and set it to CONNECTED (default NC state).
 * @param relay_pin  GPIO number driving the relay coil / gate.
 */
void qi_relay_init(uint8_t relay_pin);

/**
 * @brief Begin opening the NC relay to isolate Qi receiver.
 *
 * Non-blocking: drives the GPIO immediately and starts the settle timer.
 * Caller MUST poll qi_relay_is_ready() before assuming the relay has
 * fully transitioned.
 *
 * @return true if the transition was initiated (or already isolated).
 */
bool qi_relay_isolate(void);

/**
 * @brief Begin closing the NC relay to restore Qi charging.
 *
 * Non-blocking: drives the GPIO immediately and starts the settle timer.
 * Callers MUST ensure coil duty == 0 before calling this function.
 * Caller MUST poll qi_relay_is_ready() before assuming transition is done.
 *
 * @return true if the transition was initiated (or already connected).
 */
bool qi_relay_restore(void);

/**
 * @brief Poll whether the relay has completed its settling period.
 *
 * If the relay is in QI_STATE_SETTLING, checks whether enough time
 * has passed since the transition was initiated.  When settle time
 * is reached, automatically updates state to the target state.
 *
 * @param current_time_ms   Current millis() timestamp.
 * @return true if the relay is in a stable state (CONNECTED or ISOLATED),
 *         false if still settling.
 */
bool qi_relay_is_ready(uint32_t current_time_ms);

/**
 * @brief Query current relay state.
 */
QiRelayState qi_relay_get_state(void);

/**
 * @brief Check whether it is safe to energise the coil.
 *
 * Returns true ONLY when relay_state == QI_STATE_ISOLATED.
 * Returns false during SETTLING — the coil must not fire until
 * the relay has fully opened.
 */
bool qi_relay_is_coil_safe(void);

#ifdef __cplusplus
}
#endif

#endif /* QI_RELAY_GUARD_H */

