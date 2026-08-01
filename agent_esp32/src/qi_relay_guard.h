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
    QI_STATE_ISOLATED  = 1    /* NC relay open  — Qi circuit protected  */
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
 * @brief Open the NC relay to isolate Qi receiver BEFORE coil activation.
 * @return true if the relay was successfully commanded to isolate.
 */
bool qi_relay_isolate(void);

/**
 * @brief Close the NC relay to restore Qi charging AFTER coil is fully off.
 *
 * Callers MUST ensure coil duty == 0 before calling this function.
 * @return true if the relay was successfully commanded to connect.
 */
bool qi_relay_restore(void);

/**
 * @brief Query current relay state.
 */
QiRelayState qi_relay_get_state(void);

/**
 * @brief Check whether it is safe to energise the coil (relay must be isolated).
 */
bool qi_relay_is_coil_safe(void);

#ifdef __cplusplus
}
#endif

#endif /* QI_RELAY_GUARD_H */
