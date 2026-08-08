#ifndef COPROCESSOR_FSM_H
#define COPROCESSOR_FSM_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol_parser.h"
#include "pwm_driver.h"

/* ===================================================================
 * Coprocessor Finite State Machine — RP2040 Base Controller
 *
 * Testable state coordinator that sits between protocol_parser (input)
 * and pwm_driver (output).  Manages:
 *
 *   1. Heartbeat watchdog: transitions to SAFE_LANDING if no valid
 *      packet arrives within FSM_HEARTBEAT_TIMEOUT_US.
 *   2. OTA lock: on PREPARE_OTA, locks duty at near-field safe value
 *      and rejects dynamic packets until OTA_SUCCESS.
 *   3. Anomaly detection: rejects packets whose duty delta from the
 *      previous accepted packet exceeds ANOMALY_DELTA_THRESHOLD on
 *      any channel (solar glare / sensor fault protection).
 *   4. Ramp bypass: in ACTIVE state, duty is applied instantly to
 *      avoid PID throttling; ramp is used only in SAFE_LANDING/OTA.
 *   5. Recovery hysteresis: SAFE_LANDING → ACTIVE requires
 *      FSM_RECOVERY_THRESHOLD consecutive valid packets to prevent
 *      yo-yo oscillation on unstable UART links.
 *
 * All time is injected as uint64_t now_us for deterministic testing.
 *
 * References:
 *   - business_e2e_scenarios.md  §7 (Heartbeat), §9 (OTA), §11 (Anomaly)
 *   - quality.md                 §1 (Fault Isolation), §4 (TDD)
 *   - architecture.md            §5 (Sensor Fusion anomaly rejection)
 * =================================================================== */

/* ---- FSM States --------------------------------------------------- */

typedef enum {
    FSM_STATE_ACTIVE,        /**< Normal levitation — accepting packets */
    FSM_STATE_SAFE_LANDING,  /**< Heartbeat lost — ramping down to 0   */
    FSM_STATE_OTA_LOCKED     /**< OTA in progress — duty locked at safe */
} FsmState;

/* ---- Tunable Constants -------------------------------------------- */

/**
 * Heartbeat timeout: 250 ms (250,000 µs).
 * business_e2e_scenarios.md §7: "таймаут > 200 мс".
 */
#define FSM_HEARTBEAT_TIMEOUT_US   250000ULL

/**
 * Near-field safe duty for OTA mode.
 * ~5% of 1023 ≈ 2–3 cm hover height where neodymium N52 + coils
 * provide 100% stable passive levitation.
 * business_e2e_scenarios.md §9: "зона жесткого ближнего поля (2-3 см)".
 */
#define FSM_OTA_SAFE_DUTY          50

/**
 * Maximum allowed duty delta per packet on any single channel.
 * Packets with |new_duty - prev_duty| > threshold on ANY channel
 * are rejected as sensor anomalies (solar glare, EMI spike, etc.).
 *
 * 400 / 1023 ≈ 39% of full scale — catches 1000% jumps but allows
 * normal PID corrections (typically ≤50 per tick at 800 Hz).
 *
 * business_e2e_scenarios.md §11: "скорость изменения координат de/dt
 * превышает физические пределы свободного падения".
 */
#define ANOMALY_DELTA_THRESHOLD    400

/**
 * Recovery hysteresis: number of consecutive valid packets required
 * to transition from SAFE_LANDING back to ACTIVE.
 *
 * Prevents yo-yo oscillation when the UART link is unstable (e.g.,
 * loose connector or RPi 5 under load).  A single good packet is
 * not enough to prove the link is stable.
 *
 * At ~800 Hz packet rate, 5 packets = ~6 ms of proven stability.
 */
#define FSM_RECOVERY_THRESHOLD     5

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise FSM to ACTIVE state with all duties at 0.
 * @param now_us  Current timestamp in microseconds.
 */
void fsm_init(uint64_t now_us);

/**
 * @brief Return the current FSM state.
 */
FsmState fsm_get_state(void);

/**
 * @brief Feed a parsed packet into the FSM.
 *
 * Performs anomaly detection and OTA-lock checks before accepting.
 *
 * @param packet   Parsed packet from protocol_parser.
 * @param now_us   Current timestamp (resets heartbeat timer on accept).
 * @return true if packet accepted, false if rejected (anomaly or OTA lock).
 */
bool fsm_feed_packet(const PacketData *packet, uint64_t now_us);

/**
 * @brief Periodic tick — call every ~1 ms from the main loop.
 *
 * Checks heartbeat timeout and transitions to SAFE_LANDING if expired.
 * In SAFE_LANDING, sets all target duties to 0.
 *
 * @param now_us  Current timestamp.
 */
void fsm_tick(uint64_t now_us);

/**
 * @brief Process PREPARE_OTA command from RPi 5.
 *
 * Transitions to OTA_LOCKED, sets all target duties to FSM_OTA_SAFE_DUTY.
 * All subsequent fsm_feed_packet() calls are rejected until OTA_SUCCESS.
 *
 * @param now_us  Current timestamp (resets heartbeat to avoid false trigger).
 */
void fsm_command_prepare_ota(uint64_t now_us);

/**
 * @brief Process OTA_SUCCESS command from RPi 5.
 *
 * Transitions back to ACTIVE, resumes accepting packets.
 *
 * @param now_us  Current timestamp (resets heartbeat timer).
 */
void fsm_command_ota_success(uint64_t now_us);

/**
 * @brief Read-back current target duties (what FSM wants applied to PWM).
 * @return Pointer to internal array of NUM_PWM_VALS elements.
 */
const uint16_t* fsm_get_target_duties(void);

/**
 * @brief Should the main loop apply duty via smooth ramp?
 *
 * Returns true in SAFE_LANDING and OTA_LOCKED (duty must change
 * gradually to avoid mechanical shock).  Returns false in ACTIVE
 * (PID commands must be applied instantly — ramp would throttle
 * the D-coefficient and cause the sphere to fall).
 *
 * @return true → use ramp_pwm_duty(),  false → use set_pwm_duty().
 */
bool fsm_should_ramp(void);

#ifdef __cplusplus
}
#endif

#endif /* COPROCESSOR_FSM_H */
