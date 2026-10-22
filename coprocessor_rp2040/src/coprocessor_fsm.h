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
 *      and rejects dynamic packets until OTA_SUCCESS.  The lock expires
 *      (→ SAFE_LANDING) if PREPARE_OTA keep-alives stop arriving, so a
 *      dead RPi cannot leave the coils energised forever (review F6).
 *   3. Anomaly detection: rejects packets whose duty delta from the
 *      previous accepted packet exceeds ANOMALY_DELTA_THRESHOLD on
 *      any channel (solar glare / sensor fault protection).
 *   4. Ramp policy: in ACTIVE state duty goes through the RP-10 slew
 *      limiter (slew_pwm_duty, tunable); in SAFE_LANDING/OTA the
 *      landing ramp (ramp_pwm_duty, RAMP_STEP_DEFAULT) is used.
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
 * Heartbeat timeout: 20 ms (20,000 µs) — review F7.
 *
 * The previous 250 ms (business_e2e_scenarios.md §7 "> 200 мс") is far
 * longer than the time an unstable magnetic suspension survives without
 * feedback.  At the nominal ~860 Hz packet rate 20 ms tolerates ~17
 * consecutive lost frames before landing.  The RPi MUST keep streaming
 * duty frames (≥ 50 Hz) in every non-OTA state, including idle.
 */
#define FSM_HEARTBEAT_TIMEOUT_US   20000ULL

/**
 * OTA keep-alive timeout: 3 s.
 * While OTA_LOCKED the RPi must re-send CMD_PREPARE_OTA at least this
 * often.  If it stops (RPi crash / UART unplugged) the FSM leaves
 * OTA_LOCKED for SAFE_LANDING and ramps the coils down (review F6).
 */
#define FSM_OTA_KEEPALIVE_TIMEOUT_US  3000000ULL

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
 * Command frames (FRAME_TYPE_COMMAND) are forwarded to
 * fsm_feed_command().
 *
 * @param packet   Parsed packet from protocol_parser.
 * @param now_us   Current timestamp (resets heartbeat timer on accept).
 * @return true if packet accepted, false if rejected (anomaly or OTA lock).
 */
bool fsm_feed_packet(const PacketData *packet, uint64_t now_us);

/**
 * @brief Dispatch a protocol command (CMD_* from protocol_parser.h).
 *
 *   CMD_PREPARE_OTA — enter OTA_LOCKED, or refresh the keep-alive if
 *                     already locked.
 *   CMD_OTA_SUCCESS — leave OTA_LOCKED → ACTIVE (ignored otherwise).
 *
 * @return true if the command was accepted, false if unknown / not
 *         valid in the current state.
 */
bool fsm_feed_command(uint8_t command, uint64_t now_us);

/**
 * @brief Periodic tick — call every ~1 ms from the main loop.
 *
 * Checks heartbeat timeout (ACTIVE) and OTA keep-alive timeout
 * (OTA_LOCKED) and transitions to SAFE_LANDING if expired.
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
 * Calling it again while locked only refreshes the keep-alive timer.
 * Refused in SAFE_LANDING (coils must not be re-energised from a landing).
 *
 * @param now_us  Current timestamp (resets heartbeat to avoid false trigger).
 * @return true if locked / keep-alive refreshed, false if refused.
 */
bool fsm_command_prepare_ota(uint64_t now_us);

/**
 * @brief Process OTA_SUCCESS command from RPi 5.
 *
 * Transitions back to ACTIVE, resumes accepting packets.
 * Ignored (returns false) when not in OTA_LOCKED.
 *
 * @param now_us  Current timestamp (resets heartbeat timer).
 * @return true if the FSM left OTA_LOCKED.
 */
bool fsm_command_ota_success(uint64_t now_us);

/**
 * @brief Read-back current target duties (what FSM wants applied to PWM).
 * @return Pointer to internal array of NUM_PWM_VALS elements.
 */
const uint16_t* fsm_get_target_duties(void);

/**
 * @brief Should the main loop apply duty via the landing ramp?
 *
 * Returns true in SAFE_LANDING and OTA_LOCKED → ramp_pwm_duty() with
 * RAMP_STEP_DEFAULT (gentle, fixed).  Returns false in ACTIVE →
 * slew_pwm_duty(), i.e. the RP-10 failsafe slew limiter whose step is
 * tunable via pwm_set_slew_limit() (0 = instant).  Note: ACTIVE is NOT
 * unconditionally instant (review F7).
 *
 * @return true → use ramp_pwm_duty(),  false → use slew_pwm_duty().
 */
bool fsm_should_ramp(void);

#ifdef __cplusplus
}
#endif

#endif /* COPROCESSOR_FSM_H */
