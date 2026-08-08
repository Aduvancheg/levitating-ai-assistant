/**
 * @file coprocessor_fsm.cpp
 * @brief Coprocessor Finite State Machine — state coordination layer.
 *
 * Sits between protocol_parser (input) and pwm_driver (output).
 * Manages heartbeat watchdog, OTA locking, anomaly filtering,
 * ramp bypass policy, and recovery hysteresis.
 *
 * All time is injected externally (uint64_t now_us) so the module
 * is fully testable in [env:native] without hardware timers.
 *
 * No dynamic memory.  No blocking calls.  No hardware dependencies.
 */

#include "coprocessor_fsm.h"
#include <string.h>

/* ---- Internal State ----------------------------------------------- */

static FsmState  current_state = FSM_STATE_ACTIVE;
static uint64_t  last_packet_us = 0;
static uint16_t  target_duties[NUM_PWM_VALS]  = {0, 0, 0, 0, 0};
static uint16_t  prev_duties[NUM_PWM_VALS]    = {0, 0, 0, 0, 0};
static bool      has_prev_packet = false;  /* First packet has no delta to check */
static uint8_t   recovery_count = 0;       /* Hysteresis counter for SAFE_LANDING → ACTIVE */

/* ---- Helper: absolute difference for uint16_t --------------------- */

static inline uint16_t abs_delta(uint16_t a, uint16_t b) {
    return (a > b) ? (a - b) : (b - a);
}

/* ---- Anomaly detection -------------------------------------------- */

/**
 * @brief Check if a packet's duty values are physically plausible.
 *
 * Compares each channel against the previously accepted values.
 * If any channel's delta exceeds ANOMALY_DELTA_THRESHOLD, the packet
 * is considered an anomaly (sensor fault, solar glare, etc.).
 *
 * The first packet after init is always accepted (no baseline to compare).
 *
 * @return true if the packet is plausible, false if anomalous.
 */
static bool is_plausible(const PacketData *packet) {
    if (!has_prev_packet) {
        return true;  /* First packet — no history to compare against */
    }

    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        if (abs_delta(packet->duty_cycles[ch], prev_duties[ch]) > ANOMALY_DELTA_THRESHOLD) {
            return false;  /* Physically impossible rate of change */
        }
    }
    return true;
}

/* ---- Public API --------------------------------------------------- */

void fsm_init(uint64_t now_us) {
    current_state = FSM_STATE_ACTIVE;
    last_packet_us = now_us;
    has_prev_packet = false;
    recovery_count = 0;
    memset(target_duties, 0, sizeof(target_duties));
    memset(prev_duties, 0, sizeof(prev_duties));
}

FsmState fsm_get_state(void) {
    return current_state;
}

bool fsm_feed_packet(const PacketData *packet, uint64_t now_us) {
    if (packet == NULL || !packet->valid) {
        return false;
    }

    /* OTA lock: reject all dynamic packets until OTA_SUCCESS */
    if (current_state == FSM_STATE_OTA_LOCKED) {
        return false;
    }

    /* Anomaly detection: reject physically implausible jumps */
    if (!is_plausible(packet)) {
        return false;
    }

    /* Accept the packet */
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        target_duties[ch] = packet->duty_cycles[ch];
        prev_duties[ch]   = packet->duty_cycles[ch];
    }
    has_prev_packet = true;
    last_packet_us = now_us;

    /* Recovery hysteresis: SAFE_LANDING → ACTIVE requires
     * FSM_RECOVERY_THRESHOLD consecutive valid packets to prove
     * the link is truly stable (prevents yo-yo oscillation). */
    if (current_state == FSM_STATE_SAFE_LANDING) {
        recovery_count++;
        if (recovery_count >= FSM_RECOVERY_THRESHOLD) {
            current_state = FSM_STATE_ACTIVE;
            recovery_count = 0;
        }
    }

    return true;
}

void fsm_tick(uint64_t now_us) {
    /* Heartbeat check only applies in ACTIVE state.
     * In OTA_LOCKED, heartbeat is intentionally suspended because
     * ESP32 is flashing and won't respond (business_e2e_scenarios.md §9.4). */
    if (current_state == FSM_STATE_ACTIVE) {
        if (now_us - last_packet_us > FSM_HEARTBEAT_TIMEOUT_US) {
            current_state = FSM_STATE_SAFE_LANDING;
            recovery_count = 0;  /* Reset hysteresis on entering SAFE_LANDING */
            /* Set all targets to 0 for smooth ramp-down */
            memset(target_duties, 0, sizeof(target_duties));
        }
    }

    /* In SAFE_LANDING, targets stay at 0.
     * The caller (main loop) will call ramp_pwm_duty() toward 0. */
}

void fsm_command_prepare_ota(uint64_t now_us) {
    current_state = FSM_STATE_OTA_LOCKED;
    last_packet_us = now_us;  /* Reset heartbeat to avoid false trigger */

    /* Lock all channels to near-field safe duty */
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        target_duties[ch] = FSM_OTA_SAFE_DUTY;
    }
}

void fsm_command_ota_success(uint64_t now_us) {
    current_state = FSM_STATE_ACTIVE;
    last_packet_us = now_us;  /* Reset heartbeat timer */

    /* Keep duties at OTA_SAFE_DUTY — RPi 5 will send new targets
     * once the ESP32 reboots and reports back. */
}

const uint16_t* fsm_get_target_duties(void) {
    return target_duties;
}

bool fsm_should_ramp(void) {
    /* In ACTIVE state, PID commands must be applied instantly.
     * In SAFE_LANDING and OTA_LOCKED, duty changes must be gradual
     * to avoid mechanical shock (ramp up/down smoothly). */
    return current_state != FSM_STATE_ACTIVE;
}
