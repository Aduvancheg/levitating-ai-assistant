/**
 * @file smart_coil.cpp
 * @brief Smart DC-Coil controller with LEDC PWM, Software Fuse, and Qi interlock.
 */

#include "smart_coil.h"
#include "qi_relay_guard.h"
#include <math.h>

/* ---- Platform-specific PWM back-end ------------------------------- */

#if defined(ARDUINO)
#include <Arduino.h>
static uint8_t coil_pin = 12;

#if defined(ESP32)
/* ESP32 LEDC API — precise frequency and resolution control.
 * backlog LS-4: "ESP32 LEDC PWM"                                     */

static void hw_coil_init(uint8_t pin) {
    coil_pin = pin;
    /* Drive gate LOW immediately before attaching LEDC (review F3) */
    pinMode(coil_pin, OUTPUT);
    digitalWrite(coil_pin, LOW);
    ledcSetup(COIL_LEDC_CHANNEL, COIL_LEDC_FREQ_HZ, COIL_LEDC_RESOLUTION);
    ledcAttachPin(coil_pin, COIL_LEDC_CHANNEL);
    ledcWrite(COIL_LEDC_CHANNEL, 0);
}

static void hw_coil_set(uint16_t duty) {
    ledcWrite(COIL_LEDC_CHANNEL, duty);
}

#else
/* Non-ESP32 Arduino fallback */
static void hw_coil_init(uint8_t pin) {
    coil_pin = pin;
    pinMode(coil_pin, OUTPUT);
    analogWriteResolution(COIL_LEDC_RESOLUTION);
    analogWrite(coil_pin, 0);
}

static void hw_coil_set(uint16_t duty) {
    analogWrite(coil_pin, duty);
}
#endif  /* ESP32 */

#else
/* Native / unit-test stub */
static void hw_coil_init(uint8_t pin) { (void)pin; }
static void hw_coil_set(uint16_t duty) { (void)duty; }
#endif  /* ARDUINO */

/* ---- Public API --------------------------------------------------- */

void smart_coil_init(uint8_t mosfet_pin) {
    hw_coil_init(mosfet_pin);
}

uint16_t compute_coil_duty(float roll, float pitch, float gain) {
    float tilt = sqrtf(roll * roll + pitch * pitch);

    /* Dead-zone: no output for negligible tilt (sphere at rest) */
    if (tilt < DEADZONE_TILT_DEG) {
        return 0;
    }

    float duty_f = (tilt - DEADZONE_TILT_DEG) * gain;
    if (duty_f > (float)MAX_COIL_DUTY) duty_f = (float)MAX_COIL_DUTY;
    if (duty_f < 0.0f) duty_f = 0.0f;

    return (uint16_t)duty_f;
}

uint16_t update_smart_coil(float roll, float pitch, float gain,
                           uint32_t current_time_ms, SmartCoilState *state) {
    if (state == nullptr) return 0;

    /* ---- Safety gate 1: Software Fuse tripped --------------------- */
    if (state->fuse_tripped) {
        state->current_duty = 0;
        hw_coil_set(0);
        return 0;
    }

    uint16_t requested_duty = compute_coil_duty(roll, pitch, gain);

    /* ---- Safety gate 2: Qi Relay interlock ------------------------ */
    if (requested_duty > 0 && !state->qi_isolated) {
        /* CRITICAL: Coil cannot fire while Qi circuit is connected!
         * The back-EMF pulse would destroy the Qi diode bridge.
         * Refuse to output — caller must isolate the relay first.     */
        state->current_duty = 0;
        hw_coil_set(0);
        return 0;
    }

    /* ---- Normal duty management ----------------------------------- */
    if (requested_duty > 0) {
        if (state->current_duty == 0) {
            /* Pulse just started — record timestamp */
            state->active_start_time_ms = current_time_ms;
        } else {
            /* Pulse ongoing — check Software Fuse time limit */
            if ((current_time_ms - state->active_start_time_ms) > MAX_PULSE_DURATION_MS) {
                /* SOFTWARE FUSE TRIPPED — auto cut-off to prevent
                 * MOSFET / coil thermal damage!                       */
                state->fuse_tripped = true;
                state->current_duty = 0;
                hw_coil_set(0);
                return 0;
            }
        }
    } else {
        /* Returned to zero — reset pulse timer */
        state->active_start_time_ms = 0;
    }

    state->current_duty = requested_duty;
    hw_coil_set(requested_duty);

    /* If duty dropped to 0, allow Qi reconnection by the caller */
    return requested_duty;
}

void reset_software_fuse(SmartCoilState *state) {
    if (state != nullptr) {
        state->fuse_tripped         = false;
        state->active_start_time_ms = 0;
        state->current_duty         = 0;
    }
}

CoilRelayStepResult smart_coil_step_with_relay(float roll, float pitch, float gain,
                                               uint32_t current_time_ms,
                                               SmartCoilState *state) {
    if (state == nullptr) {
        return COIL_STEP_APPLIED;
    }

    /* 1. Compute requested duty FIRST (review F1):
     *    When at rest (duty == 0) and Qi is connected, do NOT touch the relay. */
    uint16_t requested_duty = state->fuse_tripped ? 0 : compute_coil_duty(roll, pitch, gain);

    if (requested_duty > 0) {
        state->restore_timer_armed = false;

        if (!state->qi_isolated) {
            /* Isolate Qi BEFORE applying any coil PWM */
            qi_relay_isolate();
            return COIL_STEP_WAIT_ISOLATE;
        }

        uint16_t applied = update_smart_coil(roll, pitch, gain, current_time_ms, state);
        if (applied > 0) {
            return COIL_STEP_APPLIED;
        }
        /* If update_smart_coil() just tripped the software fuse (applied == 0),
         * fall through to the zero-duty path so the Qi restore hysteresis timer
         * starts immediately instead of stranding the Qi charger disconnected. */
    } else {
        /* requested_duty == 0: ensure coil output is zeroed */
        update_smart_coil(roll, pitch, gain, current_time_ms, state);
    }

    /* Whenever coil output is 0 (including while software fuse is tripped) and
     * Qi is still isolated, reconnect Qi after QI_RESTORE_HYSTERESIS_MS so the
     * battery can charge safely while the coil is off. */
    if (state->qi_isolated) {
        if (!state->restore_timer_armed) {
            state->restore_timer_armed = true;
            state->zero_duty_since_ms  = current_time_ms;
            return COIL_STEP_APPLIED;
        }

        if ((current_time_ms - state->zero_duty_since_ms) >= QI_RESTORE_HYSTERESIS_MS) {
            state->restore_timer_armed = false;
            qi_relay_restore();
            return COIL_STEP_WAIT_RESTORE;
        }
    }

    return COIL_STEP_APPLIED;
}
