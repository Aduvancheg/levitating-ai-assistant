#ifndef SMART_COIL_H
#define SMART_COIL_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 * Smart DC-Coil Controller — ESP32-CAM Sphere Agent
 *
 * Manages the 60 mm on-board air coil via MOSFET D4184 for active
 * "current injection" to dampen centrifugal forces.
 *
 * Safety features:
 *   - Software Fuse: auto-cuts PWM after MAX_PULSE_DURATION_MS to
 *     prevent coil / MOSFET thermal damage.
 *   - Dead-zone: no output below DEADZONE_TILT_DEG to avoid jitter.
 *   - Qi Relay interlock: coil activation REQUIRES the Qi relay to
 *     be isolated first (qi_relay_guard.h).
 *
 * Uses ESP32 LEDC peripheral for precise frequency/duty control
 * (backlog LS-4 requirement).
 * =================================================================== */

#define MAX_PULSE_DURATION_MS     500
#define DEADZONE_TILT_DEG         0.5f
#define MAX_COIL_DUTY             1023

/**
 * Hysteresis delay (ms) after coil duty drops to 0 before reconnecting
 * the Qi receiver relay (review F1). Prevents relay chatter during
 * brief zero-crossings or when the sphere is at rest.
 */
#define QI_RESTORE_HYSTERESIS_MS  100

/* ESP32 LEDC configuration */
#define COIL_LEDC_CHANNEL      0
#define COIL_LEDC_FREQ_HZ      25000
#define COIL_LEDC_RESOLUTION   10     /* 10-bit: 0..1023 */

typedef struct {
    uint16_t current_duty;
    uint32_t active_start_time_ms;
    bool     fuse_tripped;
    bool     qi_isolated;           /* tracks Qi relay state for interlock */
    uint32_t zero_duty_since_ms;    /* timestamp when duty first returned to 0 */
    bool     restore_timer_armed;   /* true while waiting hysteresis before restore */
} SmartCoilState;

typedef enum {
    COIL_STEP_APPLIED      = 0, /* Duty updated; no relay transition started */
    COIL_STEP_WAIT_ISOLATE = 1, /* Relay isolation started; wait for settle  */
    COIL_STEP_WAIT_RESTORE = 2  /* Relay restore started; wait for settle    */
} CoilRelayStepResult;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise MOSFET PWM output on the specified GPIO pin.
 *
 * On ESP32, configures LEDC channel with 25 kHz / 10-bit resolution.
 * Does NOT isolate the Qi relay — caller must do that explicitly.
 */
void smart_coil_init(uint8_t mosfet_pin);

/**
 * @brief Pure function: compute duty cycle from sphere tilt angles.
 *
 * Returns 0 if tilt is within the dead-zone.
 */
uint16_t compute_coil_duty(float roll, float pitch, float gain);

/**
 * @brief Main update tick: compute duty, enforce Software Fuse + Qi interlock.
 *
 * IMPORTANT: This function will refuse to output any duty > 0 unless
 * state->qi_isolated == true.  Call qi_relay_isolate() and set the flag
 * before invoking this function.
 *
 * @return Applied duty cycle (0 if fuse tripped or Qi not isolated).
 */
uint16_t update_smart_coil(float roll, float pitch, float gain,
                           uint32_t current_time_ms, SmartCoilState *state);

/**
 * @brief Reset the software fuse after a cooldown / equilibrium return.
 */
void reset_software_fuse(SmartCoilState *state);

/**
 * @brief Coordinated Smart-Coil + Qi-Relay step (review F1).
 *
 * Evaluates compute_coil_duty() FIRST:
 *   - If duty == 0 and Qi is connected: does NOT isolate the relay (no clicking at rest).
 *   - If duty > 0 and Qi is connected: initiates qi_relay_isolate() and returns
 *     COIL_STEP_WAIT_ISOLATE without energizing the coil.
 *   - If duty > 0 and Qi is isolated: applies PWM via update_smart_coil().
 *   - If duty == 0 (or software fuse tripped) and Qi is isolated: waits
 *     QI_RESTORE_HYSTERESIS_MS of continuous zero duty before calling
 *     qi_relay_restore() and returning COIL_STEP_WAIT_RESTORE.
 */
CoilRelayStepResult smart_coil_step_with_relay(float roll, float pitch, float gain,
                                               uint32_t current_time_ms,
                                               SmartCoilState *state);

#ifdef __cplusplus
}
#endif

#endif /* SMART_COIL_H */
