#ifndef PWM_DRIVER_H
#define PWM_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 * PWM Driver — RP2040-Zero Base Coprocessor
 *
 * Generates 5 independent hardware PWM channels at >20 kHz for driving
 * TC4427 MOSFET gate drivers.  Implements:
 *   - Hard duty-cycle ceiling (architecture §2: 45-50% to stay within
 *     the 27W adapter power budget on the 8V step-up rail).
 *   - Smooth Ramp-Up for inrush-current protection of power FETs.
 *   - 10-bit resolution (0-1023).
 * =================================================================== */

#define PWM_CHANNELS        5
#define PWM_RESOLUTION_BITS 10
#define MAX_DUTY_CYCLE      1023

/**
 * Hardware duty-cycle ceiling.
 * 460 / 1023 ≈ 44.97 %  → stays within the 45 % safety margin.
 * Adjust upward (max 512 ≈ 50 %) only after thermal validation.
 */
#define MAX_DUTY_LIMIT      460

#define MIN_PWM_FREQ_HZ     20000

/**
 * Ramp-Up step: duty increments per ramp tick.
 * At 25 kHz PWM with a 1-ms ramp tick, a step of 10 reaches full
 * MAX_DUTY_LIMIT (~460) in 46 ticks = 46 ms — gentle enough to avoid
 * inrush spikes, fast enough for responsive levitation.
 */
#define RAMP_STEP_DEFAULT   10

/**
 * RP-10 Firmware Slew-Rate Failsafe: default max duty change per 1-ms
 * tick in ACTIVE state (applied via slew_pwm_duty()).
 *
 * This is the ONLY slew limiter on the power path (review C5/F7).  It
 * is a runtime parameter (pwm_set_slew_limit) so that it can be tuned
 * on the calibration stand; 0 disables limiting (instant apply).
 */
#define PWM_SLEW_LIMIT_DEFAULT  10

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise 5 PWM channels on the specified GPIO pins.
 *
 * Configures RP2040 hardware PWM slices with the requested frequency
 * and 10-bit wrap counter.  All channels start at duty = 0.
 *
 * @param pins          Array of PWM_CHANNELS GPIO pin numbers.
 * @param frequency_hz  Desired PWM frequency (must be >= 20 kHz).
 * @return true on success, false if frequency is below minimum.
 */
bool init_pwm_channels(const uint8_t pins[PWM_CHANNELS], uint32_t frequency_hz);

/**
 * @brief Set duty cycle for a specific channel (0 .. MAX_DUTY_LIMIT).
 *
 * Values above MAX_DUTY_LIMIT are clamped (not rejected) so that
 * upstream PID overshoot does not cause a silent failure.
 *
 * The value is applied IMMEDIATELY (no slew limiting).  The main loop
 * must use slew_pwm_duty() (ACTIVE) or ramp_pwm_duty() (landing / OTA)
 * for the power path; set_pwm_duty() is the raw primitive.
 *
 * @return true on success, false if channel index is out of range.
 */
bool set_pwm_duty(uint8_t channel, uint16_t duty);

/**
 * @brief RP-10 failsafe: move a channel toward `target`, limited to
 *        the configured slew step per call (call every 1 ms).
 *
 * With slew limit 0 the target is applied instantly.
 *
 * @return Current duty after the step, or 0 on invalid channel.
 */
uint16_t slew_pwm_duty(uint8_t channel, uint16_t target);

/** @brief Configure the RP-10 slew step (duty units per tick, 0 = off). */
void pwm_set_slew_limit(uint16_t step_per_tick);

/** @brief Read-back the configured RP-10 slew step. */
uint16_t pwm_get_slew_limit(void);

/**
 * @brief Smooth ramp: move a channel toward a target duty at RAMP_STEP_DEFAULT.
 *
 * Call this repeatedly (e.g. every 1 ms) to incrementally approach
 * the target.  Returns the new intermediate duty value.
 *
 * @param channel  Channel index (0..4).
 * @param target   Desired final duty (will be clamped to MAX_DUTY_LIMIT).
 * @param step     Ramp increment per call (0 = use RAMP_STEP_DEFAULT).
 * @return Current duty after the ramp step, or 0 on invalid channel.
 */
uint16_t ramp_pwm_duty(uint8_t channel, uint16_t target, uint16_t step);

/** @brief Read-back current duty cycle for a channel. */
uint16_t get_pwm_duty(uint8_t channel);

/** @brief Read-back configured PWM frequency. */
uint32_t get_pwm_frequency(void);

/**
 * @brief Emergency shutdown: set all channels to 0 immediately.
 *
 * Used by the watchdog timeout handler and safe-landing logic.
 */
void kill_all_pwm(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_DRIVER_H */
