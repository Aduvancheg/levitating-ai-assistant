/**
 * @file pwm_driver.cpp
 * @brief RP2040-Zero hardware PWM driver with safety clamping and Smooth Ramp-Up.
 *
 * On Arduino/RP2040 builds, uses the Pico SDK `hardware/pwm.h` for deterministic
 * frequency control.  On native (host) builds, the hardware calls are stubbed
 * so that pure-logic unit tests can run on the development machine.
 */

#include "pwm_driver.h"

/* ---- Internal state ----------------------------------------------- */
static uint8_t  channel_pins[PWM_CHANNELS]  = {0, 1, 2, 3, 4};
static uint16_t channel_duties[PWM_CHANNELS] = {0, 0, 0, 0, 0};
static uint32_t configured_frequency = 0;

/* ---- Platform-specific PWM back-end ------------------------------- */
#if defined(ARDUINO) && defined(TARGET_RP2040)
/*
 * RP2040 Hardware PWM via Pico SDK.
 * Each GPIO maps to a PWM slice + channel (A or B).
 * We configure the wrap value for 10-bit resolution and compute
 * the clock divider from the system clock (usually 125 MHz).
 */
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <Arduino.h>

static void hw_pwm_init_pin(uint8_t pin, uint32_t freq_hz) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    uint32_t sys_clk = clock_get_hz(clk_sys);  /* typically 125 MHz */
    /* divider = sys_clk / (freq_hz * wrap).  wrap = 1024 for 10-bit. */
    float divider = (float)sys_clk / ((float)freq_hz * (float)(MAX_DUTY_CYCLE + 1));
    if (divider < 1.0f) divider = 1.0f;

    pwm_set_clkdiv(slice, divider);
    pwm_set_wrap(slice, MAX_DUTY_CYCLE);  /* 10-bit: 0..1023 */
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice, true);
}

static void hw_pwm_set(uint8_t pin, uint16_t duty) {
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), duty);
}

#elif defined(ARDUINO)
/* Fallback for generic Arduino boards (e.g. ESP32 test builds) */
#include <Arduino.h>

static void hw_pwm_init_pin(uint8_t pin, uint32_t freq_hz) {
    (void)freq_hz;
    pinMode(pin, OUTPUT);
    analogWriteResolution(PWM_RESOLUTION_BITS);
    analogWrite(pin, 0);
}

static void hw_pwm_set(uint8_t pin, uint16_t duty) {
    analogWrite(pin, duty);
}

#else
/* Native / host build — no hardware, just state tracking for tests */
static void hw_pwm_init_pin(uint8_t pin, uint32_t freq_hz) {
    (void)pin; (void)freq_hz;
}
static void hw_pwm_set(uint8_t pin, uint16_t duty) {
    (void)pin; (void)duty;
}
#endif

/* ---- Helper: clamp to safety ceiling ------------------------------ */
static inline uint16_t clamp_duty(uint16_t duty) {
    return (duty > MAX_DUTY_LIMIT) ? MAX_DUTY_LIMIT : duty;
}

/* ---- Public API --------------------------------------------------- */

bool init_pwm_channels(const uint8_t pins[PWM_CHANNELS], uint32_t frequency_hz) {
    if (frequency_hz < MIN_PWM_FREQ_HZ) {
        return false;
    }

    configured_frequency = frequency_hz;

    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        if (pins != nullptr) {
            channel_pins[i] = pins[i];
        }
        channel_duties[i] = 0;
        hw_pwm_init_pin(channel_pins[i], frequency_hz);
    }
    return true;
}

bool set_pwm_duty(uint8_t channel, uint16_t duty) {
    if (channel >= PWM_CHANNELS) {
        return false;
    }

    /* Architecture §2: hard clamp to MAX_DUTY_LIMIT (≈45 %).
     * We clamp instead of rejecting so PID overshoot is gracefully handled. */
    uint16_t safe_duty = clamp_duty(duty);

    channel_duties[channel] = safe_duty;
    hw_pwm_set(channel_pins[channel], safe_duty);
    return true;
}

uint16_t ramp_pwm_duty(uint8_t channel, uint16_t target, uint16_t step) {
    if (channel >= PWM_CHANNELS) {
        return 0;
    }

    if (step == 0) {
        step = RAMP_STEP_DEFAULT;
    }

    uint16_t clamped_target = clamp_duty(target);
    uint16_t current = channel_duties[channel];

    if (current < clamped_target) {
        uint16_t next = current + step;
        if (next > clamped_target) next = clamped_target;
        set_pwm_duty(channel, next);
    } else if (current > clamped_target) {
        uint16_t diff = current - clamped_target;
        uint16_t next = (diff < step) ? clamped_target : (current - step);
        set_pwm_duty(channel, next);
    }
    /* else: already at target, nothing to do */

    return channel_duties[channel];
}

uint16_t get_pwm_duty(uint8_t channel) {
    if (channel >= PWM_CHANNELS) {
        return 0;
    }
    return channel_duties[channel];
}

uint32_t get_pwm_frequency(void) {
    return configured_frequency;
}

void kill_all_pwm(void) {
    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        channel_duties[i] = 0;
        hw_pwm_set(channel_pins[i], 0);
    }
}
