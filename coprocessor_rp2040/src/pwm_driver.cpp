/**
 * @file pwm_driver.cpp
 * @brief RP2040-Zero hardware PWM driver using Pico C/C++ SDK (Zero Jitter).
 *
 * On Pico SDK RP2040 builds, uses hardware/pwm.h registers & hardware APIs
 * for deterministic frequency and 10-bit resolution control.
 * On native (host) builds, hardware calls are stubbed out for host unit tests.
 */

#include "pwm_driver.h"

/* ---- Internal state ----------------------------------------------- */
static uint8_t  channel_pins[PWM_CHANNELS]   = {0, 1, 2, 3, 4};
static uint16_t channel_duties[PWM_CHANNELS] = {0, 0, 0, 0, 0};
static uint32_t configured_frequency = 0;

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)
/*
 * Bare-metal RP2040 Hardware PWM via Pico C/C++ SDK hardware APIs.
 * Maps GPIO pin to hardware PWM slice & channel (A/B).
 * Computes fractional clock divider for requested high frequency (>20 kHz)
 * and configures 10-bit TOP register (0..1023).
 */
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

static void hw_pwm_init_pin(uint8_t pin, uint32_t freq_hz) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);

    uint32_t sys_clk = clock_get_hz(clk_sys);  /* typically 125 MHz */
    /* divider = sys_clk / (freq_hz * wrap).  wrap = 1024 for 10-bit. */
    float divider = (float)sys_clk / ((float)freq_hz * (float)(MAX_DUTY_CYCLE + 1));
    if (divider < 1.0f) divider = 1.0f;

    pwm_set_clkdiv(slice, divider);
    pwm_set_wrap(slice, MAX_DUTY_CYCLE);  /* 10-bit wrap counter: 0..1023 */
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), 0);
    pwm_set_enabled(slice, true);
}

static void hw_pwm_set(uint8_t pin, uint16_t duty) {
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), duty);
}

#else
/* Native / host build — no hardware, pure state tracking for unit tests */
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
    uint16_t current = channel_duties[channel];

    /* RP-10 (Firmware Slew-Rate Limiter): limit rate of change to 10 units per ms */
    int16_t diff = (int16_t)safe_duty - (int16_t)current;
    if (diff > 10) {
        safe_duty = current + 10;
    } else if (diff < -10) {
        safe_duty = current - 10;
    }

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
        uint32_t next = (uint32_t)current + step;
        if (next > clamped_target) next = clamped_target;
        set_pwm_duty(channel, (uint16_t)next);
    } else if (current > clamped_target) {
        uint16_t diff = current - clamped_target;
        uint16_t next = (diff < step) ? clamped_target : (current - step);
        set_pwm_duty(channel, next);
    }

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
