/**
 * @file main.cpp
 * @brief RP2040-Zero Base Coprocessor — Bare-metal Pico C/C++ SDK entry point (Zero Jitter).
 *
 * Responsibilities:
 *   1. High-speed UART receiver from RPi 5 (@ 921600 baud, non-blocking stream parsing).
 *   2. Non-blocking Smooth Ramp-Up timer state machine (1 ms tick).
 *   3. 5-channel hardware PWM generator via TC4427 drivers (>20 kHz, 10-bit).
 *   4. Fast binary ACK/NAK response builder.
 *   5. Dual watchdog protection:
 *        - Soft Comm Watchdog (FSM_HEARTBEAT_TIMEOUT_US): safe landing if link lost.
 *        - Hardware Watchdog (500 ms): hardware MCU reset on EMI lock-up.
 *   6. OTA lock and anomaly filtering via coprocessor_fsm.
 */

#include "pwm_driver.h"
#include "protocol_parser.h"
#include "coprocessor_fsm.h"
#include "ring_buffer.h"

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#define UART_ID          uart1
#define UART_BAUD_RATE   921600
#define UART_TX_PIN      4
#define UART_RX_PIN      5

static const uint8_t PWM_PINS[PWM_CHANNELS] = {0, 1, 2, 3, 6};  /* Hardware PWM GPIOs */
static const uint32_t PWM_FREQ_HZ = 25000;
static const uint64_t RAMP_TICK_INTERVAL_US = 1000;  /* 1 ms */

static RingBuffer uart_rb;

static uint64_t last_ramp_tick_us = 0;

int main(void) {
    /* Initialize stdio / Pico SDK core */
    stdio_init_all();

    /* Initialize high-speed UART (UART1 @ 921600 baud) */
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    /* Initialize 5 hardware PWM channels (>20 kHz, 10-bit) */
    if (!init_pwm_channels(PWM_PINS, PWM_FREQ_HZ)) {
        kill_all_pwm();
        while (true) {
            tight_loop_contents();
        }
    }

    /* Initialize 500 ms Hardware Watchdog */
    watchdog_enable(500, 1);

    uint64_t now_us = time_us_64();
    fsm_init(now_us);
    last_ramp_tick_us = now_us;
    ring_buffer_init(&uart_rb);

    /* Main zero-jitter bare-metal execution loop */
    while (true) {
        now_us = time_us_64();

        /* 1. Feed Hardware Watchdog */
        watchdog_update();

        /* 2. Non-blocking UART RX processing */
        while (uart_is_readable(UART_ID)) {
            uint8_t byte_in = uart_getc(UART_ID);
            ring_buffer_push(&uart_rb, byte_in);

            /* Attempt stream parse when frame size threshold met */
            if (ring_buffer_available(&uart_rb) >= TOTAL_PACKET_SIZE) {
                uint8_t linear_buf[TOTAL_PACKET_SIZE * 3];
                size_t lin_len = ring_buffer_snapshot(&uart_rb, linear_buf, sizeof(linear_buf));

                PacketData packet;
                size_t consumed = 0;
                if (parse_byte_stream(linear_buf, lin_len, &packet, &consumed)) {
                    /* Delegate to FSM: anomaly check + OTA lock + heartbeat reset */
                    uint8_t ack_status;
                    if (fsm_feed_packet(&packet, now_us)) {
                        ack_status = ACK_STATUS_OK;
                    } else {
                        ack_status = ACK_STATUS_NAK;
                    }

                    uint8_t ack_buf[ACK_PACKET_SIZE];
                    build_ack_packet(ack_status, ack_buf, sizeof(ack_buf));
                    uart_write_blocking(UART_ID, ack_buf, ACK_PACKET_SIZE);

                    if (consumed > 0) {
                        ring_buffer_discard(&uart_rb, consumed);
                    }
                }
            }
        }

        /* 3. FSM tick: heartbeat watchdog + state transitions */
        fsm_tick(now_us);

        /* 4. Non-blocking Smooth Ramp tick (every 1 ms) */
        if (now_us - last_ramp_tick_us >= RAMP_TICK_INTERVAL_US) {
            last_ramp_tick_us = now_us;

            const uint16_t *targets = fsm_get_target_duties();
            bool use_ramp = fsm_should_ramp();

            for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
                if (use_ramp) {
                    ramp_pwm_duty(i, targets[i], RAMP_STEP_DEFAULT);
                } else {
                    set_pwm_duty(i, targets[i]);
                }
            }
        }
    }

    return 0;
}

#else
/* Native host test entry stub — disabled under UNIT_TEST to avoid
 * duplicate main() when PlatformIO builds src/ alongside test files. */
#ifndef UNIT_TEST
int main(void) {
    return 0;
}
#endif  /* UNIT_TEST */
#endif
