/**
 * @file main.cpp
 * @brief RP2040-Zero Base Coprocessor — Dual-Core Bare-metal Entry Point.
 *
 * Architecture (RP-9 Dual-Core Partitioning):
 *
 *   Core 1 (core1_uart_task):
 *     - UART RX polling at 921600 baud
 *     - RingBuffer byte accumulation
 *     - Binary packet parsing (header/CRC/footer)
 *     - Writes parsed PacketData to lock-free mailbox
 *
 *   Core 0 (this file — main loop):
 *     - Reads PacketData from mailbox
 *     - FSM state machine (heartbeat, OTA lock, anomaly filter)
 *     - ACK/NAK response via UART TX
 *     - 5-channel hardware PWM generation (25 kHz, 10-bit)
 *     - Smooth Ramp-Up/Down timer (1 ms tick)
 *     - Hardware Watchdog (500 ms)
 *
 * References:
 *   - levitation_solution_backlog_2-v4.md: RP-1 through RP-9
 *   - quality-v4.md: RP-CL-1 through RP-CL-9
 *   - base_power_assembly_guide-v3.md: power topology & pin mapping
 */

#include "pwm_driver.h"
#include "protocol_parser.h"
#include "coprocessor_fsm.h"
#include "core1_uart_task.h"

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* ---- UART configuration ------------------------------------------- */
#define UART_ID          uart1
#define UART_BAUD_RATE   921600
#define UART_TX_PIN      4
#define UART_RX_PIN      5

/* ---- PWM pin mapping (base_power_assembly_guide-v3.md §3) --------- */
static const uint8_t PWM_PINS[PWM_CHANNELS] = {0, 1, 2, 3, 6};
static const uint32_t PWM_FREQ_HZ = 25000;

/* ---- Ramp tick interval ------------------------------------------- */
static const uint64_t RAMP_TICK_INTERVAL_US = 1000;  /* 1 ms */

static uint64_t last_ramp_tick_us = 0;

int main(void) {
    /* ============================================================
     * PHASE 1: Hardware initialisation (Core 0 only)
     * ============================================================ */

    /* Initialise stdio / Pico SDK core */
    stdio_init_all();

    /* Initialise high-speed UART (UART1 @ 921600 baud).
     * Both TX and RX are configured here on Core 0.
     * Core 1 will only READ from the UART FIFO (safe cross-core). */
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(UART_ID, true);

    /* Initialise 5 hardware PWM channels (25 kHz, 10-bit) */
    if (!init_pwm_channels(PWM_PINS, PWM_FREQ_HZ)) {
        kill_all_pwm();
        while (true) {
            tight_loop_contents();
        }
    }

    /* Initialise 500 ms Hardware Watchdog (RP-5) */
    watchdog_enable(500, 1);

    /* Initialise FSM and inter-core mailbox */
    uint64_t now_us = time_us_64();
    fsm_init(now_us);
    mailbox_init();
    last_ramp_tick_us = now_us;

    /* ============================================================
     * PHASE 2: Launch Core 1 — dedicated UART RX + parsing
     * ============================================================ */
    multicore_launch_core1(core1_entry);

    /* ============================================================
     * PHASE 3: Core 0 main loop — FSM + PWM (zero-jitter)
     * ============================================================ */
    while (true) {
        now_us = time_us_64();

        /* 1. Feed Hardware Watchdog — must be called frequently
         *    to prevent 500 ms timeout reset (RP-5) */
        watchdog_update();

        /* 2. Consume parsed packets from Core 1 mailbox.
         *    Core 1 handles all UART RX and parsing (RP-9).
         *    Core 0 only processes validated PacketData here. */
        PacketData rx_packet;
        if (mailbox_consume(&rx_packet)) {
            /* Delegate to FSM: anomaly check + OTA lock + heartbeat reset */
            uint8_t ack_status;
            if (fsm_feed_packet(&rx_packet, now_us)) {
                ack_status = ACK_STATUS_OK;
            } else {
                ack_status = ACK_STATUS_NAK;
            }

            /* Send ACK/NAK back to RPi 5.
             * UART TX from Core 0 is safe — Core 1 only reads RX. */
            uint8_t ack_buf[ACK_PACKET_SIZE];
            build_ack_packet(ack_status, ack_buf, sizeof(ack_buf));
            uart_write_blocking(UART_ID, ack_buf, ACK_PACKET_SIZE);
        }

        /* 3. FSM tick: heartbeat watchdog + state transitions (RP-4, RP-5) */
        fsm_tick(now_us);

        /* 4. Non-blocking Smooth Ramp tick every 1 ms (RP-8).
         *    In ACTIVE state: instant duty application (no ramp).
         *    In SAFE_LANDING / OTA_LOCKED: smooth ramp for safety. */
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
