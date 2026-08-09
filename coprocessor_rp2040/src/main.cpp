/**
 * @file main.cpp
 * @brief RP2040-Zero Base Coprocessor — Main firmware entry point.
 *
 * Responsibilities:
 *   1. Receive 5× duty-cycle commands from RPi 5 via high-speed UART.
 *   2. Apply Smooth Ramp-Up to each channel (no inrush spikes).
 *   3. Drive 5 hardware PWM outputs through TC4427 gate drivers.
 *   4. Send ACK/NAK back to RPi 5 for each received packet.
 *   5. Hardware Watchdog: if no valid packet arrives within WATCHDOG_TIMEOUT_MS,
 *      all PWM channels are killed and the MCU resets.
 *
 * Communication link: UART1 @ 921600 baud (architecture §2 latency < 1 ms).
 */

#if defined(ARDUINO)
#include <Arduino.h>
#include "pwm_driver.h"
#include "protocol_parser.h"
#include "ring_buffer.h"

/* ---- Configuration ------------------------------------------------ */

/** GPIO pins for 5 PWM channels driving TC4427 gate drivers.
 *  TODO: Update to match final PCB routing.                           */
static const uint8_t PWM_PINS[PWM_CHANNELS] = {0, 1, 2, 3, 4};

/** UART baud rate — 921600 provides ~92 KB/s, enough for 800 Hz × 15B
 *  packets with margin.  Must match RPi 5 serial configuration.       */
static const uint32_t UART_BAUD = 921600;

/** PWM carrier frequency in Hz (> 20 kHz to be inaudible).            */
static const uint32_t PWM_FREQ_HZ = 25000;

/** Communication watchdog: if no valid packet is received within this
 *  window, assume link loss and execute safe shutdown (all PWM → 0).  */
static const uint32_t COMM_WATCHDOG_TIMEOUT_MS = 100;

/** Ramp tick interval: how often ramp_pwm_duty() is called.           */
static const uint32_t RAMP_TICK_INTERVAL_US = 1000;  /* 1 ms */

/* ---- Runtime state ------------------------------------------------ */

static RingBuffer rx_ring;

/** Linear snapshot buffer for parse_byte_stream() — avoids wrapping
 *  complexity during parsing.  Sized to hold enough bytes for scan.   */
static uint8_t snapshot_buf[TOTAL_PACKET_SIZE * 3];

/** Target duties received from RPi 5 (ramped toward gradually).       */
static uint16_t target_duties[PWM_CHANNELS] = {0, 0, 0, 0, 0};

/** Timestamp of the last successfully parsed packet (millis).         */
static uint32_t last_valid_packet_ms = 0;

/** Ramp timer (micros).                                               */
static uint32_t last_ramp_tick_us = 0;

/** Communication-loss flag — cleared when a valid packet arrives.     */
static bool comm_lost = false;

/* ---- Hardware Watchdog -------------------------------------------- */
#if defined(TARGET_RP2040)
#include "hardware/watchdog.h"

static void wdt_init(void) {
    /* 500 ms hardware WDT — if the main loop hangs for more than
     * half a second (e.g. due to EMI lock-up), the MCU resets.
     * This is the last line of defence: coils cannot stay powered
     * indefinitely.                                                   */
    watchdog_enable(500, true);  /* 500 ms, pause on debug */
}

static void wdt_feed(void) {
    watchdog_update();
}
#else
/* Non-RP2040 Arduino boards — stub */
static void wdt_init(void) {}
static void wdt_feed(void) {}
#endif

/* ---- Safe shutdown ------------------------------------------------ */

static void safe_shutdown(void) {
    kill_all_pwm();
    for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
        target_duties[i] = 0;
    }
    comm_lost = true;
}

/* ---- Arduino entry points ---------------------------------------- */

void setup() {
    Serial.begin(115200);          /* Debug console */
    Serial1.begin(UART_BAUD);      /* High-speed link from RPi 5 */

    if (!init_pwm_channels(PWM_PINS, PWM_FREQ_HZ)) {
        Serial.println(F("[FATAL] PWM init failed — frequency below 20 kHz"));
        while (true) { /* halt */ }
    }

    ring_buffer_init(&rx_ring);
    last_valid_packet_ms = millis();
    last_ramp_tick_us    = micros();

    wdt_init();
    Serial.println(F("[OK] RP2040 Base coprocessor ready"));
}

void loop() {
    uint32_t now_ms = millis();
    uint32_t now_us = micros();

    /* ---- 1. Feed hardware watchdog -------------------------------- */
    wdt_feed();

    /* ---- 2. Receive UART bytes into RingBuffer --------------------- */
    while (Serial1.available() > 0) {
        uint8_t byte_in = (uint8_t)Serial1.read();
        ring_buffer_push(&rx_ring, byte_in);
    }

    /* ---- 2b. Attempt parse when enough bytes are buffered ---------- */
    if (ring_buffer_available(&rx_ring) >= TOTAL_PACKET_SIZE) {
        size_t snap_len = ring_buffer_snapshot(
            &rx_ring, snapshot_buf, sizeof(snapshot_buf));

        PacketData packet;
        if (parse_byte_stream(snapshot_buf, snap_len, &packet)) {
            /* Valid packet — update targets and send ACK */
            for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
                target_duties[i] = packet.duty_cycles[i];
            }

            uint8_t ack_buf[ACK_PACKET_SIZE];
            build_ack_packet(ACK_STATUS_OK, ack_buf, sizeof(ack_buf));
            Serial1.write(ack_buf, ACK_PACKET_SIZE);

            last_valid_packet_ms = now_ms;
            comm_lost = false;

            /* Discard the parsed packet from the ring buffer.
             * We consumed TOTAL_PACKET_SIZE bytes worth of frame.     */
            ring_buffer_discard(&rx_ring, TOTAL_PACKET_SIZE);
        }
    }

    /* ---- 3. Communication watchdog check -------------------------- */
    if (!comm_lost && (now_ms - last_valid_packet_ms > COMM_WATCHDOG_TIMEOUT_MS)) {
        Serial.println(F("[WARN] Comm watchdog — safe shutdown"));
        safe_shutdown();
    }

    /* ---- 4. Smooth Ramp-Up tick (every 1 ms) ---------------------- */
    if (now_us - last_ramp_tick_us >= RAMP_TICK_INTERVAL_US) {
        last_ramp_tick_us = now_us;

        for (uint8_t i = 0; i < PWM_CHANNELS; i++) {
            ramp_pwm_duty(i, target_duties[i], RAMP_STEP_DEFAULT);
        }
    }
}

#endif /* ARDUINO */
