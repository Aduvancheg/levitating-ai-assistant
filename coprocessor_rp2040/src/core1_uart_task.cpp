/**
 * @file core1_uart_task.cpp
 * @brief Core 1 UART Receiver & Packet Parser — implementation.
 *
 * Dedicated Core 1 task for the RP2040 dual-core architecture (RP-9).
 * Handles all UART I/O and protocol parsing, keeping Core 0 free
 * for deterministic PWM generation and FSM execution.
 *
 * No dynamic memory. No blocking delays. All time via hardware timer.
 *
 * References:
 *   - levitation_solution_backlog_2-v4.md: RP-9
 *   - quality-v4.md: RP-CL-1, RP-CL-2, RP-CL-4
 */

#include "core1_uart_task.h"
#include "ring_buffer.h"
#include <string.h>

/* ---- Global mailbox instance -------------------------------------- */

PacketMailbox g_packet_mailbox;

/* ---- Memory barrier abstraction ----------------------------------- */

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)
  #include "pico/stdlib.h"
  #include "hardware/uart.h"
  #include "hardware/timer.h"
  #include "hardware/sync.h"   /* __dmb() */

  #define MEMORY_BARRIER()  __dmb()
#else
  /* Native / host build — no real barrier needed for single-threaded tests */
  #define MEMORY_BARRIER()  do {} while(0)
#endif

/* ---- Mailbox implementation --------------------------------------- */

void mailbox_init(void) {
    memset(&g_packet_mailbox.packet, 0, sizeof(PacketData));
    g_packet_mailbox.packet.valid = false;
    MEMORY_BARRIER();
    g_packet_mailbox.ready = false;
    MEMORY_BARRIER();
}

/**
 * @brief Core 1: Write a parsed packet into the mailbox.
 *
 * Overwrites any unconsumed packet (newest-wins policy).
 * This is acceptable because at ~800 Hz packet rate and ~860 Hz
 * consumption rate, overwrite is extremely rare and the newest
 * data is always more relevant than stale data.
 */
static void mailbox_produce(const PacketData *packet) {
    /* Copy packet data first, then set ready flag with barrier.
     * This ensures Core 0 sees complete data when it reads ready=true. */
    g_packet_mailbox.packet = *packet;
    MEMORY_BARRIER();
    g_packet_mailbox.ready = true;
    MEMORY_BARRIER();
}

bool mailbox_consume(PacketData *out_packet) {
    if (out_packet == NULL) {
        return false;
    }

    if (!g_packet_mailbox.ready) {
        return false;
    }

    MEMORY_BARRIER();
    *out_packet = g_packet_mailbox.packet;
    MEMORY_BARRIER();
    g_packet_mailbox.ready = false;
    MEMORY_BARRIER();

    return true;
}

/* ---- Core 1 entry point ------------------------------------------- */

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)

/* UART configuration — must match main.cpp */
#define CORE1_UART_ID        uart1
#define CORE1_UART_BAUD_RATE 921600

void core1_entry(void) {
    /* Core 1 owns its own RingBuffer instance — no sharing with Core 0.
     * The RingBuffer is local to Core 1's producer/consumer cycle:
     * UART bytes go in, parsed packets come out into the mailbox. */
    static RingBuffer uart_rb;
    ring_buffer_init(&uart_rb);

    /* Note: UART hardware is already initialised by Core 0 in main().
     * Core 1 only reads from the UART FIFO — no re-initialisation needed.
     * The RP2040 UART FIFO is safe to read from any core. */

    while (true) {
        /* 1. Poll UART FIFO — non-blocking, zero-jitter (RP-CL-1) */
        while (uart_is_readable(CORE1_UART_ID)) {
            uint8_t byte_in = (uint8_t)uart_getc(CORE1_UART_ID);
            ring_buffer_push(&uart_rb, byte_in);
        }

        /* 2. Attempt packet parsing when enough bytes buffered */
        if (ring_buffer_available(&uart_rb) >= TOTAL_PACKET_SIZE) {
            uint8_t linear_buf[TOTAL_PACKET_SIZE * 3];
            size_t lin_len = ring_buffer_snapshot(
                &uart_rb, linear_buf, sizeof(linear_buf)
            );

            PacketData packet;
            size_t consumed = 0;
            if (parse_byte_stream(linear_buf, lin_len, &packet, &consumed)) {
                /* 3. Write parsed packet to mailbox for Core 0 */
                mailbox_produce(&packet);

                if (consumed > 0) {
                    ring_buffer_discard(&uart_rb, consumed);
                }
            }
        }

        /* Yield CPU briefly to avoid starving hardware interrupts.
         * tight_loop_contents() is a Pico SDK no-op that prevents
         * the compiler from optimising away the loop. */
        tight_loop_contents();
    }
}

#else
/* Native / host build — Core 1 entry is a no-op stub.
 * In unit tests, mailbox is fed directly via mailbox_produce(). */
void core1_entry(void) {
    /* No-op on host builds */
}
#endif
