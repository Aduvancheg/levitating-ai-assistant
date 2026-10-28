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

/** Sequence number of the last packet consumed by Core 0. */
static uint32_t s_last_consumed_seq = 0;

/* ---- Memory barrier abstraction ----------------------------------- */

#if defined(TARGET_RP2040) || defined(PICO_BOARD) || defined(PICO_ON_DEVICE)
  #include "pico/stdlib.h"
  #include "hardware/uart.h"
  #include "hardware/timer.h"
  #include "hardware/sync.h"   /* __dmb() */

  #define MEMORY_BARRIER()  __dmb()
#else
  /* Native / host build — compiler barrier for deterministic ordering */
  #if defined(__GNUC__) || defined(__clang__)
    #define MEMORY_BARRIER()  __asm__ __volatile__("" ::: "memory")
  #else
    #define MEMORY_BARRIER()  do {} while(0)
  #endif
#endif

/* ---- Mailbox implementation (single-writer seqlock, review F5) ---- */

void mailbox_init(void) {
    memset(&g_packet_mailbox.packet, 0, sizeof(PacketData));
    g_packet_mailbox.packet.valid = false;
    MEMORY_BARRIER();
    g_packet_mailbox.seq = 0;
    s_last_consumed_seq  = 0;
    MEMORY_BARRIER();
}

void mailbox_produce(const PacketData *packet) {
    if (packet == NULL) {
        return;
    }

    /* Odd sequence number signals write-in-progress to Core 0 */
    g_packet_mailbox.seq++;
    MEMORY_BARRIER();

    g_packet_mailbox.packet = *packet;
    MEMORY_BARRIER();

    /* Even sequence number signals stable snapshot */
    g_packet_mailbox.seq++;
    MEMORY_BARRIER();
}

bool mailbox_consume(PacketData *out_packet) {
    if (out_packet == NULL) {
        return false;
    }

    for (uint8_t attempt = 0; attempt < MAILBOX_MAX_READ_RETRIES; attempt++) {
        uint32_t s1 = g_packet_mailbox.seq;

        /* Odd seq means producer is currently writing — retry */
        if ((s1 & 1u) != 0u) {
            MEMORY_BARRIER();
            continue;
        }

        /* Nothing new published since the last successful consume */
        if (s1 == s_last_consumed_seq) {
            return false;
        }

        MEMORY_BARRIER();
        PacketData snapshot = g_packet_mailbox.packet;
        MEMORY_BARRIER();

        uint32_t s2 = g_packet_mailbox.seq;
        if (s1 == s2) {
            *out_packet = snapshot;
            s_last_consumed_seq = s1;
            return true;
        }
        /* s1 != s2: producer overwrote while we were reading — retry */
    }

    return false;
}

bool core1_extract_frame(RingBuffer *rb, PacketData *out_packet) {
    if (rb == NULL || out_packet == NULL) {
        return false;
    }

    if (ring_buffer_available(rb) < MIN_FRAME_SIZE) {
        return false;
    }

    uint8_t linear_buf[TOTAL_PACKET_SIZE * 3];
    size_t lin_len = ring_buffer_snapshot(rb, linear_buf, sizeof(linear_buf));

    size_t consumed = 0;
    if (parse_byte_stream(linear_buf, lin_len, out_packet, &consumed)) {
        if (consumed > 0) {
            ring_buffer_discard(rb, consumed);
        }
        return true;
    }

    /* Discard leading bytes that cannot possibly start a full frame so
     * that a burst of garbage cannot permanently fill the parse window. */
    if (lin_len >= TOTAL_PACKET_SIZE) {
        size_t stale = lin_len - (TOTAL_PACKET_SIZE - 1);
        ring_buffer_discard(rb, stale);
    }

    return false;
}

#if defined(UNIT_TEST)
void mailbox_test_begin_write(void) {
    if ((g_packet_mailbox.seq & 1u) == 0u) {
        g_packet_mailbox.seq++;
        MEMORY_BARRIER();
    }
}

void mailbox_test_end_write(void) {
    if ((g_packet_mailbox.seq & 1u) != 0u) {
        MEMORY_BARRIER();
        g_packet_mailbox.seq++;
        MEMORY_BARRIER();
    }
}
#endif

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

        /* 2. Extract and publish all complete frames buffered so far */
        PacketData packet;
        while (core1_extract_frame(&uart_rb, &packet)) {
            mailbox_produce(&packet);
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
