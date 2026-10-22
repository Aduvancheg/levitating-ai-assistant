/**
 * @file core1_uart_task.h
 * @brief Core 1 UART Receiver & Packet Parser — Dual-Core Partitioning (RP-9).
 *
 * Core 1 is dedicated exclusively to:
 *   1. Polling UART FIFO for incoming bytes at 921600 baud.
 *   2. Pushing bytes into the lock-free RingBuffer.
 *   3. Parsing complete binary frames (duty + command, header/CRC/footer).
 *   4. Publishing parsed PacketData through a seqlock mailbox for
 *      consumption by Core 0 (FSM + PWM).
 *
 * Inter-core communication uses a single-writer seqlock (review F5):
 * the previous "ready flag" mailbox let Core 1 overwrite the packet while
 * Core 0 was copying it, producing torn reads that the (already checked)
 * CRC could no longer catch.
 *
 * References:
 *   - levitation_solution_backlog_2-v4.md: RP-9 (Dual-Core Partitioning)
 *   - quality-v4.md: RP-CL-2 (Zero Dynamic Memory), RP-CL-4 (Volatile/Atomic)
 */

#ifndef CORE1_UART_TASK_H
#define CORE1_UART_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol_parser.h"
#include "ring_buffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Seqlock single-slot mailbox ---------------------------------- */

/**
 * @brief Single-slot seqlock mailbox for inter-core PacketData transfer.
 *
 * Protocol:
 *   Producer (Core 1, the ONLY writer):
 *     seq++ (odd = write in progress) → barrier → write packet →
 *     barrier → seq++ (even = stable).
 *   Consumer (Core 0, read-only on shared state):
 *     s1 = seq; if odd → retry; barrier; copy packet; barrier;
 *     s2 = seq; if s1 != s2 → retry (torn read discarded).
 *     A packet is "new" when s1 differs from the last consumed seq.
 *
 * Newest-wins semantics are preserved; a reader can never observe a
 * half-written packet.  Consumer retries are bounded so Core 0 never
 * spins indefinitely.
 */
typedef struct {
    PacketData        packet;
    volatile uint32_t seq;   /**< even = stable, odd = write in progress */
} PacketMailbox;

/** Max consumer retries before giving up for this loop iteration. */
#define MAILBOX_MAX_READ_RETRIES  4

/**
 * @brief Global mailbox instance — shared between Core 0 and Core 1.
 */
extern PacketMailbox g_packet_mailbox;

/**
 * @brief Initialise the mailbox to empty state.
 *
 * Must be called from Core 0 BEFORE launching Core 1.
 */
void mailbox_init(void);

/**
 * @brief Core 1: publish a packet (newest-wins).
 *
 * Must only ever be called from a single producer context.
 */
void mailbox_produce(const PacketData *packet);

/**
 * @brief Core 0: consume the newest packet if one was published since
 *        the last successful consume.
 *
 * @param out_packet  Destination for the consumed packet.
 * @return true if a new, untorn packet was copied; false if nothing new
 *         (or the producer kept writing for MAILBOX_MAX_READ_RETRIES).
 */
bool mailbox_consume(PacketData *out_packet);

/**
 * @brief Core 1 RX step: try to extract one frame from the ring buffer.
 *
 * Portable (no hardware calls) so it is unit-testable on [env:native].
 * On success the frame and everything before it are discarded.  On
 * failure, bytes that can no longer start a complete frame are
 * discarded so that a burst of garbage cannot stall parsing (the parse
 * window is bounded, so undiscarded garbage would otherwise hide valid
 * frames behind it).
 *
 * @return true if a valid frame was written to out_packet.
 */
bool core1_extract_frame(RingBuffer *rb, PacketData *out_packet);

#if defined(UNIT_TEST)
/** Test hook: simulate the producer being preempted mid-write. */
void mailbox_test_begin_write(void);
void mailbox_test_end_write(void);
#endif

/**
 * @brief Core 1 entry point — UART RX polling + parsing + mailbox.
 *
 * This function never returns. On native/host builds, it is a no-op stub.
 */
void core1_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE1_UART_TASK_H */
