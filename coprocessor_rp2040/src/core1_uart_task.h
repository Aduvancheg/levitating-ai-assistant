/**
 * @file core1_uart_task.h
 * @brief Core 1 UART Receiver & Packet Parser — Dual-Core Partitioning (RP-9).
 *
 * Core 1 is dedicated exclusively to:
 *   1. Polling UART FIFO for incoming bytes at 921600 baud.
 *   2. Pushing bytes into the lock-free RingBuffer.
 *   3. Parsing complete binary packets (header/CRC/footer).
 *   4. Writing parsed PacketData into a lock-free single-slot mailbox
 *      for consumption by Core 0 (FSM + PWM).
 *
 * Inter-core communication uses a volatile single-slot mailbox with
 * __dmb() memory barriers to guarantee visibility across Cortex-M0+ cores.
 * This is simpler and faster than SIO FIFO for our use case (~800 packets/s).
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

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Lock-free single-slot mailbox -------------------------------- */

/**
 * @brief Single-slot mailbox for inter-core PacketData transfer.
 *
 * Producer (Core 1) writes packet data and sets `ready` flag.
 * Consumer (Core 0) reads packet data and clears `ready` flag.
 *
 * Thread-safety:
 *   - Only Core 1 writes `packet` and sets `ready = true`.
 *   - Only Core 0 reads `packet` and sets `ready = false`.
 *   - Memory barriers (__dmb) ensure visibility across cores.
 *   - If Core 0 hasn't consumed the previous packet before Core 1
 *     produces a new one, the old packet is overwritten (newest-wins).
 */
typedef struct {
    PacketData packet;
    volatile bool ready;    /**< true = new packet available for Core 0 */
} PacketMailbox;

/**
 * @brief Global mailbox instance — shared between Core 0 and Core 1.
 *
 * Declared extern here, defined in core1_uart_task.cpp.
 * Core 0 reads from this in its main loop.
 */
extern PacketMailbox g_packet_mailbox;

/**
 * @brief Initialise the mailbox to empty state.
 *
 * Must be called from Core 0 BEFORE launching Core 1.
 */
void mailbox_init(void);

/**
 * @brief Core 0: Check if a new packet is available and consume it.
 *
 * If a packet is ready, copies it to `out_packet` and clears the
 * ready flag. Uses __dmb() for cross-core memory ordering.
 *
 * @param out_packet  Destination for the consumed packet.
 * @return true if a packet was consumed, false if mailbox was empty.
 */
bool mailbox_consume(PacketData *out_packet);

/**
 * @brief Core 1 entry point — UART RX polling + parsing + mailbox.
 *
 * This function never returns. It runs an infinite loop on Core 1:
 *   1. Poll UART FIFO for available bytes.
 *   2. Push bytes into the local RingBuffer.
 *   3. When enough bytes are buffered, attempt packet parsing.
 *   4. On successful parse, write to g_packet_mailbox.
 *
 * On native/host builds, this is a no-op stub.
 */
void core1_entry(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE1_UART_TASK_H */
