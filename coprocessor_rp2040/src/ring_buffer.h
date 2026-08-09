/**
 * @file ring_buffer.h
 * @brief Lock-free SPSC Ring Buffer for RP2040 UART reception.
 *
 * Design decisions:
 *   - CAPACITY is a power of two (256) so that modulo can be replaced
 *     with a bit-mask `& (CAPACITY - 1)` for O(1) index wrapping.
 *   - `head_` and `tail_` are marked `volatile` to prevent the compiler
 *     from caching their values in registers across ISR / main-loop
 *     boundaries when optimising at -O2 / -O3.
 *   - For future Dual-Core RP2040 operation (Core 0 produces, Core 1
 *     consumes), upgrade `volatile` to `std::atomic<size_t>` with
 *     `memory_order_acquire / release` semantics.
 *
 * Thread-safety model (current, bare-metal single-core + ISR):
 *   - Producer (ISR / UART read) calls push() only.
 *   - Consumer (main loop parser) calls pop() / peek() / available().
 *   - One slot is always wasted to distinguish full from empty
 *     (classic SPSC pattern).
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ---- Capacity must be a power of two ------------------------------ */
#define RING_BUFFER_CAPACITY 256u

/* Compile-time check: CAPACITY must be power-of-two */
#if (RING_BUFFER_CAPACITY & (RING_BUFFER_CAPACITY - 1)) != 0
  #error "RING_BUFFER_CAPACITY must be a power of two for bit-mask optimisation"
#endif

#define RING_BUFFER_MASK (RING_BUFFER_CAPACITY - 1u)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Opaque ring buffer handle.
 *
 * `head_` points to the next write position (producer).
 * `tail_` points to the next read position (consumer).
 * Buffer is empty when head_ == tail_.
 * Buffer is full  when ((head_ + 1) & MASK) == tail_.
 */
typedef struct {
    uint8_t  data[RING_BUFFER_CAPACITY];
    volatile size_t head_;   /**< Write index — modified by producer only */
    volatile size_t tail_;   /**< Read  index — modified by consumer only */
} RingBuffer;

/**
 * @brief Initialise / reset ring buffer to empty state.
 */
void ring_buffer_init(RingBuffer *rb);

/**
 * @brief Push one byte into the buffer (producer side).
 *
 * If the buffer is full, the oldest byte is silently overwritten
 * (tail is advanced) to ensure the newest data is always available.
 * This is critical for UART streams where losing old bytes is
 * preferable to losing new ones.
 *
 * @return true if pushed without overflow, false if overflow occurred
 *         (oldest byte was dropped).
 */
bool ring_buffer_push(RingBuffer *rb, uint8_t byte);

/**
 * @brief Pop one byte from the buffer (consumer side).
 *
 * @param rb         Ring buffer instance.
 * @param out_byte   Destination for the popped byte.
 * @return true if a byte was available and popped, false if empty.
 */
bool ring_buffer_pop(RingBuffer *rb, uint8_t *out_byte);

/**
 * @brief Peek at a byte at a given offset from tail without consuming it.
 *
 * Offset 0 = oldest available byte (tail).
 *
 * @return true if offset is within available data, false otherwise.
 */
bool ring_buffer_peek(const RingBuffer *rb, size_t offset, uint8_t *out_byte);

/**
 * @brief Number of bytes currently available for reading.
 */
size_t ring_buffer_available(const RingBuffer *rb);

/**
 * @brief Discard `count` bytes from the read side (advance tail).
 *
 * If count > available(), all bytes are discarded.
 */
void ring_buffer_discard(RingBuffer *rb, size_t count);

/**
 * @brief Reset the buffer to empty.
 */
void ring_buffer_clear(RingBuffer *rb);

/**
 * @brief Copy up to `max_len` bytes from the buffer into a linear
 *        snapshot array WITHOUT consuming them.
 *
 * Useful for passing a contiguous byte array to `parse_byte_stream()`.
 *
 * @return Number of bytes actually copied.
 */
size_t ring_buffer_snapshot(const RingBuffer *rb, uint8_t *out_buf, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
