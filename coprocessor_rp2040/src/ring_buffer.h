#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* ===================================================================
 * Ring Buffer — O(1) Circular Byte Buffer for UART RX
 *
 * Replaces the linear memmove-based buffer in the UART handler.
 * At 921600 baud (~10 µs per byte), memmove of the entire buffer
 * on every byte wastes CPU cycles.  This ring buffer provides:
 *
 *   - O(1) push: overwrites oldest byte when full (no data loss
 *     for streaming protocols — old bytes are already stale).
 *   - O(N) linearize: copies ring contents into a flat buffer
 *     for parse_byte_stream().  Called only when enough bytes
 *     have accumulated for a potential packet (≥ TOTAL_PACKET_SIZE).
 *   - O(1) consume: advances the tail pointer after a successful
 *     parse, discarding processed bytes.
 *
 * No dynamic memory.  No blocking calls.  ISR-safe for single
 * producer (UART RX ISR) / single consumer (main loop parser).
 *
 * References:
 *   - architecture.md §6 (high-speed UART protocol)
 * =================================================================== */

typedef struct {
    uint8_t *buf;       /**< Backing storage (caller-provided, static) */
    size_t   capacity;  /**< Total buffer capacity in bytes             */
    size_t   head;      /**< Next write position (wraps around)         */
    size_t   count;     /**< Current number of valid bytes in buffer    */
} RingBuffer;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise ring buffer with caller-provided backing storage.
 *
 * @param rb        Ring buffer instance.
 * @param backing   Static byte array for storage.
 * @param capacity  Size of the backing array in bytes.
 */
void rb_init(RingBuffer *rb, uint8_t *backing, size_t capacity);

/**
 * @brief Push one byte into the ring buffer.
 *
 * If the buffer is full, the oldest byte is silently overwritten.
 * This is the correct behavior for streaming UART protocols where
 * stale bytes are worthless — we always want the freshest data.
 *
 * @param rb    Ring buffer instance.
 * @param byte  The byte to push.
 */
void rb_push(RingBuffer *rb, uint8_t byte);

/**
 * @brief Copy ring buffer contents into a flat (linear) buffer.
 *
 * Necessary because parse_byte_stream() expects a contiguous array.
 * Only call this when rb_count() >= TOTAL_PACKET_SIZE.
 *
 * @param rb       Ring buffer instance.
 * @param out      Destination flat buffer.
 * @param max_len  Size of the destination buffer.
 * @return Number of bytes copied (min of count and max_len).
 */
size_t rb_linearize(const RingBuffer *rb, uint8_t *out, size_t max_len);

/**
 * @brief Consume (discard) the oldest N bytes from the buffer.
 *
 * Called after parse_byte_stream() reports how many bytes it consumed.
 *
 * @param rb  Ring buffer instance.
 * @param n   Number of bytes to consume. Clamped to count.
 */
void rb_consume(RingBuffer *rb, size_t n);

/**
 * @brief Return the current number of valid bytes in the buffer.
 */
size_t rb_count(const RingBuffer *rb);

/**
 * @brief Reset the ring buffer to empty state.
 */
void rb_reset(RingBuffer *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
