/**
 * @file ring_buffer.cpp
 * @brief O(1) circular byte buffer implementation for UART RX.
 *
 * Pure C implementation with no dynamic memory, no blocking calls,
 * and no hardware dependencies.  Fully testable in [env:native].
 *
 * When the buffer is full, rb_push() overwrites the oldest byte.
 * This ensures the parser always sees the freshest data from the
 * UART stream, which is critical at 921600 baud (~10 µs per byte).
 */

#include "ring_buffer.h"
#include <string.h>

void rb_init(RingBuffer *rb, uint8_t *backing, size_t capacity) {
    rb->buf      = backing;
    rb->capacity = capacity;
    rb->head     = 0;
    rb->count    = 0;
}

void rb_push(RingBuffer *rb, uint8_t byte) {
    rb->buf[rb->head] = byte;
    rb->head = (rb->head + 1) % rb->capacity;

    if (rb->count < rb->capacity) {
        rb->count++;
    }
    /* If count == capacity, we just overwrote the oldest byte.
     * head already advanced past it, so count stays at capacity. */
}

size_t rb_linearize(const RingBuffer *rb, uint8_t *out, size_t max_len) {
    size_t to_copy = (rb->count < max_len) ? rb->count : max_len;
    if (to_copy == 0) return 0;

    /* Calculate tail (oldest byte position) */
    size_t tail;
    if (rb->head >= rb->count) {
        tail = rb->head - rb->count;
    } else {
        tail = rb->capacity - (rb->count - rb->head);
    }

    /* Copy in one or two chunks depending on wrap */
    size_t first_chunk = rb->capacity - tail;
    if (first_chunk >= to_copy) {
        /* No wrap: single contiguous copy */
        memcpy(out, rb->buf + tail, to_copy);
    } else {
        /* Wrap: copy tail..end, then start..remainder */
        memcpy(out, rb->buf + tail, first_chunk);
        memcpy(out + first_chunk, rb->buf, to_copy - first_chunk);
    }

    return to_copy;
}

void rb_consume(RingBuffer *rb, size_t n) {
    if (n >= rb->count) {
        rb->count = 0;
        /* head stays where it is — next push goes to current head */
    } else {
        rb->count -= n;
    }
    /* No need to move head — consumed bytes are simply forgotten.
     * The tail implicitly moves forward (tail = head - count). */
}

size_t rb_count(const RingBuffer *rb) {
    return rb->count;
}

void rb_reset(RingBuffer *rb) {
    rb->head  = 0;
    rb->count = 0;
}
