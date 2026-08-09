/**
 * @file ring_buffer.cpp
 * @brief Lock-free SPSC Ring Buffer implementation.
 *
 * All index arithmetic uses `& RING_BUFFER_MASK` instead of
 * `% RING_BUFFER_CAPACITY` for deterministic O(1) performance
 * on the RP2040 (which lacks a hardware divider in Cortex-M0+).
 */

#include "ring_buffer.h"

/* ---- Init / Clear ------------------------------------------------- */

void ring_buffer_init(RingBuffer *rb) {
    if (rb == nullptr) return;
    rb->head_ = 0;
    rb->tail_ = 0;
    /* No need to zero data[] — only indices matter for correctness */
}

void ring_buffer_clear(RingBuffer *rb) {
    if (rb == nullptr) return;
    rb->head_ = 0;
    rb->tail_ = 0;
}

/* ---- Producer: push ----------------------------------------------- */

bool ring_buffer_push(RingBuffer *rb, uint8_t byte) {
    if (rb == nullptr) return false;

    size_t next_head = (rb->head_ + 1u) & RING_BUFFER_MASK;

    if (next_head == rb->tail_) {
        /* Buffer full — overwrite oldest byte by advancing tail.
         * For UART streams, newest data is always more relevant
         * than stale bytes.                                           */
        rb->tail_ = (rb->tail_ + 1u) & RING_BUFFER_MASK;
        rb->data[rb->head_] = byte;
        rb->head_ = next_head;
        return false;  /* Overflow indicator */
    }

    rb->data[rb->head_] = byte;
    rb->head_ = next_head;
    return true;
}

/* ---- Consumer: pop ------------------------------------------------ */

bool ring_buffer_pop(RingBuffer *rb, uint8_t *out_byte) {
    if (rb == nullptr || out_byte == nullptr) return false;

    if (rb->tail_ == rb->head_) {
        return false;  /* Empty */
    }

    *out_byte = rb->data[rb->tail_];
    rb->tail_ = (rb->tail_ + 1u) & RING_BUFFER_MASK;
    return true;
}

/* ---- Peek without consuming --------------------------------------- */

bool ring_buffer_peek(const RingBuffer *rb, size_t offset, uint8_t *out_byte) {
    if (rb == nullptr || out_byte == nullptr) return false;

    size_t avail = ring_buffer_available(rb);
    if (offset >= avail) {
        return false;
    }

    size_t index = (rb->tail_ + offset) & RING_BUFFER_MASK;
    *out_byte = rb->data[index];
    return true;
}

/* ---- Available count ---------------------------------------------- */

size_t ring_buffer_available(const RingBuffer *rb) {
    if (rb == nullptr) return 0;

    /* Cast to signed-safe arithmetic via the mask.
     * (head - tail) & MASK works correctly even when head < tail
     * because CAPACITY is a power of two.                             */
    return (rb->head_ - rb->tail_) & RING_BUFFER_MASK;
}

/* ---- Discard (advance tail) --------------------------------------- */

void ring_buffer_discard(RingBuffer *rb, size_t count) {
    if (rb == nullptr) return;

    size_t avail = ring_buffer_available(rb);
    if (count > avail) {
        count = avail;
    }

    rb->tail_ = (rb->tail_ + count) & RING_BUFFER_MASK;
}

/* ---- Snapshot: copy to linear buffer without consuming ------------ */

size_t ring_buffer_snapshot(const RingBuffer *rb, uint8_t *out_buf, size_t max_len) {
    if (rb == nullptr || out_buf == nullptr) return 0;

    size_t avail = ring_buffer_available(rb);
    size_t to_copy = (avail < max_len) ? avail : max_len;

    for (size_t i = 0; i < to_copy; i++) {
        size_t index = (rb->tail_ + i) & RING_BUFFER_MASK;
        out_buf[i] = rb->data[index];
    }

    return to_copy;
}
