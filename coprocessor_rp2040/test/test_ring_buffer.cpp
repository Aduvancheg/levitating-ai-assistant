/**
 * @file test_ring_buffer.cpp
 * @brief Unit tests for lock-free SPSC Ring Buffer — push, pop, overflow,
 *        snapshot, peek, discard, and bitmask correctness.
 */

#include "../src/ring_buffer.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void test_init_empty() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    assert(ring_buffer_available(&rb) == 0);

    uint8_t byte;
    assert(ring_buffer_pop(&rb, &byte) == false);

    printf("[PASS] test_init_empty\n");
}

void test_push_pop_basic() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    assert(ring_buffer_push(&rb, 0xAA) == true);
    assert(ring_buffer_push(&rb, 0xBB) == true);
    assert(ring_buffer_available(&rb) == 2);

    uint8_t byte;
    assert(ring_buffer_pop(&rb, &byte) == true);
    assert(byte == 0xAA);

    assert(ring_buffer_pop(&rb, &byte) == true);
    assert(byte == 0xBB);

    assert(ring_buffer_available(&rb) == 0);
    assert(ring_buffer_pop(&rb, &byte) == false);

    printf("[PASS] test_push_pop_basic\n");
}

void test_fill_to_capacity() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* SPSC ring buffer holds CAPACITY-1 elements (one slot wasted) */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        bool ok = ring_buffer_push(&rb, (uint8_t)(i & 0xFF));
        assert(ok == true);
    }

    assert(ring_buffer_available(&rb) == RING_BUFFER_CAPACITY - 1);

    /* Verify FIFO order */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        uint8_t byte;
        assert(ring_buffer_pop(&rb, &byte) == true);
        assert(byte == (uint8_t)(i & 0xFF));
    }

    printf("[PASS] test_fill_to_capacity\n");
}

void test_overflow_drops_oldest() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* Fill completely (CAPACITY-1 items) */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        ring_buffer_push(&rb, (uint8_t)i);
    }

    /* Push one more — should overflow, dropping oldest */
    bool overflow = ring_buffer_push(&rb, 0xFF);
    assert(overflow == false);  /* Overflow indicator */

    /* Available should still be CAPACITY-1 */
    assert(ring_buffer_available(&rb) == RING_BUFFER_CAPACITY - 1);

    /* First byte should be 0x01 (0x00 was dropped) */
    uint8_t byte;
    assert(ring_buffer_pop(&rb, &byte) == true);
    assert(byte == 0x01);

    printf("[PASS] test_overflow_drops_oldest\n");
}

void test_peek_without_consuming() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 'H');
    ring_buffer_push(&rb, 'i');
    ring_buffer_push(&rb, '!');

    uint8_t byte;
    assert(ring_buffer_peek(&rb, 0, &byte) == true);
    assert(byte == 'H');
    assert(ring_buffer_peek(&rb, 1, &byte) == true);
    assert(byte == 'i');
    assert(ring_buffer_peek(&rb, 2, &byte) == true);
    assert(byte == '!');
    assert(ring_buffer_peek(&rb, 3, &byte) == false);

    /* Available unchanged — peek doesn't consume */
    assert(ring_buffer_available(&rb) == 3);

    printf("[PASS] test_peek_without_consuming\n");
}

void test_discard() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    for (uint8_t i = 0; i < 10; i++) {
        ring_buffer_push(&rb, i);
    }

    ring_buffer_discard(&rb, 5);
    assert(ring_buffer_available(&rb) == 5);

    uint8_t byte;
    assert(ring_buffer_pop(&rb, &byte) == true);
    assert(byte == 5);  /* First 5 bytes (0..4) were discarded */

    printf("[PASS] test_discard\n");
}

void test_discard_more_than_available() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 0xAA);
    ring_buffer_push(&rb, 0xBB);

    ring_buffer_discard(&rb, 100);  /* More than available */
    assert(ring_buffer_available(&rb) == 0);

    printf("[PASS] test_discard_more_than_available\n");
}

void test_snapshot_copies_without_consuming() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    uint8_t input[] = {0xAA, 0x55, 0x01, 0x02, 0x03};
    for (size_t i = 0; i < sizeof(input); i++) {
        ring_buffer_push(&rb, input[i]);
    }

    uint8_t snapshot[10];
    size_t copied = ring_buffer_snapshot(&rb, snapshot, sizeof(snapshot));
    assert(copied == 5);
    assert(memcmp(snapshot, input, 5) == 0);

    /* Available unchanged */
    assert(ring_buffer_available(&rb) == 5);

    printf("[PASS] test_snapshot_copies_without_consuming\n");
}

void test_snapshot_partial() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    for (uint8_t i = 0; i < 20; i++) {
        ring_buffer_push(&rb, i);
    }

    uint8_t snapshot[5];
    size_t copied = ring_buffer_snapshot(&rb, snapshot, 5);
    assert(copied == 5);
    assert(snapshot[0] == 0);
    assert(snapshot[4] == 4);

    printf("[PASS] test_snapshot_partial\n");
}

void test_clear() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 0xAA);
    ring_buffer_push(&rb, 0xBB);
    ring_buffer_clear(&rb);

    assert(ring_buffer_available(&rb) == 0);

    uint8_t byte;
    assert(ring_buffer_pop(&rb, &byte) == false);

    printf("[PASS] test_clear\n");
}

void test_wrap_around_correctness() {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* Push and pop many times to wrap head/tail around the buffer */
    for (int cycle = 0; cycle < 3; cycle++) {
        for (size_t i = 0; i < 200; i++) {
            ring_buffer_push(&rb, (uint8_t)(i & 0xFF));
        }
        for (size_t i = 0; i < 200; i++) {
            uint8_t byte;
            bool ok = ring_buffer_pop(&rb, &byte);
            assert(ok == true);
            assert(byte == (uint8_t)(i & 0xFF));
        }
        assert(ring_buffer_available(&rb) == 0);
    }

    printf("[PASS] test_wrap_around_correctness\n");
}

void test_null_safety() {
    assert(ring_buffer_push(nullptr, 0) == false);
    assert(ring_buffer_pop(nullptr, nullptr) == false);
    assert(ring_buffer_available(nullptr) == 0);
    assert(ring_buffer_peek(nullptr, 0, nullptr) == false);

    RingBuffer rb;
    ring_buffer_init(&rb);
    assert(ring_buffer_pop(&rb, nullptr) == false);
    assert(ring_buffer_peek(&rb, 0, nullptr) == false);
    assert(ring_buffer_snapshot(&rb, nullptr, 10) == 0);
    assert(ring_buffer_snapshot(nullptr, nullptr, 10) == 0);

    /* These should not crash */
    ring_buffer_init(nullptr);
    ring_buffer_clear(nullptr);
    ring_buffer_discard(nullptr, 5);

    printf("[PASS] test_null_safety\n");
}

void test_bitmask_vs_modulo() {
    /* Verify that & (CAPACITY - 1) produces the same result
     * as % CAPACITY for all relevant index values.                    */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY * 3; i++) {
        assert((i & RING_BUFFER_MASK) == (i % RING_BUFFER_CAPACITY));
    }

    printf("[PASS] test_bitmask_vs_modulo\n");
}

int main() {
    printf("--- Running Ring Buffer Unit Tests ---\n");
    test_init_empty();
    test_push_pop_basic();
    test_fill_to_capacity();
    test_overflow_drops_oldest();
    test_peek_without_consuming();
    test_discard();
    test_discard_more_than_available();
    test_snapshot_copies_without_consuming();
    test_snapshot_partial();
    test_clear();
    test_wrap_around_correctness();
    test_null_safety();
    test_bitmask_vs_modulo();
    printf("All Ring Buffer tests passed successfully.\n");
    return 0;
}
