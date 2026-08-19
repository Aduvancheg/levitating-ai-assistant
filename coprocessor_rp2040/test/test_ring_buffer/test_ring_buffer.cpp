/**
 * @file test_ring_buffer.cpp
 * @brief Unit tests for lock-free SPSC Ring Buffer — push, pop, overflow,
 *        snapshot, peek, discard, and bitmask correctness.
 */

#include "../../src/ring_buffer.h"
#include <unity.h>
#include <string.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_init_empty(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&rb));

    uint8_t byte;
    TEST_ASSERT_FALSE(ring_buffer_pop(&rb, &byte));
}

void test_push_pop_basic(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    TEST_ASSERT_TRUE(ring_buffer_push(&rb, 0xAA));
    TEST_ASSERT_TRUE(ring_buffer_push(&rb, 0xBB));
    TEST_ASSERT_EQUAL_UINT32(2, ring_buffer_available(&rb));

    uint8_t byte;
    TEST_ASSERT_TRUE(ring_buffer_pop(&rb, &byte));
    TEST_ASSERT_EQUAL_HEX8(0xAA, byte);

    TEST_ASSERT_TRUE(ring_buffer_pop(&rb, &byte));
    TEST_ASSERT_EQUAL_HEX8(0xBB, byte);

    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&rb));
    TEST_ASSERT_FALSE(ring_buffer_pop(&rb, &byte));
}

void test_fill_to_capacity(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* SPSC ring buffer holds CAPACITY-1 elements (one slot wasted) */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        bool ok = ring_buffer_push(&rb, (uint8_t)(i & 0xFF));
        TEST_ASSERT_TRUE(ok);
    }

    TEST_ASSERT_EQUAL_UINT32(RING_BUFFER_CAPACITY - 1, ring_buffer_available(&rb));

    /* Verify FIFO order */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        uint8_t byte;
        TEST_ASSERT_TRUE(ring_buffer_pop(&rb, &byte));
        TEST_ASSERT_EQUAL_HEX8((uint8_t)(i & 0xFF), byte);
    }
}

void test_overflow_drops_oldest(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* Fill completely (CAPACITY-1 items) */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY - 1; i++) {
        ring_buffer_push(&rb, (uint8_t)i);
    }

    /* Push one more — should overflow, dropping oldest */
    bool no_overflow = ring_buffer_push(&rb, 0xFF);
    TEST_ASSERT_FALSE(no_overflow);  /* Overflow indicator */

    /* Available should still be CAPACITY-1 */
    TEST_ASSERT_EQUAL_UINT32(RING_BUFFER_CAPACITY - 1, ring_buffer_available(&rb));

    /* First byte should be 0x01 (0x00 was dropped) */
    uint8_t byte;
    TEST_ASSERT_TRUE(ring_buffer_pop(&rb, &byte));
    TEST_ASSERT_EQUAL_HEX8(0x01, byte);
}

void test_peek_without_consuming(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 'H');
    ring_buffer_push(&rb, 'i');
    ring_buffer_push(&rb, '!');

    uint8_t byte;
    TEST_ASSERT_TRUE(ring_buffer_peek(&rb, 0, &byte));
    TEST_ASSERT_EQUAL_HEX8('H', byte);
    TEST_ASSERT_TRUE(ring_buffer_peek(&rb, 1, &byte));
    TEST_ASSERT_EQUAL_HEX8('i', byte);
    TEST_ASSERT_TRUE(ring_buffer_peek(&rb, 2, &byte));
    TEST_ASSERT_EQUAL_HEX8('!', byte);
    TEST_ASSERT_FALSE(ring_buffer_peek(&rb, 3, &byte));

    /* Available unchanged — peek doesn't consume */
    TEST_ASSERT_EQUAL_UINT32(3, ring_buffer_available(&rb));
}

void test_discard(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    for (uint8_t i = 0; i < 10; i++) {
        ring_buffer_push(&rb, i);
    }

    ring_buffer_discard(&rb, 5);
    TEST_ASSERT_EQUAL_UINT32(5, ring_buffer_available(&rb));

    uint8_t byte;
    TEST_ASSERT_TRUE(ring_buffer_pop(&rb, &byte));
    TEST_ASSERT_EQUAL_HEX8(5, byte);  /* First 5 bytes (0..4) were discarded */
}

void test_discard_more_than_available(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 0xAA);
    ring_buffer_push(&rb, 0xBB);

    ring_buffer_discard(&rb, 100);  /* More than available */
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&rb));
}

void test_snapshot_copies_without_consuming(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    uint8_t input[] = {0xAA, 0x55, 0x01, 0x02, 0x03};
    for (size_t i = 0; i < sizeof(input); i++) {
        ring_buffer_push(&rb, input[i]);
    }

    uint8_t snapshot[10];
    size_t copied = ring_buffer_snapshot(&rb, snapshot, sizeof(snapshot));
    TEST_ASSERT_EQUAL_UINT32(5, copied);
    TEST_ASSERT_EQUAL_MEMORY(input, snapshot, 5);

    /* Available unchanged */
    TEST_ASSERT_EQUAL_UINT32(5, ring_buffer_available(&rb));
}

void test_snapshot_partial(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    for (uint8_t i = 0; i < 20; i++) {
        ring_buffer_push(&rb, i);
    }

    uint8_t snapshot[5];
    size_t copied = ring_buffer_snapshot(&rb, snapshot, 5);
    TEST_ASSERT_EQUAL_UINT32(5, copied);
    TEST_ASSERT_EQUAL_HEX8(0, snapshot[0]);
    TEST_ASSERT_EQUAL_HEX8(4, snapshot[4]);
}

void test_clear(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    ring_buffer_push(&rb, 0xAA);
    ring_buffer_push(&rb, 0xBB);
    ring_buffer_clear(&rb);

    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&rb));

    uint8_t byte;
    TEST_ASSERT_FALSE(ring_buffer_pop(&rb, &byte));
}

void test_wrap_around_correctness(void) {
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
            TEST_ASSERT_TRUE(ok);
            TEST_ASSERT_EQUAL_HEX8((uint8_t)(i & 0xFF), byte);
        }
        TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(&rb));
    }
}

void test_null_safety(void) {
    TEST_ASSERT_FALSE(ring_buffer_push(nullptr, 0));
    TEST_ASSERT_FALSE(ring_buffer_pop(nullptr, nullptr));
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_available(nullptr));
    TEST_ASSERT_FALSE(ring_buffer_peek(nullptr, 0, nullptr));

    RingBuffer rb;
    ring_buffer_init(&rb);
    TEST_ASSERT_FALSE(ring_buffer_pop(&rb, nullptr));
    TEST_ASSERT_FALSE(ring_buffer_peek(&rb, 0, nullptr));
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_snapshot(&rb, nullptr, 10));
    TEST_ASSERT_EQUAL_UINT32(0, ring_buffer_snapshot(nullptr, nullptr, 10));

    /* These should not crash */
    ring_buffer_init(nullptr);
    ring_buffer_clear(nullptr);
    ring_buffer_discard(nullptr, 5);
}

void test_bitmask_vs_modulo(void) {
    /* Verify that & (CAPACITY - 1) produces the same result
     * as % CAPACITY for all relevant index values. */
    for (size_t i = 0; i < RING_BUFFER_CAPACITY * 3; i++) {
        TEST_ASSERT_EQUAL_UINT32((i % RING_BUFFER_CAPACITY), (i & RING_BUFFER_MASK));
    }
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_init_empty);
    RUN_TEST(test_push_pop_basic);
    RUN_TEST(test_fill_to_capacity);
    RUN_TEST(test_overflow_drops_oldest);
    RUN_TEST(test_peek_without_consuming);
    RUN_TEST(test_discard);
    RUN_TEST(test_discard_more_than_available);
    RUN_TEST(test_snapshot_copies_without_consuming);
    RUN_TEST(test_snapshot_partial);
    RUN_TEST(test_clear);
    RUN_TEST(test_wrap_around_correctness);
    RUN_TEST(test_null_safety);
    RUN_TEST(test_bitmask_vs_modulo);
    return UNITY_END();
}
