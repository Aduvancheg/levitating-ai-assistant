/**
 * @file test_ring_buffer.cpp
 * @brief Unit tests for O(1) circular byte buffer.
 */

#include "../../src/ring_buffer.h"
#include <unity.h>
#include <stdio.h>
#include <string.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

static void test_rb_push_and_linearize(void) {
    uint8_t backing[10];
    RingBuffer rb;
    rb_init(&rb, backing, sizeof(backing));

    TEST_ASSERT_EQUAL_UINT32(0, rb_count(&rb));

    rb_push(&rb, 0xAA);
    rb_push(&rb, 0xBB);
    rb_push(&rb, 0xCC);
    TEST_ASSERT_EQUAL_UINT32(3, rb_count(&rb));

    uint8_t out[10];
    size_t len = rb_linearize(&rb, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(3, len);
    TEST_ASSERT_EQUAL_HEX8(0xAA, out[0]);
    TEST_ASSERT_EQUAL_HEX8(0xBB, out[1]);
    TEST_ASSERT_EQUAL_HEX8(0xCC, out[2]);
}

static void test_rb_wrap_around(void) {
    uint8_t backing[5];
    RingBuffer rb;
    rb_init(&rb, backing, sizeof(backing));

    /* Push 7 bytes into a 5-byte capacity buffer */
    for (uint8_t i = 1; i <= 7; i++) {
        rb_push(&rb, i);
    }

    TEST_ASSERT_EQUAL_UINT32(5, rb_count(&rb));  /* Capped at capacity */

    uint8_t out[5];
    size_t len = rb_linearize(&rb, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(5, len);
    
    /* The first 2 bytes (1, 2) were overwritten.
     * We should see 3, 4, 5, 6, 7. */
    TEST_ASSERT_EQUAL_UINT8(3, out[0]);
    TEST_ASSERT_EQUAL_UINT8(4, out[1]);
    TEST_ASSERT_EQUAL_UINT8(5, out[2]);
    TEST_ASSERT_EQUAL_UINT8(6, out[3]);
    TEST_ASSERT_EQUAL_UINT8(7, out[4]);
}

static void test_rb_consume(void) {
    uint8_t backing[5];
    RingBuffer rb;
    rb_init(&rb, backing, sizeof(backing));

    rb_push(&rb, 1);
    rb_push(&rb, 2);
    rb_push(&rb, 3);
    TEST_ASSERT_EQUAL_UINT32(3, rb_count(&rb));

    rb_consume(&rb, 2);
    TEST_ASSERT_EQUAL_UINT32(1, rb_count(&rb));

    uint8_t out[5];
    size_t len = rb_linearize(&rb, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(1, len);
    TEST_ASSERT_EQUAL_UINT8(3, out[0]);

    /* Push more to wrap */
    rb_push(&rb, 4);
    rb_push(&rb, 5);
    rb_push(&rb, 6);
    rb_push(&rb, 7);
    
    TEST_ASSERT_EQUAL_UINT32(5, rb_count(&rb));
    len = rb_linearize(&rb, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(5, len);
    TEST_ASSERT_EQUAL_UINT8(3, out[0]);
    TEST_ASSERT_EQUAL_UINT8(4, out[1]);
    TEST_ASSERT_EQUAL_UINT8(5, out[2]);
    TEST_ASSERT_EQUAL_UINT8(6, out[3]);
    TEST_ASSERT_EQUAL_UINT8(7, out[4]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_rb_push_and_linearize);
    RUN_TEST(test_rb_wrap_around);
    RUN_TEST(test_rb_consume);
    return UNITY_END();
}
