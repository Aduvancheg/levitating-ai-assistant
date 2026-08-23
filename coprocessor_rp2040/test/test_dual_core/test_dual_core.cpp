/**
 * @file test_dual_core.cpp
 * @brief Unit tests for the inter-core mailbox and Core 1 UART task (RP-9).
 *
 * Tests verify:
 *   1. Mailbox initialisation to empty state.
 *   2. Produce/consume cycle with correct packet data transfer.
 *   3. Newest-wins overwrite semantics when consumer is slow.
 *   4. Consume from empty mailbox returns false.
 *   5. Integration: serialised packet → RingBuffer → parse → mailbox → consume.
 *
 * These tests run on [env:native] without hardware — the mailbox
 * and RingBuffer are fully portable.  Core 1 entry is a no-op stub
 * on native builds; we test the mailbox API directly.
 */

#include <unity.h>
#include <string.h>

#include "core1_uart_task.h"
#include "protocol_parser.h"
#include "ring_buffer.h"

/* ---- Setup / Teardown --------------------------------------------- */

void setUp(void) {
    mailbox_init();
}

void tearDown(void) {
    /* Nothing to clean up */
}

/* ---- Test: mailbox starts empty ----------------------------------- */

void test_mailbox_init_empty(void) {
    PacketData pkt;
    bool consumed = mailbox_consume(&pkt);
    TEST_ASSERT_FALSE_MESSAGE(consumed, "Mailbox should be empty after init");
}

/* ---- Test: produce then consume ----------------------------------- */

void test_mailbox_produce_consume(void) {
    /* Simulate Core 1 producing a packet */
    PacketData produced;
    produced.valid = true;
    produced.duty_cycles[0] = 100;
    produced.duty_cycles[1] = 200;
    produced.duty_cycles[2] = 300;
    produced.duty_cycles[3] = 400;
    produced.duty_cycles[4] = 460;

    /* We need to write directly to the mailbox since mailbox_produce is static.
     * Use the global mailbox directly for testing. */
    g_packet_mailbox.packet = produced;
    g_packet_mailbox.ready = true;

    /* Consume on Core 0 side */
    PacketData consumed;
    bool got = mailbox_consume(&consumed);

    TEST_ASSERT_TRUE_MESSAGE(got, "Should consume a packet");
    TEST_ASSERT_TRUE(consumed.valid);
    TEST_ASSERT_EQUAL_UINT16(100, consumed.duty_cycles[0]);
    TEST_ASSERT_EQUAL_UINT16(200, consumed.duty_cycles[1]);
    TEST_ASSERT_EQUAL_UINT16(300, consumed.duty_cycles[2]);
    TEST_ASSERT_EQUAL_UINT16(400, consumed.duty_cycles[3]);
    TEST_ASSERT_EQUAL_UINT16(460, consumed.duty_cycles[4]);

    /* Mailbox should now be empty */
    PacketData again;
    TEST_ASSERT_FALSE(mailbox_consume(&again));
}

/* ---- Test: newest-wins overwrite ---------------------------------- */

void test_mailbox_overwrite_newest_wins(void) {
    /* First packet */
    PacketData pkt1;
    pkt1.valid = true;
    pkt1.duty_cycles[0] = 50;
    pkt1.duty_cycles[1] = 50;
    pkt1.duty_cycles[2] = 50;
    pkt1.duty_cycles[3] = 50;
    pkt1.duty_cycles[4] = 50;
    g_packet_mailbox.packet = pkt1;
    g_packet_mailbox.ready = true;

    /* Second packet overwrites before consumer reads */
    PacketData pkt2;
    pkt2.valid = true;
    pkt2.duty_cycles[0] = 250;
    pkt2.duty_cycles[1] = 250;
    pkt2.duty_cycles[2] = 250;
    pkt2.duty_cycles[3] = 250;
    pkt2.duty_cycles[4] = 250;
    g_packet_mailbox.packet = pkt2;
    g_packet_mailbox.ready = true;

    /* Consumer should get the NEWEST packet */
    PacketData consumed;
    bool got = mailbox_consume(&consumed);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL_UINT16(250, consumed.duty_cycles[0]);
}

/* ---- Test: consume with NULL pointer ------------------------------ */

void test_mailbox_consume_null_safe(void) {
    bool result = mailbox_consume(NULL);
    TEST_ASSERT_FALSE_MESSAGE(result, "Consume with NULL should return false");
}

/* ---- Test: end-to-end serialize → RingBuffer → parse → mailbox ---- */

void test_e2e_serialize_parse_mailbox(void) {
    /* 1. Serialize a known duty vector into a binary packet */
    uint16_t duties[NUM_PWM_VALS] = {100, 200, 300, 400, 460};
    uint8_t frame[TOTAL_PACKET_SIZE];
    size_t written = serialize_packet(duties, frame, sizeof(frame));
    TEST_ASSERT_EQUAL(TOTAL_PACKET_SIZE, written);

    /* 2. Push into RingBuffer byte-by-byte (simulates UART RX) */
    RingBuffer rb;
    ring_buffer_init(&rb);
    for (size_t i = 0; i < TOTAL_PACKET_SIZE; i++) {
        ring_buffer_push(&rb, frame[i]);
    }
    TEST_ASSERT_EQUAL(TOTAL_PACKET_SIZE, ring_buffer_available(&rb));

    /* 3. Snapshot and parse */
    uint8_t linear_buf[TOTAL_PACKET_SIZE * 3];
    size_t lin_len = ring_buffer_snapshot(&rb, linear_buf, sizeof(linear_buf));

    PacketData parsed;
    size_t consumed_bytes = 0;
    bool ok = parse_byte_stream(linear_buf, lin_len, &parsed, &consumed_bytes);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(parsed.valid);

    /* 4. Write to mailbox and consume */
    g_packet_mailbox.packet = parsed;
    g_packet_mailbox.ready = true;

    PacketData final_pkt;
    TEST_ASSERT_TRUE(mailbox_consume(&final_pkt));
    TEST_ASSERT_EQUAL_UINT16(100, final_pkt.duty_cycles[0]);
    TEST_ASSERT_EQUAL_UINT16(200, final_pkt.duty_cycles[1]);
    TEST_ASSERT_EQUAL_UINT16(300, final_pkt.duty_cycles[2]);
    TEST_ASSERT_EQUAL_UINT16(400, final_pkt.duty_cycles[3]);
    TEST_ASSERT_EQUAL_UINT16(460, final_pkt.duty_cycles[4]);
}

/* ---- Test Runner -------------------------------------------------- */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mailbox_init_empty);
    RUN_TEST(test_mailbox_produce_consume);
    RUN_TEST(test_mailbox_overwrite_newest_wins);
    RUN_TEST(test_mailbox_consume_null_safe);
    RUN_TEST(test_e2e_serialize_parse_mailbox);
    return UNITY_END();
}
