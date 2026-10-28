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
    memset(&produced, 0, sizeof(produced));
    produced.valid = true;
    produced.type  = FRAME_TYPE_DUTY;
    produced.duty_cycles[0] = 100;
    produced.duty_cycles[1] = 200;
    produced.duty_cycles[2] = 300;
    produced.duty_cycles[3] = 400;
    produced.duty_cycles[4] = 460;

    mailbox_produce(&produced);

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

    /* Mailbox should now report no new packet */
    PacketData again;
    TEST_ASSERT_FALSE(mailbox_consume(&again));
}

/* ---- Test: newest-wins overwrite ---------------------------------- */

void test_mailbox_overwrite_newest_wins(void) {
    /* First packet */
    PacketData pkt1;
    memset(&pkt1, 0, sizeof(pkt1));
    pkt1.valid = true;
    pkt1.duty_cycles[0] = 50;
    pkt1.duty_cycles[1] = 50;
    pkt1.duty_cycles[2] = 50;
    pkt1.duty_cycles[3] = 50;
    pkt1.duty_cycles[4] = 50;
    mailbox_produce(&pkt1);

    /* Second packet overwrites before consumer reads */
    PacketData pkt2;
    memset(&pkt2, 0, sizeof(pkt2));
    pkt2.valid = true;
    pkt2.duty_cycles[0] = 250;
    pkt2.duty_cycles[1] = 250;
    pkt2.duty_cycles[2] = 250;
    pkt2.duty_cycles[3] = 250;
    pkt2.duty_cycles[4] = 250;
    mailbox_produce(&pkt2);

    /* Consumer should get the NEWEST packet */
    PacketData consumed;
    bool got = mailbox_consume(&consumed);
    TEST_ASSERT_TRUE(got);
    TEST_ASSERT_EQUAL_UINT16(250, consumed.duty_cycles[0]);
}

/* ---- Test: seqlock rejects mid-write (torn read protection, F5) --- */

void test_mailbox_seqlock_rejects_in_progress_write(void) {
    PacketData pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.valid = true;
    pkt.duty_cycles[0] = 333;
    mailbox_produce(&pkt);

    /* Simulate Core 1 starting another write and being preempted */
    mailbox_test_begin_write();
    g_packet_mailbox.packet.duty_cycles[0] = 999; /* half-written */

    PacketData out;
    memset(&out, 0, sizeof(out));
    TEST_ASSERT_FALSE_MESSAGE(
        mailbox_consume(&out),
        "Consumer must reject mailbox while producer write is in progress"
    );

    /* Finish the write; consumer now sees the complete new packet */
    mailbox_test_end_write();
    TEST_ASSERT_TRUE(mailbox_consume(&out));
    TEST_ASSERT_EQUAL_UINT16(999, out.duty_cycles[0]);
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

    /* 3. Extract frame via core1_extract_frame and publish */
    PacketData parsed;
    bool ok = core1_extract_frame(&rb, &parsed);
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_TRUE(parsed.valid);
    TEST_ASSERT_EQUAL(0, ring_buffer_available(&rb));

    /* 4. Write to mailbox and consume */
    mailbox_produce(&parsed);

    PacketData final_pkt;
    TEST_ASSERT_TRUE(mailbox_consume(&final_pkt));
    TEST_ASSERT_EQUAL_UINT16(100, final_pkt.duty_cycles[0]);
    TEST_ASSERT_EQUAL_UINT16(200, final_pkt.duty_cycles[1]);
    TEST_ASSERT_EQUAL_UINT16(300, final_pkt.duty_cycles[2]);
    TEST_ASSERT_EQUAL_UINT16(400, final_pkt.duty_cycles[3]);
    TEST_ASSERT_EQUAL_UINT16(460, final_pkt.duty_cycles[4]);
}

/* ---- Test: core1_extract_frame discards long garbage burst --------- */

void test_core1_extract_frame_recovers_after_long_garbage(void) {
    RingBuffer rb;
    ring_buffer_init(&rb);

    /* Push 80 bytes of garbage (> 3 * TOTAL_PACKET_SIZE snapshot window) */
    for (int i = 0; i < 80; i++) {
        ring_buffer_push(&rb, 0xEE);
        PacketData dummy;
        TEST_ASSERT_FALSE(core1_extract_frame(&rb, &dummy));
    }

    /* Now push a valid command frame (7 bytes) */
    uint8_t cmd_frame[CMD_PACKET_SIZE];
    size_t n = serialize_command(CMD_PREPARE_OTA, 0, cmd_frame, sizeof(cmd_frame));
    TEST_ASSERT_EQUAL(CMD_PACKET_SIZE, n);
    for (size_t i = 0; i < n; i++) {
        ring_buffer_push(&rb, cmd_frame[i]);
    }

    PacketData out;
    TEST_ASSERT_TRUE(core1_extract_frame(&rb, &out));
    TEST_ASSERT_TRUE(out.valid);
    TEST_ASSERT_EQUAL(FRAME_TYPE_COMMAND, out.type);
    TEST_ASSERT_EQUAL_UINT8(CMD_PREPARE_OTA, out.command);
}

/* ---- Test Runner -------------------------------------------------- */

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mailbox_init_empty);
    RUN_TEST(test_mailbox_produce_consume);
    RUN_TEST(test_mailbox_overwrite_newest_wins);
    RUN_TEST(test_mailbox_seqlock_rejects_in_progress_write);
    RUN_TEST(test_mailbox_consume_null_safe);
    RUN_TEST(test_e2e_serialize_parse_mailbox);
    RUN_TEST(test_core1_extract_frame_recovers_after_long_garbage);
    return UNITY_END();
}
