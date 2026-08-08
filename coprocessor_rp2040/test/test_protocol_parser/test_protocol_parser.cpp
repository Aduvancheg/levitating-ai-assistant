/**
 * @file test_protocol_parser.cpp
 * @brief Unit tests for binary protocol parser — CRC, corruption, ACK.
 */

#include "../../src/protocol_parser.h"
#include <unity.h>
#include <stdio.h>
#include <string.h>

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_ideal_packet_parsing(void) {
    uint16_t original_duties[NUM_PWM_VALS] = {100, 250, 512, 800, 1023};
    uint8_t buffer[TOTAL_PACKET_SIZE];

    size_t written = serialize_packet(original_duties, buffer, sizeof(buffer));
    TEST_ASSERT_EQUAL_INT(TOTAL_PACKET_SIZE, written);

    PacketData parsed_packet;
    bool success = parse_byte_stream(buffer, written, &parsed_packet);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_TRUE(parsed_packet.valid);

    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        TEST_ASSERT_EQUAL_UINT16(original_duties[i], parsed_packet.duty_cycles[i]);
    }
}

void test_corrupted_crc_continues_scanning(void) {
    uint16_t duties1[NUM_PWM_VALS] = {10, 20, 30, 40, 50};
    uint16_t duties2[NUM_PWM_VALS] = {100, 200, 300, 400, 500};

    uint8_t combined[TOTAL_PACKET_SIZE * 2];

    /* First packet: valid, then corrupt its payload */
    serialize_packet(duties1, combined, TOTAL_PACKET_SIZE);
    combined[3] ^= 0xFF;  /* Corrupt payload byte → CRC will fail */

    /* Second packet: valid */
    serialize_packet(duties2, combined + TOTAL_PACKET_SIZE, TOTAL_PACKET_SIZE);

    PacketData parsed;
    bool success = parse_byte_stream(combined, sizeof(combined), &parsed);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_TRUE(parsed.valid);

    /* Should have found the SECOND packet */
    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        TEST_ASSERT_EQUAL_UINT16(duties2[i], parsed.duty_cycles[i]);
    }
}

void test_garbage_stream_rejected(void) {
    uint8_t garbage[30];
    memset(garbage, 0x77, sizeof(garbage));

    PacketData parsed;
    bool success = parse_byte_stream(garbage, sizeof(garbage), &parsed);
    TEST_ASSERT_FALSE(success);
    TEST_ASSERT_FALSE(parsed.valid);
}

void test_packet_after_garbage(void) {
    uint8_t buffer[40];
    memset(buffer, 0xFF, 15);  /* Garbage prefix */

    uint16_t duties[NUM_PWM_VALS] = {500, 500, 500, 500, 500};
    serialize_packet(duties, buffer + 15, TOTAL_PACKET_SIZE);

    memset(buffer + 15 + TOTAL_PACKET_SIZE, 0xEE, 10);  /* Garbage suffix */

    PacketData parsed;
    bool success = parse_byte_stream(buffer, sizeof(buffer), &parsed);
    TEST_ASSERT_TRUE(success);
    TEST_ASSERT_TRUE(parsed.valid);
    TEST_ASSERT_EQUAL_UINT16(500, parsed.duty_cycles[0]);
}

void test_null_buffer_safety(void) {
    PacketData parsed;
    TEST_ASSERT_FALSE(parse_byte_stream(NULL, 15, &parsed));
    TEST_ASSERT_FALSE(parse_byte_stream(NULL, 0, NULL));
}

void test_truncated_buffer_rejected(void) {
    uint16_t duties[NUM_PWM_VALS] = {100, 100, 100, 100, 100};
    uint8_t buffer[TOTAL_PACKET_SIZE];
    serialize_packet(duties, buffer, sizeof(buffer));

    PacketData parsed;
    /* Pass 14 bytes instead of 15 */
    bool success = parse_byte_stream(buffer, TOTAL_PACKET_SIZE - 1, &parsed);
    TEST_ASSERT_FALSE(success);
}

void test_ack_packet_builder(void) {
    uint8_t ack_buf[ACK_PACKET_SIZE];
    size_t len = build_ack_packet(ACK_STATUS_OK, ack_buf, sizeof(ack_buf));

    TEST_ASSERT_EQUAL_INT(ACK_PACKET_SIZE, len);
    TEST_ASSERT_EQUAL_HEX8(PACKET_HEADER_1, ack_buf[0]);
    TEST_ASSERT_EQUAL_HEX8(ACK_STATUS_OK, ack_buf[1]);

    uint8_t expected_crc = calculate_crc8(&ack_buf[1], 1);
    TEST_ASSERT_EQUAL_HEX8(expected_crc, ack_buf[2]);
}

void test_nak_packet_builder(void) {
    uint8_t nak_buf[ACK_PACKET_SIZE];
    size_t len = build_ack_packet(ACK_STATUS_NAK, nak_buf, sizeof(nak_buf));

    TEST_ASSERT_EQUAL_INT(ACK_PACKET_SIZE, len);
    TEST_ASSERT_EQUAL_HEX8(ACK_STATUS_NAK, nak_buf[1]);
}

void test_ack_buffer_too_small(void) {
    uint8_t tiny_buf[2];
    size_t len = build_ack_packet(ACK_STATUS_OK, tiny_buf, sizeof(tiny_buf));
    TEST_ASSERT_EQUAL_INT(0, len);
}

void test_serialize_clamps_duty(void) {
    uint16_t duties[NUM_PWM_VALS] = {0, 0, 0, 0, 2000}; /* 2000 > 1023 */
    uint8_t buffer[TOTAL_PACKET_SIZE];

    serialize_packet(duties, buffer, sizeof(buffer));

    /* Parse and verify clamping */
    PacketData parsed;
    parse_byte_stream(buffer, TOTAL_PACKET_SIZE, &parsed);
    TEST_ASSERT_EQUAL_UINT16(1023, parsed.duty_cycles[4]);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_ideal_packet_parsing);
    RUN_TEST(test_corrupted_crc_continues_scanning);
    RUN_TEST(test_garbage_stream_rejected);
    RUN_TEST(test_packet_after_garbage);
    RUN_TEST(test_null_buffer_safety);
    RUN_TEST(test_truncated_buffer_rejected);
    RUN_TEST(test_ack_packet_builder);
    RUN_TEST(test_nak_packet_builder);
    RUN_TEST(test_ack_buffer_too_small);
    RUN_TEST(test_serialize_clamps_duty);
    return UNITY_END();
}
