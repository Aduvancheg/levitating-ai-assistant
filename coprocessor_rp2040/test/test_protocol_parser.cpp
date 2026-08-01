/**
 * @file test_protocol_parser.cpp
 * @brief Unit tests for binary protocol parser — CRC, corruption, ACK.
 */

#include "../src/protocol_parser.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

void test_ideal_packet_parsing() {
    uint16_t original_duties[NUM_PWM_VALS] = {100, 250, 512, 800, 1023};
    uint8_t buffer[TOTAL_PACKET_SIZE];

    size_t written = serialize_packet(original_duties, buffer, sizeof(buffer));
    assert(written == TOTAL_PACKET_SIZE);

    PacketData parsed_packet;
    bool success = parse_byte_stream(buffer, written, &parsed_packet);
    assert(success == true);
    assert(parsed_packet.valid == true);

    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        assert(parsed_packet.duty_cycles[i] == original_duties[i]);
    }
    printf("[PASS] test_ideal_packet_parsing\n");
}

void test_corrupted_crc_continues_scanning() {
    /*
     * Place a corrupted packet followed by a valid one.
     * The parser should skip the corrupt frame and find the valid one.
     * (This was a bug: the original parser returned false on first CRC mismatch.)
     */
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
    assert(success == true);
    assert(parsed.valid == true);

    /* Should have found the SECOND packet */
    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        assert(parsed.duty_cycles[i] == duties2[i]);
    }
    printf("[PASS] test_corrupted_crc_continues_scanning\n");
}

void test_garbage_stream_rejected() {
    uint8_t garbage[30];
    memset(garbage, 0xEE, sizeof(garbage));

    PacketData parsed;
    bool success = parse_byte_stream(garbage, sizeof(garbage), &parsed);
    assert(success == false);
    assert(parsed.valid == false);

    printf("[PASS] test_garbage_stream_rejected\n");
}

void test_packet_after_garbage() {
    uint16_t duties[NUM_PWM_VALS] = {50, 100, 150, 200, 250};
    uint8_t combined[40];
    memset(combined, 0xFF, 10);  /* 10 bytes of garbage prefix */
    serialize_packet(duties, combined + 10, sizeof(combined) - 10);

    PacketData parsed;
    bool success = parse_byte_stream(combined, 10 + TOTAL_PACKET_SIZE, &parsed);
    assert(success == true);
    assert(parsed.valid == true);
    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        assert(parsed.duty_cycles[i] == duties[i]);
    }

    printf("[PASS] test_packet_after_garbage\n");
}

void test_null_buffer_safety() {
    PacketData parsed;
    bool success = parse_byte_stream(NULL, 0, &parsed);
    assert(success == false);

    success = parse_byte_stream(NULL, 100, NULL);
    assert(success == false);

    printf("[PASS] test_null_buffer_safety\n");
}

void test_truncated_buffer_rejected() {
    uint16_t duties[NUM_PWM_VALS] = {100, 200, 300, 400, 500};
    uint8_t buffer[TOTAL_PACKET_SIZE];
    serialize_packet(duties, buffer, sizeof(buffer));

    /* Pass only 10 bytes of a 15-byte packet */
    PacketData parsed;
    bool success = parse_byte_stream(buffer, 10, &parsed);
    assert(success == false);

    printf("[PASS] test_truncated_buffer_rejected\n");
}

void test_ack_packet_builder() {
    uint8_t ack_buf[ACK_PACKET_SIZE];

    size_t len = build_ack_packet(ACK_STATUS_OK, ack_buf, sizeof(ack_buf));
    assert(len == ACK_PACKET_SIZE);
    assert(ack_buf[0] == PACKET_HEADER_1);
    assert(ack_buf[1] == ACK_STATUS_OK);

    /* CRC of status byte */
    uint8_t expected_crc = calculate_crc8(&ack_buf[1], 1);
    assert(ack_buf[2] == expected_crc);

    printf("[PASS] test_ack_packet_builder\n");
}

void test_nak_packet_builder() {
    uint8_t ack_buf[ACK_PACKET_SIZE];

    size_t len = build_ack_packet(ACK_STATUS_NAK, ack_buf, sizeof(ack_buf));
    assert(len == ACK_PACKET_SIZE);
    assert(ack_buf[1] == ACK_STATUS_NAK);

    printf("[PASS] test_nak_packet_builder\n");
}

void test_ack_buffer_too_small() {
    uint8_t tiny_buf[1];
    size_t len = build_ack_packet(ACK_STATUS_OK, tiny_buf, sizeof(tiny_buf));
    assert(len == 0);

    printf("[PASS] test_ack_buffer_too_small\n");
}

void test_serialize_clamps_duty() {
    uint16_t duties[NUM_PWM_VALS] = {0, 0, 0, 0, 2000}; /* 2000 > 1023 */
    uint8_t buffer[TOTAL_PACKET_SIZE];

    serialize_packet(duties, buffer, sizeof(buffer));

    /* Parse and verify clamping */
    PacketData parsed;
    parse_byte_stream(buffer, TOTAL_PACKET_SIZE, &parsed);
    assert(parsed.duty_cycles[4] == 1023);

    printf("[PASS] test_serialize_clamps_duty\n");
}

int main() {
    printf("--- Running Protocol Parser Unit Tests ---\n");
    test_ideal_packet_parsing();
    test_corrupted_crc_continues_scanning();
    test_garbage_stream_rejected();
    test_packet_after_garbage();
    test_null_buffer_safety();
    test_truncated_buffer_rejected();
    test_ack_packet_builder();
    test_nak_packet_builder();
    test_ack_buffer_too_small();
    test_serialize_clamps_duty();
    printf("All Protocol Parser tests passed successfully.\n");
    return 0;
}
