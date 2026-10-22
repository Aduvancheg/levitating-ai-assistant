/**
 * @file protocol_parser.cpp
 * @brief Binary protocol serialization, parsing, CRC-8, and ACK/NAK.
 */

#include "protocol_parser.h"
#include <string.h>

/* ---- CRC-8 (polynomial 0x07, init 0x00) --------------------------- */

uint8_t calculate_crc8(const uint8_t *data, size_t length) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ---- Serialize 5 duty-cycle values into a binary frame ------------ */

size_t serialize_packet(const uint16_t duties[NUM_PWM_VALS], uint8_t *out_buffer, size_t max_len) {
    if (max_len < TOTAL_PACKET_SIZE || out_buffer == nullptr || duties == nullptr) {
        return 0;
    }

    out_buffer[0] = PACKET_HEADER_1;
    out_buffer[1] = PACKET_HEADER_2;

    uint8_t payload[PACKET_PAYLOAD_SIZE];
    for (size_t i = 0; i < NUM_PWM_VALS; i++) {
        uint16_t duty = duties[i] > 1023 ? 1023 : duties[i];
        payload[i * 2]     = (uint8_t)(duty & 0xFF);
        payload[i * 2 + 1] = (uint8_t)((duty >> 8) & 0xFF);
    }

    memcpy(&out_buffer[2], payload, PACKET_PAYLOAD_SIZE);

    uint8_t crc = calculate_crc8(payload, PACKET_PAYLOAD_SIZE);
    out_buffer[2 + PACKET_PAYLOAD_SIZE] = crc;

    out_buffer[2 + PACKET_PAYLOAD_SIZE + 1] = PACKET_FOOTER_1;
    out_buffer[2 + PACKET_PAYLOAD_SIZE + 2] = PACKET_FOOTER_2;

    return TOTAL_PACKET_SIZE;
}

/* ---- Serialize a command frame (review F6) ------------------------ */

size_t serialize_command(uint8_t command, uint8_t arg, uint8_t *out_buffer, size_t max_len) {
    if (out_buffer == nullptr || max_len < CMD_PACKET_SIZE) {
        return 0;
    }

    const uint8_t payload[CMD_PAYLOAD_SIZE] = {command, arg};
    out_buffer[0] = PACKET_HEADER_1;
    out_buffer[1] = CMD_HEADER_2;
    out_buffer[2] = command;
    out_buffer[3] = arg;
    out_buffer[4] = calculate_crc8(payload, CMD_PAYLOAD_SIZE);
    out_buffer[5] = PACKET_FOOTER_1;
    out_buffer[6] = PACKET_FOOTER_2;
    return CMD_PACKET_SIZE;
}

/* ---- Frame matchers ------------------------------------------------ */

static bool try_parse_duty_at(const uint8_t *buffer, size_t length, size_t i,
                              PacketData *out_packet) {
    if (length - i < TOTAL_PACKET_SIZE || buffer[i + 1] != PACKET_HEADER_2) {
        return false;
    }

    const uint8_t *payload = &buffer[i + 2];
    uint8_t expected_crc   = buffer[i + 2 + PACKET_PAYLOAD_SIZE];
    uint8_t footer1        = buffer[i + 2 + PACKET_PAYLOAD_SIZE + 1];
    uint8_t footer2        = buffer[i + 2 + PACKET_PAYLOAD_SIZE + 2];

    /* Footer mismatch — not a real frame */
    if (footer1 != PACKET_FOOTER_1 || footer2 != PACKET_FOOTER_2) {
        return false;
    }
    /* CRC mismatch — corrupted frame */
    if (calculate_crc8(payload, PACKET_PAYLOAD_SIZE) != expected_crc) {
        return false;
    }

    for (size_t k = 0; k < NUM_PWM_VALS; k++) {
        uint16_t duty = (uint16_t)payload[k * 2] | ((uint16_t)payload[k * 2 + 1] << 8);
        out_packet->duty_cycles[k] = duty > 1023 ? 1023 : duty;
    }
    out_packet->type        = FRAME_TYPE_DUTY;
    out_packet->command     = 0;
    out_packet->command_arg = 0;
    out_packet->valid       = true;
    return true;
}

static bool try_parse_command_at(const uint8_t *buffer, size_t length, size_t i,
                                 PacketData *out_packet) {
    if (length - i < CMD_PACKET_SIZE || buffer[i + 1] != CMD_HEADER_2) {
        return false;
    }

    const uint8_t *payload = &buffer[i + 2];
    if (buffer[i + 5] != PACKET_FOOTER_1 || buffer[i + 6] != PACKET_FOOTER_2) {
        return false;
    }
    if (calculate_crc8(payload, CMD_PAYLOAD_SIZE) != buffer[i + 4]) {
        return false;
    }

    memset(out_packet->duty_cycles, 0, sizeof(out_packet->duty_cycles));
    out_packet->type        = FRAME_TYPE_COMMAND;
    out_packet->command     = payload[0];
    out_packet->command_arg = payload[1];
    out_packet->valid       = true;
    return true;
}

/* ---- Parse byte-stream, scanning past CRC errors ------------------ */

bool parse_byte_stream(const uint8_t *buffer, size_t length, PacketData *out_packet, size_t *out_consumed) {
    if (out_consumed) *out_consumed = 0;
    if (buffer == nullptr || out_packet == nullptr || length < MIN_FRAME_SIZE) {
        if (out_packet) out_packet->valid = false;
        return false;
    }

    /* Scan for every possible header position.
     * On CRC mismatch we continue searching rather than aborting,
     * so a valid frame after a corrupted one is still found.          */
    for (size_t i = 0; i + MIN_FRAME_SIZE <= length; i++) {
        if (buffer[i] != PACKET_HEADER_1) {
            continue;
        }

        if (try_parse_duty_at(buffer, length, i, out_packet)) {
            if (out_consumed) *out_consumed = i + TOTAL_PACKET_SIZE;
            return true;
        }
        if (try_parse_command_at(buffer, length, i, out_packet)) {
            if (out_consumed) *out_consumed = i + CMD_PACKET_SIZE;
            return true;
        }
    }

    out_packet->valid = false;
    return false;
}

/* ---- Build ACK / NAK response ------------------------------------- */

size_t build_ack_packet(uint8_t status, uint8_t *out_buffer, size_t max_len) {
    if (out_buffer == nullptr || max_len < ACK_PACKET_SIZE) {
        return 0;
    }

    out_buffer[0] = PACKET_HEADER_1;        /* 0xAA marker */
    out_buffer[1] = status;                 /* ACK (0x06) or NAK (0x15) */
    out_buffer[2] = calculate_crc8(&status, 1);

    return ACK_PACKET_SIZE;
}
