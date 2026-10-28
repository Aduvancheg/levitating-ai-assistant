#ifndef PROTOCOL_PARSER_H
#define PROTOCOL_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ===================================================================
 * Binary Protocol — RPi 5 ↔ RP2040 Communication
 *
 * Packet structure (15 bytes):
 *   [Header 0xAA 0x55] [Payload: 5× uint16_t LE] [CRC8] [Footer 0x55 0xAA]
 *
 * ACK response (3 bytes):
 *   [0xAA] [STATUS_BYTE] [CRC8 of STATUS_BYTE]
 *
 *   STATUS_BYTE:
 *     0x06 (ACK)  — packet accepted and applied
 *     0x15 (NAK)  — packet received but CRC failed
 *
 * Command frame (7 bytes) — review F6:
 *   [Header 0xAA 0xC3] [CMD] [ARG] [CRC8(CMD,ARG)] [Footer 0x55 0xAA]
 *
 *   CMD:
 *     0x01 PREPARE_OTA — enter OTA_LOCKED (re-send ≤ every
 *                        FSM_OTA_KEEPALIVE_TIMEOUT_US as keep-alive)
 *     0x02 OTA_SUCCESS — leave OTA_LOCKED → ACTIVE
 *   ARG: reserved, must be 0x00.
 *
 *   Why 0xC3: duty values are ≤ 1023, so every payload high byte is
 *   ≤ 0x03 and the pair 0xAA 0xC3 can never occur inside a well-formed
 *   duty frame.  CRC + footer guard against accidental matches in noise.
 * =================================================================== */

#define PACKET_HEADER_1    0xAA
#define PACKET_HEADER_2    0x55
#define PACKET_FOOTER_1    0x55
#define PACKET_FOOTER_2    0xAA
#define NUM_PWM_VALS       5
#define PACKET_PAYLOAD_SIZE (NUM_PWM_VALS * 2)
#define TOTAL_PACKET_SIZE  (2 + PACKET_PAYLOAD_SIZE + 1 + 2)  /* 15 bytes */

#define CMD_HEADER_2       0xC3
#define CMD_PAYLOAD_SIZE   2
#define CMD_PACKET_SIZE    (2 + CMD_PAYLOAD_SIZE + 1 + 2)     /* 7 bytes */

/** Smallest frame of any type — minimum bytes needed to attempt a parse. */
#define MIN_FRAME_SIZE     CMD_PACKET_SIZE

#define CMD_PREPARE_OTA    0x01
#define CMD_OTA_SUCCESS    0x02

#define ACK_PACKET_SIZE    3
#define ACK_STATUS_OK      0x06
#define ACK_STATUS_NAK     0x15

typedef enum {
    FRAME_TYPE_DUTY    = 0,   /**< duty_cycles[] is valid  */
    FRAME_TYPE_COMMAND = 1    /**< command / command_arg are valid */
} FrameType;

typedef struct {
    uint16_t  duty_cycles[NUM_PWM_VALS];
    bool      valid;
    FrameType type;
    uint8_t   command;
    uint8_t   command_arg;
} PacketData;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate CRC-8 (polynomial 0x07, init 0x00).
 */
uint8_t calculate_crc8(const uint8_t *data, size_t length);

/**
 * @brief Parse a byte-stream buffer searching for a valid packet.
 *
 * Scans for the header signature, validates CRC and footer.
 * On CRC mismatch, continues scanning for the next header instead
 * of aborting — this avoids losing valid packets that follow a
 * corrupted one in a noisy stream.
 *
 * Recognises both duty frames (15 B) and command frames (7 B); the
 * earliest valid frame in the buffer wins.  out_packet->type tells
 * which one was found.
 *
 * @param buffer        Input byte stream.
 * @param length        Number of bytes available.
 * @param out_packet    Parsed result (valid flag set on success).
 * @param out_consumed  Optional pointer to receive count of bytes consumed up to end of valid packet.
 * @return true if a valid packet was found, false otherwise.
 */
bool parse_byte_stream(const uint8_t *buffer, size_t length, PacketData *out_packet, size_t *out_consumed = NULL);

/**
 * @brief Serialize 5 duty-cycle values into a binary packet.
 *
 * Duty values are clamped to 0..1023 during serialization.
 *
 * @return Number of bytes written (TOTAL_PACKET_SIZE on success, 0 on error).
 */
size_t serialize_packet(const uint16_t duties[NUM_PWM_VALS], uint8_t *out_buffer, size_t max_len);

/**
 * @brief Serialize a command frame (see CMD_* codes).
 *
 * @return Number of bytes written (CMD_PACKET_SIZE on success, 0 on error).
 */
size_t serialize_command(uint8_t command, uint8_t arg, uint8_t *out_buffer, size_t max_len);

/**
 * @brief Build an ACK/NAK response packet.
 *
 * @param status      ACK_STATUS_OK or ACK_STATUS_NAK.
 * @param out_buffer  At least ACK_PACKET_SIZE bytes.
 * @param max_len     Size of output buffer.
 * @return Bytes written (ACK_PACKET_SIZE on success, 0 on error).
 */
size_t build_ack_packet(uint8_t status, uint8_t *out_buffer, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_PARSER_H */
