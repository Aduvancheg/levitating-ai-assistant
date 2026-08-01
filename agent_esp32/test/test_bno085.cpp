/**
 * @file test_bno085.cpp
 * @brief Unit tests for BNO085 Sensor Fusion — quaternion parsing,
 *        normalization, I2C timeout / bus recovery.
 */

#include "../src/bno085_fusion.h"
#include <assert.h>
#include <stdio.h>
#include <math.h>

/* Tolerance for float comparisons */
#define FLOAT_EPS 0.01f

void test_identity_quaternion_parsing() {
    /*
     * Identity quaternion: q_w = 1.0, q_x = q_y = q_z = 0.0
     * In Q14 encoding: q_real = 16384 (0x4000), others = 0
     */
    uint8_t raw_identity[10] = {
        0x01, 0x00,   /* Report ID */
        0x00, 0x00,   /* Q_i = 0 */
        0x00, 0x00,   /* Q_j = 0 */
        0x00, 0x00,   /* Q_k = 0 */
        0x00, 0x40    /* Q_real = 16384 (0x4000 LE) */
    };

    OrientationData orientation;
    bool res = bno085_parse_raw_packet(raw_identity, 10, &orientation);
    assert(res == true);
    assert(orientation.valid == true);

    /* Quaternion components */
    assert(fabsf(orientation.q_w - 1.0f) < FLOAT_EPS);
    assert(fabsf(orientation.q_x - 0.0f) < FLOAT_EPS);
    assert(fabsf(orientation.q_y - 0.0f) < FLOAT_EPS);
    assert(fabsf(orientation.q_z - 0.0f) < FLOAT_EPS);

    /* Euler angles — identity should give 0 degrees */
    assert(fabsf(orientation.roll)  < FLOAT_EPS);
    assert(fabsf(orientation.pitch) < FLOAT_EPS);
    assert(fabsf(orientation.yaw)   < FLOAT_EPS);

    printf("[PASS] test_identity_quaternion_parsing\n");
}

void test_quaternion_normalization() {
    /*
     * Construct a quaternion that is NOT unit-length after Q14 decode.
     * The parser must normalize it before Euler conversion.
     *
     * q_i = 8192 (0x2000) → 0.5
     * q_j = 8192 → 0.5
     * q_k = 8192 → 0.5
     * q_real = 8192 → 0.5
     * Magnitude before normalization: sqrt(4 * 0.25) = 1.0 — already unit.
     *
     * Use a non-unit case: all = 4096 → 0.25 each
     * Magnitude = sqrt(4 * 0.0625) = 0.5 → must be normalized to 1.0
     */
    uint8_t raw[10] = {
        0x01, 0x00,         /* Report ID */
        0x00, 0x10,         /* Q_i = 4096 (0x1000 LE) */
        0x00, 0x10,         /* Q_j = 4096 */
        0x00, 0x10,         /* Q_k = 4096 */
        0x00, 0x10          /* Q_real = 4096 */
    };

    OrientationData orientation;
    bool res = bno085_parse_raw_packet(raw, 10, &orientation);
    assert(res == true);
    assert(orientation.valid == true);

    /* After normalization, magnitude should be 1.0 */
    float mag = sqrtf(orientation.q_w * orientation.q_w +
                      orientation.q_x * orientation.q_x +
                      orientation.q_y * orientation.q_y +
                      orientation.q_z * orientation.q_z);
    assert(fabsf(mag - 1.0f) < FLOAT_EPS);

    printf("[PASS] test_quaternion_normalization\n");
}

void test_degenerate_quaternion_rejected() {
    /* All zeros — magnitude = 0, cannot normalize → invalid */
    uint8_t raw_zero[10] = {
        0x01, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00
    };

    OrientationData orientation;
    bool res = bno085_parse_raw_packet(raw_zero, 10, &orientation);
    assert(res == false);
    assert(orientation.valid == false);

    printf("[PASS] test_degenerate_quaternion_rejected\n");
}

void test_short_packet_rejected() {
    uint8_t short_buf[5] = {0x01, 0x00, 0x00, 0x00, 0x00};

    OrientationData orientation;
    bool res = bno085_parse_raw_packet(short_buf, 5, &orientation);
    assert(res == false);
    assert(orientation.valid == false);

    printf("[PASS] test_short_packet_rejected\n");
}

void test_null_args_safety() {
    OrientationData orientation;
    assert(bno085_parse_raw_packet(NULL, 10, &orientation) == false);
    assert(bno085_parse_raw_packet(NULL, 0, NULL) == false);

    printf("[PASS] test_null_args_safety\n");
}

void test_i2c_timeout_no_reset() {
    uint32_t last_read = 1000;
    uint32_t current   = 1030;   /* 30 ms elapsed (< 50 ms timeout) */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    assert(reset == false);
    assert(bno085_was_bus_reset() == false);

    printf("[PASS] test_i2c_timeout_no_reset\n");
}

void test_i2c_timeout_triggers_reset() {
    uint32_t last_read = 1000;
    uint32_t current   = 1100;   /* 100 ms elapsed (> 50 ms timeout) */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    assert(reset == true);
    assert(bno085_was_bus_reset() == true);

    printf("[PASS] test_i2c_timeout_triggers_reset\n");
}

void test_i2c_timeout_exactly_at_boundary() {
    uint32_t last_read = 1000;
    uint32_t current   = 1050;   /* Exactly 50 ms — NOT timed out */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    assert(reset == false);

    printf("[PASS] test_i2c_timeout_exactly_at_boundary\n");
}

int main() {
    printf("--- Running BNO085 Sensor Fusion Unit Tests ---\n");
    test_identity_quaternion_parsing();
    test_quaternion_normalization();
    test_degenerate_quaternion_rejected();
    test_short_packet_rejected();
    test_null_args_safety();
    test_i2c_timeout_no_reset();
    test_i2c_timeout_triggers_reset();
    test_i2c_timeout_exactly_at_boundary();
    printf("All BNO085 tests passed successfully.\n");
    return 0;
}
