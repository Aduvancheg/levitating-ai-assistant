/**
 * @file test_bno085.cpp
 * @brief Unit tests for BNO085 Sensor Fusion — quaternion parsing,
 *        normalization, I2C timeout / bus recovery.
 */

#include "../../src/bno085_fusion.h"
#include <unity.h>
#include <stdio.h>
#include <math.h>

/* Tolerance for float comparisons */
#define FLOAT_EPS 0.01f

void setUp(void) {
    // Empty
}

void tearDown(void) {
    // Empty
}

void test_identity_quaternion_parsing(void) {
    OrientationData orientation;
    bool res = bno085_test_compute_euler(1.0f, 0.0f, 0.0f, 0.0f, &orientation);
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_TRUE(orientation.valid);

    /* Quaternion components */
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 1.0f, orientation.q_w);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.q_x);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.q_y);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.q_z);

    /* Euler angles — identity should give 0 degrees */
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.roll);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.pitch);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 0.0f, orientation.yaw);
}

void test_quaternion_normalization(void) {
    OrientationData orientation;
    /* Pass an unnormalized quaternion */
    bool res = bno085_test_compute_euler(10.0f, 10.0f, 10.0f, 10.0f, &orientation);
    TEST_ASSERT_TRUE(res);
    TEST_ASSERT_TRUE(orientation.valid);

    /* After normalization, magnitude should be 1.0 */
    float mag = sqrtf(orientation.q_w * orientation.q_w +
                      orientation.q_x * orientation.q_x +
                      orientation.q_y * orientation.q_y +
                      orientation.q_z * orientation.q_z);
    TEST_ASSERT_FLOAT_WITHIN(FLOAT_EPS, 1.0f, mag);
}

void test_degenerate_quaternion_rejected(void) {
    /* All zeros — magnitude = 0, cannot normalize → invalid */
    OrientationData orientation;
    bool res = bno085_test_compute_euler(0.0f, 0.0f, 0.0f, 0.0f, &orientation);
    TEST_ASSERT_FALSE(res);
    TEST_ASSERT_FALSE(orientation.valid);
}

void test_null_args_safety(void) {
    TEST_ASSERT_FALSE(bno085_test_compute_euler(1.0f, 0.0f, 0.0f, 0.0f, NULL));
}

void test_i2c_timeout_no_reset(void) {
    uint32_t last_read = 1000;
    uint32_t current   = 1030;   /* 30 ms elapsed (< 50 ms timeout) */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    TEST_ASSERT_FALSE(reset);
    TEST_ASSERT_FALSE(bno085_was_bus_reset());
}

void test_i2c_timeout_triggers_reset(void) {
    uint32_t last_read = 1000;
    uint32_t current   = 1100;   /* 100 ms elapsed (> 50 ms timeout) */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    TEST_ASSERT_TRUE(reset);
    TEST_ASSERT_TRUE(bno085_was_bus_reset());
}

void test_i2c_timeout_exactly_at_boundary(void) {
    uint32_t last_read = 1000;
    uint32_t current   = 1050;   /* Exactly 50 ms — NOT timed out */

    bool reset = bno085_handle_i2c_timeout(last_read, current, 50);
    TEST_ASSERT_FALSE(reset);
}

void test_telemetry_json_formatting(void) {
    OrientationData orientation;
    TEST_ASSERT_TRUE(bno085_test_compute_euler(1.0f, 0.0f, 0.0f, 0.0f, &orientation));

    char buf[160];
    bool ok = bno085_format_telemetry_json(&orientation, 87.5f, 320u, true, -52, buf, sizeof(buf));
    TEST_ASSERT_TRUE(ok);
    TEST_ASSERT_EQUAL_STRING(
        "{\"pitch\":0.00,\"roll\":0.00,\"yaw\":0.00,\"battery\":87.5,\"coil_pwm\":320,\"relay\":\"Isolated\",\"rssi\":-52}",
        buf
    );
}

void test_telemetry_json_rejects_invalid_or_small_buffer(void) {
    OrientationData orientation;
    TEST_ASSERT_TRUE(bno085_test_compute_euler(1.0f, 0.0f, 0.0f, 0.0f, &orientation));

    char tiny[16];
    TEST_ASSERT_FALSE(bno085_format_telemetry_json(&orientation, 90.0f, 0u, false, -60, tiny, sizeof(tiny)));
    TEST_ASSERT_FALSE(bno085_format_telemetry_json(NULL, 90.0f, 0u, false, -60, tiny, sizeof(tiny)));

    orientation.valid = false;
    char buf[160];
    TEST_ASSERT_FALSE(bno085_format_telemetry_json(&orientation, 90.0f, 0u, false, -60, buf, sizeof(buf)));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_identity_quaternion_parsing);
    RUN_TEST(test_quaternion_normalization);
    RUN_TEST(test_degenerate_quaternion_rejected);
    RUN_TEST(test_null_args_safety);
    RUN_TEST(test_i2c_timeout_no_reset);
    RUN_TEST(test_i2c_timeout_triggers_reset);
    RUN_TEST(test_i2c_timeout_exactly_at_boundary);
    RUN_TEST(test_telemetry_json_formatting);
    RUN_TEST(test_telemetry_json_rejects_invalid_or_small_buffer);
    return UNITY_END();
}
