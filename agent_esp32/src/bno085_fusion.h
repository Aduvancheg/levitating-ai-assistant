#ifndef BNO085_FUSION_H
#define BNO085_FUSION_H

#include <stdint.h>
#include <stdbool.h>

/* ===================================================================
 * BNO085 Sensor Fusion — ESP32-CAM Sphere Agent
 *
 * Reads rotation vectors (quaternions) from the BNO085 IMU over I2C,
 * converts to Euler angles (Roll, Pitch, Yaw).
 *
 * Key features:
 *   - Quaternion normalization after Q14 decode (prevents drift).
 *   - I2C bus watchdog with hardware bus reset (9× SCL toggle).
 *   - EMI-resilient: designed for operation near high-power PWM coils.
 *
 * Backlog LS-3: ≥ 100 Hz sampling without blocking the ESP32 core.
 * =================================================================== */

typedef struct {
    float roll;     /* degrees, X-axis rotation */
    float pitch;    /* degrees, Y-axis rotation */
    float yaw;      /* degrees, Z-axis rotation */
    float q_w;      /* quaternion real part      */
    float q_x;      /* quaternion i component    */
    float q_y;      /* quaternion j component    */
    float q_z;      /* quaternion k component    */
    bool  valid;    /* false if data is stale or parsing failed */
} OrientationData;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise BNO085 I2C interface.
 *
 * @param sda_pin  I2C SDA GPIO pin.
 * @param scl_pin  I2C SCL GPIO pin.
 * @return true on successful initialization.
 */
bool bno085_init(uint8_t sda_pin, uint8_t scl_pin);

/**
 * @brief Read the latest orientation sample from BNO085.
 *
 * Non-blocking: returns false immediately if no data is available
 * or the I2C bus is stalled.
 */
bool bno085_read_orientation(OrientationData *out_data);

/**
 * @brief Decode a raw BNO085 rotation vector report into quaternion + Euler.
 *
 * Packet format: [Report ID (2B)] [Q_i Q14] [Q_j Q14] [Q_k Q14] [Q_real Q14]
 * Performs normalization after Q14 scaling.
 *
 * Exposed for unit testing with mock I2C data.
 */
bool bno085_parse_raw_packet(const uint8_t *raw_bytes, uint16_t len, OrientationData *out_data);

/**
 * @brief I2C bus watchdog — detects timeout and triggers bus reset.
 *
 * @param last_read_timestamp_ms  millis() of the last successful read.
 * @param current_time_ms         Current millis().
 * @param timeout_ms              Max allowed silence before reset.
 * @return true if a bus reset was triggered.
 */
bool bno085_handle_i2c_timeout(uint32_t last_read_timestamp_ms,
                               uint32_t current_time_ms,
                               uint32_t timeout_ms);

/** @brief Query whether the last call to handle_i2c_timeout triggered a reset. */
bool bno085_was_bus_reset(void);

/**
 * @brief Perform a hardware I2C bus reset (9× SCL toggles + SDA release).
 *
 * This recovers the bus from a stuck state caused by EMI glitches.
 * After calling this, bno085_init() should be called to re-establish
 * communication with the sensor.
 */
void bno085_i2c_bus_reset(uint8_t sda_pin, uint8_t scl_pin);

#ifdef __cplusplus
}
#endif

#endif /* BNO085_FUSION_H */
