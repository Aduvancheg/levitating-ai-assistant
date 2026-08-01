/**
 * @file bno085_fusion.cpp
 * @brief BNO085 Sensor Fusion — quaternion decode, Euler conversion, I2C watchdog.
 */

#include "bno085_fusion.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---- Internal state ----------------------------------------------- */
static bool     bus_reset_triggered = false;
static uint32_t last_success_time   = 0;
static uint8_t  stored_sda_pin      = 14;
static uint8_t  stored_scl_pin      = 15;

/* ---- Quaternion normalization ------------------------------------- */

/**
 * Normalize a quaternion in-place to unit length.
 * Without this, Q14 rounding errors accumulate and Euler conversion
 * produces drifting / nonsensical angles.
 */
static void normalize_quaternion(OrientationData *d) {
    float mag = sqrtf(d->q_w * d->q_w + d->q_x * d->q_x +
                      d->q_y * d->q_y + d->q_z * d->q_z);
    if (mag < 1e-8f) {
        /* Degenerate quaternion — mark invalid rather than dividing by ~0 */
        d->valid = false;
        return;
    }
    float inv_mag = 1.0f / mag;
    d->q_w *= inv_mag;
    d->q_x *= inv_mag;
    d->q_y *= inv_mag;
    d->q_z *= inv_mag;
}

/* ---- Raw packet parsing + Euler conversion ------------------------ */

bool bno085_parse_raw_packet(const uint8_t *raw_bytes, uint16_t len, OrientationData *out_data) {
    if (raw_bytes == nullptr || out_data == nullptr || len < 10) {
        if (out_data) out_data->valid = false;
        return false;
    }

    /* Packet layout: [Report ID (2B)] [Q_i LE16] [Q_j LE16] [Q_k LE16] [Q_real LE16] */
    int16_t q_i_raw = (int16_t)((uint16_t)raw_bytes[2] | ((uint16_t)raw_bytes[3] << 8));
    int16_t q_j_raw = (int16_t)((uint16_t)raw_bytes[4] | ((uint16_t)raw_bytes[5] << 8));
    int16_t q_k_raw = (int16_t)((uint16_t)raw_bytes[6] | ((uint16_t)raw_bytes[7] << 8));
    int16_t q_r_raw = (int16_t)((uint16_t)raw_bytes[8] | ((uint16_t)raw_bytes[9] << 8));

    /* Q14 fixed-point to float: scale = 1 / 2^14 = 1 / 16384.0 */
    static const float Q14_SCALE = 1.0f / 16384.0f;
    out_data->q_x = q_i_raw * Q14_SCALE;
    out_data->q_y = q_j_raw * Q14_SCALE;
    out_data->q_z = q_k_raw * Q14_SCALE;
    out_data->q_w = q_r_raw * Q14_SCALE;

    /* Normalize to prevent drift from Q14 rounding errors */
    out_data->valid = true;  /* set before normalize — it may clear it */
    normalize_quaternion(out_data);
    if (!out_data->valid) {
        return false;
    }

    /* Convert normalized quaternion to Euler angles (ZYX convention) */
    float qx = out_data->q_x;
    float qy = out_data->q_y;
    float qz = out_data->q_z;
    float qw = out_data->q_w;

    /* Roll (X-axis rotation) */
    float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    out_data->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    /* Pitch (Y-axis rotation) — clamped at gimbal lock */
    float sinp = 2.0f * (qw * qy - qz * qx);
    if (fabsf(sinp) >= 1.0f) {
        out_data->pitch = copysignf(90.0f, sinp);
    } else {
        out_data->pitch = asinf(sinp) * (180.0f / (float)M_PI);
    }

    /* Yaw (Z-axis rotation) */
    float siny_cosp = 2.0f * (qw * qz + qx * qy);
    float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
    out_data->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

    return true;
}

/* ---- I2C bus watchdog --------------------------------------------- */

bool bno085_handle_i2c_timeout(uint32_t last_read_timestamp_ms,
                               uint32_t current_time_ms,
                               uint32_t timeout_ms) {
    if (current_time_ms - last_read_timestamp_ms > timeout_ms) {
        bus_reset_triggered = true;
        /* Trigger hardware bus reset */
        bno085_i2c_bus_reset(stored_sda_pin, stored_scl_pin);
        return true;
    }
    bus_reset_triggered = false;
    return false;
}

bool bno085_was_bus_reset(void) {
    return bus_reset_triggered;
}

/* ---- Platform-specific I2C implementation ------------------------- */

#if defined(ARDUINO)
#include <Wire.h>

void bno085_i2c_bus_reset(uint8_t sda_pin, uint8_t scl_pin) {
    /*
     * Hardware I2C bus recovery procedure:
     * When a slave holds SDA low (stuck), toggling SCL 9 times forces
     * the slave to release. Then generate a STOP condition.
     */
    Wire.end();

    pinMode(sda_pin, INPUT_PULLUP);
    pinMode(scl_pin, OUTPUT);

    for (uint8_t i = 0; i < 9; i++) {
        digitalWrite(scl_pin, LOW);
        delayMicroseconds(5);
        digitalWrite(scl_pin, HIGH);
        delayMicroseconds(5);
    }

    /* Generate STOP: SDA LOW → SCL HIGH → SDA HIGH */
    pinMode(sda_pin, OUTPUT);
    digitalWrite(sda_pin, LOW);
    delayMicroseconds(5);
    digitalWrite(scl_pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sda_pin, HIGH);
    delayMicroseconds(5);

    /* Re-initialise Wire with the stored pins */
    Wire.begin(sda_pin, scl_pin, 400000);
}

bool bno085_init(uint8_t sda_pin, uint8_t scl_pin) {
    stored_sda_pin = sda_pin;
    stored_scl_pin = scl_pin;
    bus_reset_triggered = false;

    Wire.begin(sda_pin, scl_pin, 400000);  /* 400 kHz fast-mode I2C */

    /* NOTE: A production implementation should use the Adafruit_BNO08x
     * library to configure SHTP reports (rotation vector at 100 Hz).
     * The raw Wire.requestFrom() approach below is a simplified
     * placeholder — it works for basic I2C communication validation
     * but does not implement the full SHTP handshake.
     *
     * TODO: Replace with Adafruit_BNO08x::begin() + enableReport()
     * once the lib_deps are installed and validated on hardware.       */
    return true;
}

bool bno085_read_orientation(OrientationData *out_data) {
    if (out_data == nullptr) return false;

    uint8_t raw[10];
    Wire.requestFrom((uint8_t)0x4A, (uint8_t)10);

    uint16_t idx = 0;
    uint32_t start = millis();
    while (Wire.available() && idx < 10) {
        raw[idx++] = Wire.read();
        /* Guard against infinite loop from EMI-induced clock stretching */
        if (millis() - start > 10) {
            break;
        }
    }

    if (idx < 10) {
        /* Incomplete read — trigger watchdog check with actual timestamps */
        out_data->valid = false;
        return false;
    }

    last_success_time = millis();
    return bno085_parse_raw_packet(raw, 10, out_data);
}

#else
/* Native / unit-test stubs */
void bno085_i2c_bus_reset(uint8_t sda_pin, uint8_t scl_pin) {
    (void)sda_pin; (void)scl_pin;
    /* In test builds, just record that a reset was requested */
}

bool bno085_init(uint8_t sda_pin, uint8_t scl_pin) {
    stored_sda_pin = sda_pin;
    stored_scl_pin = scl_pin;
    bus_reset_triggered = false;
    return true;
}

bool bno085_read_orientation(OrientationData *out_data) {
    if (out_data) out_data->valid = false;
    return false;
}
#endif
