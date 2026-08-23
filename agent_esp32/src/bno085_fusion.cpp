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

#ifdef UNIT_TEST
bool bno085_test_compute_euler(float qw, float qx, float qy, float qz, OrientationData *out_data) {
    if (out_data == nullptr) return false;
    out_data->q_w = qw;
    out_data->q_x = qx;
    out_data->q_y = qy;
    out_data->q_z = qz;
    out_data->valid = true;
    normalize_quaternion(out_data);
    if (!out_data->valid) return false;

    /* Compute Euler (same as compute_euler but duplicating here since compute_euler is static, 
     * or we can just call compute_euler if we pull it up... Wait, compute_euler is not compiled in UNIT_TEST 
     * because it's inside #if defined(ARDUINO)? Let's check. 
     * Actually, let's just do the math here to be safe and independent. */
    float qx2 = out_data->q_x;
    float qy2 = out_data->q_y;
    float qz2 = out_data->q_z;
    float qw2 = out_data->q_w;

    float sinr_cosp = 2.0f * (qw2 * qx2 + qy2 * qz2);
    float cosr_cosp = 1.0f - 2.0f * (qx2 * qx2 + qy2 * qy2);
    out_data->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / (float)M_PI);

    float sinp = 2.0f * (qw2 * qy2 - qz2 * qx2);
    if (fabsf(sinp) >= 1.0f) {
        out_data->pitch = copysignf(90.0f, sinp);
    } else {
        out_data->pitch = asinf(sinp) * (180.0f / (float)M_PI);
    }

    float siny_cosp = 2.0f * (qw2 * qz2 + qx2 * qy2);
    float cosy_cosp = 1.0f - 2.0f * (qy2 * qy2 + qz2 * qz2);
    out_data->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / (float)M_PI);

    return true;
}
#endif

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
#include <Adafruit_BNO08x.h>

static Adafruit_BNO08x bno08x;

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

    /* Initialize Adafruit_BNO08x over I2C at address 0x4A */
    if (!bno08x.begin_I2C(0x4A, &Wire, 0)) {
        return false;
    }
    
    /* Request Rotation Vector (quaternions) at 100 Hz (10,000 microseconds) */
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR, 10000)) {
        return false;
    }

    return true;
}

static void compute_euler(OrientationData *out_data) {
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
}

bool bno085_read_orientation(OrientationData *out_data) {
    if (out_data == nullptr) return false;

    sh2_SensorValue_t sensorValue;
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            out_data->q_x = sensorValue.un.rotationVector.i;
            out_data->q_y = sensorValue.un.rotationVector.j;
            out_data->q_z = sensorValue.un.rotationVector.k;
            out_data->q_w = sensorValue.un.rotationVector.real;
            
            out_data->valid = true;
            normalize_quaternion(out_data);
            if (!out_data->valid) return false;
            
            compute_euler(out_data);
            
            last_success_time = millis();
            return true;
        }
    }
    
    return false;
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
