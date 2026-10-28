/**
 * @file main.cpp
 * @brief ESP32-CAM Sphere Agent — Main firmware entry point.
 *
 * Responsibilities:
 *   1. Read orientation from BNO085 IMU at ≥ 100 Hz (I2C).
 *   2. Manage Qi Relay isolation before/after coil activation.
 *   3. Compute and apply Smart Coil current injection.
 *   4. Monitor I2C bus health with watchdog + auto-recovery.
 *   5. (Future) Transmit orientation data to RPi 5 via Wi-Fi.
 *
 * Architecture: non-blocking state machine — zero use of delay().
 */

#if defined(ARDUINO)
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "bno085_fusion.h"
#include "smart_coil.h"
#include "qi_relay_guard.h"
#include "wifi_ble_onboarding.h"

/* ---- Configuration ------------------------------------------------ */

static const uint8_t BNO085_SDA_PIN    = 14;
static const uint8_t BNO085_SCL_PIN    = 15;
/**
 * Smart-coil MOSFET gate pin (review F3):
 *   On ESP32-CAM, GPIO12 is the MTDI strapping pin (controls flash VDD_SDIO
 *   voltage at reset: HIGH = 1.8V → bootloop on 3.3V flash modules).
 *   A 10 kΩ hardware pull-down from MOSFET Gate to GND is MANDATORY if GPIO12
 *   is used, or override via -DMOSFET_PIN_OVERRIDE=<pin> (e.g., GPIO4 when
 *   onboard flash LED is unused).
 */
#ifndef MOSFET_PIN_OVERRIDE
#define MOSFET_PIN_OVERRIDE 12
#endif
static const uint8_t MOSFET_PIN        = MOSFET_PIN_OVERRIDE;
static const uint8_t QI_RELAY_PIN      = 13;

/** Loop tick interval: 10 ms = 100 Hz (BNO085 max report rate). */
static const uint32_t LOOP_INTERVAL_MS = 10;

/** I2C watchdog timeout: if no successful read for 50 ms, reset bus. */
static const uint32_t I2C_TIMEOUT_MS   = 50;

/** Coil gain factor: tilt degrees → duty mapping.
 *  Tune this experimentally during levitation calibration.            */
static const float COIL_GAIN = 50.0f;

/* ---- State machine ------------------------------------------------ */

typedef enum {
    STATE_IDLE,               /* Waiting for loop tick */
    STATE_READ_IMU,           /* Request orientation from BNO085 */
    STATE_RELAY_WAIT_ISOLATE, /* Wait for Qi relay to finish isolating */
    STATE_COIL_UPDATE,        /* Compute and apply coil duty */
    STATE_RELAY_WAIT_RESTORE, /* Wait for Qi relay to finish restoring */
    STATE_HEALTH_CHECK,       /* I2C watchdog check */
    STATE_OTA_LOCKED          /* Over-The-Air firmware update in progress */
} AgentState;

/* ---- Runtime state ------------------------------------------------ */

static SmartCoilState coil_state;
static AgentState     agent_state    = STATE_IDLE;
static uint32_t       last_loop_ms   = 0;
static uint32_t       last_imu_ok_ms = 0;
static OrientationData latest_orientation;

/* ---- Hardware Watchdog -------------------------------------------- */
#if defined(ESP32)
#include <esp_task_wdt.h>

static void wdt_init(void) {
    /* 3-second hardware WDT.  If the main loop hangs for > 3s
     * (e.g. I2C stuck + software watchdog also stuck), the ESP32
     * reboots.  This is the last safety net.                          */
    esp_task_wdt_init(3, true);
    esp_task_wdt_add(NULL);
}

static void wdt_feed(void) {
    esp_task_wdt_reset();
}
#else
static void wdt_init(void) {}
static void wdt_feed(void) {}
#endif

/* ---- Arduino entry points ---------------------------------------- */

void setup() {
    Serial.begin(115200);

    /* Initialise Wi-Fi, BLE GATT Server (if needed), and UDP Handover listener */
    wifi_ble_onboarding_init();

    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.onStart([]() {
            Serial.println("[OTA] Update started. Locking coil.");
            agent_state = STATE_OTA_LOCKED;
            // Force smart coil off during OTA
            update_smart_coil(0, 0, COIL_GAIN, millis(), &coil_state);
            qi_relay_restore(); // Safely connect Qi for passive levitation power
        });
        ArduinoOTA.onEnd([]() {
            Serial.println("\n[OTA] Update complete.");
        });
        ArduinoOTA.begin();
    }

    /* Initialise subsystems */
    bno085_init(BNO085_SDA_PIN, BNO085_SCL_PIN);
    smart_coil_init(MOSFET_PIN);
    qi_relay_init(QI_RELAY_PIN);

    /* Initialise coil state */
    memset(&coil_state, 0, sizeof(coil_state));
    coil_state.fuse_tripped = false;
    coil_state.qi_isolated  = false;

    memset(&latest_orientation, 0, sizeof(latest_orientation));

    last_loop_ms   = millis();
    last_imu_ok_ms = millis();

    wdt_init();
    Serial.println(F("[OK] ESP32 Sphere Agent ready"));
}

void loop() {
    uint32_t now = millis();

    /* Feed hardware watchdog every iteration */
    wdt_feed();

    if (WiFi.status() == WL_CONNECTED) {
        ArduinoOTA.handle();
    }

    /* ---- State machine -------------------------------------------- */
    switch (agent_state) {

    case STATE_OTA_LOCKED:
        /* Do nothing, wait for OTA to complete and reboot */
        break;

    case STATE_IDLE:
        if (now - last_loop_ms >= LOOP_INTERVAL_MS) {
            last_loop_ms = now;
            agent_state  = STATE_READ_IMU;
        }
        break;

    case STATE_READ_IMU:
        if (bno085_read_orientation(&latest_orientation)) {
            last_imu_ok_ms = now;
            agent_state    = STATE_COIL_UPDATE;
        } else {
            /* Read failed — go to health check */
            agent_state = STATE_HEALTH_CHECK;
        }
        break;

    case STATE_RELAY_WAIT_ISOLATE:
        /* Poll relay settle — non-blocking wait for 5 ms settling */
        if (qi_relay_is_ready(now)) {
            coil_state.qi_isolated = true;
            agent_state = STATE_COIL_UPDATE;
        }
        /* else: stay in this state until relay settles */
        break;

    case STATE_COIL_UPDATE: {
        /* Review F1: compute_coil_duty() is evaluated FIRST inside
         * smart_coil_step_with_relay(). The Qi relay is isolated ONLY when
         * duty > 0, and restored only after QI_RESTORE_HYSTERESIS_MS of
         * zero duty — eliminating 50 Hz relay clicking at rest. */
        CoilRelayStepResult step = smart_coil_step_with_relay(
            latest_orientation.roll,
            latest_orientation.pitch,
            COIL_GAIN,
            now,
            &coil_state
        );

        if (step == COIL_STEP_WAIT_ISOLATE) {
            agent_state = STATE_RELAY_WAIT_ISOLATE;
        } else if (step == COIL_STEP_WAIT_RESTORE) {
            agent_state = STATE_RELAY_WAIT_RESTORE;
        } else {
            agent_state = STATE_HEALTH_CHECK;
        }
        break;
    }

    case STATE_RELAY_WAIT_RESTORE:
        /* Poll relay settle for restore transition */
        if (qi_relay_is_ready(now)) {
            coil_state.qi_isolated = false;
            agent_state = STATE_HEALTH_CHECK;
        }
        /* else: stay in this state until relay settles */
        break;

    case STATE_HEALTH_CHECK:
        /* I2C bus watchdog — detect hung bus and recover */
        if (bno085_handle_i2c_timeout(last_imu_ok_ms, now, I2C_TIMEOUT_MS)) {
            Serial.println(F("[WARN] I2C bus reset triggered"));
            /* After bus reset, re-init sensor */
            bno085_init(BNO085_SDA_PIN, BNO085_SCL_PIN);
            last_imu_ok_ms = now;  /* Give it a fresh timeout window */
        }

        /* If software fuse tripped and tilt returned to zero, reset.
         * Qi relay restoration is handled cleanly with hysteresis and
         * settle-polling by smart_coil_step_with_relay() -> STATE_RELAY_WAIT_RESTORE. */
        if (coil_state.fuse_tripped) {
            float tilt = sqrtf(latest_orientation.roll * latest_orientation.roll +
                               latest_orientation.pitch * latest_orientation.pitch);
            if (tilt < DEADZONE_TILT_DEG) {
                reset_software_fuse(&coil_state);
            }
        }

        agent_state = STATE_IDLE;
        break;
    }
}

#endif /* ARDUINO */
