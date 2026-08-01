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
#include "bno085_fusion.h"
#include "smart_coil.h"
#include "qi_relay_guard.h"

/* ---- Configuration ------------------------------------------------ */

static const uint8_t BNO085_SDA_PIN    = 14;
static const uint8_t BNO085_SCL_PIN    = 15;
static const uint8_t MOSFET_PIN        = 12;
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
    STATE_IDLE,          /* Waiting for loop tick */
    STATE_READ_IMU,      /* Request orientation from BNO085 */
    STATE_COIL_UPDATE,   /* Compute and apply coil duty */
    STATE_HEALTH_CHECK   /* I2C watchdog check */
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

    /* ---- State machine -------------------------------------------- */
    switch (agent_state) {

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

    case STATE_COIL_UPDATE: {
        /* Ensure Qi relay is isolated before coil activation */
        if (!coil_state.qi_isolated) {
            qi_relay_isolate();
            coil_state.qi_isolated = true;
        }

        uint16_t duty = update_smart_coil(
            latest_orientation.roll,
            latest_orientation.pitch,
            COIL_GAIN,
            now,
            &coil_state
        );

        /* If coil returned to zero, restore Qi charging */
        if (duty == 0 && coil_state.qi_isolated && !coil_state.fuse_tripped) {
            qi_relay_restore();
            coil_state.qi_isolated = false;
        }

        agent_state = STATE_HEALTH_CHECK;
        break;
    }

    case STATE_HEALTH_CHECK:
        /* I2C bus watchdog — detect hung bus and recover */
        if (bno085_handle_i2c_timeout(last_imu_ok_ms, now, I2C_TIMEOUT_MS)) {
            Serial.println(F("[WARN] I2C bus reset triggered"));
            /* After bus reset, re-init sensor */
            bno085_init(BNO085_SDA_PIN, BNO085_SCL_PIN);
            last_imu_ok_ms = now;  /* Give it a fresh timeout window */
        }

        /* If software fuse tripped and tilt returned to zero, reset */
        if (coil_state.fuse_tripped) {
            float tilt = sqrtf(latest_orientation.roll * latest_orientation.roll +
                               latest_orientation.pitch * latest_orientation.pitch);
            if (tilt < DEADZONE_TILT_DEG) {
                reset_software_fuse(&coil_state);
                /* Restore Qi once fuse is cleared and coil is off */
                if (coil_state.qi_isolated) {
                    qi_relay_restore();
                    coil_state.qi_isolated = false;
                }
            }
        }

        agent_state = STATE_IDLE;
        break;
    }
}

#endif /* ARDUINO */
