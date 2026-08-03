/**
 * @file test_e2e_integration.cpp
 * @brief End-to-End integration tests — full data pipeline validation.
 *
 * Covers 8 business scenarios from business_e2e_scenarios.md:
 *
 *   Scenario 1 (Deep Work / Smooth Ramp-Down):
 *     RPi 5 sends a stream of decreasing Z-coordinates encoded as duty-cycle
 *     packets.  Validates that the protocol_parser → pwm_driver pipeline
 *     produces a monotonically decreasing, jitter-free PWM envelope on all
 *     5 channels.
 *
 *   Scenario 2 (Crafting Nod / ACTION_NOD spike):
 *     RPi 5 sends a short high-duty "spike" packet followed by an immediate
 *     return-to-baseline packet.  Validates that the PWM driver correctly
 *     applies the spike impulse to all 5 channels and then restores the
 *     previous duty.
 *
 *   Scenario 3 (Storytelling / high-freq PWM oscillation):
 *     RP2040 correctly handles rapid duty modulation (tremor) and slow
 *     bidirectional ramp (calm sway) without dropping the sphere.
 *
 *   EMI Garbage Rejection:
 *     A corrupted UART stream (random bytes with broken CRC) arrives due to
 *     electromagnetic interference.  Validates that the parser rejects the
 *     frame and the PWM driver retains its last-known-good duty values.
 *
 *   Scenario 7 (Wi-Fi Heartbeat Loss / Safe Landing):
 *     FSM watchdog detects missing heartbeat and transitions to SAFE_LANDING,
 *     ramping all channels to 0.  Recovery on reconnect.
 *
 *   Scenario 9 (OTA In-Air / Firmware Update):
 *     FSM locks duty at near-field safe value during OTA, rejects all dynamic
 *     packets, and resumes after OTA_SUCCESS.
 *
 *   Scenario 10 (Smooth Morning Boot / Ramp-Up):
 *     All channels ramp from 0 to working duty without inrush spikes,
 *     monotonically increasing, capped at MAX_DUTY_LIMIT.
 *
 *   Scenario 11 (Sensor Anomaly / False Reading Rejection):
 *     FSM rejects packets with physically impossible duty deltas while
 *     accepting normal PID corrections.
 *
 * All tests run in [env:native] without real RP2040 hardware.
 *
 * Test framework: Unity (PlatformIO native runner).
 *
 * References:
 *   - requirements/business_e2e_scenarios.md  §1–§3, §7, §9–§11
 *   - requirements/quality.md                 §1, §4 (TDD, Boundary Testing)
 *   - requirements/architecture.md            §2, §5, §6
 */

#include "../../src/protocol_parser.h"
#include "../../src/pwm_driver.h"
#include "../../src/coprocessor_fsm.h"
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* ===================================================================
 * Helpers
 * =================================================================== */

/** @brief Default GPIO pins for native test init. */
static const uint8_t TEST_PINS[PWM_CHANNELS] = {0, 1, 2, 3, 4};
static const uint32_t TEST_PWM_FREQ = 25000;

/**
 * @brief Reinitialise PWM subsystem to a clean state for each test.
 *
 * Calls kill_all_pwm() then re-inits channels to ensure zero cross-test
 * contamination in static driver state.
 */
static void reset_pwm(void) {
    kill_all_pwm();
    init_pwm_channels(TEST_PINS, TEST_PWM_FREQ);
}

/**
 * @brief Simulate the main-loop pipeline for a single packet.
 *
 * Mirrors the data flow from main.cpp:
 *   serialize → parse_byte_stream → apply duty cycles directly via set_pwm_duty.
 *
 * "Direct apply" mode (no ramp) is used when we want to verify the exact
 * duty values the parser extracted, without ramp smoothing.
 *
 * @return true if the packet was parsed and applied successfully.
 */
static bool apply_packet_direct(const uint16_t duties[NUM_PWM_VALS]) {
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    size_t written = serialize_packet(duties, wire_buf, sizeof(wire_buf));
    if (written != TOTAL_PACKET_SIZE) return false;

    PacketData parsed;
    size_t consumed = 0;
    if (!parse_byte_stream(wire_buf, written, &parsed, &consumed)) return false;
    if (!parsed.valid) return false;

    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, parsed.duty_cycles[ch]);
    }
    return true;
}

/**
 * @brief Simulate the main-loop pipeline via ramp (Smooth Ramp tick).
 *
 * Mirrors ramp_pwm_duty() path from main.cpp: sets target duties, then
 * ticks the ramp N times (simulating N milliseconds of ramp ticks).
 *
 * @param target_duties  Desired duty values from the parsed packet.
 * @param ramp_ticks     Number of 1-ms ramp ticks to simulate.
 */
static void apply_packet_ramped(const uint16_t target_duties[NUM_PWM_VALS],
                                uint16_t ramp_ticks) {
    for (uint16_t t = 0; t < ramp_ticks; t++) {
        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            ramp_pwm_duty(ch, target_duties[ch], RAMP_STEP_DEFAULT);
        }
    }
}


/* ===================================================================
 * SCENARIO 1: Deep Work — Smooth Ramp-Down
 *
 * business_e2e_scenarios.md §1:
 *   "RPi 5 генерирует массив убывающих по оси Z координат.
 *    RP2040 плавно (без джиттера) пересчитывает скважность ШИМ
 *    для алгоритма Smooth Ramp-Down."
 *
 * We simulate 20 packets with linearly decreasing duty (400 → 0).
 * After each packet + sufficient ramp ticks, PWM must be:
 *   a) Monotonically non-increasing (no upward jitter).
 *   b) Never exceed the safety ceiling MAX_DUTY_LIMIT.
 *   c) Reach target 0 at the end.
 * =================================================================== */

static void test_e2e_deep_work_smooth_ramp_down(void) {
    reset_pwm();

    const int NUM_STEPS = 20;
    const uint16_t START_DUTY = 400;    /* Below MAX_DUTY_LIMIT (460) */

    /* Pre-set all channels to START_DUTY to simulate steady-state hover */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, START_DUTY);
    }

    uint16_t prev_duty_ch0 = get_pwm_duty(0);

    for (int step = 1; step <= NUM_STEPS; step++) {
        /* RPi 5 computes next target: linear descent from START_DUTY to 0 */
        uint16_t target = (uint16_t)(START_DUTY - (START_DUTY * step) / NUM_STEPS);

        /* Simulate RPi 5 sending a duty packet over UART */
        uint16_t packet_duties[NUM_PWM_VALS];
        for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
            packet_duties[ch] = target;
        }

        /* Serialize → parse → extract duty (full pipeline validation) */
        uint8_t wire_buf[TOTAL_PACKET_SIZE];
        size_t written = serialize_packet(packet_duties, wire_buf, sizeof(wire_buf));
        assert(written == TOTAL_PACKET_SIZE);

        PacketData parsed;
        size_t consumed = 0;
        bool ok = parse_byte_stream(wire_buf, written, &parsed, &consumed);
        assert(ok);
        assert(parsed.valid);

        /* Verify parsed values match what RPi 5 intended */
        for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
            assert(parsed.duty_cycles[ch] == target);
        }

        /* Apply via ramp — enough ticks to fully converge.
         * Max ramp distance per step ≈ START_DUTY/NUM_STEPS = 20.
         * With RAMP_STEP_DEFAULT=10, need ceil(20/10) = 2 ticks minimum.
         * Use 50 ticks for safety margin. */
        apply_packet_ramped(parsed.duty_cycles, 50);

        uint16_t current = get_pwm_duty(0);

        /* Monotonic decrease: current ≤ previous */
        assert(current <= prev_duty_ch0);

        /* Safety ceiling: never exceed MAX_DUTY_LIMIT */
        assert(current <= MAX_DUTY_LIMIT);

        prev_duty_ch0 = current;
    }

    /* Final: all channels at 0 (sphere landed) */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == 0);
    }

    printf("[PASS] test_e2e_deep_work_smooth_ramp_down\n");
}


/* ===================================================================
 * SCENARIO 1b: Deep Work — Fine-grain monotonicity during ramp
 *
 * Verifies that intermediate ramp ticks between two packets also
 * produce strictly non-increasing duty (no micro-jitter mid-ramp).
 * =================================================================== */

static void test_e2e_deep_work_ramp_monotonicity_per_tick(void) {
    reset_pwm();

    /* Set initial duty to 300 on channel 0 */
    set_pwm_duty(0, 300);

    /* Target: ramp down to 100 */
    uint16_t prev = get_pwm_duty(0);
    for (int tick = 0; tick < 100; tick++) {
        ramp_pwm_duty(0, 100, RAMP_STEP_DEFAULT);
        uint16_t current = get_pwm_duty(0);

        /* Strictly non-increasing on each tick */
        assert(current <= prev);
        prev = current;
    }

    /* Should have reached target */
    assert(get_pwm_duty(0) == 100);

    printf("[PASS] test_e2e_deep_work_ramp_monotonicity_per_tick\n");
}


/* ===================================================================
 * SCENARIO 2: Crafting Nod — ACTION_NOD Spike Impulse
 *
 * business_e2e_scenarios.md §2:
 *   "RPi 5 генерирует команду ACTION_NOD → мгновенный импульс
 *    D-коэффициента в ПИД-регуляторе для микро-прыжка."
 *
 * Simulation:
 *   1. Sphere hovering at baseline duty (200).
 *   2. RPi 5 sends spike packet (duty=400 on all 5 channels).
 *   3. Verify all 5 channels received the spike.
 *   4. RPi 5 immediately sends return-to-baseline packet (200).
 *   5. Verify all channels return to 200.
 *
 * This is a "direct apply" test (not ramped), because the ACTION_NOD
 * spike is designed to be instantaneous, bypassing the ramp.
 * =================================================================== */

static void test_e2e_crafting_nod_spike_impulse(void) {
    reset_pwm();

    const uint16_t BASELINE = 200;
    const uint16_t SPIKE    = 400;

    /* 1. Establish baseline hover on all 5 channels */
    uint16_t baseline_duties[NUM_PWM_VALS];
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        baseline_duties[ch] = BASELINE;
    }
    assert(apply_packet_direct(baseline_duties));

    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == BASELINE);
    }

    /* 2. RPi 5 sends ACTION_NOD spike packet */
    uint16_t spike_duties[NUM_PWM_VALS];
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        spike_duties[ch] = SPIKE;
    }
    assert(apply_packet_direct(spike_duties));

    /* 3. Verify spike applied on ALL 5 channels */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == SPIKE);
    }

    /* 4. RPi 5 immediately returns to baseline */
    assert(apply_packet_direct(baseline_duties));

    /* 5. Verify all channels restored */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == BASELINE);
    }

    printf("[PASS] test_e2e_crafting_nod_spike_impulse\n");
}


/* ===================================================================
 * SCENARIO 2b: Crafting Nod — Spike clamped by safety ceiling
 *
 * If the PID overshoot causes a spike beyond MAX_DUTY_LIMIT (460),
 * the PWM driver must clamp it — never exceed the safety ceiling.
 * =================================================================== */

static void test_e2e_crafting_nod_spike_clamped(void) {
    reset_pwm();

    const uint16_t DANGEROUS_SPIKE = 900;  /* Far above MAX_DUTY_LIMIT (460) */

    uint16_t spike_duties[NUM_PWM_VALS];
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        spike_duties[ch] = DANGEROUS_SPIKE;
    }
    assert(apply_packet_direct(spike_duties));

    /* All channels must be clamped to MAX_DUTY_LIMIT, not 900 */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == MAX_DUTY_LIMIT);
    }

    printf("[PASS] test_e2e_crafting_nod_spike_clamped\n");
}


/* ===================================================================
 * SCENARIO 3: EMI Garbage Rejection
 *
 * business_e2e_scenarios.md §6 + architecture.md §6:
 *   "Высокоскоростной SPI/UART со строго валидируемым бинарным
 *    пакетом (маркеры кадра + 5× uint16 duty cycle + CRC8)."
 *
 * quality.md §4:
 *   "Код должен корректно отбрасывать такие пакеты."
 *
 * Simulation:
 *   1. Sphere hovering at known duty (250).
 *   2. EMI noise corrupts the UART stream: random garbage bytes arrive.
 *   3. Parser rejects the corrupted frame (CRC8 mismatch).
 *   4. PWM duties remain unchanged — sphere stays stable.
 * =================================================================== */

static void test_e2e_emi_garbage_preserves_pwm(void) {
    reset_pwm();

    const uint16_t STABLE_DUTY = 250;

    /* 1. Establish stable hover */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, STABLE_DUTY);
    }

    /* Snapshot duties before attack */
    uint16_t before[PWM_CHANNELS];
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        before[ch] = get_pwm_duty(ch);
        assert(before[ch] == STABLE_DUTY);
    }

    /* 2. Simulate pure EMI garbage: 30 random bytes, no valid header */
    uint8_t garbage[30];
    memset(garbage, 0xDE, sizeof(garbage));
    garbage[0] = 0xAB; garbage[1] = 0xCD;  /* Not a valid header (0xAA 0x55) */
    garbage[5] = 0x00; garbage[10] = 0xFF;  /* Random noise */

    PacketData parsed;
    size_t consumed = 0;
    bool found = parse_byte_stream(garbage, sizeof(garbage), &parsed, &consumed);
    assert(found == false);
    assert(parsed.valid == false);

    /* 3. PWM duties unchanged — sphere did NOT fall */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == before[ch]);
    }

    printf("[PASS] test_e2e_emi_garbage_preserves_pwm\n");
}


/* ===================================================================
 * SCENARIO 3b: EMI — Corrupted CRC in otherwise valid-looking frame
 *
 * The most dangerous case: a packet arrives with correct header and
 * footer bytes but with a single bit flipped in the payload (typical
 * EMI-induced error).  CRC8 must catch it.
 * =================================================================== */

static void test_e2e_emi_bitflip_crc_rejection(void) {
    reset_pwm();

    const uint16_t STABLE_DUTY = 300;

    /* Establish stable hover */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, STABLE_DUTY);
    }

    /* Build a legitimate packet, then corrupt ONE bit in the payload */
    uint16_t attack_duties[NUM_PWM_VALS] = {0, 0, 0, 0, 0};  /* Would crash sphere */
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(attack_duties, wire_buf, sizeof(wire_buf));

    /* Flip bit 2 of payload byte index 4 (duty channel 2 LSB) */
    wire_buf[6] ^= 0x04;

    /* Parser MUST reject this frame */
    PacketData parsed;
    bool found = parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed);
    assert(found == false);  /* CRC mismatch → rejected */

    /* PWM stays at STABLE_DUTY — sphere is safe */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == STABLE_DUTY);
    }

    printf("[PASS] test_e2e_emi_bitflip_crc_rejection\n");
}


/* ===================================================================
 * SCENARIO 3c: EMI — Valid packet recovers after garbage burst
 *
 * architecture.md §6:
 *   "На CRC mismatch, парсер продолжает сканировать (continue
 *    scanning) для следующего заголовка."
 *
 * Simulation: garbage + corrupted packet + valid packet in one stream.
 * The parser must skip the noise and find the good packet at the end.
 * =================================================================== */

static void test_e2e_emi_recovery_after_garbage(void) {
    reset_pwm();

    const uint16_t INITIAL_DUTY   = 200;
    const uint16_t RECOVERED_DUTY = 350;

    /* Establish baseline */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, INITIAL_DUTY);
    }

    /* Build a stream: [garbage 8B] [corrupted packet 15B] [valid packet 15B] */
    uint8_t stream[8 + TOTAL_PACKET_SIZE + TOTAL_PACKET_SIZE];
    memset(stream, 0xBB, 8);  /* garbage prefix */

    /* Corrupted packet */
    uint16_t bad_duties[NUM_PWM_VALS] = {999, 999, 999, 999, 999};
    serialize_packet(bad_duties, stream + 8, TOTAL_PACKET_SIZE);
    stream[8 + 5] ^= 0xFF;  /* corrupt payload → CRC fail */

    /* Valid packet */
    uint16_t good_duties[NUM_PWM_VALS];
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        good_duties[ch] = RECOVERED_DUTY;
    }
    serialize_packet(good_duties, stream + 8 + TOTAL_PACKET_SIZE, TOTAL_PACKET_SIZE);

    /* Parse the whole stream */
    PacketData parsed;
    size_t consumed = 0;
    bool found = parse_byte_stream(stream, sizeof(stream), &parsed, &consumed);
    assert(found == true);
    assert(parsed.valid == true);

    /* Should find the VALID packet with RECOVERED_DUTY */
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(parsed.duty_cycles[ch] == RECOVERED_DUTY);
    }

    /* Apply recovered duties */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, parsed.duty_cycles[ch]);
    }

    /* Verify PWM updated to recovered values */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == RECOVERED_DUTY);
    }

    printf("[PASS] test_e2e_emi_recovery_after_garbage\n");
}


/* ===================================================================
 * SCENARIO 3: Storytelling — High-Frequency PWM Oscillation (Tremor)
 *
 * business_e2e_scenarios.md §3:
 *   "RP2040 корректно отрабатывает микро-смещения фаз ШИМ,
 *    не роняя сферу при искусственном программном дрожании."
 *
 * Simulation:
 *   Sphere at baseline duty (250).  RPi 5 rapidly alternates
 *   duty ±30 for 50 cycles (simulating FEAR tremor at ~50 Hz).
 *   Verify:
 *     a) Duty never drops below safe minimum (BASELINE - AMPLITUDE).
 *     b) Duty never exceeds MAX_DUTY_LIMIT.
 *     c) All 5 channels track the oscillation correctly.
 * =================================================================== */

static void test_e2e_storytelling_tremor_no_drop(void) {
    reset_pwm();

    const uint16_t BASELINE  = 250;
    const uint16_t AMPLITUDE = 30;
    const int CYCLES = 50;

    /* Establish baseline hover */
    uint16_t base_duties[NUM_PWM_VALS];
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        base_duties[ch] = BASELINE;
    }
    assert(apply_packet_direct(base_duties));

    for (int cycle = 0; cycle < CYCLES; cycle++) {
        /* Odd ticks: spike up */
        uint16_t high_duties[NUM_PWM_VALS];
        for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
            high_duties[ch] = BASELINE + AMPLITUDE;
        }
        assert(apply_packet_direct(high_duties));

        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            assert(get_pwm_duty(ch) == BASELINE + AMPLITUDE);
            assert(get_pwm_duty(ch) <= MAX_DUTY_LIMIT);
        }

        /* Even ticks: dip down */
        uint16_t low_duties[NUM_PWM_VALS];
        for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
            low_duties[ch] = BASELINE - AMPLITUDE;
        }
        assert(apply_packet_direct(low_duties));

        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            assert(get_pwm_duty(ch) == BASELINE - AMPLITUDE);
            assert(get_pwm_duty(ch) >= (BASELINE - AMPLITUDE)); /* Never below floor */
        }
    }

    /* Return to baseline */
    assert(apply_packet_direct(base_duties));
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == BASELINE);
    }

    printf("[PASS] test_e2e_storytelling_tremor_no_drop\n");
}


/* ===================================================================
 * SCENARIO 3b: Storytelling — Calm Sway (bidirectional ramp)
 *
 * business_e2e_scenarios.md §3:
 *   "плавно покачивается влево-вправо на спокойных этапах."
 *
 * Simulation:
 *   Ramp up from 150 to 300, then ramp back down to 150.
 *   Verify bidirectional ramp monotonicity.
 * =================================================================== */

static void test_e2e_storytelling_calm_sway(void) {
    reset_pwm();

    const uint16_t LOW  = 150;
    const uint16_t HIGH = 300;

    /* Start at LOW */
    set_pwm_duty(0, LOW);

    /* Phase 1: ramp UP — monotonic increase */
    uint16_t prev = get_pwm_duty(0);
    for (int tick = 0; tick < 100; tick++) {
        ramp_pwm_duty(0, HIGH, RAMP_STEP_DEFAULT);
        uint16_t current = get_pwm_duty(0);
        assert(current >= prev);  /* Monotonic increase */
        prev = current;
    }
    assert(get_pwm_duty(0) == HIGH);

    /* Phase 2: ramp DOWN — monotonic decrease */
    prev = get_pwm_duty(0);
    for (int tick = 0; tick < 100; tick++) {
        ramp_pwm_duty(0, LOW, RAMP_STEP_DEFAULT);
        uint16_t current = get_pwm_duty(0);
        assert(current <= prev);  /* Monotonic decrease */
        prev = current;
    }
    assert(get_pwm_duty(0) == LOW);

    printf("[PASS] test_e2e_storytelling_calm_sway\n");
}


/* ===================================================================
 * SCENARIO 7: Wi-Fi Heartbeat Loss — Safe Landing
 *
 * business_e2e_scenarios.md §7:
 *   "В ESP32-CAM срабатывает программный Watchdog отсутствия
 *    Heartbeat-пакетов от RPi 5 (таймаут > 200 мс)."
 *   "RPi 5 передает команду сопроцессору RP2040 на инициализацию
 *    алгоритма Smooth Ramp-Down."
 *
 * Simulation:
 *   FSM in ACTIVE state at t=0.  No packets arrive.  At t=250ms+1,
 *   fsm_tick() triggers SAFE_LANDING.  Target duties go to 0.
 * =================================================================== */

static void test_e2e_heartbeat_loss_triggers_safe_landing(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);
    assert(fsm_get_state() == FSM_STATE_ACTIVE);

    /* Feed one valid packet to establish hover at duty=300 */
    uint16_t hover_duties[NUM_PWM_VALS] = {300, 300, 300, 300, 300};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(hover_duties, wire_buf, sizeof(wire_buf));

    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Apply to PWM */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, targets[ch]);
    }

    /* Advance time just past heartbeat timeout (250 ms = 250000 µs) */
    t += FSM_HEARTBEAT_TIMEOUT_US + 1;
    fsm_tick(t);

    /* FSM must transition to SAFE_LANDING */
    assert(fsm_get_state() == FSM_STATE_SAFE_LANDING);

    /* Target duties must be 0 */
    targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 0);
    }

    printf("[PASS] test_e2e_heartbeat_loss_triggers_safe_landing\n");
}


/* ===================================================================
 * SCENARIO 7b: Heartbeat Loss — Ramp-Down completes to 0
 *
 * After SAFE_LANDING, simulating the main loop's ramp ticks should
 * smoothly bring all channels from hover duty to 0.
 * =================================================================== */

static void test_e2e_heartbeat_loss_ramp_down_completes(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Establish hover */
    uint16_t hover_duties[NUM_PWM_VALS] = {400, 400, 400, 400, 400};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(hover_duties, wire_buf, sizeof(wire_buf));

    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Apply hover to PWM */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, 400);
    }

    /* Trigger heartbeat loss */
    t += FSM_HEARTBEAT_TIMEOUT_US + 1;
    fsm_tick(t);
    assert(fsm_get_state() == FSM_STATE_SAFE_LANDING);

    /* Simulate main loop ramp ticks toward target=0 */
    const uint16_t *targets = fsm_get_target_duties();
    for (int tick = 0; tick < 200; tick++) {
        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            ramp_pwm_duty(ch, targets[ch], RAMP_STEP_DEFAULT);
        }
    }

    /* All channels must reach 0 — sphere safely landed */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == 0);
    }

    printf("[PASS] test_e2e_heartbeat_loss_ramp_down_completes\n");
}


/* ===================================================================
 * SCENARIO 7c: Heartbeat Recovery — reconnect restores ACTIVE
 *
 * business_e2e_scenarios.md §7:
 *   "до восстановления сети"
 *
 * FSM is in SAFE_LANDING.  A valid packet arrives → FSM recovers
 * to ACTIVE.
 * =================================================================== */

static void test_e2e_heartbeat_recovery_after_reconnect(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Feed initial packet to establish baseline */
    uint16_t initial[NUM_PWM_VALS] = {200, 200, 200, 200, 200};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(initial, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Trigger heartbeat loss */
    t += FSM_HEARTBEAT_TIMEOUT_US + 1;
    fsm_tick(t);
    assert(fsm_get_state() == FSM_STATE_SAFE_LANDING);

    /* Wi-Fi reconnects: a new valid packet arrives */
    t += 50000;  /* 50 ms later */
    uint16_t recovery[NUM_PWM_VALS] = {250, 250, 250, 250, 250};
    serialize_packet(recovery, wire_buf, sizeof(wire_buf));
    consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* FSM must return to ACTIVE */
    assert(fsm_get_state() == FSM_STATE_ACTIVE);

    /* Target duties must reflect the recovery packet */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 250);
    }

    printf("[PASS] test_e2e_heartbeat_recovery_after_reconnect\n");
}


/* ===================================================================
 * SCENARIO 9: OTA In-Air — PREPARE_OTA locks duty
 *
 * business_e2e_scenarios.md §9:
 *   "RPi 5 шлет команду PREPARE_OTA."
 *   "База (RP2040) плавно опускает Агента в зону жесткого ближнего
 *    поля (2-3 см)"
 *
 * Simulation:
 *   FSM in ACTIVE.  PREPARE_OTA command → OTA_LOCKED.
 *   All target duties locked at FSM_OTA_SAFE_DUTY (50).
 * =================================================================== */

static void test_e2e_ota_locks_duty_at_safe_level(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Establish normal hover */
    uint16_t hover[NUM_PWM_VALS] = {300, 300, 300, 300, 300};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(hover, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* RPi 5 sends PREPARE_OTA */
    t += 100000;  /* 100 ms later */
    fsm_command_prepare_ota(t);

    /* State must be OTA_LOCKED */
    assert(fsm_get_state() == FSM_STATE_OTA_LOCKED);

    /* All targets must be at near-field safe duty */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == FSM_OTA_SAFE_DUTY);
    }

    printf("[PASS] test_e2e_ota_locks_duty_at_safe_level\n");
}


/* ===================================================================
 * SCENARIO 9b: OTA — Dynamic packets rejected during lock
 *
 * business_e2e_scenarios.md §9:
 *   "ESP32-CAM отключает силовые ключи, подтверждает статус
 *    READY_FOR_OTA и начинает прошивку."
 *
 * While OTA_LOCKED, any incoming duty packets must be rejected.
 * =================================================================== */

static void test_e2e_ota_rejects_dynamic_packets(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Feed initial packet */
    uint16_t initial[NUM_PWM_VALS] = {200, 200, 200, 200, 200};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(initial, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Enter OTA mode */
    t += 50000;
    fsm_command_prepare_ota(t);
    assert(fsm_get_state() == FSM_STATE_OTA_LOCKED);

    /* Try to feed a new duty packet — must be REJECTED */
    t += 10000;
    uint16_t new_duties[NUM_PWM_VALS] = {400, 400, 400, 400, 400};
    serialize_packet(new_duties, wire_buf, sizeof(wire_buf));
    consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t) == false);  /* REJECTED */

    /* Targets remain at OTA safe duty */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == FSM_OTA_SAFE_DUTY);
    }

    /* State still OTA_LOCKED */
    assert(fsm_get_state() == FSM_STATE_OTA_LOCKED);

    printf("[PASS] test_e2e_ota_rejects_dynamic_packets\n");
}


/* ===================================================================
 * SCENARIO 9c: OTA — OTA_SUCCESS resumes ACTIVE mode
 *
 * business_e2e_scenarios.md §9:
 *   "RPi 5 удерживает Сферу, пока ESP32 не перезагрузится и не
 *    пришлет пакет OTA_SUCCESS."
 *
 * After OTA_SUCCESS, FSM returns to ACTIVE and accepts packets.
 * =================================================================== */

static void test_e2e_ota_success_resumes_active(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Feed initial packet to build baseline */
    uint16_t initial[NUM_PWM_VALS] = {200, 200, 200, 200, 200};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(initial, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Enter OTA lock */
    t += 50000;
    fsm_command_prepare_ota(t);
    assert(fsm_get_state() == FSM_STATE_OTA_LOCKED);

    /* ESP32 reboots and reports OTA_SUCCESS */
    t += 15000000;  /* 15 seconds later (OTA flash time) */
    fsm_command_ota_success(t);
    assert(fsm_get_state() == FSM_STATE_ACTIVE);

    /* Now packets should be accepted again.
     * Note: prev_duties is still at the initial 200 from before OTA.
     * A small step to 250 (delta=50) should be within anomaly threshold. */
    t += 1000;
    uint16_t post_ota[NUM_PWM_VALS] = {250, 250, 250, 250, 250};
    serialize_packet(post_ota, wire_buf, sizeof(wire_buf));
    consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t) == true);  /* ACCEPTED */

    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 250);
    }

    printf("[PASS] test_e2e_ota_success_resumes_active\n");
}


/* ===================================================================
 * SCENARIO 10: Smooth Morning Boot — Ramp-Up from 0
 *
 * business_e2e_scenarios.md §10:
 *   "RPi 5 подает команду на RP2040: алгоритм Smooth Ramp-Up
 *    увеличивает ШИМ на 5 катушках от 0 до рабочего минимума
 *    в течение 3 секунд."
 *
 * Simulation:
 *   All channels start at 0.  Ramp to target duty=200 over
 *   many ticks.  Verify monotonic increase with no downward jitter.
 * =================================================================== */

static void test_e2e_morning_boot_smooth_ramp_up(void) {
    reset_pwm();

    const uint16_t TARGET = 200;

    /* All channels at 0 (sphere resting on platform) */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == 0);
    }

    /* Ramp all 5 channels toward TARGET simultaneously */
    uint16_t prev[PWM_CHANNELS];
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        prev[ch] = 0;
    }

    for (int tick = 0; tick < 100; tick++) {
        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            ramp_pwm_duty(ch, TARGET, RAMP_STEP_DEFAULT);
            uint16_t current = get_pwm_duty(ch);

            /* Monotonic increase: current >= previous */
            assert(current >= prev[ch]);

            /* Safety: never exceed MAX_DUTY_LIMIT */
            assert(current <= MAX_DUTY_LIMIT);

            prev[ch] = current;
        }
    }

    /* All channels must reach TARGET */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == TARGET);
    }

    printf("[PASS] test_e2e_morning_boot_smooth_ramp_up\n");
}


/* ===================================================================
 * SCENARIO 10b: Morning Boot — Ramp-Up capped at MAX_DUTY_LIMIT
 *
 * If the target is above MAX_DUTY_LIMIT, ramp stops at the ceiling.
 * =================================================================== */

static void test_e2e_morning_boot_ramp_up_capped(void) {
    reset_pwm();

    const uint16_t OVER_TARGET = 800;  /* Above MAX_DUTY_LIMIT (460) */

    for (int tick = 0; tick < 200; tick++) {
        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            ramp_pwm_duty(ch, OVER_TARGET, RAMP_STEP_DEFAULT);
        }
    }

    /* All channels capped at MAX_DUTY_LIMIT */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == MAX_DUTY_LIMIT);
    }

    printf("[PASS] test_e2e_morning_boot_ramp_up_capped\n");
}


/* ===================================================================
 * SCENARIO 11: Sensor Anomaly — Impossible Jump Rejected
 *
 * business_e2e_scenarios.md §11:
 *   "Алгоритм Комплементарного фильтра жестко отбрасывает (Reject)
 *    этот кадр матрицы."
 *   "ПИД-регулятор не получает ложную ошибку, и RP2040 продолжает
 *    генерировать предыдущий стабильный ШИМ."
 *
 * Simulation:
 *   FSM at prev_duty=200.  A packet with duty=900 arrives
 *   (delta=700 > ANOMALY_DELTA_THRESHOLD=400).  FSM rejects it.
 * =================================================================== */

static void test_e2e_anomaly_rejects_impossible_jump(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Feed first packet to establish baseline at 200 */
    uint16_t baseline[NUM_PWM_VALS] = {200, 200, 200, 200, 200};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(baseline, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Apply to PWM */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, 200);
    }

    /* Solar glare! Sensor reports duty=900 (delta=700 > threshold=400) */
    t += 1000;  /* 1 ms later */
    uint16_t glare[NUM_PWM_VALS] = {900, 900, 900, 900, 900};
    serialize_packet(glare, wire_buf, sizeof(wire_buf));
    consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(parsed.valid);

    /* FSM MUST reject this anomalous packet */
    assert(fsm_feed_packet(&parsed, t) == false);

    /* PWM unchanged — sphere stays stable */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == 200);
    }

    /* Targets unchanged */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 200);
    }

    printf("[PASS] test_e2e_anomaly_rejects_impossible_jump\n");
}


/* ===================================================================
 * SCENARIO 11b: Anomaly — Normal PID correction accepted
 *
 * A reasonable correction (delta=50 << threshold=400) is accepted.
 * =================================================================== */

static void test_e2e_anomaly_accepts_normal_correction(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* Establish baseline at 200 */
    uint16_t baseline[NUM_PWM_VALS] = {200, 200, 200, 200, 200};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(baseline, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));
    assert(fsm_feed_packet(&parsed, t));

    /* Normal PID correction: 200 → 250 (delta=50, well within threshold) */
    t += 1000;
    uint16_t correction[NUM_PWM_VALS] = {250, 250, 250, 250, 250};
    serialize_packet(correction, wire_buf, sizeof(wire_buf));
    consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));

    /* FSM MUST accept this normal correction */
    assert(fsm_feed_packet(&parsed, t) == true);

    /* Targets updated */
    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 250);
    }

    printf("[PASS] test_e2e_anomaly_accepts_normal_correction\n");
}


/* ===================================================================
 * SCENARIO 11c: Anomaly — First packet always accepted
 *
 * After fsm_init(), there is no previous baseline.  The first packet
 * must always be accepted regardless of its duty values.
 * =================================================================== */

static void test_e2e_anomaly_first_packet_always_accepted(void) {
    reset_pwm();

    uint64_t t = 0;
    fsm_init(t);

    /* First packet with high duty values — no baseline to compare */
    uint16_t first[NUM_PWM_VALS] = {450, 450, 450, 450, 450};
    uint8_t wire_buf[TOTAL_PACKET_SIZE];
    serialize_packet(first, wire_buf, sizeof(wire_buf));
    PacketData parsed;
    size_t consumed = 0;
    assert(parse_byte_stream(wire_buf, sizeof(wire_buf), &parsed, &consumed));

    /* Must be accepted — first packet has no delta to check */
    assert(fsm_feed_packet(&parsed, t) == true);

    const uint16_t *targets = fsm_get_target_duties();
    for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
        assert(targets[ch] == 450);
    }

    printf("[PASS] test_e2e_anomaly_first_packet_always_accepted\n");
}


/* ===================================================================
 * SCENARIO BONUS: Full 45-minute session simulation (mini version)
 *
 * Simulates a condensed version of the full Deep Work session:
 *   - 100 packets from max duty (400) to 0
 *   - Each packet followed by ramp convergence
 *   - Checks end-to-end pipeline integrity across the full sweep
 * =================================================================== */

static void test_e2e_full_session_sweep(void) {
    reset_pwm();

    const int TOTAL_PACKETS = 100;
    const uint16_t MAX_START = 400;

    /* Pre-set starting duty */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        set_pwm_duty(ch, MAX_START);
    }

    for (int i = 1; i <= TOTAL_PACKETS; i++) {
        uint16_t target = (uint16_t)(MAX_START - (uint32_t)(MAX_START * i) / TOTAL_PACKETS);

        uint16_t duties[NUM_PWM_VALS];
        for (uint8_t ch = 0; ch < NUM_PWM_VALS; ch++) {
            duties[ch] = target;
        }

        /* Full pipeline: serialize → parse → ramp */
        uint8_t wire_buf[TOTAL_PACKET_SIZE];
        size_t written = serialize_packet(duties, wire_buf, sizeof(wire_buf));
        assert(written == TOTAL_PACKET_SIZE);

        PacketData parsed;
        size_t consumed = 0;
        bool ok = parse_byte_stream(wire_buf, written, &parsed, &consumed);
        assert(ok && parsed.valid);

        /* Ramp to convergence (100 ticks = 100 ms simulated) */
        apply_packet_ramped(parsed.duty_cycles, 100);

        /* All channels converged to target */
        for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
            assert(get_pwm_duty(ch) == target);
        }
    }

    /* Final: all channels at 0 */
    for (uint8_t ch = 0; ch < PWM_CHANNELS; ch++) {
        assert(get_pwm_duty(ch) == 0);
    }

    printf("[PASS] test_e2e_full_session_sweep\n");
}


/* ===================================================================
 * Test Runner
 * =================================================================== */

int main(void) {
    printf("=== Running E2E Integration Tests ===\n");
    printf("  (business_e2e_scenarios.md coverage)\n\n");

    /* Scenario 1: Deep Work — Smooth Ramp-Down */
    printf("--- Scenario 1: Deep Work (Smooth Ramp-Down) ---\n");
    test_e2e_deep_work_smooth_ramp_down();
    test_e2e_deep_work_ramp_monotonicity_per_tick();

    /* Scenario 2: Crafting Nod — ACTION_NOD Spike */
    printf("\n--- Scenario 2: Crafting Nod (ACTION_NOD Spike) ---\n");
    test_e2e_crafting_nod_spike_impulse();
    test_e2e_crafting_nod_spike_clamped();

    /* Scenario 3: Storytelling — High-Freq Oscillation */
    printf("\n--- Scenario 3: Storytelling (High-Freq Oscillation) ---\n");
    test_e2e_storytelling_tremor_no_drop();
    test_e2e_storytelling_calm_sway();

    /* EMI Garbage Rejection */
    printf("\n--- EMI Garbage Rejection ---\n");
    test_e2e_emi_garbage_preserves_pwm();
    test_e2e_emi_bitflip_crc_rejection();
    test_e2e_emi_recovery_after_garbage();

    /* Scenario 7: Wi-Fi Heartbeat Loss — Safe Landing */
    printf("\n--- Scenario 7: Wi-Fi Heartbeat Loss (Safe Landing) ---\n");
    test_e2e_heartbeat_loss_triggers_safe_landing();
    test_e2e_heartbeat_loss_ramp_down_completes();
    test_e2e_heartbeat_recovery_after_reconnect();

    /* Scenario 9: OTA In-Air — Firmware Update */
    printf("\n--- Scenario 9: OTA In-Air (Firmware Update) ---\n");
    test_e2e_ota_locks_duty_at_safe_level();
    test_e2e_ota_rejects_dynamic_packets();
    test_e2e_ota_success_resumes_active();

    /* Scenario 10: Smooth Morning Boot — Ramp-Up */
    printf("\n--- Scenario 10: Smooth Morning Boot (Ramp-Up) ---\n");
    test_e2e_morning_boot_smooth_ramp_up();
    test_e2e_morning_boot_ramp_up_capped();

    /* Scenario 11: Sensor Anomaly — False Reading Rejection */
    printf("\n--- Scenario 11: Sensor Anomaly (False Reading Rejection) ---\n");
    test_e2e_anomaly_rejects_impossible_jump();
    test_e2e_anomaly_accepts_normal_correction();
    test_e2e_anomaly_first_packet_always_accepted();

    /* Bonus: Full session sweep */
    printf("\n--- Bonus: Full Session Sweep ---\n");
    test_e2e_full_session_sweep();

    printf("\n=== All E2E Integration Tests PASSED (21 tests) ===\n");
    return 0;
}
