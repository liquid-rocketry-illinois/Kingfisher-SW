//
// Created by dyrel on 4/16/2026.
//

#ifndef KINGFISHER_SW_CONSTANTS_H
#define KINGFISHER_SW_CONSTANTS_H

#include "stm32h7xx_hal.h"

// ── Handshake ─────────────────────────────────────────────────────────────────
#define HANDSHAKE_GND_BYTE      0xA1u
#define HANDSHAKE_FC_BYTE       0xB2u

// ── Ground station polling ────────────────────────────────────────────────────
#define GND_SENSOR_PERIOD_MS    100u    // 10 Hz local sensor update

// ── Abort accumulator ─────────────────────────────────────────────────────────
#define ABORT_ACCUM_MS          1500u   // continuous signal time required to latch abort
#define ABORT_DROPOUT_MS        300u    // gap that resets the accumulator
#define COMMS_ERR_THRESHOLD     5       // consecutive TX/RX failures before warning

// ── Liftoff detection ─────────────────────────────────────────────────────────
#define LIFTOFF_ACCEL_G         3.0f    // acceleration threshold (G)
#define LIFTOFF_SUSTAIN_MS      100u    // must exceed threshold for this long
#define LIFTOFF_ALT_DELTA_M     5.0f    // must gain this many metres from pad

// ── Stage transition thresholds ───────────────────────────────────────────────
#define BURNOUT_ACCEL_G         1.5f    // below this G = motor burnout
#define APOGEE_APPROACH_VEL_MS  20.0f   // < 20 m/s upward = apogee approach
#define APOGEE_VEL_MS           5.0f    // <  5 m/s upward = near apogee
#define APOGEE_PASS_VEL_MS      (-3.0f) // < -3 m/s = confirmed past apogee
#define DESCENT_VEL_MS          (-7.0f) // < -7 m/s = descent established
#define FINAL_DESCENT_VEL_MS    (-5.0f) // > -5 m/s = stable under main chute

// ── Altitude thresholds (metres MSL — calibrate to launch site) ───────────────
#define MIN_APOGEE_ALT_M        2000.0f  // guard against false apogee at ground
#define MAX_MAIN_DEPLOY_ALT_M   400.0f  // top of main-chute deploy window
#define MIN_MAIN_DEPLOY_ALT_M   250.0f  // safety floor for main deploy
#define TARGET_MAIN_ALT_M       300.0f  // fire main chute at or below this altitude

// ── GPIO ──────────────────────────────────────────────────────────────────────
#define USR_BUTTON_ACTIVE_STATE GPIO_PIN_SET

// ── Sensor sanity bounds ──────────────────────────────────────────────────────
#define BARO_ALT_MIN_M          (-500.0f)
#define BARO_ALT_MAX_M          30000.0f
#define BARO_PRESS_MIN_PA       10000.0f
#define BARO_PRESS_MAX_PA       110000.0f

#endif //KINGFISHER_SW_CONSTANTS_H

//
// Flight constants for HAL-1 / Kingfisher
//

#ifndef KINGFISHER_SW_CONSTANTS_H
#define KINGFISHER_SW_CONSTANTS_H

#include <cstdint>

// ── Liftoff detection ────────────────────────────────────────────────────────
constexpr float    LIFTOFF_ACCEL_G       = 3.0f;    // accel magnitude threshold (G)
constexpr uint32_t LIFTOFF_SUSTAIN_MS    = 200;     // must be sustained this long
constexpr float    LIFTOFF_ALT_DELTA_M   = 10.0f;   // min altitude gain from launch point (m)

// ── Stage-transition velocity thresholds  (positive = up, negative = down) ──
constexpr float    BURNOUT_ACCEL_G       = 1.5f;    // ASCENT→CONTROLS_TEST: accel drops below this
constexpr float    APOGEE_APPROACH_VEL_MS = 20.0f;  // CONTROLS_TEST→APOGEE_APPROACH: vert vel (m/s)
constexpr float    APOGEE_VEL_MS         = 5.0f;    // APOGEE_APPROACH→APOGEE:  vert vel (m/s)
constexpr float    MIN_APOGEE_ALT_M      = 2500.0f; // guard: must be above this to trigger APOGEE
constexpr float    APOGEE_PASS_VEL_MS    = -3.0f;   // APOGEE→APOGEE_PASS:   vert vel (m/s)
constexpr float    DESCENT_VEL_MS        = -7.0f;   // APOGEE_PASS→DESCENT:  vert vel (m/s)
constexpr float    FINAL_DESCENT_VEL_MS  = -5.0f;   // MAIN→FINAL_DESCENT:   vert vel > this (m/s)

// ── Main parachute deployment window ────────────────────────────────────────
constexpr float    MAX_MAIN_DEPLOY_ALT_M = 500.0f;  // enter window below this AGL (m)
constexpr float    MIN_MAIN_DEPLOY_ALT_M = 200.0f;  // safety floor — do not deploy below this (m)
constexpr float    TARGET_MAIN_ALT_M     = 300.0f;  // deploy main at this AGL (m)

// ── Abort accumulator ────────────────────────────────────────────────────────
constexpr uint32_t ABORT_ACCUM_MS        = 1500;    // total active time needed to trigger abort
constexpr uint32_t ABORT_DROPOUT_MS      = 200;     // max gap before accumulator resets

// ── Ground station ───────────────────────────────────────────────────────────
constexpr uint32_t GND_SENSOR_PERIOD_MS  = 3000;    // how often GND updates local sensors

// ── Comms health ─────────────────────────────────────────────────────────────
constexpr int8_t   COMMS_ERR_THRESHOLD   = 10;      // consecutive TX failures before logging warning

// ── Handshake bytes (keepAliveIn / keepAliveStatus channel) ─────────────────
constexpr uint8_t  HANDSHAKE_GND_BYTE    = 0x01;    // GND sends this to initiate handshake
constexpr uint8_t  HANDSHAKE_FC_BYTE     = 0x02;    // FC replies with this to confirm

// ── Hardware polarity ────────────────────────────────────────────────────────
// USR_BUTTON is GPIO_NOPULL — verify active state against hardware schematic
#define USR_BUTTON_ACTIVE_STATE GPIO_PIN_SET

#endif //KINGFISHER_SW_CONSTANTS_H