//
// Created by dyrel on 4/16/2026.
//

#ifndef KINGFISHER_SW_CONSTANTS_H
#define KINGFISHER_SW_CONSTANTS_H

#include "stm32h7xx_hal.h"
#include "Ebyte_E22_900T22S_defs.h"

// -- MASTER RADIO FREQUENCY --
#define GLOBAL_RADIO_CHANNEL CH909

// ── Handshake / protocol bytes ────────────────────────────────────────────────
#define HANDSHAKE_GND_BYTE      0xA1u   // GND→FC: (re)connection handshake
#define HANDSHAKE_FC_BYTE       0xB2u   // FC→GND: handshake acknowledgement (heartbeat)
#define REQUEST_DATA_BYTE       0xC3u   // GND→FC: "send me a telemetry packet now"
#define BYTE_DEFLECT_TEST       150u    // GND→FC: commanded servo deflection test — matches DEFLECT_TEST in Telemetry.h and GND RADIO_DEFNS.h

// ── Ground station polling ────────────────────────────────────────────────────
#define GND_SENSOR_PERIOD_MS    100u    // 10 Hz local sensor update

// ── Abort accumulator ─────────────────────────────────────────────────────────
#define ABORT_ACCUM_MS          1500u   // continuous signal time required to latch abort
#define ABORT_DROPOUT_MS        300u    // gap that resets the accumulator
#define COMMS_ERR_THRESHOLD     5       // consecutive TX/RX failures before warning

// ── Liftoff detection ─────────────────────────────────────────────────────────
#define LIFTOFF_ACCEL_G         3.0f    // acceleration threshold (G) *****
#define LIFTOFF_SUSTAIN_MS      200u    // must exceed threshold for this long
#define LIFTOFF_ALT_DELTA_M     20.0f    // must gain this many metres from pad *****

// ── Stage transition thresholds ───────────────────────────────────────────────
#define BURNOUT_ACCEL_G         (-1.0f)    // below this G = motor burnout. THIS MUST BE ABSOLUTE MAGNITUDE. Was 1.5f
#define APOGEE_APPROACH_VEL_MS  10.0f   // < 20 m/s upward = apogee approach
#define APOGEE_VEL_MS           3.0f    // <  5 m/s upward = near apogee
#define APOGEE_PASS_VEL_MS      (-3.0f) // < -3 m/s = confirmed past apogee
#define DESCENT_VEL_MS          (-7.0f) // < -7 m/s = descent established
#define FINAL_DESCENT_VEL_MS    (-9.0f) // > -9 m/s = stable under main chute

// ── Altitude thresholds (metres MSL — calibrate to launch site) ───────────────
#define MIN_APOGEE_ALT_M        2500.0f  // guard against false apogee at ground
#define MAX_MAIN_DEPLOY_ALT_M   400.0f  // top of main-chute deploy window
#define MIN_MAIN_DEPLOY_ALT_M   250.0f  // safety floor for main deploy
#define TARGET_MAIN_ALT_M       300.0f  // fire main chute at or below this altitude

// ── Ignition detection ────────────────────────────────────────────────────────
#define IGNITION_SUSTAIN_MS     300u    // vertical accel must exceed LIFTOFF_ACCEL_G for this long
#define IGNITION_MIN_VEL_MS     5.0f    // vertical velocity must exceed this (m/s) to confirm ignition

// ── Burnout detection ─────────────────────────────────────────────────────────
#define BURNOUT_SUSTAIN_MS          200u    // total accel must stay below BURNOUT_ACCEL_G for this long
#define MAX_ACTUATION_DURATION_MS   25000u  // disable actuation this many ms after burnout confirmed

// ── Controls activation gate ──────────────────────────────────────────────────
#define CTRLS_MIN_ALT_M         100.0f  // AGL metres before roll control arms
#define MAX_PITCH_ANGLE_DEG     30.0f   // if pitch exceeds this, permanently disable actuation
#define MIN_ACTUATION_VEL_MS    2.0f    // if |vertical velocity| drops below this, permanently disable actuation

// ── Vertical velocity ─────────────────────────────────────────────────────────
#define VVEL_UPDATE_PERIOD_MS   50u    // fixed interval for altitude delta sampling

// ── Initial condition tare ────────────────────────────────────────────────────
#define TARE_STABILITY_MS       (3u * 60u * 1000u)  // 3 minutes of continuous stability required
#define TARE_ACC_DELTA_G        0.1f    // max inter-sample acceleration change (g) to count as stable
#define TARE_GYRO_DELTA_DPS     5.0f    // max inter-sample gyro change (deg/s) to count as stable

// ── Backup drogue autonomous trigger ─────────────────────────────────────────
#define BACKUP_DROGUE_MIN_ALT_M 1000.0f // minimum altitude (m) to trigger backup
#define BACKUP_DROGUE_VEL_MS    (-40.0f)// falling faster than this without primary

// ── Pyrotechnic pending bitmask (written by Radio task, consumed by PyroTask) ─
#define PYRO_DROGUE_MAIN_BIT    (1u << 0)
#define PYRO_DROGUE_BKP_BIT     (1u << 1)
#define PYRO_MAIN_BIT           (1u << 2)
extern volatile uint32_t g_pyroPending;

// ── GPIO ──────────────────────────────────────────────────────────────────────
#define USR_BUTTON_ACTIVE_STATE GPIO_PIN_RESET

// ── Sensor sanity bounds ──────────────────────────────────────────────────────
#define BARO_ALT_MIN_M          (-500.0f)
#define BARO_ALT_MAX_M          30000.0f
#define BARO_PRESS_MIN_PA       10000.0f
#define BARO_PRESS_MAX_PA       110000.0f

#endif //KINGFISHER_SW_CONSTANTS_H