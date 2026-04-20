//
// Created by dyrel on 4/13/2026.
//

#ifndef KINGFISHER_SW_FLIGHT_PROCEDURES_H
#define KINGFISHER_SW_FLIGHT_PROCEDURES_H

#include <stdint.h>
#include "Servo_Axon_Mini_MKII.h"
#include "IMUs.h"
#include "Barometer.h"
#include "LIS2MDL.h"
#include "Telemetry.h"
#include "constants.h"
#include "SDCard.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "MAXM10S.h"

// Servo canard trim offsets defined in main.cpp — edit there before flashing GND
extern float GND_SERVO_OFFSET_S1;
extern float GND_SERVO_OFFSET_S2;

typedef enum : int8_t {
    STATUS_ABORT_ACTIVE     = -4,  // Abort already latched, redundant trigger
    STATUS_SENSOR_FAIL      = -3,  // GND/FC sensor data exceeds 5% tolerance
    STATUS_COMMS_FAILURE    = -2,  // Handshake or radio TX/RX failure
    STATUS_INIT_FAILURE     = -1,  // One or more devices failed to initialize
    STATUS_OK               =  0,  // Nominal
    STATUS_TIMEOUT          =  1,  // Operation exceeded allowed time window
    STATUS_LOCAL_READ_ERR   =  2,  // Local sensor read returned bad data
    STATUS_LOCAL_DATA_ERR   =  3,  // Local sensor data out of expected range
    STATUS_REMOTE_READ_ERR  =  4,  // Radio/FC read returned bad data
    STATUS_REMOTE_DATA_ERR  =  5,  // FC data out of expected range
    STATUS_ABORT_TRIGGERED  =  127 // Abort signal freshly activated
} ProcedureStatus;

typedef enum {
    PREFLIGHT,
    ASCENT,
    CONTROLS_TEST,
    APOGEE_APPROACH,
    APOGEE,
    APOGEE_PASS,
    DESCENT,
    MAIN_APPROACH,
    MAIN,
    FINAL_DESCENT
} STAGE;

// Both ground and FC record the same data
typedef struct {
    GndStationData          dat_GND_Data;
    telemetryData           dat_FC_Data;
    DATA_Axon_Mini_MKII     dat_Servos;
    BMI_Data                dat_BMI_IMUs;
    BARO_DATA               dat_Barometers;
    MAXM10S::gpsData        dat_GPS;
} Data;

// Devices shared by both ground and flight computers
typedef struct {
    Telemetry               dev_telemetry;
    Baro_Unified            dev_BarometerEngine;
    Servo_Axon_Mini_MKII    dev_servoSet;
    IMUs                    dev_IMU_Engine;
    MAXM10S                 dev_GPS;
    // Magnetometer         dev_Magnetometer;  (add when integrated)
} Devices;


// ════════════════════════════════════════════════════════════════════════════
//  GROUND STATION
// ════════════════════════════════════════════════════════════════════════════

class GroundStation {

    // ── State ────────────────────────────────────────────────────────────────
    bool     initDone                = false;
    bool     abortLatched            = false;
    bool     radioConnected          = false;  // true once handshake done; false on link loss
    int8_t   commsErrCount           = 0;      // consecutive TX/RX failures
    uint32_t holdStartMs             = 0;      // millis() when button first went active
    uint32_t lastLEDToggleMs         = 0;      // rate-limits abort LED blink
    uint32_t lastLogMs               = 0;      // rate-limits SD logging

    Data    GNDData    = {};
    Devices GNDDevices = {};

public:
    GroundStation();

    /* Init()
     *
     * Init all devices; aggregate return values.
     * If any Init() != 0: return STATUS_INIT_FAILURE
     *
     * Handshake (poll via osDelay(10) loops, 2s timeout each):
     *   set GNDData.dat_GND_Data.keepAliveIn = HANDSHAKE_GND_BYTE, call UpdateTelemetry()
     *   poll UpdateTelemetry() until dat_FC_Data.keepAliveStatus == HANDSHAKE_FC_BYTE
     *   if 2s elapsed without valid response: return STATUS_COMMS_FAILURE
     *
     * Sensor verification:
     *   call UpdateSensors() 5 times
     *   if any call != STATUS_OK: return STATUS_LOCAL_READ_ERR
     *   if any sensor value out of expected range: return STATUS_LOCAL_DATA_ERR
     *   poll UpdateTelemetry() up to 2s for FC sensor data
     *   if timeout: return STATUS_TIMEOUT
     *   if |GND reading - FC reading| > 5%: return STATUS_SENSOR_FAIL
     *
     * Servo verification:
     *   for each angle in {±1, ±3, ±5, ±10, ±20, ±30, ±45}:
     *     dev_servoSet.Update(angle, angle); read back currentAngle; accumulate error
     *   if cumulative error > 2%: return STATUS_LOCAL_DATA_ERR
     *   if cumulative error > 1%: SD_LogNewline("SERVO WARN")
     *
     * initDone = true
     * return STATUS_OK
     */
    int8_t Init();


    /* Update()  — called every FreeRTOS task iteration
     *
     * if !initDone: return STATUS_INIT_FAILURE
     *
     * if millis() - lastLocalUpdateMs >= GND_SENSOR_PERIOD_MS:
     *   if UpdateSensors() != STATUS_OK: SD_LogNewline("LOCAL SENSOR WARN")
     *   pendingLocalLog   = true
     *   lastLocalUpdateMs = millis()
     *
     * status = UpdateKeepAlive()
     * if status == STATUS_ABORT_TRIGGERED:
     *   GNDData.dat_GND_Data.keepAliveIn = SHUTDOWN_KEEPALIVE
     *
     * UpdateTelemetry()
     * UpdateLogging()
     * return STATUS_OK
     */
    int8_t Update();

private:

    /* UpdateSensors()
     *
     * dev_BarometerEngine.Update() → GNDData.dat_Barometers
     * dev_IMU_Engine.Update()      → GNDData.dat_BMI_IMUs
     * // dev_Magnetometer.Update() (when integrated)
     * if any update failed: return STATUS_LOCAL_READ_ERR
     * return STATUS_OK
     */
    int8_t UpdateSensors();


    /* UpdateTelemetry()
     *
     * dev_telemetry.GNDOutData = GNDData.dat_GND_Data
     * if dev_telemetry.Update() != STATUS_OK: return STATUS_REMOTE_READ_ERR
     * GNDData.dat_FC_Data = dev_telemetry.HALOutData
     * return STATUS_OK
     */
    int8_t UpdateTelemetry();


    /* UpdateKeepAlive()
     *
     * if abortLatched: return STATUS_ABORT_ACTIVE
     *
     * buttonActive = (HAL_GPIO_ReadPin(USR_BUTTON_GPIO_Port, USR_BUTTON_Pin) == USR_BUTTON_ACTIVE_STATE)
     *
     * // NOTE TODO: ensure that 1.5ms include 0.1 grace for button release
     * if buttonActive:
     *   if holdStartMs == 0: holdStartMs = millis()
     *   if millis() - lastLEDToggleMs >= 200:          // blink at 2.5 Hz while held
     *     HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin)
     *     lastLEDToggleMs = millis()
     *   if millis() - holdStartMs >= ABORT_ACCUM_MS:
     *     abortLatched = true
     *     SD_LogNewline("ABORT TRIGGERED")
     *     return STATUS_ABORT_TRIGGERED
     * else:
     *   holdStartMs = 0                                 // button released — reset hold timer
     *   stop the timer completely, until next button activation
     *
     * return STATUS_OK
     */
    int8_t UpdateKeepAlive();


    /* UpdateLogging()
     *
     * if pendingLocalLog:
     *   SD_LogNewline(timestamp + GNDData local sensor fields)
     *   pendingLocalLog = false
     * SD_LogNewline(timestamp + GNDData.dat_FC_Data fields)
     * return STATUS_OK
     */
    int8_t UpdateLogging();
};



// ════════════════════════════════════════════════════════════════════════════
//  FLIGHT COMPUTER
// ════════════════════════════════════════════════════════════════════════════

class FlightComputer {

    // ── CONOPS state ─────────────────────────────────────────────────────────
    STAGE    currentStage          = PREFLIGHT;
    bool     initDone              = false;
    float    launchAltM            = 0.0f;   // altitude at launch pad (AGL reference, metres)
    uint32_t liftoffDetectMs       = 0;      // millis() when liftoff accel first exceeded threshold

    // ── Abort accumulator ────────────────────────────────────────────────────
    bool     abortLatched          = false;
    uint32_t abortAccumMs          = 0;      // accumulated active-signal time
    uint32_t abortDropoutMs        = 0;      // elapsed time since signal went inactive
    uint32_t lastAbortUpdateMs     = 0;      // last millis() sample for delta-time

    // ── Pyro state ───────────────────────────────────────────────────────────
    bool     pyroMainDrogueFired   = false;
    bool     pyroBackupDrogueFired = false;
    bool     pyroMainChuteFired    = false;

    // ── Telemetry health ─────────────────────────────────────────────────────
    int8_t   commsErrCount         = 0;      // consecutive TX/RX failures
    bool     radioConnected        = false;  // true once handshake done; false on link loss

    // ── SD logging ───────────────────────────────────────────────────────────
    uint32_t lastLogMs             = 0;      // rate-limits SD log writes

    // ── Servo trim ───────────────────────────────────────────────────────────
    bool     servoOffsetApplied    = false;  // prevents repeated SD log entries for same offset

    // ── Controls test sequence ───────────────────────────────────────────────
    uint8_t  ctrlTestStep          = 0;      // current step index (0–2)
    uint32_t ctrlTestStepStartMs   = 0;      // millis() when current step began

    // ── Altitude spike filter (median-of-3) ──────────────────────────────────
    float    altFilterBuf[3]       = {0.0f, 0.0f, 0.0f};
    uint8_t  altFilterIdx          = 0;
    float    filteredAltM          = 0.0f;  // spike-filtered altitude — use for all decisions

    // ── Vertical velocity (barometric dAlt/dt) ───────────────────────────────
    float    vertVelocityMs        = 0.0f;   // positive = ascending, negative = descending
    float    prevAltM              = 0.0f;
    uint32_t prevAltTimeMs         = 0;

    Data    FCData    = {};
    Devices FCDevices = {};

public:
    FlightComputer();

    /* Init()
     *
     * Init IMU engine and barometer engine.
     * // magnetometer + init here when integrated
     * If any status != 0: return STATUS_INIT_FAILURE
     *
     * if dev_telemetry.Init(TELEMETRY_MODE_FLIGHT) != 0: return STATUS_COMMS_FAILURE
     *
     * Handshake (poll via osDelay(10) loops, 3s timeout):
     *   poll dev_telemetry.Update() until GNDOutData.keepAliveIn == HANDSHAKE_GND_BYTE
     *   if 3s elapsed: return STATUS_COMMS_FAILURE
     *   set dev_telemetry.HALOutData.keepAliveStatus = HANDSHAKE_FC_BYTE
     *   call dev_telemetry.Update() to transmit response
     *
     * Sensor sync (poll via osDelay(10), 2s timeout):
     *   UpdateSensors()
     *   launchAltM = FCData.dat_Barometers.Filtered.heightMeters
     *   successCount = 0
     *   while millis() - syncStartMs < 2000:
     *     UpdateSensors(); UpdateTelemetry()
     *     if GND sensor data received && |local - GND| <= 5% for all sensors: successCount++
     *     if successCount >= 10: initDone = true; return STATUS_OK
     *     osDelay(10)
     *   return STATUS_TIMEOUT
     */
    int8_t Init();


    /* Update()  — called every FreeRTOS task iteration
     *
     * if !initDone: return STATUS_INIT_FAILURE
     *
     * UpdateTelemetry()            // TX: last cycle's sensor data  RX: current GND commands
     * UpdateAbortAccumulator()     // checks GND abort signal via SHUTDOWN_KEEPALIVE; calls Abort() if threshold met (no return)
     * UpdateSensors()
     * UpdateVerticalVelocity()
     * TrackCONOPS()
     * UpdatePyroTrack()
     *
     * if commsErrCount > COMMS_ERR_THRESHOLD: SD_LogNewline("COMMS WARNING")
     * return STATUS_OK
     */
    int8_t Update();

private:

    /* UpdateAbortAccumulator()
     *
     * signalActive = (FCData.dat_GND_Data.keepAliveIn == SHUTDOWN_KEEPALIVE)
     * dt = millis() - lastAbortUpdateMs; lastAbortUpdateMs = millis()
     *
     * if signalActive:
     *   abortDropoutMs = 0
     *   abortAccumMs  += dt
     * else:
     *   abortDropoutMs += dt
     *   if abortDropoutMs > ABORT_DROPOUT_MS: abortAccumMs = 0
     *
     * if abortAccumMs >= ABORT_ACCUM_MS: Abort()   // does not return
     */
    int8_t UpdateAbortAccumulator();


    /* UpdateSensors()
     *
     * imuStatus = dev_IMU_Engine.Update()    → FCData.dat_BMI_IMUs
     * baroStatus = dev_BarometerEngine.Update() → FCData.dat_Barometers
     * // dev_Magnetometer.Update() when integrated
     * if any status field != 0: return STATUS_LOCAL_READ_ERR
     * return STATUS_OK
     *
     * Note: IMUs::Update() returns IMUsStatus {A, B, C, ISM} — check each field individually.
     */
    int8_t UpdateSensors();


    /* UpdateVerticalVelocity()
     *
     * currentAltM = FCData.dat_Barometers.Filtered.heightMeters
     * now = millis()
     * dt  = now - prevAltTimeMs
     * if dt > 0: vertVelocityMs = (currentAltM - prevAltM) / (dt / 1000.0f)
     * prevAltM      = currentAltM
     * prevAltTimeMs = now
     */
    void UpdateVerticalVelocity();


    /* UpdateTelemetry()
     *
     * Pack FCData into dev_telemetry.HALOutData:
     *   altitude              = FCData.dat_Barometers.Filtered.heightMeters
     *   mAccX/Y/Z             = FCData.dat_BMI_IMUs.accel_linear.x/y/z
     *   servoPos1             = FCData.dat_Servos.currentAngle.S1
     *   servoPos2             = FCData.dat_Servos.currentAngle.S2
     *   pyroMainDrogueFired   = pyroMainDrogueFired
     *   pyroBackupDrogueFired = pyroBackupDrogueFired
     *   pyroMainChuteFired    = pyroMainChuteFired
     *   // GPS, quaternion, pitch/yaw/roll when available
     *
     * if dev_telemetry.Update() != STATUS_OK: commsErrCount++; return STATUS_REMOTE_READ_ERR
     * commsErrCount = 0
     * FCData.dat_GND_Data = dev_telemetry.GNDOutData
     * return STATUS_OK
     */
    int8_t UpdateTelemetry();


    /* TrackCONOPS()
     * Advances currentStage based on sensor data. Only the active stage's condition is evaluated.
     *
     * switch(currentStage):
     *
     *   PREFLIGHT:
     *     if FCData.dat_BMI_IMUs.accel_linear.magnitude() > LIFTOFF_ACCEL_G:
     *       if liftoffDetectMs == 0: liftoffDetectMs = millis()
     *       if millis()-liftoffDetectMs > LIFTOFF_SUSTAIN_MS
     *          && FCData.dat_Barometers.Filtered.heightMeters > launchAltM + LIFTOFF_ALT_DELTA_M:
     *         currentStage = ASCENT; SD_LogNewline("STAGE: ASCENT")
     *     else: liftoffDetectMs = 0    // reset if accel drops before sustain
     *
     *   ASCENT:
     *     if FCData.dat_BMI_IMUs.accel_linear.magnitude() < BURNOUT_ACCEL_G:
     *       currentStage = CONTROLS_TEST; SD_LogNewline("STAGE: CONTROLS_TEST")
     *
     *   CONTROLS_TEST:
     *     if vertVelocityMs < APOGEE_APPROACH_VEL_MS:   // < 20.0 m/s upward
     *       currentStage = APOGEE_APPROACH; SD_LogNewline("STAGE: APOGEE_APPROACH")
     *
     *   APOGEE_APPROACH:
     *     if vertVelocityMs < APOGEE_VEL_MS             // < 5.0 m/s upward
     *        && FCData.dat_Barometers.Filtered.heightMeters > MIN_APOGEE_ALT_M:
     *       currentStage = APOGEE; SD_LogNewline("STAGE: APOGEE")
     *
     *   APOGEE:
     *     if vertVelocityMs < APOGEE_PASS_VEL_MS:       // < -3.0 m/s — confirmed descending
     *       currentStage = APOGEE_PASS; SD_LogNewline("STAGE: APOGEE_PASS")
     *
     *   APOGEE_PASS:
     *     if vertVelocityMs < DESCENT_VEL_MS:           // < -7.0 m/s — descent established
     *       currentStage = DESCENT; SD_LogNewline("STAGE: DESCENT")
     *
     *   DESCENT:
     *     if FCData.dat_Barometers.Filtered.heightMeters < MAX_MAIN_DEPLOY_ALT_M
     *        && FCData.dat_Barometers.Filtered.heightMeters > MIN_MAIN_DEPLOY_ALT_M:
     *       currentStage = MAIN_APPROACH; SD_LogNewline("STAGE: MAIN_APPROACH")
     *
     *   MAIN_APPROACH:
     *     if FCData.dat_Barometers.Filtered.heightMeters <= TARGET_MAIN_ALT_M:
     *       currentStage = MAIN; SD_LogNewline("STAGE: MAIN")
     *
     *   MAIN:
     *     if vertVelocityMs > FINAL_DESCENT_VEL_MS:     // > -5.0 m/s — stable under main chute
     *       currentStage = FINAL_DESCENT; SD_LogNewline("STAGE: FINAL_DESCENT")
     *
     *   FINAL_DESCENT: break
     *
     * return STATUS_OK
     */
    int8_t TrackCONOPS();


    /* UpdatePyroTrack()
     *
     * if currentStage == APOGEE && !pyroMainDrogueFired:
     *   firePyroMainDrogue(); pyroMainDrogueFired = true
     *
     * if currentStage == APOGEE_PASS && !pyroBackupDrogueFired:
     *   firePyroBackupDrogue(); pyroBackupDrogueFired = true   // unconditional — max redundancy
     *
     * if currentStage == MAIN && !pyroMainChuteFired:
     *   firePyroMainParachute(); pyroMainChuteFired = true
     *
     * // Pyro fired flags are packed into HALOutData by UpdateTelemetry()
     *
     * return STATUS_OK
     */
    int8_t UpdatePyroTrack();
    int8_t UpdateLogging();

    void firePyroMainDrogue();      // HAL_GPIO_WritePin DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin
    void firePyroBackupDrogue();    // HAL_GPIO_WritePin DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin
    void firePyroMainParachute();   // HAL_GPIO_WritePin MAIN_GPIO_Port, MAIN_Pin


    /* Abort()
     *
     * dev_FCDevices.dev_servoSet.Update(0.0f, 0.0f)
     * abortLatched = true
     * SD_LogNewline("ABORT")
     *
     * while (1):                   // FreeRTOS task loop — intentionally never returns
     *   UpdateSensors()
     *   UpdatePyroTrack()          // pyros continue to sequence normally
     *   UpdateTelemetry()          // keep ground informed
     *   Log_data() // TODO: still need to log data
     *   osDelay(10)
     */
    int8_t Abort();
};

#endif //KINGFISHER_SW_FLIGHT_PROCEDURES_H