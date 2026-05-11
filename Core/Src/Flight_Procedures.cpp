//
// Created by dyrel on 4/13/2026.
//

#include "Flight_Procedures.h"
#include "timing.h"
#include "cmsis_os2.h"
#include "main.h"
#include "i2c.h"
#include "CTRLS_Controls.h"
#include <cstdio>
#include <cmath>

// ════════════════════════════════════════════════════════════════════════════
//  GROUND STATION
// ════════════════════════════════════════════════════════════════════════════

GroundStation::GroundStation() : GNDDevices(), GNDData() {}

int8_t GroundStation::Init()
{
    for (;;) {

        // ── Device init ───────────────────────────────────────────────────────
        bool anyFail = false;
        if (SD_Init() != 0)                                             anyFail = true;
        if (GNDDevices.dev_telemetry.Init(TELEMETRY_MODE_GROUND) != 0)  anyFail = true;
        if (GNDDevices.dev_GPS.Init(&hi2c4) != 0)                       anyFail = true;

        if (anyFail) {
            SD_LogNewline("GND INIT: device init failed, retrying");
            osDelay(1000);
            continue;
        }

        SD_LogNewline("\n"); SD_LogNewline("\n");
        SD_LogNewline("BEGINNING NEW FLIGHT DATA");

        // ── Handshake: send GND byte, wait up to 5 s for FC echo ─────────────
        // No retry delay here — GND keeps broadcasting so FC can come online at
        // any time without needing both units activated simultaneously.
        GNDData.dat_GND_Data.CommandByteIn = HANDSHAKE_GND_BYTE;
        uint32_t t0 = millis();
        bool handshakeOK = false;
        while (millis() - t0 < 5000) {
            UpdateTelemetry();
            if (GNDData.dat_FC_Data.CommandResponseByte == HANDSHAKE_FC_BYTE) {
                handshakeOK = true;
                break;
            }
            osDelay(10);
        }
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        if (!handshakeOK) continue;   // retry — FC not online yet

        radioConnected = true;
        initDone = true;
        return STATUS_OK;

    } // retry loop
}

int8_t GroundStation::Update()
{
    if (!initDone) return STATUS_INIT_FAILURE;

    int8_t ka = UpdateKeepAlive();
    if (ka == STATUS_ABORT_TRIGGERED) {
        GNDData.dat_GND_Data.CommandByteIn = SHUTDOWN_KEEPALIVE;
    }

    UpdateTelemetry();
    UpdateLogging();
    return STATUS_OK;
}

int8_t GroundStation::UpdateSensors()
{
    GNDDevices.dev_GPS.update();
    GNDData.dat_GPS = GNDDevices.dev_GPS.getData();
    return STATUS_OK;
}

int8_t GroundStation::UpdateTelemetry()
{
    GNDDevices.dev_telemetry.GNDOutData = GNDData.dat_GND_Data;

    // Always carry servo offsets in every packet — FC applies them each cycle
    // (idempotent: re-applying the same value has no effect).
    // SERVO_OFFSET_CMD_BYTE replaces REQUEST_DATA_BYTE as the normal operating
    // command; FC is already configured to respond to it with a telemetry packet.
    // Only SHUTDOWN_KEEPALIVE and HANDSHAKE_GND_BYTE take priority.
    GNDDevices.dev_telemetry.GNDOutData.servoOffset1 = GND_SERVO_OFFSET_S1;
    GNDDevices.dev_telemetry.GNDOutData.servoOffset2 = GND_SERVO_OFFSET_S2;

    if (GNDDevices.dev_telemetry.GNDOutData.CommandByteIn != SHUTDOWN_KEEPALIVE &&
        GNDDevices.dev_telemetry.GNDOutData.CommandByteIn != HANDSHAKE_GND_BYTE)
    {
        GNDDevices.dev_telemetry.GNDOutData.CommandByteIn = SERVO_OFFSET_CMD_BYTE;
    }

    if (GNDDevices.dev_telemetry.Update() != 0) {
        if (commsErrCount < 127) commsErrCount++;
        if (commsErrCount == COMMS_ERR_THRESHOLD && radioConnected) {
            radioConnected = false;
            SD_LogNewline("COMMS LOST");
        }
        return STATUS_REMOTE_READ_ERR;
    }

    // Any successful receive = link alive. FC always responds with HANDSHAKE_FC_BYTE,
    // so a packet getting through is all the confirmation we need.
    commsErrCount = 0;
    GNDData.dat_FC_Data = GNDDevices.dev_telemetry.HALOutData;

    if (!radioConnected) {
        radioConnected = true;
        SD_LogNewline("COMMS RECONNECTED");
    }

    return STATUS_OK;
}

int8_t GroundStation::UpdateKeepAlive()
{
    if (abortLatched) return STATUS_ABORT_ACTIVE;

    bool buttonActive =
        (HAL_GPIO_ReadPin(USR_BUTTON_GPIO_Port, USR_BUTTON_Pin) == USR_BUTTON_ACTIVE_STATE);

    if (buttonActive) {
        if (holdStartMs == 0) holdStartMs = millis();

        if (millis() - lastLEDToggleMs >= 200) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
            lastLEDToggleMs = millis();
        }

        if (millis() - holdStartMs >= ABORT_ACCUM_MS) {
            abortLatched = true;
            SD_LogNewline("ABORT TRIGGERED");
            return STATUS_ABORT_TRIGGERED;
        }
    } else {
        holdStartMs = 0;
    }

    return STATUS_OK;
}

int8_t GroundStation::UpdateLogging()
{
    if (millis() - lastLogMs < 500) return STATUS_OK;   // 2 Hz log rate
    lastLogMs = millis();

    char buf[192];
    snprintf(buf, sizeof(buf),
        "T=%lu FC ALT=%.1f ACC=%.2f,%.2f,%.2f GYR=%.2f,%.2f,%.2f SRV_TGT=%.1f,%.1f SRV_POS=%.1f,%.1f PYRO=%d%d%d",
        static_cast<unsigned long>(millis()),
        GNDData.dat_FC_Data.altitude,
        GNDData.dat_FC_Data.mAccX,
        GNDData.dat_FC_Data.mAccY,
        GNDData.dat_FC_Data.mAccZ,
        GNDData.dat_FC_Data.mGyrX,
        GNDData.dat_FC_Data.mGyrY,
        GNDData.dat_FC_Data.mGyrZ,
        GNDData.dat_FC_Data.servoTarget1,
        GNDData.dat_FC_Data.servoTarget2,
        GNDData.dat_FC_Data.servoPos1,
        GNDData.dat_FC_Data.servoPos2,
        static_cast<int>(GNDData.dat_FC_Data.pyroMainDrogueFired),
        static_cast<int>(GNDData.dat_FC_Data.pyroBackupDrogueFired),
        static_cast<int>(GNDData.dat_FC_Data.pyroMainChuteFired));
    SD_LogNewline(buf);
    return STATUS_OK;
}


// ════════════════════════════════════════════════════════════════════════════
//  FLIGHT COMPUTER
// ════════════════════════════════════════════════════════════════════════════

FlightComputer::FlightComputer() : FCDevices(), FCData() {}

int8_t FlightComputer::Init()
{
    // ── Phase 1: Device init — retry until hardware is healthy ───────────────
    for (;;) {
        if (SD_Init() != 0) {
            osDelay(1000);
            continue;
        }

        SD_LogNewline("\n"); SD_LogNewline("\n");
        SD_LogNewline("BEGINNING NEW FLIGHT DATA");

        auto imuSt  = FCDevices.dev_IMU_Engine.Init(true);
        auto baroSt = FCDevices.dev_BarometerEngine.Init(true);
        FCDevices.dev_servoSet.Init({GND_SERVO_OFFSET_S1, GND_SERVO_OFFSET_S2}, TENTH_DEGREE, false);
        FCDevices.dev_GPS.Init(&hi2c4);

        if (imuSt.A != 0 || /* imuSt.ISM != 0 || */ baroSt.A != 0) {
            SD_LogNewline("FC INIT: sensor init failed, retrying");
            osDelay(1000);
            continue;
        }

        if (FCDevices.dev_telemetry.Init(TELEMETRY_MODE_FLIGHT) != 0) {
            SD_LogNewline("FC INIT: radio init failed, retrying");
            osDelay(1000);
            continue;
        }

        break;  // all devices healthy
    }

    // ── Phase 2: Handshake — wait for GND byte, echo back FC byte ────────────
    // CommandResponseByte is set BEFORE the loop so that the very cycle FC
    // receives HANDSHAKE_GND_BYTE it immediately responds with HANDSHAKE_FC_BYTE
    // inside the same Update() call. No second Update() is needed after the break
    // (that would call receiveCommands() first, blocking while GND waits — deadlock).
    // osDelay removed: FC must keep HAL_UART_Receive running continuously;
    // GndStationData (20 bytes) > UART FIFO (16 bytes) so any gap causes overflow.
    FCDevices.dev_telemetry.HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;
    for (;;) {
        FCDevices.dev_telemetry.Update();   // receive GND cmd, send HANDSHAKE_FC_BYTE
        if (FCDevices.dev_telemetry.GNDOutData.CommandByteIn == HANDSHAKE_GND_BYTE)
            break;
    }
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    radioConnected = true;

    // ── Phase 3: Sensor sync — establish launch altitude reference ────────────
    // launchAltM is set from the first UpdateSensors() call so baseline is valid
    // even if the sync loop does not reach 10 successes.
    UpdateSensors();
    // Use the raw TMR baro reading here — filteredAltM is 0 until UpdateVerticalVelocity runs
    launchAltM        = FCData.dat_Barometers.Filtered.heightMeters;
    prevAltM          = launchAltM;
    prevAltTimeMs     = millis();
    lastAbortUpdateMs = millis();

    uint32_t syncStart   = millis();
    int      successCount = 0;
    while (millis() - syncStart < 5000) {
        IMUsStatus syncImuSt  = FCDevices.dev_IMU_Engine.Update();
        auto       syncBaroSt = FCDevices.dev_BarometerEngine.Update();
        FCData.dat_Barometers = FCDevices.dev_BarometerEngine.getData();
        if (syncImuSt.A == 0 && syncBaroSt.A == 0) {
            if (++successCount >= 10)
                break;
        }
        osDelay(10);
    }

    if (successCount < 10)
        SD_LogNewline("FC INIT: sensor sync incomplete, proceeding");

    initDone = true;
    return STATUS_OK;
}

int8_t FlightComputer::Update()
{
    if (!initDone) return STATUS_INIT_FAILURE;

    UpdateTelemetry();
    UpdateAbortAccumulator();

    // acquire mutex to sample independently. if sampling, dont use sensor data yet
    osMutexAcquire(g_ctrls_sensor_mutex, osWaitForever);
    UpdateSensors();
    UpdateVerticalVelocity();
    osMutexRelease(g_ctrls_sensor_mutex);

    TrackCONOPS();
    UpdatePyroTrack();
    UpdateLogging();

    return STATUS_OK;
}

int8_t FlightComputer::UpdateAbortAccumulator()
{
    bool     signalActive = (FCData.dat_GND_Data.CommandByteIn == SHUTDOWN_KEEPALIVE);
    uint32_t now          = millis();
    uint32_t dt           = now - lastAbortUpdateMs;
    lastAbortUpdateMs     = now;

    if (signalActive) {
        abortDropoutMs = 0;
        abortAccumMs  += dt;
    } else {
        abortDropoutMs += dt;
        if (abortDropoutMs > ABORT_DROPOUT_MS) abortAccumMs = 0;
    }

    if (abortAccumMs >= ABORT_ACCUM_MS) Abort();   // does not return
    return STATUS_OK;
}

int8_t FlightComputer::UpdateSensors()
{
    IMUsStatus imuSt  = FCDevices.dev_IMU_Engine.Update();
    auto       baroSt = FCDevices.dev_BarometerEngine.Update();
    FCDevices.dev_GPS.update();
    // FCDevices.dev_servoSet.Update(0.0f, 0.0f); // Done in TrackCONOPS()

    FCData.dat_BMI_IMUs   = FCDevices.dev_IMU_Engine.getVotedBMI();
    FCData.dat_Barometers = FCDevices.dev_BarometerEngine.getData();

    FCData.dat_Servos     = FCDevices.dev_servoSet.getData();
    FCData.dat_GPS        = FCDevices.dev_GPS.getData();

    if (imuSt.A  != 0 || imuSt.B  != 0 ||
        imuSt.C  != 0 || /* imuSt.ISM != 0 || */ baroSt.A != 0) {
        return STATUS_LOCAL_READ_ERR;
    }
    return STATUS_OK;
}

void FlightComputer::UpdateVerticalVelocity()
{
    float rawAlt = FCData.dat_Barometers.Filtered.heightMeters;

    // Median-of-3 spike filter: store 3 samples, return the middle value.
    // A single-point baro glitch is completely rejected with no lag on valid data.
    // Cost: 3 comparisons + temp swaps — negligible on Cortex-M7.
    altFilterBuf[altFilterIdx] = rawAlt;
    altFilterIdx = (altFilterIdx + 1) % 3;

    float a = altFilterBuf[0], b = altFilterBuf[1], c = altFilterBuf[2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    filteredAltM = b;  // median — feeds all altitude-based decisions

    uint32_t now = millis();
    uint32_t dt  = now - prevAltTimeMs;
    if (dt > 0) {
        vertVelocityMs = (filteredAltM - prevAltM) / (dt / 1000.0f);
    }
    prevAltM      = filteredAltM;
    prevAltTimeMs = now;
}

int8_t FlightComputer::UpdateTelemetry()
{
    // Always send HANDSHAKE_FC_BYTE as a continuous heartbeat so GND can track
    // link health without needing a separate reconnection exchange.
    telemetryData &tx = FCDevices.dev_telemetry.HALOutData;

    tx.CommandResponseByte   = HANDSHAKE_FC_BYTE;

    // Barometric altitude (spike-filtered)
    tx.altitude              = filteredAltM;

    // Primary IMU (BMI sensor 0) — linear acceleration + angular velocity
    tx.mAccX                 = FCData.dat_BMI_IMUs.accel_linear.x;
    tx.mAccY                 = FCData.dat_BMI_IMUs.accel_linear.y;
    tx.mAccZ                 = FCData.dat_BMI_IMUs.accel_linear.z;
    tx.mGyrX                 = FCData.dat_BMI_IMUs.ang_vel.x;
    tx.mGyrY                 = FCData.dat_BMI_IMUs.ang_vel.y;
    tx.mGyrZ                 = FCData.dat_BMI_IMUs.ang_vel.z;

    // Backup IMU (BMI sensor 1)
    BMI_Data bmi1            = FCDevices.dev_IMU_Engine.getRawBMI(1);
    tx.bAccX                 = bmi1.accel_linear.x;
    tx.bAccY                 = bmi1.accel_linear.y;
    tx.bAccZ                 = bmi1.accel_linear.z;

    // GPS (float precision sufficient for telemetry; full doubles kept in FCData.dat_GPS)
    tx.latitude              = (float)FCData.dat_GPS.latitude;
    tx.longitude             = (float)FCData.dat_GPS.longitude;
    tx.GPSaltitude           = (float)FCData.dat_GPS.altitude;

    // Servos — commanded target angle and encoder-read current angle
    tx.servoTarget1          = FCData.dat_Servos.targetAngle.S1;
    tx.servoTarget2          = FCData.dat_Servos.targetAngle.S2;
    tx.servoPos1             = FCData.dat_Servos.currentAngle.S1;
    tx.servoPos2             = FCData.dat_Servos.currentAngle.S2;

    // Pyro state
    tx.pyroMainDrogueFired   = pyroMainDrogueFired;
    tx.pyroBackupDrogueFired = pyroBackupDrogueFired;
    tx.pyroMainChuteFired    = pyroMainChuteFired;

    if (FCDevices.dev_telemetry.Update() != 0) {
        if (commsErrCount < 127) commsErrCount++;
        if (commsErrCount == COMMS_ERR_THRESHOLD && radioConnected) {
            radioConnected = false;
            SD_LogNewline("COMMS LOST");
        }
        return STATUS_REMOTE_READ_ERR;
    }

    // Any successful receive = link alive. Abort detection reads CommandByteIn
    // independently in UpdateAbortAccumulator() — no byte check needed here.
    commsErrCount = 0;
    FCData.dat_FC_Data  = FCDevices.dev_telemetry.HALOutData;   // mirror outgoing packet so debugger can inspect it
    FCData.dat_GND_Data = FCDevices.dev_telemetry.GNDOutData;

    // Servo trim: apply zero-point offsets when GND broadcasts SERVO_OFFSET_CMD_BYTE.
    // Log once on first application; idempotent so subsequent identical packets are harmless.
    if (FCData.dat_GND_Data.CommandByteIn == SERVO_OFFSET_CMD_BYTE) {
        FCDevices.dev_servoSet.SetOffset({
            FCData.dat_GND_Data.servoOffset1,
            FCData.dat_GND_Data.servoOffset2
        });
        if (!servoOffsetApplied) {
            servoOffsetApplied = true;
            char trimBuf[80];
            snprintf(trimBuf, sizeof(trimBuf),
                "FC: servo offset applied S1=%.2f S2=%.2f",
                FCData.dat_GND_Data.servoOffset1,
                FCData.dat_GND_Data.servoOffset2);
            SD_LogNewline(trimBuf);
        }
    }

    if (!radioConnected) {
        radioConnected = true;
        SD_LogNewline("COMMS RECONNECTED");
    }

    return STATUS_OK;
}

int8_t FlightComputer::TrackCONOPS()
{
    switch (currentStage) {

        case PREFLIGHT:
            //FCDevices.dev_servoSet.SetOffset({FCData.dat_GND_Data.servoOffset1, FCData.dat_GND_Data.servoOffset2});
            FCDevices.dev_servoSet.Update(0.0f, 0.0f);   // hold neutral so GND offset is live
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() > LIFTOFF_ACCEL_G) {
                if (liftoffDetectMs == 0) liftoffDetectMs = millis();
                if ((millis() - liftoffDetectMs > LIFTOFF_SUSTAIN_MS) &&
                    (filteredAltM > launchAltM + LIFTOFF_ALT_DELTA_M)) {
                    currentStage = ASCENT;
                    SD_LogNewline("STAGE: ASCENT");
                }
            } else {
                liftoffDetectMs = 0;
            }
            break;

        case ASCENT:
            // TODO This needs to account for drag!!!!!
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() < BURNOUT_ACCEL_G) {
                currentStage = CONTROLS_TEST;
                SD_LogNewline("STAGE: CONTROLS_TEST");
            }
            break;

        case CONTROLS_TEST:
        {
            // Non-blocking servo sequence: 8° → -2° → 5° → repeat, 1.5 s per step.
            // Runs until APOGEE_APPROACH transition, then servos zero.
            static const float    ctrlAngles[3] = { 8.0f, -2.0f, 5.0f };
            static const uint32_t ctrlHoldMs    = 1500;

            if (ctrlTestStepStartMs == 0)
                ctrlTestStepStartMs = millis();

            if (millis() - ctrlTestStepStartMs >= ctrlHoldMs) {
                ctrlTestStep        = (ctrlTestStep + 1) % 3;
                ctrlTestStepStartMs = millis();
            }

            FCDevices.dev_servoSet.Update(ctrlAngles[ctrlTestStep],
                                          ctrlAngles[ctrlTestStep]);

            if (vertVelocityMs < APOGEE_APPROACH_VEL_MS) {
                currentStage = APOGEE_APPROACH;
                FCDevices.dev_servoSet.Update(0.0f, 0.0f);
                SD_LogNewline("STAGE: APOGEE_APPROACH");
            }
            break;
        }

        case APOGEE_APPROACH:
            if (vertVelocityMs < APOGEE_VEL_MS &&
                filteredAltM > launchAltM + MIN_APOGEE_ALT_M) {
                currentStage = APOGEE;
                SD_LogNewline("STAGE: APOGEE");
            }
            break;

        case APOGEE:
            if (vertVelocityMs < APOGEE_PASS_VEL_MS) {
                currentStage = APOGEE_PASS;
                SD_LogNewline("STAGE: APOGEE_PASS");
            }
            break;

        case APOGEE_PASS:
            if (vertVelocityMs < DESCENT_VEL_MS) {
                currentStage = DESCENT;
                SD_LogNewline("STAGE: DESCENT");
            }
            break;

        case DESCENT:
            if (filteredAltM < launchAltM + MAX_MAIN_DEPLOY_ALT_M &&
                filteredAltM > launchAltM + MIN_MAIN_DEPLOY_ALT_M) {
                currentStage = MAIN_APPROACH;
                SD_LogNewline("STAGE: MAIN_APPROACH");
            }
            break;

        case MAIN_APPROACH:
            if (filteredAltM <= launchAltM + TARGET_MAIN_ALT_M) {
                currentStage = MAIN;
                SD_LogNewline("STAGE: MAIN");
            }
            break;

        case MAIN:
            if (vertVelocityMs > FINAL_DESCENT_VEL_MS) {
                currentStage = FINAL_DESCENT;
                SD_LogNewline("STAGE: FINAL_DESCENT");
            }
            break;

        case FINAL_DESCENT:
            break;
    }
    return STATUS_OK;
}

int8_t FlightComputer::UpdatePyroTrack()
{
    if (currentStage == APOGEE && !pyroMainDrogueFired) {
        firePyroMainDrogue();
        pyroMainDrogueFired = true;
    }

    // Backup drogue fires unconditionally at APOGEE_PASS for maximum redundancy
    if (currentStage == APOGEE_PASS && !pyroBackupDrogueFired) {
        firePyroBackupDrogue();
        pyroBackupDrogueFired = true;
    }

    if (currentStage == MAIN && !pyroMainChuteFired) {
        firePyroMainParachute();
        pyroMainChuteFired = true;
    }

    uint32_t now = millis();
    if (pyroMainDrogueArmMs && now - pyroMainDrogueArmMs >= PYRO_PULSE_MS) {
        HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);
        SD_LogNewline("PYRO DROGUE_MAIN OFF");
        pyroMainDrogueArmMs = 0;
    }

    if (pyroBackupDrogueArmMs && now - pyroBackupDrogueArmMs >= PYRO_PULSE_MS) {
        HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_RESET);
        SD_LogNewline("PYRO DROGUE_BACKUP OFF");
        pyroBackupDrogueArmMs = 0;
    }

    if (pyroMainChuteArmMs && now - pyroMainChuteArmMs >= PYRO_PULSE_MS) {
        HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);
        SD_LogNewline("PYRO MAIN_CHUTE OFF");
        pyroMainChuteArmMs = 0;
    }

    return STATUS_OK;
}

int8_t FlightComputer::UpdateLogging()
{
    if (millis() - lastLogMs < 100) return STATUS_OK;  // 10 Hz
    lastLogMs = millis();

    static const char* const stageNames[] = {
        "PRE", "ASC", "CTL", "APR", "APO", "APS", "DES", "MAR", "MAI", "FIN"
    };

    char buf[160];

    // IMU line: timestamp, stage, altitude, vertical velocity, acceleration, angular velocity
    snprintf(buf, sizeof(buf),
        "T=%lu S=%s ALT=%.1f VV=%.2f AX=%.2f AY=%.2f AZ=%.2f GX=%.2f GY=%.2f GZ=%.2f",
        static_cast<unsigned long>(millis()),
        stageNames[static_cast<int>(currentStage)],
        filteredAltM,
        vertVelocityMs,
        FCData.dat_BMI_IMUs.accel_linear.x,
        FCData.dat_BMI_IMUs.accel_linear.y,
        FCData.dat_BMI_IMUs.accel_linear.z,
        FCData.dat_BMI_IMUs.ang_vel.x,
        FCData.dat_BMI_IMUs.ang_vel.y,
        FCData.dat_BMI_IMUs.ang_vel.z);
    SD_LogNewline(buf);

    // Servo line: commanded target angle and encoder-read current angle
    char srvBuf[80];
    snprintf(srvBuf, sizeof(srvBuf),
        "SRV TGT=%.2f,%.2f POS=%.2f,%.2f",
        FCData.dat_Servos.targetAngle.S1,
        FCData.dat_Servos.targetAngle.S2,
        FCData.dat_Servos.currentAngle.S1,
        FCData.dat_Servos.currentAngle.S2);
    SD_LogNewline(srvBuf);

    // GPS line (always written; coordinates = 0,0 until fix acquired)
    char gpsBuf[96];
    snprintf(gpsBuf, sizeof(gpsBuf),
        "GPS SAT=%u FIX=%d LAT=%.6f LON=%.6f ALT=%.1f T=%02u:%02u:%02u",
        (unsigned)FCData.dat_GPS.satellites,
        (int)FCData.dat_GPS.hasFix,
        FCData.dat_GPS.latitude,
        FCData.dat_GPS.longitude,
        FCData.dat_GPS.altitude,
        (unsigned)FCData.dat_GPS.hour,
        (unsigned)FCData.dat_GPS.minute,
        (unsigned)FCData.dat_GPS.second);
    SD_LogNewline(gpsBuf);

    return STATUS_OK;
}

void FlightComputer::firePyroMainDrogue()
{
    char buf[80];
    snprintf(buf, sizeof(buf), "PYRO DROGUE_MAIN FIRE T=%lu ALT=%.1f",
             static_cast<unsigned long>(millis()), filteredAltM);
    SD_LogNewline(buf);                                             // log before firing — survives a crash
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
    SD_LogNewline("PYRO DROGUE_MAIN OFF");
    pyroMainDrogueArmMs = millis();
}

void FlightComputer::firePyroBackupDrogue()
{
    char buf[80];
    snprintf(buf, sizeof(buf), "PYRO DROGUE_BACKUP FIRE T=%lu ALT=%.1f",
             static_cast<unsigned long>(millis()), filteredAltM);
    SD_LogNewline(buf);
    HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_SET);
    SD_LogNewline("PYRO DROGUE_BACKUP OFF");
    pyroBackupDrogueArmMs = millis();
}

void FlightComputer::firePyroMainParachute()
{
    char buf[80];
    snprintf(buf, sizeof(buf), "PYRO MAIN_CHUTE FIRE T=%lu ALT=%.1f",
             static_cast<unsigned long>(millis()), filteredAltM);
    SD_LogNewline(buf);
    HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);
    SD_LogNewline("PYRO MAIN_CHUTE OFF");
    pyroMainChuteArmMs = millis();
}

int8_t FlightComputer::Abort()
{
    FCDevices.dev_servoSet.Update(0.0f, 0.0f);
    abortLatched = true;
    SD_LogNewline("ABORT");

    char buf[80];
    for (;;) {
        UpdateSensors();
        UpdateVerticalVelocity();
        TrackCONOPS();
        FCDevices.dev_servoSet.Update(0.0f, 0.0f);
        UpdatePyroTrack();
        UpdateTelemetry();

        if (!pyroMainDrogueFired && vertVelocityMs < 0.0f) {
            SD_LogNewline("ABORT: drogue fallback (descending, not yet deployed)");
            HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
            pyroMainDrogueArmMs = millis();
            pyroMainDrogueFired = true;
            currentStage = APOGEE_PASS;
        }

        snprintf(buf, sizeof(buf),
            "T=%lu ABORT ALT=%.1f VEL=%.2f",
            static_cast<unsigned long>(millis()),
            filteredAltM,
            vertVelocityMs);
        SD_LogNewline(buf);

        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        osDelay(1);
    }
}
