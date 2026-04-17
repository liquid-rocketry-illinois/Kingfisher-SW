//
// Created by dyrel on 4/13/2026.
//

#include "Flight_Procedures.h"
#include "timing.h"
#include "cmsis_os2.h"
#include "main.h"
#include "i2c.h"
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
        if (!handshakeOK) continue;   // retry — FC not online yet

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
    if (GNDDevices.dev_telemetry.Update() != 0) return STATUS_REMOTE_READ_ERR;
    GNDData.dat_FC_Data = GNDDevices.dev_telemetry.HALOutData;
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

    char buf[160];
    snprintf(buf, sizeof(buf),
        "T=%lu FC ALT=%.1f ACC=%.2f,%.2f,%.2f SRV=%.1f,%.1f PYRO=%d%d%d",
        (unsigned long)millis(),
        GNDData.dat_FC_Data.altitude,
        GNDData.dat_FC_Data.mAccX,
        GNDData.dat_FC_Data.mAccY,
        GNDData.dat_FC_Data.mAccZ,
        GNDData.dat_FC_Data.servoPos1,
        GNDData.dat_FC_Data.servoPos2,
        (int)GNDData.dat_FC_Data.pyroMainDrogueFired,
        (int)GNDData.dat_FC_Data.pyroBackupDrogueFired,
        (int)GNDData.dat_FC_Data.pyroMainChuteFired);
    SD_LogNewline(buf);
    return STATUS_OK;
}


// ════════════════════════════════════════════════════════════════════════════
//  FLIGHT COMPUTER
// ════════════════════════════════════════════════════════════════════════════

FlightComputer::FlightComputer() : FCDevices(), FCData() {}

int8_t FlightComputer::Init()
{
    for (;;) {

        // ── SD card ───────────────────────────────────────────────────────────
        if (SD_Init() != 0) {
            osDelay(1000);
            continue;
        }

        // ── Sensor init ───────────────────────────────────────────────────────
        auto imuSt  = FCDevices.dev_IMU_Engine.Init(true);
        auto baroSt = FCDevices.dev_BarometerEngine.Init(true);
        FCDevices.dev_servoSet.Init({0.0f, 0.0f}, TENTH_DEGREE, false);
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

        // ── Handshake: wait up to 5 s for GND byte, echo back FC byte ────────
        // No delay on retry — FC simply listens again immediately so GND can
        // come online at any time without both units being activated together.
        uint32_t t0 = millis();
        bool handshakeOK = false;
        while (millis() - t0 < 5000) {
            FCDevices.dev_telemetry.Update();
            if (FCDevices.dev_telemetry.GNDOutData.CommandByteIn == HANDSHAKE_GND_BYTE) {
                handshakeOK = true;
                break;
            }
            osDelay(10);
        }
        if (!handshakeOK) continue;   // retry — GND not online yet

        FCDevices.dev_telemetry.HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;
        FCDevices.dev_telemetry.Update();   // transmit response

        // ── Sensor sync: establish launch altitude reference ──────────────────
        UpdateSensors();
        launchAltM        = FCData.dat_Barometers.Filtered.heightMeters;
        prevAltM          = launchAltM;
        prevAltTimeMs     = millis();
        lastAbortUpdateMs = millis();

        uint32_t syncStart = millis();
        int successCount = 0;
        while (millis() - syncStart < 2000) {
            if (UpdateSensors() == STATUS_OK) {
                UpdateTelemetry();
                if (++successCount >= 10) {
                    initDone = true;
                    return STATUS_OK;
                }
            }
            osDelay(10);
        }
        SD_LogNewline("FC INIT: sensor sync failed, retrying");
        osDelay(500);

    } // retry loop
}

int8_t FlightComputer::Update()
{
    if (!initDone) return STATUS_INIT_FAILURE;

    UpdateTelemetry();
    UpdateAbortAccumulator();
    UpdateSensors();
    UpdateVerticalVelocity();
    TrackCONOPS();
    UpdatePyroTrack();

    if (commsErrCount > COMMS_ERR_THRESHOLD) SD_LogNewline("COMMS WARNING");
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

    FCData.dat_BMI_IMUs   = FCDevices.dev_IMU_Engine.getRawBMI(0);
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
    float    alt = FCData.dat_Barometers.Filtered.heightMeters;
    uint32_t now = millis();
    uint32_t dt  = now - prevAltTimeMs;

    if (dt > 0) {
        vertVelocityMs = (alt - prevAltM) / (dt / 1000.0f);
    }

    prevAltM      = alt;
    prevAltTimeMs = now;
}

int8_t FlightComputer::UpdateTelemetry()
{
    FCDevices.dev_telemetry.HALOutData.altitude              = FCData.dat_Barometers.Filtered.heightMeters;
    FCDevices.dev_telemetry.HALOutData.mAccX                 = FCData.dat_BMI_IMUs.accel_linear.x;
    FCDevices.dev_telemetry.HALOutData.mAccY                 = FCData.dat_BMI_IMUs.accel_linear.y;
    FCDevices.dev_telemetry.HALOutData.mAccZ                 = FCData.dat_BMI_IMUs.accel_linear.z;
    FCDevices.dev_telemetry.HALOutData.servoPos1             = FCData.dat_Servos.currentAngle.S1;
    FCDevices.dev_telemetry.HALOutData.servoPos2             = FCData.dat_Servos.currentAngle.S2;
    FCDevices.dev_telemetry.HALOutData.pyroMainDrogueFired   = pyroMainDrogueFired;
    FCDevices.dev_telemetry.HALOutData.pyroBackupDrogueFired = pyroBackupDrogueFired;
    FCDevices.dev_telemetry.HALOutData.pyroMainChuteFired    = pyroMainChuteFired;

    if (FCDevices.dev_telemetry.Update() != 0) {
        commsErrCount++;
        return STATUS_REMOTE_READ_ERR;
    }
    commsErrCount = 0;
    FCData.dat_GND_Data = FCDevices.dev_telemetry.GNDOutData;
    return STATUS_OK;
}

int8_t FlightComputer::TrackCONOPS()
{
    switch (currentStage) {

        case PREFLIGHT:
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() > LIFTOFF_ACCEL_G) {
                if (liftoffDetectMs == 0) liftoffDetectMs = millis();
                if ((millis() - liftoffDetectMs > LIFTOFF_SUSTAIN_MS) &&
                    (FCData.dat_Barometers.Filtered.heightMeters > launchAltM + LIFTOFF_ALT_DELTA_M)) {
                    currentStage = ASCENT;
                    SD_LogNewline("STAGE: ASCENT");
                }
            } else {
                liftoffDetectMs = 0;
            }
            break;

        case ASCENT:
            if (FCData.dat_BMI_IMUs.accel_linear.magnitude() < BURNOUT_ACCEL_G) {
                currentStage = CONTROLS_TEST;
                SD_LogNewline("STAGE: CONTROLS_TEST");
            }
            break;

        case CONTROLS_TEST:
            if (vertVelocityMs < APOGEE_APPROACH_VEL_MS) {
                currentStage = APOGEE_APPROACH;
                SD_LogNewline("STAGE: APOGEE_APPROACH");
            }
            break;

        case APOGEE_APPROACH:
            if (vertVelocityMs < APOGEE_VEL_MS &&
                FCData.dat_Barometers.Filtered.heightMeters > launchAltM + MIN_APOGEE_ALT_M) {
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
            if (FCData.dat_Barometers.Filtered.heightMeters < launchAltM + MAX_MAIN_DEPLOY_ALT_M &&
                FCData.dat_Barometers.Filtered.heightMeters > launchAltM + MIN_MAIN_DEPLOY_ALT_M) {
                currentStage = MAIN_APPROACH;
                SD_LogNewline("STAGE: MAIN_APPROACH");
            }
            break;

        case MAIN_APPROACH:
            if (FCData.dat_Barometers.Filtered.heightMeters <= launchAltM + TARGET_MAIN_ALT_M) {
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

    return STATUS_OK;
}

void FlightComputer::firePyroMainDrogue()
{
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
    osDelay(2000);
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);
}

void FlightComputer::firePyroBackupDrogue()
{
    HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_SET);
    osDelay(2000);
    HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_RESET);
}

void FlightComputer::firePyroMainParachute()
{
    HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);
    osDelay(2000);
    HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);
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
        UpdatePyroTrack();
        UpdateTelemetry();

        snprintf(buf, sizeof(buf),
            "T=%lu ABORT ALT=%.1f VEL=%.2f",
            (unsigned long)millis(),
            FCData.dat_Barometers.Filtered.heightMeters,
            vertVelocityMs);
        SD_LogNewline(buf);

        osDelay(10);
    }
}
