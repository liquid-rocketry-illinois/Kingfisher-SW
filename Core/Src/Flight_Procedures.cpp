//
// Created by dyrel on 4/13/2026.
//

#include "Flight_Procedures.h"
#include "timing.h"
#include "cmsis_os2.h"
#include "main.h"
#include <cstdio>
#include <cmath>

// ════════════════════════════════════════════════════════════════════════════
//  GROUND STATION
// ════════════════════════════════════════════════════════════════════════════

GroundStation::GroundStation() : GNDDevices(), GNDData() {}

int8_t GroundStation::Init()
{
    bool anyFail = false;

    if (SD_Init() != 0)                                                     anyFail = true;
    if (GNDDevices.dev_telemetry.Init(TELEMETRY_MODE_GROUND) != 0)         anyFail = true;
    if (GNDDevices.dev_BarometerEngine.Init(true).A != 0)                   anyFail = true;
    if (GNDDevices.dev_IMU_Engine.Init(true).A != 0)                        anyFail = true;
    if (!GNDDevices.dev_servoSet.Init({0.0f, 0.0f}, TENTH_DEGREE, false))   anyFail = true;

    if (anyFail) return STATUS_INIT_FAILURE;

    // ── Handshake: send GND byte, wait up to 2 s for FC echo ─────────────────
    GNDData.dat_GND_Data.CommandByteIn = HANDSHAKE_GND_BYTE;
    uint32_t t0 = millis();
    bool handshakeOK = false;
    while (millis() - t0 < 2000) {
        UpdateTelemetry();
        if (GNDData.dat_FC_Data.CommandResponseByte == HANDSHAKE_FC_BYTE) {
            handshakeOK = true;
            break;
        }
        osDelay(10);
    }
    if (!handshakeOK) return STATUS_COMMS_FAILURE;

    // ── Local sensor verification ─────────────────────────────────────────────
    for (int i = 0; i < 5; i++) {
        if (UpdateSensors() != STATUS_OK) return STATUS_LOCAL_READ_ERR;
    }

    BARO_DATA baro = GNDDevices.dev_BarometerEngine.getData();
    if (baro.Filtered.heightMeters < BARO_ALT_MIN_M ||
        baro.Filtered.heightMeters > BARO_ALT_MAX_M ||
        baro.Filtered.Pressure     < BARO_PRESS_MIN_PA ||
        baro.Filtered.Pressure     > BARO_PRESS_MAX_PA) {
        return STATUS_LOCAL_DATA_ERR;
    }

    // ── Wait for FC telemetry (up to 2 s) ────────────────────────────────────
    uint32_t fcWait = millis();
    bool fcDataOK = false;
    while (millis() - fcWait < 2000) {
        UpdateTelemetry();
        if (GNDData.dat_FC_Data.altitude != 0.0f) { fcDataOK = true; break; }
        osDelay(10);
    }
    if (!fcDataOK) return STATUS_TIMEOUT;

    // Cross-check GND vs FC altitude within 5 %
    float gndAlt = baro.Filtered.heightMeters;
    float fcAlt  = GNDData.dat_FC_Data.altitude;
    if (gndAlt != 0.0f) {
        if (fabsf((fcAlt - gndAlt) / gndAlt) > 0.05f) return STATUS_SENSOR_FAIL;
    }

    // ── Servo verification ────────────────────────────────────────────────────
    static const float kTestAngles[] = {
        1.0f, -1.0f, 3.0f, -3.0f, 5.0f, -5.0f,
        10.0f, -10.0f, 20.0f, -20.0f, 30.0f, -30.0f, 45.0f, -45.0f
    };
    const int kCount = (int)(sizeof(kTestAngles) / sizeof(kTestAngles[0]));
    float totalErr = 0.0f, totalCmd = 0.0f;

    for (int i = 0; i < kCount; i++) {
        float a = kTestAngles[i];
        GNDDevices.dev_servoSet.Update(a, a);
        osDelay(200);
        DATA_Axon_Mini_MKII sd = GNDDevices.dev_servoSet.getData();
        totalErr += fabsf(sd.currentAngle.S1 - a) + fabsf(sd.currentAngle.S2 - a);
        totalCmd += 2.0f * fabsf(a);
    }

    if (totalCmd > 0.0f) {
        float errPct = totalErr / totalCmd;
        if (errPct > 0.02f) return STATUS_LOCAL_DATA_ERR;
        if (errPct > 0.01f) SD_LogNewline("SERVO WARN");
    }

    // Return servos to neutral
    GNDDevices.dev_servoSet.Update(0.0f, 0.0f);

    initDone = true;
    return STATUS_OK;
}

int8_t GroundStation::Update()
{
    if (!initDone) return STATUS_INIT_FAILURE;

    if (millis() - lastLocalUpdateMs >= GND_SENSOR_PERIOD_MS) {
        if (UpdateSensors() != STATUS_OK) SD_LogNewline("LOCAL SENSOR WARN");
        pendingLocalLog   = true;
        lastLocalUpdateMs = millis();
    }

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
    auto baroSt = GNDDevices.dev_BarometerEngine.Update();
    auto imuSt  = GNDDevices.dev_IMU_Engine.Update();

    GNDData.dat_Barometers = GNDDevices.dev_BarometerEngine.getData();
    GNDData.dat_BMI_IMUs   = GNDDevices.dev_IMU_Engine.getRawBMI(0);

    if (baroSt.A != 0 || imuSt.A != 0) return STATUS_LOCAL_READ_ERR;
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
    char buf[160];

    if (pendingLocalLog) {
        snprintf(buf, sizeof(buf),
            "T=%lu GND ALT=%.1f PRESS=%.1f ACCEL=%.3f",
            (unsigned long)millis(),
            GNDData.dat_Barometers.Filtered.heightMeters,
            GNDData.dat_Barometers.Filtered.Pressure,
            GNDData.dat_BMI_IMUs.accel_linear.magnitude());
        SD_LogNewline(buf);
        pendingLocalLog = false;
    }

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
    // ── SD card ───────────────────────────────────────────────────────────────
    if (SD_Init() != 0) return STATUS_INIT_FAILURE;

    // ── Sensor init ───────────────────────────────────────────────────────────
    auto imuSt  = FCDevices.dev_IMU_Engine.Init(true);
    auto baroSt = FCDevices.dev_BarometerEngine.Init(true);
    FCDevices.dev_servoSet.Init({0.0f, 0.0f}, TENTH_DEGREE, false);

    if (imuSt.A != 0 || imuSt.ISM != 0 || baroSt.A != 0) return STATUS_INIT_FAILURE;

    if (FCDevices.dev_telemetry.Init(TELEMETRY_MODE_FLIGHT) != 0) return STATUS_COMMS_FAILURE;

    // ── Handshake: wait up to 3 s for GND byte, echo back FC byte ────────────
    uint32_t t0 = millis();
    bool handshakeOK = false;
    while (millis() - t0 < 3000) {
        FCDevices.dev_telemetry.Update();
        if (FCDevices.dev_telemetry.GNDOutData.CommandByteIn == HANDSHAKE_GND_BYTE) {
            handshakeOK = true;
            break;
        }
        osDelay(10);
    }
    if (!handshakeOK) return STATUS_COMMS_FAILURE;

    FCDevices.dev_telemetry.HALOutData.CommandResponseByte = HANDSHAKE_FC_BYTE;
    FCDevices.dev_telemetry.Update();   // transmit response

    // ── Sensor sync: establish launch altitude reference ──────────────────────
    UpdateSensors();
    launchAltM        = FCData.dat_Barometers.Filtered.heightMeters;
    prevAltM          = launchAltM;
    prevAltTimeMs     = millis();
    lastAbortUpdateMs = millis();

    // GndStationData only carries keepAlive + pyro keys — no GND sensor data
    // is transmitted in the current protocol, so we count successful local
    // sensor+telemetry round-trips instead of cross-checking altitudes.
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
    return STATUS_TIMEOUT;
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

    FCData.dat_BMI_IMUs   = FCDevices.dev_IMU_Engine.getRawBMI(0);
    FCData.dat_Barometers = FCDevices.dev_BarometerEngine.getData();
    FCData.dat_Servos     = FCDevices.dev_servoSet.getData();

    if (imuSt.A  != 0 || imuSt.B  != 0 ||
        imuSt.C  != 0 || imuSt.ISM != 0 || baroSt.A != 0) {
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
                FCData.dat_Barometers.Filtered.heightMeters > MIN_APOGEE_ALT_M) {
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
            if (FCData.dat_Barometers.Filtered.heightMeters < MAX_MAIN_DEPLOY_ALT_M &&
                FCData.dat_Barometers.Filtered.heightMeters > MIN_MAIN_DEPLOY_ALT_M) {
                currentStage = MAIN_APPROACH;
                SD_LogNewline("STAGE: MAIN_APPROACH");
            }
            break;

        case MAIN_APPROACH:
            if (FCData.dat_Barometers.Filtered.heightMeters <= TARGET_MAIN_ALT_M) {
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
