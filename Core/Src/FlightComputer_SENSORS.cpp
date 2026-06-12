//
// Created by bzhan on 5/15/2026.
//
#include "FlightComputer_SENSORS.h"
#include "i2c.h"
#include "CTRLS_Controls.h"
#include "constants.h"

// GLOBAL SENSOR DATA (DEFINED HERE) [We can just directly make these vars here]
telemetryData     g_telemNow;
telemetryData     g_telemPrev;
GndStationData    g_gndData;
BMI_Data          g_BMI;
BMP_Data          g_BMP;
GPS_Data          g_GPS;
ServoAngles       g_servoAngles;


Sensors::Sensors() : mag(&hspi1) {}

int8_t Sensors::Init()
{
    int8_t result = STATUS_OK;

    IMUsStatus imuSt = IMU.Init(true);
    if (imuSt.A != 0 || imuSt.B != 0 || imuSt.C != 0)
        result = STATUS_SENSOR_FAIL;

    auto baroSt = Baro.Init(true);
    if (baroSt.A != 0 || baroSt.B != 0 || baroSt.C != 0)
        result = STATUS_SENSOR_FAIL;

    if (GPS.Init(&hi2c4) != 0)
        result = STATUS_SENSOR_FAIL;

    bool servoOk = Servos.Init({g_gndData.servoOffset1, g_gndData.servoOffset2},
                                PRECISION::TENTH_DEGREE, false);
    //if (!servoOk)
    //result = STATUS_SENSOR_FAIL;

    if (mag.Init() != 0)
        //result = STATUS_SENSOR_FAIL;

    if (result == STATUS_OK) {
        g_telemNow  = {0};
        g_telemPrev = {0};
    }

    return result;
}

extern bool SensorsUpdated;

int8_t Sensors::Update()
{
    IMU.Update();
    Baro.Update();
    GPS.update();

    // Servos' positions are updated in control algo
    Servos.SetOffset({g_gndData.servoOffset1, g_gndData.servoOffset2});

    SAsym<float> c_Servos = Servos.getCurrentAngle();
    MAXM10S::gpsData c_GPS = GPS.getData();
    g_BMI = IMU.getVotedBMI();
    g_BMP = Baro.getData().Filtered;

    if (c_GPS.satellites > 0)
        vTaskDelay(1);

    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;
    static uint32_t liftoffDetectMs = 0U;
    static uint32_t liftoffMs       = 0U;

    const uint32_t nowMs = HAL_GetTick();
    if (liftoffMs == 0U) {
        if (g_BMI.accel_linear.magnitude() > LIFTOFF_ACCEL_G) {
            if (liftoffDetectMs == 0U) liftoffDetectMs = nowMs;
            if (static_cast<uint32_t>(nowMs - liftoffDetectMs) > LIFTOFF_SUSTAIN_MS) {
                liftoffMs = liftoffDetectMs;
            }
        } else {
            liftoffDetectMs = 0U;
        }
    }

    const float flightTimeS =
        (liftoffMs == 0U) ? 0.0f : static_cast<float>(nowMs - liftoffMs) * 1.0e-3f;

    // Sensors mutex is acquired when updating g_SensorData and g_telemNow.
    // The return value MUST be checked: releasing a mutex you don't own corrupts
    // FreeRTOS internal state and triggers configASSERT.
    if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {

        g_telemNow.latitude    = c_GPS.latitude;
        g_telemNow.longitude   = c_GPS.longitude;
        g_telemNow.GPSaltitude = c_GPS.altitude;
        g_GPS.latitude         = c_GPS.latitude;
        g_GPS.longitude        = c_GPS.longitude;
        g_GPS.altitude         = c_GPS.altitude;
        g_GPS.hour             = c_GPS.hour;
        g_GPS.minute           = c_GPS.minute;
        g_GPS.second           = c_GPS.second;
        g_GPS.satellites       = c_GPS.satellites;
        g_GPS.hdop             = c_GPS.hdop;
        g_telemNow.altitude    = g_BMP.heightMeters;  // absolute height — delta computed in getVerticalVelocity
        g_telemNow.temperature = g_BMP.Temperature;

        // NOTE THAT Y AND Z ARE FLIPPED
        // Y points upwards in the IMU, but the controls algorithm expects that
        // Z is upwards.
        g_telemNow.mAccX = g_BMI.accel_linear.x;
        g_telemNow.mAccY = g_BMI.accel_linear.z;
        g_telemNow.mAccZ = g_BMI.accel_linear.y;
        g_telemNow.mGyrX = g_BMI.ang_vel.x;
        g_telemNow.mGyrY = g_BMI.ang_vel.z;
        g_telemNow.mGyrZ = g_BMI.ang_vel.y;
        g_telemNow.servoPos1 = c_Servos.S1;
        g_telemNow.servoPos2 = c_Servos.S2;

        // Populate controls sensor snapshot (same axes/units as g_telemNow above).
        // ang_vel from BMI is in deg/s — EKF expects rad/s.
        g_SensorData.accel_g[0]    = g_BMI.accel_linear.x;
        g_SensorData.accel_g[1]    = g_BMI.accel_linear.z;
        g_SensorData.accel_g[2]    = g_BMI.accel_linear.y;
        g_SensorData.gyro_rad_s[0] = g_BMI.ang_vel.x * DEG_TO_RAD;
        g_SensorData.gyro_rad_s[1] = g_BMI.ang_vel.z * DEG_TO_RAD;
        g_SensorData.gyro_rad_s[2] = g_BMI.ang_vel.y * DEG_TO_RAD;
        g_SensorData.altitude_m    = g_BMP.heightMeters;
        // Kept for telemetry/diagnostics; controls use weather-profile temperature.
        g_SensorData.temperature_K = g_BMP.Temperature + 273.15f;
        g_SensorData.flight_time_s = flightTimeS;
        g_SensorData.timestamp_ms  = HAL_GetTick();
        g_SensorData.fresh         = true;

        osMutexRelease(g_ctrls_sensor_mutex);
    } // osMutexAcquire

    return STATUS_OK;
}

// ============= PRIVATE FUNCS ============
