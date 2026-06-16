//
// Created by bzhan on 6/1/2026.
//

#include "DataFilter.h"
#include "Telemetry.h"
#include "CTRLS_Controls.h"

extern telemetryData       g_telemNow;
extern CtrlsSensorSnapshot g_SensorData;

SensorFilters::SensorFilters()
    : pressureFilter(0.1f),
      accelFilter   (0.5f),
      gyroFilter    (0.5f)
{}

SensorFilters g_filters;

void SensorFilters::apply()
{
    // ── Altitude / pressure ───────────────────────────────────────────────────
    g_telemNow.altitude      = pressureFilter.update(g_telemNow.altitude);
    g_SensorData.altitude_m  = g_telemNow.altitude;

    // ── Acceleration (g units, Y/Z axis swap already applied by Sensors::Update) ─
    accelFilter.update(g_telemNow.mAccX, g_telemNow.mAccY, g_telemNow.mAccZ);
    g_SensorData.accel_g[0] = g_telemNow.mAccX;
    g_SensorData.accel_g[1] = g_telemNow.mAccY;
    g_SensorData.accel_g[2] = g_telemNow.mAccZ;

    // ── Gyroscope (deg/s in g_telemNow; EKF snapshot expects rad/s) ───────────
    gyroFilter.update(g_telemNow.mGyrX, g_telemNow.mGyrY, g_telemNow.mGyrZ);
    g_SensorData.gyro_rad_s[0] = g_telemNow.mGyrX * DEG_TO_RAD;
    g_SensorData.gyro_rad_s[1] = g_telemNow.mGyrY * DEG_TO_RAD;
    g_SensorData.gyro_rad_s[2] = g_telemNow.mGyrZ * DEG_TO_RAD;
}