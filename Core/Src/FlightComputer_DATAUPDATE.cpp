//
// Created by admin on 5/19/2026.
//

#include "FlightComputer_DATAUPDATE.h"


extern telemetryData g_telemNow;
extern telemetryData g_telemPrev;

float g_dt = 0.0f;

void DataUpdate::ComputeDt()
{
    static uint32_t lastMicros = 0;
    uint32_t now = micros();
    g_dt = (lastMicros == 0) ? 0.0f : static_cast<float>(now - lastMicros) * 1e-6f;
    lastMicros = now;
}

void DataUpdate::FuseQuat(Quaternion* QObj)
{
    // Drive the filter at the actual measured dt so integration is accurate.
    // Falls back to 1 kHz on the first call (g_dt == 0) before ComputeDt has run.
    if (g_dt > 0.0f)
        QObj->sampleFreq = 1.0f / g_dt;

    static constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    Vector3D<float> a = Vector3D(g_telemNow.mAccX, g_telemNow.mAccY, g_telemNow.mAccZ);
    // g_telemNow gyro is in deg/s (raw from BMI); Madgwick integrates in rad/s.
    Vector3D<float> w = Vector3D(g_telemNow.mGyrX * DEG_TO_RAD,
                                  g_telemNow.mGyrY * DEG_TO_RAD,
                                  g_telemNow.mGyrZ * DEG_TO_RAD);
    MATHEMATICS::Quaternion_Madgwick(QObj, a, w);
    g_telemNow.Qw = QObj->q.w;
    g_telemNow.Qx = QObj->q.x;
    g_telemNow.Qy = QObj->q.y;
    g_telemNow.Qz = QObj->q.z;
}

// Returns yaw, pitch, roll, all in degrees
void DataUpdate::QuatToYPR(Q* quaternionIn)
{
    float w = quaternionIn->w;
    float x = quaternionIn->x;
    float y = quaternionIn->y;
    float z = quaternionIn->z;

    // Clamp the asinf argument to [-1, 1] to prevent NaN from floating-point
    // rounding on a near-unit quaternion (would poison all three outputs).
    float sinp = 2.0f * (w*y - z*x);
    if (sinp >  1.0f) sinp =  1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    float roll  = atan2f(2.0f * (w*x + y*z), 1.0f - 2.0f * (x*x + y*y));
    float pitch = asinf(sinp);
    float yaw   = atan2f(2.0f * (w*z + x*y), 1.0f - 2.0f * (y*y + z*z));

    Vector3D<float> deg = Vector3D(yaw, pitch, roll) * (180.0f / static_cast<float>(M_PI));
    g_telemNow.yaw      = roundf(deg.x * 100.0f) / 100.0f;
    g_telemNow.pitch    = roundf(deg.y * 100.0f) / 100.0f;
    g_telemNow.roll     = roundf(deg.z * 100.0f) / 100.0f;
}

float DataUpdate::getVerticalVelocity()
{
    if (g_dt <= 0.0f) return 0.0f;
    return (g_telemNow.altitude - g_telemPrev.altitude) / g_dt;
}