//
// Created by admin on 5/19/2026.
//

#include "FlightComputer_DATAUPDATE.h"
#include "constants.h"


extern telemetryData g_telemNow;
extern telemetryData g_telemPrev;

float g_dt = 0.0f;

void DataUpdate::ComputeDt()
{
    // Use raw DWT cycle counts rather than micros().
    //
    // micros() = DWT->CYCCNT / us_f, which wraps at (2^32 / us_f) ≈ 8.95 s
    // at 480 MHz — not at 2^32.  Subtracting two values that wrap at a
    // non-power-of-two boundary with uint32 arithmetic gives a spurious
    // ~4286 s spike every ~8.95 s, which caused the Madgwick integrator to
    // apply a massive rotation step and produce the periodic quaternion jumps.
    //
    // DWT->CYCCNT wraps at exactly 2^32 cycles, so uint32 subtraction is
    // always correct.  We convert to seconds only after the subtraction.
    static uint32_t lastCycles = 0;
    const uint32_t now = DWT->CYCCNT;
    if (lastCycles == 0) {
        g_dt = 0.0f;
    } else {
        const uint32_t elapsed = now - lastCycles;   // correct across 2^32 rollover
        g_dt = static_cast<float>(elapsed) /
               static_cast<float>(HAL_RCC_GetHCLKFreq());
    }
    lastCycles = now;
}

void DataUpdate::FuseAttitude(stateestimation::AttitudeEstimator* est)
{
    if (g_dt <= 0.0f) return;

    static constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;
    static constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

    // FlightComputer_SENSORS already remapped BMI axes so that:
    //   mAccX / mGyrX  = BMI.x  (lateral 1)
    //   mAccY / mGyrY  = BMI.z  (lateral 2)
    //   mAccZ / mGyrZ  = BMI.y  (rocket longitudinal axis, pointing up)
    // Pass these directly: no second remap. The estimator's Z axis = rocket-up.
    const double gx = g_telemNow.mGyrX * DEG_TO_RAD;
    const double gy = g_telemNow.mGyrY * DEG_TO_RAD;
    const double gz = g_telemNow.mGyrZ * DEG_TO_RAD;
    const double ax = g_telemNow.mAccX;
    const double ay = g_telemNow.mAccY;
    const double az = g_telemNow.mAccZ;

    est->update(static_cast<double>(g_dt),
                gx, gy, gz,
                ax, ay, az,
                0.0, 0.0, 0.0);   // no magnetometer

    double q[4];
    est->getAttitude(q);  // q[0]=w, q[1]=x, q[2]=y, q[3]=z
    g_telemNow.Qw = static_cast<float>(q[0]);
    g_telemNow.Qx = static_cast<float>(q[1]);
    g_telemNow.Qy = static_cast<float>(q[2]);
    g_telemNow.Qz = static_cast<float>(q[3]);

    // ZYX Euler convention with Z = rocket longitudinal axis:
    //   eulerYaw()   = rotation about Z = spin about rocket axis  → physical roll
    //   eulerPitch() = rotation about Y = lateral tilt            → physical yaw
    //   eulerRoll()  = rotation about X = lateral tilt            → physical pitch
    g_telemNow.roll  = static_cast<float>(est->eulerPitch() * RAD_TO_DEG);
    g_telemNow.yaw   = static_cast<float>(est->eulerYaw()   * RAD_TO_DEG);
    g_telemNow.pitch = static_cast<float>(est->eulerRoll()  * RAD_TO_DEG);
}

float DataUpdate::getVerticalVelocity()
{
    static uint32_t lastVvelMs = 0U;
    static float    lastAlt    = 0.0f;
    static float    lastVvel   = 0.0f;

    const uint32_t nowMs  = HAL_GetTick();
    const float    altNow = g_telemNow.altitude;

    if (lastVvelMs == 0U) {
        lastVvelMs = nowMs;
        lastAlt    = altNow;
        return 0.0f;
    }

    const uint32_t elapsed = nowMs - lastVvelMs;
    if (elapsed < VVEL_UPDATE_PERIOD_MS) {
        return lastVvel;
    }

    if (fabs(altNow - lastAlt) < 0.1f) { // delta v < 0.1f = m/s
        lastVvel   = 0.0f;
        lastVvelMs = nowMs;
        lastAlt    = altNow;
        return 0.0f;
    }

    float vvel = (altNow - lastAlt) / (static_cast<float>(elapsed) * 1.0e-3f);
    // Clamp to physically plausible range — suppresses ADC glitches and
    // timing anomalies from producing absurd readings that could trip the
    // actuation lockout or ignition state machine.
    if (vvel >  300.0f) vvel =  300.0f;
    if (vvel < -300.0f) vvel = -300.0f;

    lastVvel   = vvel;
    lastVvelMs = nowMs;
    lastAlt    = altNow;
    return lastVvel;
}