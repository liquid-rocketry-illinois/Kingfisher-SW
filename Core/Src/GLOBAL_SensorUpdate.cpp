//
// Created by dyrel on 5/11/2026.
//

#include "GLOBAL_SensorUpdate.h"

// ── Device instances owned by this thread ─────────────────────────────────────
static IMUs                 s_imu;
static Baro_Unified         s_baro;
static MAXM10S              s_gps;
static Servo_Axon_Mini_MKII s_servos;

// ── UpdateData ────────────────────────────────────────────────────────────────
// Dedicated sensor-read thread. Initialises hardware, then continuously reads
// all sensors and publishes a fresh CtrlsSensorSnapshot under g_ctrls_sensor_mutex.
// The mutex is held for the full iteration so consumers always see a consistent
// snapshot. osDelay(1) at the end yields the CPU between iterations (≤1 kHz
// ceiling; effective baro rate is gated by BMP390 ODR ~200 Hz).
extern "C" void UpdateData(void *argument) {

    // ── Phase 1: initialise ───────────────────────────────────────────────────
    s_baro.Init(true);                                      // TMR on (3× BMP390)
    s_imu.Init(true);                                       // TMR on (3× BMI323)
    s_gps.Init(&hi2c4);
    s_servos.Init({0.0f, 0.0f}, TENTH_DEGREE, false);

    const uint32_t t0_ms = millis();

    // ── Phase 2: sensor loop ──────────────────────────────────────────────────
    for (;;) {
        osMutexAcquire(g_ctrls_sensor_mutex, osWaitForever);

        // Read hardware
        s_imu.Update();
        s_baro.Update();
        s_gps.update();

        // TMR-vote IMU (three BMI323 readings → single best estimate)
        BMI_Data rawA = s_imu.getRawBMI(0);
        BMI_Data rawB = s_imu.getRawBMI(1);
        BMI_Data rawC = s_imu.getRawBMI(2);
        Vector3D<float> acc = TMR::Vote(rawA.accel_linear, rawB.accel_linear, rawC.accel_linear);
        Vector3D<float> gyr = TMR::Vote(rawA.ang_vel,      rawB.ang_vel,      rawC.ang_vel);

        // Publish snapshot
        BARO_DATA bd = s_baro.getData();
        g_ctrls_sensor.flight_time_s = static_cast<float>(millis() - t0_ms) * 1e-3f;
        g_ctrls_sensor.altitude_m    = bd.Filtered.heightMeters;
        g_ctrls_sensor.temperature_K = bd.Filtered.Temperature + 273.15f;
        g_ctrls_sensor.accel_ms2     = acc;
        g_ctrls_sensor.gyro_rads     = gyr;
        g_ctrls_sensor.fresh         = true;

        osMutexRelease(g_ctrls_sensor_mutex);

        osDelay(1);
    }
}
