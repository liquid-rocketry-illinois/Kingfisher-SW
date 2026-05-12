//
// Created by dyrel on 5/11/2026.
//

#include "GLOBAL_SensorUpdate.h"

// ── Device instances owned by this thread ─────────────────────────────────────
static IMUs                 s_imu;
static Baro_Unified         s_baro;
static MAXM10S              s_gps;
static Servo_Axon_Mini_MKII s_servos;

// Mutex for controls
extern osMutexId_t g_ctrls_sensor_mutex;
// Sensor data
extern CtrlsSensorSnapshot g_SensorData;

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
        Vector3D<float> accV = TMR::Vote(rawA.accel_linear, rawB.accel_linear, rawC.accel_linear);
        Vector3D<float> gyrV = TMR::Vote(rawA.ang_vel,      rawB.ang_vel,      rawC.ang_vel);

        BARO_DATA bd = s_baro.getData();

        // Publish snapshot


        g_SensorData.flight_time_s = static_cast<float>(millis() - t0_ms) * 1e-3f;
        g_SensorData.altitude_m    = bd.Filtered.heightMeters;
        g_SensorData.temperature_K = bd.Filtered.Temperature + 273.15f;
        g_SensorData.accel_g[0]    = accV.x;
        g_SensorData.accel_g[1]    = accV.y;
        g_SensorData.accel_g[2]    = accV.z;
        g_SensorData.gyro_rad_s[0] = gyrV.x;
        g_SensorData.gyro_rad_s[1] = gyrV.y;
        g_SensorData.gyro_rad_s[2] = gyrV.z;
        g_SensorData.fresh         = true;

        osMutexRelease(g_ctrls_sensor_mutex);

        osDelay(1);
    }
}
