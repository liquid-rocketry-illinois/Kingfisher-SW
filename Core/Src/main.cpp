//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "cmsis_os.h"
#include "constants.h"
#include "TEST_FUNCTIONS.h"
#include "timing.h"

#include "FlightComputer_SENSORS.h"
#include "CTRLS_Controls.h"

TaskHandle_t updateDataTaskHandle;
TaskHandle_t CTRLIndicationHandle;


// ======================= THREADS =========================

Sensors S;
Telemetry telem;
Servo_Axon_Mini_MKII Servos;

static volatile bool initDone = false;

static float modelVerticalVelocityMS(const StateVec& x)
{
    float R[3][3];
    Physics::R_BW(x(6,0), x(7,0), x(8,0), x(9,0), R);

    // R_BW maps world vectors into body frame, so R^T maps body velocity
    // back to world.  The third world component is vertical velocity.
    return R[0][2] * x(3,0) + R[1][2] * x(4,0) + R[2][2] * x(5,0);
}

extern "C" void FC_Init(void*) {
    MICROS_DWT_Timebase_Init();

    while (S.Init() != STATUS_OK)
        osDelay(100);

    while (telem.Init() != STATUS_OK)
        osDelay(100);
    
    while (Servos.Init({0,0}, TENTH_DEGREE, false) != true)
        osDelay(100);

    // SD is non-critical: log if it fails but don't block init.
    // Required for ground testing where SD may not be available.
    SD_Init();

    initDone = true;
    vTaskSuspend(NULL); // suspend permanently
}


extern "C" void updateSensorTask(void*) {
    while (!initDone) osDelay(10);

    for (;;) {
        S.Update();
        xTaskNotifyGive(updateDataTaskHandle);
        osDelay(2); // ~500 Hz ceiling; yields CPU to lower-priority tasks (e.g. Radio)
    }
}

#include "FlightComputer_DATAUPDATE.h"
#include "DataFilter.h"

extern telemetryData g_telemNow;
extern telemetryData g_telemPrev;
extern GndStationData g_gndData;
extern osMutexId_t g_ctrls_sensor_mutex;

extern "C" void updateDataTask(void*)
{
    while (!initDone) osDelay(10);

    static stateestimation::AttitudeEstimator Est;

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); // 100ms watchdog: if sensor task stalls, don't block forever

        if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
            DataUpdate::ComputeDt();
            g_filters.apply();
            DataUpdate::FuseAttitude(&Est);
            g_telemNow.verticalVelocity = DataUpdate::getVerticalVelocity();
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Wake CTRLs immediately so it reads the freshest gyro data with
        // minimum latency rather than waiting for its next poll interval.
        xTaskNotifyGive(CTRLIndicationHandle);
    }
}


extern "C" void Radio(void*)
{
    while (!initDone)
        osDelay(10);

    for (;;)
    {
        // Snapshot g_telemNow into the radio's outbound buffer under the mutex,
        // then release before TX so other tasks aren't blocked during transmission.
        if (osMutexAcquire(g_ctrls_sensor_mutex, 20U) == osOK) {
            telem.HALOutData = g_telemNow;
            g_telemPrev      = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }
}


extern volatile uint32_t g_pyroPending;

extern "C" void PyroTask(void*)
{
    while (!initDone)
        osDelay(10);

    for (;;)
    {
        // Manual trigger
        if (g_pyroPending & PYRO_DROGUE_MAIN_BIT) {
            g_pyroPending &= ~PYRO_DROGUE_MAIN_BIT;
            HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);
            osMutexAcquire(g_ctrls_sensor_mutex, 10);
            g_telemNow.pyroMainDrogueFired = true;
            osMutexRelease(g_ctrls_sensor_mutex);

            SD_LogNewline("FirePyroDrogueMain");
            SD_LogNewline("FirePyroDrogueMain");
            SD_LogNewline("FirePyroDrogueMain");
        }
        if (g_pyroPending & PYRO_DROGUE_BKP_BIT) {
            g_pyroPending &= ~PYRO_DROGUE_BKP_BIT;
            HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_RESET);
            osMutexAcquire(g_ctrls_sensor_mutex, 10);
            g_telemNow.pyroBackupDrogueFired = true;
            osMutexRelease(g_ctrls_sensor_mutex);

            SD_LogNewline("FirePyroDrogueBackup");
            SD_LogNewline("FirePyroDrogueBackup");
            SD_LogNewline("FirePyroDrogueBackup");
        }
        if (g_pyroPending & PYRO_MAIN_BIT) {
            g_pyroPending &= ~PYRO_MAIN_BIT;
            HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);
            osMutexAcquire(g_ctrls_sensor_mutex, 10);
            g_telemNow.pyroMainChuteFired = true;
            osMutexRelease(g_ctrls_sensor_mutex);

            SD_LogNewline("FirePyroMain");
            SD_LogNewline("FirePyroMain");
            SD_LogNewline("FirePyroMain");
        }

        // ── Autonomous pyro logic ────────────────────────────────────────────
        float alt = 0.0f, vvel = 0.0f;
        bool drogueMainFired = false, drogueBkpFired = false;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
            alt           = g_telemNow.altitude;
            vvel          = g_telemNow.verticalVelocity;
            drogueMainFired = g_telemNow.pyroMainDrogueFired;
            drogueBkpFired  = g_telemNow.pyroBackupDrogueFired;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Primary drogue at apogee: vvel below APOGEE_VEL_MS and above MIN_APOGEE_ALT_M
        if (!drogueMainFired && alt > MIN_APOGEE_ALT_M && vvel < APOGEE_VEL_MS) {
            g_pyroPending |= PYRO_DROGUE_MAIN_BIT;
            SD_LogNewline("AutoDrogue_Primary");
        }

        // Backup drogue: falling faster than BACKUP_DROGUE_VEL_MS above BACKUP_DROGUE_MIN_ALT_M
        if (!drogueBkpFired && alt > BACKUP_DROGUE_MIN_ALT_M && vvel < BACKUP_DROGUE_VEL_MS) {
            g_pyroPending |= PYRO_DROGUE_BKP_BIT;
            SD_LogNewline("AutoDrogue_Backup");
        }

        // Main chute: below MAX_MAIN_DEPLOY_ALT_M and at least one drogue has fired
        if (!g_telemNow.pyroMainChuteFired && alt < MAX_MAIN_DEPLOY_ALT_M
                && alt > MIN_MAIN_DEPLOY_ALT_M && (drogueMainFired || drogueBkpFired)) {
            g_pyroPending |= PYRO_MAIN_BIT;
            SD_LogNewline("AutoMain_Chute");
        }

        osDelay(10);
    }
}


extern "C" void SDLogTask(void*) {
    while (!initDone)
        osDelay(10);

    // CSV headers — two-line schema per sample
    SD_LogNewline("tick_ms,lat,lon,baro_alt,gps_alt,vvel_ms,temp_c");
    SD_LogNewline("accX,accY,accZ,gyrX,gyrY,gyrZ,roll,pitch,yaw,s1cmd,s2cmd,rssi,pyro_dm,pyro_db,pyro_mc");

    for (;;) {
        // Snapshot under mutex — release before slow SD write
        telemetryData s = {};
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
            s = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        char ln1[100], ln2[140];

        snprintf(ln1, sizeof(ln1),
            "%lu,%.5f,%.5f,%.1f,%.1f,%.2f,%.1f",
            HAL_GetTick(),
            (double)s.latitude, (double)s.longitude,
            (double)s.altitude, (double)s.GPSaltitude,
            (double)s.verticalVelocity, (double)s.temperature);

        snprintf(ln2, sizeof(ln2),
            "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d",
            (double)s.mAccX,  (double)s.mAccY,  (double)s.mAccZ,
            (double)s.mGyrX,  (double)s.mGyrY,  (double)s.mGyrZ,
            (double)s.roll,   (double)s.pitch,   (double)s.yaw,
            (double)s.servoTarget1, (double)s.servoTarget2,
            (int)s.RSSI,
            (int)s.pyroMainDrogueFired,
            (int)s.pyroBackupDrogueFired,
            (int)s.pyroMainChuteFired);

        SD_LogNewline(ln1);
        SD_LogNewline(ln2);

        osDelay(20); // 50 Hz
    }
}


extern "C" void CTRLs(void*)
{
    // ── Init: build algorithm objects ────────────────────────────────────────
    // Mutexes (g_ctrls_sensor_mutex, g_ctrls_output_mutex) are created by
    // ctrlsInit() in MX_FREERTOS_Init before the scheduler starts, so they are
    // safe to acquire as soon as this task runs.
    static RocketConfig cfg;
    loadControlFreakRocketConfig(cfg);
    cfg.computeDerived();
    static Physics    phys(cfg);
    static EKF        ekf(phys, cfg);
    static ControlLaw ctrl(phys, cfg);

    while (!initDone) osDelay(10);

    {
        // ── Update loop ──────────────────────────────────────────────────────────
        static uint32_t prev_ms         = 0U;
        static uint32_t launch_start_ms = 0U;
        static float    model_alt_m     = 0.0f;
        static bool     apogee_latched  = false;
        static uint32_t apogee_blink_ms = 0U;
        static bool     apogee_blink_done = false;

        float u_last_rad         = 0.0f;  // previous canard command (rad) — fed back into EKF EOM
        CtrlsSensorSnapshot snap = {};
        snap.temperature_K       = 288.15f;

        for (;;)
        {
            //if (!g_ctrls_enabled) { osDelay(10); continue; }

            const uint32_t now_ms = millis();
            if (launch_start_ms == 0U) launch_start_ms = now_ms;

            const float dt = (prev_ms == 0U) ? 0.01f
                             : static_cast<float>(now_ms - prev_ms) * 1e-3f;
            prev_ms = now_ms;
            if (dt <= 0.0f || dt > 0.5f) { osDelay(1); continue; }

            // Dynamics-test mode: time starts at controls-task start, not at
            // sensor-detected liftoff.  This lets the flight computer propagate
            // the model from launch for bench/ground dynamics tests.
            const float t = static_cast<float>(now_ms - launch_start_ms) * 1.0e-3f;

            // ── Read sensor snapshot ─────────────────────────────────────────────
            // The sensor task still populates full telemetry, but controls and
            // dynamics intentionally consume only gyro.  Accel, baro altitude,
            // GPS, temperature, and sensor-derived flight_time_s are left out
            // of the loop so they cannot disturb model propagation.
            bool fresh = false;
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                if (g_SensorData.fresh) {
                    snap = g_SensorData;
                    g_SensorData.fresh = false;
                    fresh = true;
                }
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            // ── EKF step ─────────────────────────────────────────────────────────
            const float model_alt_for_step = model_alt_m;
            if (fresh) {
                const float vx=ekf.xhat(3,0), vy=ekf.xhat(4,0), vz=ekf.xhat(5,0);
                ctrl.updateRollEffectivenessSign(t, snap.gyro_rad_s[2], u_last_rad,
                                                  sqrtf(vx*vx + vy*vy + vz*vz));
                MeasVec y;
                // y(i, 0) used to carry accelerometer data for full EKF correction.
                // It is deliberately left at zero in dynamics-test mode.
                // y(i,   0) = snap.accel_g[i];
                // y(3+i, 0) = snap.gyro_rad_s[i];  // all three gyro axes
                y(5, 0) = snap.gyro_rad_s[2];        // roll axis (w3) only
                // Old full-sensor correction path:
                // ekf.update(t, dt, y, u_last_rad, snap.altitude_m);
                ekf.updateGyroOnly(t, dt, y, u_last_rad, model_alt_for_step);
            } else {
                // Old baro-altitude propagation path:
                // ekf.predictOnly(t, dt, u_last_rad, snap.altitude_m);
                ekf.predictOnly(t, dt, u_last_rad, model_alt_for_step);
            }

            const float model_vz_ms = modelVerticalVelocityMS(ekf.xhat);
            model_alt_m += model_vz_ms * dt;
            if (model_alt_m < 0.0f) model_alt_m = 0.0f;

            // Model-based apogee: trigger when vertical velocity drops below 3 m/s
            // after burnout. saw_climb guard removed for ground testing — EKF
            // velocity stays near 0 on the bench so a climb is never observed.
            if (!apogee_latched && t > cfg.t_burnout && model_vz_ms < 3.0f) {
                apogee_latched  = true;
                apogee_blink_ms = now_ms;
                g_ctrls_enabled = false;   // zero servo output immediately
                Servos.Update(0.0f, 0.0f);
                SD_LogNewline("ModelApogee");
            }

            // Non-blocking apogee indication: rapid LED strobe for 2 seconds.
            if (apogee_latched && !apogee_blink_done) {
                const uint32_t blink_elapsed = now_ms - apogee_blink_ms;
                if (blink_elapsed < 2000U) {
                    const bool led_on = (blink_elapsed % 200U) < 100U;
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin,
                                      led_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
                    apogee_blink_done = true;
                }
            }

            // ── Control law ───────────────────────────────────────────────────────
            // EKF always runs above to keep the state estimate warm.
            // Output is zeroed when controls are disabled so servos hold neutral.
            //
            // Old ground-test behavior injected fixed time/airspeed and shut
            // controls off after 60 s.  For dynamics testing, ground test now
            // means "run the model from launch and observe roll response."
            //
            // static constexpr float    GROUND_TEST_AIRSPEED_MS = 50.0f;
            // static constexpr float    GROUND_TEST_TIME_S      = 5.0f;
            // static constexpr uint32_t GROUND_TEST_DURATION_MS = 60000U;
            // static uint32_t           test_start_ms           = 0U;
            // if (g_ctrls_test_mode) { ... }
            // if (g_ctrls_test_mode) {
            //     t_ctrl          = GROUND_TEST_TIME_S;
            //     xhat_ctrl(5, 0) = GROUND_TEST_AIRSPEED_MS;
            // }
            const float u_rad = g_ctrls_enabled ? ctrl.computeControl(t, ekf.xhat) : 0.0f;
            u_last_rad = u_rad;
            const float u_deg = u_rad * (180.0f / static_cast<float>(M_PI));

            // ── Write outputs ─────────────────────────────────────────────────────
            static float servo_offset1_deg = 0.0f;
            static float servo_offset2_deg = 0.0f;
            if (osMutexAcquire(g_ctrls_output_mutex, 2U) == osOK) {
                g_ctrls_canard_cmd_deg = u_deg;
                osMutexRelease(g_ctrls_output_mutex);
            }

            // Both canards receive the same deflection command to produce roll.
            // Ground-station offsets apply the per-servo zero-point trim.
            if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
                servo_offset1_deg = g_gndData.servoOffset1;
                servo_offset2_deg = g_gndData.servoOffset2;
                g_telemNow.servoTarget1 = u_deg + servo_offset1_deg;
                g_telemNow.servoTarget2 = u_deg + servo_offset2_deg;
                g_telemNow.altitude = model_alt_m;
                g_telemNow.verticalVelocity = model_vz_ms;
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            Servos.SetOffset({servo_offset1_deg, servo_offset2_deg});
            Servos.Update(u_deg, u_deg);

            osDelay(5); // ~200 Hz ceiling; yields CPU between EKF iterations
        }
    }
}
