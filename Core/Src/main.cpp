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
    return R[0][2]*x(3,0) + R[1][2]*x(4,0) + R[2][2]*x(5,0);
}

extern "C" void FC_Init(void*) {
    MICROS_DWT_Timebase_Init();

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

    while (S.Init() != STATUS_OK)
        osDelay(100);

    while (telem.Init() != STATUS_OK)
        osDelay(100);

    while (Servos.Init({0,0}, TENTH_DEGREE, false) != true)
        osDelay(100);

    while (SD_Init() != 0) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        osDelay(100);
    }

    initDone = true;
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);

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

        osDelay(100); // 10 Hz
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

    SD_DynInit();

    {
        // ── Update loop ──────────────────────────────────────────────────────────
        static uint32_t prev_ms         = 0U;
        static uint32_t launch_start_ms = 0U;
        static float    model_alt_m     = 0.0f;
        static bool     apogee_latched  = false;
        static uint32_t apogee_blink_ms = 0U;
        static bool     apogee_blink_done = false;
        static uint32_t dyn_log_ms      = 0U;

        float u_last_rad         = 0.0f;
        CtrlsSensorSnapshot snap = {};
        snap.temperature_K       = 288.15f;

        for (;;)
        {
            const uint32_t now_ms = millis();
            const float dt = (prev_ms == 0U) ? 0.01f
                             : static_cast<float>(now_ms - prev_ms) * 1e-3f;
            prev_ms = now_ms;
            if (dt <= 0.0f || dt > 0.5f) { osDelay(1); continue; }

            // ── Read sensor snapshot ─────────────────────────────────────────────
            bool fresh = false;
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                if (g_SensorData.fresh) {
                    snap = g_SensorData;
                    g_SensorData.fresh = false;
                    fresh = true;
                }
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            // Ground test: start timer the moment g_ctrls_test_mode is set.
            // Real flight: use sensor-detected flight time.
            if (g_ctrls_test_mode && launch_start_ms == 0U)
                launch_start_ms = now_ms;
            const float t = (launch_start_ms > 0U)
                ? static_cast<float>(now_ms - launch_start_ms) * 1e-3f
                : snap.flight_time_s;
            const float model_alt_for_step = model_alt_m;
            const float ekf_alt_for_step =
                g_ctrls_test_mode ? model_alt_for_step : snap.altitude_m;

            // ── EKF step ─────────────────────────────────────────────────────────
            if (fresh) {
                const WeatherSample wx = phys.weatherAtAltitude(ekf_alt_for_step);
                float va[3];
                phys.airRelativeVelocity(ekf.xhat, wx.wind_x, wx.wind_y, va);
                ctrl.updateRollEffectivenessSign(t, snap.gyro_rad_s[2], u_last_rad,
                                                  sqrtf(va[0]*va[0] + va[1]*va[1] + va[2]*va[2]));
                MeasVec y;
                for (int i = 0; i < 3; i++) {
                    y(i,   0) = snap.accel_g[i];
                    y(3+i, 0) = snap.gyro_rad_s[i];
                }
                if (g_ctrls_test_mode)
                    ekf.updateGyroOnly(t, dt, y, u_last_rad, ekf_alt_for_step);
                else
                    ekf.update(t, dt, y, u_last_rad, ekf_alt_for_step);
            } else {
                ekf.predictOnly(t, dt, u_last_rad, ekf_alt_for_step);
            }

            // ── Dynamics model altitude + apogee detection ────────────────────────
            const float model_vz_ms = modelVerticalVelocityMS(ekf.xhat);
            model_alt_m += model_vz_ms * dt;
            if (model_alt_m < 0.0f) model_alt_m = 0.0f;

            // DYN log at 2 Hz
            if (!apogee_latched && now_ms - dyn_log_ms >= 100U) {
                dyn_log_ms = now_ms;
                const float vx = ekf.xhat(3,0), vy = ekf.xhat(4,0);
                const float horiz_v   = sqrtf(vx*vx + vy*vy);
                const float roll_rate = ekf.xhat(2,0);
                char dyn_ln[64];
                snprintf(dyn_ln, sizeof(dyn_ln), "%.2f,%.1f,%.2f,%.2f,%.3f",
                    t, model_alt_m, model_vz_ms, horiz_v, roll_rate);
                SD_LogDyn(dyn_ln);
            }

            // Apogee: vz drops below 3 m/s after burnout
            if (!apogee_latched && t > cfg.t_burnout && model_vz_ms < 3.0f) {
                apogee_latched  = true;
                apogee_blink_ms = now_ms;
                g_ctrls_enabled = false;
                Servos.Update(0.0f, 0.0f);
                SD_LogNewline("ModelApogee");
            }

            // Non-blocking LED strobe for 2 s on apogee
            if (apogee_latched && !apogee_blink_done) {
                const uint32_t elapsed = now_ms - apogee_blink_ms;
                if (elapsed < 2000U) {
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin,
                        (elapsed % 200U) < 100U ? GPIO_PIN_SET : GPIO_PIN_RESET);
                } else {
                    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_RESET);
                    apogee_blink_done = true;
                }
            }

            // ── Control law ───────────────────────────────────────────────────────
            // EKF always runs above to keep the state estimate warm.
            // Output is zeroed when controls are disabled so servos hold neutral.
            const float control_alt =
                g_ctrls_test_mode ? model_alt_m : snap.altitude_m;
            const float u_rad = g_ctrls_enabled
                ? ctrl.computeControl(t, ekf.xhat, control_alt)
                : 0.0f;
            u_last_rad = u_rad;
            const float u_deg = u_rad * (180.0f / static_cast<float>(M_PI));

            // ── Write outputs ─────────────────────────────────────────────────────
            if (osMutexAcquire(g_ctrls_output_mutex, 2U) == osOK) {
                g_ctrls_canard_cmd_deg = u_deg;
                osMutexRelease(g_ctrls_output_mutex);
            }

            // Both canards receive the same deflection command to produce roll.
            // Ground-station offsets apply the per-servo zero-point trim.
            Servos.Update(u_deg, u_deg);
            if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
                g_telemNow.servoTarget1 = u_deg + g_gndData.servoOffset1;
                g_telemNow.servoTarget2 = u_deg + g_gndData.servoOffset2;
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            osDelay(5); // ~200 Hz ceiling; yields CPU between EKF iterations
        }
    }
}
