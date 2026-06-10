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
static volatile bool initDone = false;

extern "C" void FC_Init(void*) {
    MICROS_DWT_Timebase_Init();

    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);

    // Retry until every sensor (IMU A/B/C, baro A/B/C, GPS) reports success.
    // Radio init is handled independently in the Radio task.
    while (S.Init() != STATUS_OK)
        osDelay(100);

    while (telem.Init() != STATUS_OK)
        osDelay(100);

    initDone = true;
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);

    vTaskSuspend(NULL); // init complete — suspend permanently; FreeRTOS tasks must not return
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

extern telemetryData g_telemNow;
extern telemetryData g_telemPrev;
extern GndStationData g_gndData;
extern osMutexId_t g_ctrls_sensor_mutex;

extern "C" void updateDataTask(void*)
{
    while (!initDone) osDelay(10);

    static Quaternion QObj;
    QObj.q = {1,0,0,0};

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        osMutexAcquire(g_ctrls_sensor_mutex, 20);

        DataUpdate::ComputeDt();

        DataUpdate::FuseQuat(&QObj);
        DataUpdate::QuatToYPR(&QObj.q);

        g_telemNow.verticalVelocity = DataUpdate::getVerticalVelocity();

        osMutexRelease(g_ctrls_sensor_mutex);
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
        uint8_t status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }
}


extern volatile uint32_t g_pyroPending;

extern "C" void PyroTask(void*)
{
    while (!initDone) osDelay(10);

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
        }
        if (g_pyroPending & PYRO_DROGUE_BKP_BIT) {
            g_pyroPending &= ~PYRO_DROGUE_BKP_BIT;
            HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(DROUGE_BACK_GPIO_Port, DROUGE_BACK_Pin, GPIO_PIN_RESET);
            osMutexAcquire(g_ctrls_sensor_mutex, 10);
            g_telemNow.pyroBackupDrogueFired = true;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        if (g_pyroPending & PYRO_MAIN_BIT) {
            g_pyroPending &= ~PYRO_MAIN_BIT;
            HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);
            osDelay(2000);
            HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_RESET);
            osMutexAcquire(g_ctrls_sensor_mutex, 10);
            g_telemNow.pyroMainChuteFired = true;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        osDelay(10);
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
        static uint32_t prev_ms  = 0U;
        float u_last_rad         = 0.0f;  // previous canard command (rad) — fed back into EKF EOM
        CtrlsSensorSnapshot snap = {};
        snap.temperature_K       = 288.15f;

        for (;;)
        {
            if (!g_ctrls_enabled) { osDelay(10); continue; }

            const uint32_t now_ms = millis();
            const float dt = (prev_ms == 0U) ? 0.01f
                             : static_cast<float>(now_ms - prev_ms) * 1e-3f;
            prev_ms = now_ms;
            if (dt <= 0.0f || dt > 0.5f) { osDelay(1); continue; }

            // ── Read sensor snapshot ─────────────────────────────────────────────
            // snap retains last known values if no fresh data is available,
            // so predictOnly always has a valid flight time and altitude.
            bool fresh = false;
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                if (g_SensorData.fresh) {
                    snap = g_SensorData;
                    g_SensorData.fresh = false;
                    fresh = true;
                }
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            const float t = snap.flight_time_s;

            // ── EKF step ─────────────────────────────────────────────────────────
            if (fresh) {
                const float vx=ekf.xhat(3,0), vy=ekf.xhat(4,0), vz=ekf.xhat(5,0);
                ctrl.updateRollEffectivenessSign(t, snap.gyro_rad_s[2], u_last_rad,
                                                  sqrtf(vx*vx + vy*vy + vz*vz));
                MeasVec y;
                for (int i = 0; i < 3; i++) {
                    y(i,   0) = snap.accel_g[i];
                    y(3+i, 0) = snap.gyro_rad_s[i];
                }
                ekf.update(t, dt, y, u_last_rad, snap.altitude_m);
            } else {
                ekf.predictOnly(t, dt, u_last_rad, snap.altitude_m);
            }

            // ── Control law ───────────────────────────────────────────────────────
            // EKF always runs above to keep the state estimate warm.
            // Output is zeroed when controls are disabled so servos hold neutral.
            const float u_rad = g_ctrls_enabled ? ctrl.computeControl(t, ekf.xhat) : 0.0f;
            u_last_rad = u_rad;
            const float u_deg = u_rad * (180.0f / static_cast<float>(M_PI));

            // ── Write outputs ─────────────────────────────────────────────────────
            if (osMutexAcquire(g_ctrls_output_mutex, 2U) == osOK) {
                g_ctrls_canard_cmd_deg = u_deg;
                osMutexRelease(g_ctrls_output_mutex);
            }

            // Both canards receive the same deflection command to produce roll.
            // Ground-station offsets apply the per-servo zero-point trim.
            if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
                g_telemNow.servoTarget1 = u_deg + g_gndData.servoOffset1;
                g_telemNow.servoTarget2 = u_deg + g_gndData.servoOffset2;
                osMutexRelease(g_ctrls_sensor_mutex);
            }

            osDelay(5); // ~200 Hz ceiling; yields CPU between EKF iterations
        }
    }
}
