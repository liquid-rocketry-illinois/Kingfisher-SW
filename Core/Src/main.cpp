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

    // Retry until every sensor (IMU A/B/C, baro A/B/C, GPS) reports success.
    // Radio init is handled independently in the Radio task.
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

bool STATE_IGNITION  = false;
bool STATE_BURNOUT   = false;
bool STATE_BACKUP_PID = true; // default to on; cleared when ignition is latched
bool ALLOW_ACTUATION  = false;
uint32_t CTRLs_t      = 0;   // kernel tick at ignition moment (0 = pre-launch)

static volatile bool g_request_tare = false; // set by Initial_Condition_Tare, consumed by updateDataTask
static float g_alt_pad_m = 0.0f; // MSL altitude of launch pad; applied at radio snapshot, not in g_telemNow

extern "C" void updateDataTask(void*)
{
    while (!initDone) osDelay(10);

    static stateestimation::AttitudeEstimator Est;

    // Sustain timers for latched state transitions
    static uint32_t ignDetectMs   = 0U;
    static uint32_t burnDetectMs  = 0U;
    static uint32_t burnoutMs     = 0U; // kernel tick when burnout was confirmed
    static bool     actuation_locked = false; // permanent actuation disable latch

    while (1)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)); // 100 ms watchdog if sensor task stalls

        if (osMutexAcquire(g_ctrls_sensor_mutex, 20) == osOK) {
            // Apply tare requested by Initial_Condition_Tare task
            if (g_request_tare) {
                g_request_tare = false;
                Est.reset(true, false); // reset attitude to identity, keep gyro bias from 3-min window
                g_alt_pad_m = g_telemNow.altitude;
                CTRLs_t = 0;

                // Full flight state machine reset — clears any lockouts accumulated
                // while the rocket was powered on the ground before tare.
                STATE_IGNITION   = false;
                STATE_BURNOUT    = false;
                STATE_BACKUP_PID = true;   // PID on by default until ignition
                ALLOW_ACTUATION  = false;
                g_ctrls_enabled  = false;
                actuation_locked = false;
                ignDetectMs      = 0U;
                burnDetectMs     = 0U;
                burnoutMs        = 0U;
            }

            DataUpdate::ComputeDt();
            g_filters.apply();
            DataUpdate::FuseAttitude(&Est);
            g_telemNow.verticalVelocity = DataUpdate::getVerticalVelocity();
            // g_telemNow.altitude stays MSL; AGL offset applied at radio snapshot only
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Snapshot state machine inputs (benign race — worst case is one stale sample).
        // mAccZ = remapped BMI Y axis = physical rocket vertical (reads +1g upright at rest).
        const float az   = g_telemNow.mAccZ;
        const float pitch = g_telemNow.pitch;
        //const float yaw = g_telemNow.yaw;
        //const float roll = g_telemNow.roll;
        const float vvel  = g_telemNow.verticalVelocity;
        const uint32_t nowMs = osKernelGetTickCount();

        // Ignition: vertical-axis accel exceeds LIFTOFF_ACCEL_G for IGNITION_SUSTAIN_MS
        // Latched permanently once confirmed.
        if (!STATE_IGNITION) {
            if (az > LIFTOFF_ACCEL_G) {
                if (ignDetectMs == 0U) ignDetectMs = nowMs;
                if (nowMs - ignDetectMs >= IGNITION_SUSTAIN_MS) {
                    CTRLs_t          = nowMs;
                    STATE_IGNITION   = true;
                    STATE_BACKUP_PID = false;
                }
            } else {
                ignDetectMs = 0U;
            }
        }

        // Burnout: vertical axis reads downward (negative) with magnitude > BURNOUT_ACCEL_G
        // for BURNOUT_SUSTAIN_MS.
        if (!STATE_BURNOUT) {
            if (az < BURNOUT_ACCEL_G) {
                if (burnDetectMs == 0U) burnDetectMs = nowMs;
                if (nowMs - burnDetectMs >= BURNOUT_SUSTAIN_MS) {
                    STATE_BURNOUT   = true;
                    burnoutMs       = nowMs;
                    ALLOW_ACTUATION = !actuation_locked;
                }
            } else {
                burnDetectMs = 0U;
            }
        }

        // Permanent actuation disable: pitch tipped too far, velocity too low post-burnout,
        // or MAX_ACTUATION_DURATION_MS has elapsed since burnout.
        // vvel check is gated on STATE_BURNOUT — vvel is 0 on the pad and must not
        // trip the lockout before the rocket has left the rail.
        if (!actuation_locked) {
            const bool timeout  = STATE_BURNOUT &&
                                  (nowMs - burnoutMs) >= MAX_ACTUATION_DURATION_MS;
            const bool vvel_low = STATE_BURNOUT &&
                                  fabsf(vvel) < MIN_ACTUATION_VEL_MS;
            if (fabsf(pitch) > MAX_PITCH_ANGLE_DEG || vvel_low || timeout) {
                actuation_locked = true;
                ALLOW_ACTUATION  = false;
                g_ctrls_enabled  = false;
            }
        }

        // Wake CTRLs so it reads the freshest gyro data with minimum latency.
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
        // AGL offset is applied here rather than mutating g_telemNow, so the
        // sensor task's continuous MSL writes don't race against the correction.
        if (osMutexAcquire(g_ctrls_sensor_mutex, 20U) == osOK) {
            telem.HALOutData          = g_telemNow;
            telem.HALOutData.altitude = g_telemNow.altitude - g_alt_pad_m;
            g_telemPrev               = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        uint8_t status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }
}




extern GPS_Data g_GPS;

extern "C" void SDLogTask(void*) {
    while (!initDone)
        osDelay(10);

    SD_LogNewline("=== HAL-1 LOG START ===");
    SD_LogNewline("SENS1,tick_ms\tlat,lon\tbaro_alt_m,gps_alt_m\tvvel_ms\ttemp_c,");
    SD_LogNewline("SENS2,tick_ms\taccX,accY,accZ,gyrX,gyrY,gyrZ,roll,pitch,yaw,s1cmd_deg,s2cmd_deg,rssi,pyro_dm,pyro_db,pyro_mc");
    SD_LogNewline("GPS,tick_ms\tlat,lon\talt_m\thh:mm:ss\tsats\thdop");
    SD_LogNewline("DYN,tick_ms\tt_s\talt_m\tvz_ms,vh_ms\troll_rate_rads");
    SD_LogNewline("EVENT,tick_ms\tdescription");

    for (;;) {
        telemetryData s = {};
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) == osOK) {
            s = g_telemNow;
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        GPS_Data gps = g_GPS;

        char ln1[112], ln2[152], ln3[96];

        snprintf(ln1, sizeof(ln1),
            "SENS1,%lu\t%.5f,%.5f\t%.1f,%.1f\t%.2f\t%.1f",
            HAL_GetTick(),
            (double)s.latitude, (double)s.longitude,
            (double)s.altitude, (double)s.GPSaltitude,
            (double)s.verticalVelocity, (double)s.temperature);

        snprintf(ln2, sizeof(ln2),
            "SENS2,%lu\t%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%d,%d,%d,%d",
            HAL_GetTick(),
            (double)s.mAccX,  (double)s.mAccY,  (double)s.mAccZ,
            (double)s.mGyrX,  (double)s.mGyrY,  (double)s.mGyrZ,
            (double)s.roll,   (double)s.pitch,   (double)s.yaw,
            (double)s.servoTarget1, (double)s.servoTarget2,
            (int)s.RSSI,
            (int)s.pyroMainDrogueFired,
            (int)s.pyroBackupDrogueFired,
            (int)s.pyroMainChuteFired);

        snprintf(ln3, sizeof(ln3),
            "GPS,%lu\t%.7f,%.7f\t%.2f\t%02u:%02u:%02u\t%u\t%.2f",
            HAL_GetTick(),
            gps.latitude, gps.longitude, gps.altitude,
            (unsigned)gps.hour, (unsigned)gps.minute, (unsigned)gps.second,
            (unsigned)gps.satellites, (double)gps.hdop);

        SD_LogNewline(ln1);
        SD_LogNewline(ln2);
        SD_LogNewline(ln3);

        osDelay(10); // 100 Hz
    }
}


// ------------- BACKUP PID CONTROLLER IF INITIAL ACCEL NOT DETECTED -------------

void CTRLs_PID()
{
    // Proportional-only roll-rate damper.
    // Closed-loop: J*wdot = -(b + Keff*KP)*w  => exponential decay for any KP > 0.
    // Theoretically stable regardless of gain magnitude; larger KP = faster settling
    // at the cost of larger canard deflections.  Tune KP on the bench: spin the
    // rocket by hand, confirm canards oppose the spin, then increase until satisfied.
    static constexpr float KP              = 0.30f;  // deg deflection per deg/s roll rate -- TUNE
    static constexpr float MAX_DEFLECT_DEG = 7.0f;  // canard travel limit -- verify against stops
    // Dead-band: suppress control below 6 deg/s to avoid fighting
    // natural spin-up and reduce servo wear at low roll rates.
    static constexpr float ROLL_RATE_THRESHOLD_RPS = M_PI/180.0f * 1.0; // last number is deg/sec

    while (!initDone) osDelay(10);

        // if (!g_ctrls_enabled) {
        //     Servos.Update(0.0f, 0.0f);
        //     continue;
        // }

        // mGyrZ = BMI.ang_vel.y = rocket spin-axis rate in deg/s (after axis remap)
        float roll_rate_dps = 0.0f;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
            roll_rate_dps = g_telemNow.mGyrZ;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        if (fabsf(roll_rate_dps) <= ROLL_RATE_THRESHOLD_RPS) {
            Servos.Update(0.0f, 0.0f);
            if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
                g_telemNow.servoTarget1 = g_gndData.servoOffset1;
                g_telemNow.servoTarget2 = g_gndData.servoOffset2;
                osMutexRelease(g_ctrls_sensor_mutex);
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            return;
        }

        // Oppose the roll: positive rate -> negative deflection, and vice versa.
        // "positive angle -> positive roll moment" (servo docstring), so negating
        // the rate produces a restoring moment.
        float cmd = -KP * roll_rate_dps;

        if (cmd >  MAX_DEFLECT_DEG) cmd =  MAX_DEFLECT_DEG;
        if (cmd < -MAX_DEFLECT_DEG) cmd = -MAX_DEFLECT_DEG;

        // AngleOffsetDEGREES (ground-station trim) is applied inside Actuate().
        Servos.Update(cmd, cmd);

        if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
            g_telemNow.servoTarget1 = cmd + g_gndData.servoOffset1;
            g_telemNow.servoTarget2 = cmd + g_gndData.servoOffset2;
            osMutexRelease(g_ctrls_sensor_mutex);
        }

        // Block until updateDataTask signals that a fresh sensor frame is ready.
        // 10 ms timeout is a safety fallback only — under normal operation the
        // notification arrives every ~2 ms (updateSensorTask rate).
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
}


// ------------------------- STATE SPACE MODEL -------------------------
void CTRLs_STATESPACE() {
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


    // ── Update loop ──────────────────────────────────────────────────────────
    static uint32_t prev_ms         = 0U;
    static uint32_t launch_start_ms = 0U;
    static float    model_alt_m     = 0.0f;
    static float    t_zero_s        = 0.0f;  // subtracted from snap.flight_time_s to reset EKF clock
    static bool     apogee_latched  = false;
    static uint32_t apogee_blink_ms = 0U;
    static bool     apogee_blink_done = false;
    static uint32_t dyn_log_ms      = 0U;

    float u_last_rad         = 0.0f;
    CtrlsSensorSnapshot snap = {};
    snap.temperature_K       = 288.15f;

    for (;;)
        {
            // Remote kill or algorithm switch — return so the task loop re-evaluates.
            if (!g_ctrls_enabled || !ALLOW_ACTUATION || STATE_BACKUP_PID) {
                Servos.Update(0.0f, 0.0f);
                return;
            }

            // Reset flight clock if GND commanded state-space from t=0.
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

            // Reset flight clock if GND commanded state-space from t=0.
            // Capture current snap.flight_time_s as the new zero so that
            // subsequent t values are relative to the moment of the command.
            if (g_ctrls_reset_time) {
                g_ctrls_reset_time = false;
                launch_start_ms    = 0U;
                t_zero_s           = snap.flight_time_s;
                ekf.reset(cfg);
                ctrl.reset();
            }

            const float t = snap.flight_time_s - t_zero_s;
            const float sensor_alt_m = snap.altitude_m;
            const float ekf_alt_for_step = fresh ? sensor_alt_m : model_alt_m;

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
                ekf.update(t, dt, y, u_last_rad, ekf_alt_for_step);
            } else {
                ekf.predictOnly(t, dt, u_last_rad, ekf_alt_for_step);
            }

            // ── Sensor-aided model altitude + apogee detection ────────────────────
            const float model_vz_ms = modelVerticalVelocityMS(ekf.xhat);
            if (fresh)
                model_alt_m = sensor_alt_m;
            else
                model_alt_m += model_vz_ms * dt;
            if (model_alt_m < 0.0f) model_alt_m = 0.0f;

            // DYN log at 10 Hz — only while controls are active
            if (g_ctrls_enabled && !apogee_latched && now_ms - dyn_log_ms >= 20U) {
                dyn_log_ms = now_ms;
                const float vx = ekf.xhat(3,0), vy = ekf.xhat(4,0);
                const float horiz_v   = sqrtf(vx*vx + vy*vy);
                const float roll_rate = ekf.xhat(2,0);
                char dyn_ln[72];
                snprintf(dyn_ln, sizeof(dyn_ln), "DYN,%lu\t%.2f\t%.1f\t%.2f,%.2f\t%.3f",
                    now_ms, t, model_alt_m, model_vz_ms, horiz_v, roll_rate);
                SD_LogNewline(dyn_ln);
            }

            // Apogee: vz drops below 3 m/s after burnout
            if (!apogee_latched && t > cfg.t_burnout && model_vz_ms < 3.0f) {
                apogee_latched  = true;
                apogee_blink_ms = now_ms;
                g_ctrls_enabled = false;
                Servos.Update(0.0f, 0.0f);
                SD_LogNewline("EVENT,ModelApogee");
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
            const float control_alt = model_alt_m;
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

            // Block until updateDataTask signals that a fresh sensor frame is ready.
            // 10 ms timeout is a safety fallback only.
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
        }
}


// CONTROLS TASK. Selects between PID and state space controllers based on flight conditions.
// Algorithm selection is one-way: PID runs until ignition latches STATE_BACKUP_PID=false,
// then CTRLs_STATESPACE() is called and never returns (its own for(;;) loop).
// When ALLOW_ACTUATION is false, yield via ulTaskNotifyTake rather than busy-loop.
extern "C" void CTRLs(void*)
{
    while (!initDone) osDelay(10);

    for (;;) {
        if (!ALLOW_ACTUATION) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
            continue;
        }
        if (STATE_BACKUP_PID) {
            CTRLs_PID();
        } else {
            CTRLs_STATESPACE();
        }
    }
}

// Monitors IMU for a 3-minute stability window (rocket sitting upright on pad), then
// sets g_request_tare so updateDataTask can zero the attitude estimator, pad altitude,
// and CTRLs_t in a mutex-safe context. Suspends itself after the tare is requested.
extern "C" void Initial_Condition_Tare(void*) {
    while (!initDone) osDelay(10);

    float    prevAx = 0.0f, prevAy = 0.0f, prevAz = 0.0f;
    float    prevGx = 0.0f, prevGy = 0.0f, prevGz = 0.0f;
    uint32_t stableStart = 0U;
    bool     firstSample = true;

    for (;;) {
        osDelay(100U); // 10 Hz sampling

        float ax, ay, az, gx, gy, gz;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 5) != osOK) continue;
        ax = g_telemNow.mAccX; ay = g_telemNow.mAccY; az = g_telemNow.mAccZ;
        gx = g_telemNow.mGyrX; gy = g_telemNow.mGyrY; gz = g_telemNow.mGyrZ;
        osMutexRelease(g_ctrls_sensor_mutex);

        if (firstSample) {
            prevAx = ax; prevAy = ay; prevAz = az;
            prevGx = gx; prevGy = gy; prevGz = gz;
            stableStart = osKernelGetTickCount();
            firstSample = false;
            continue;
        }

        const bool stable =
            fabsf(ax - prevAx) < TARE_ACC_DELTA_G  &&
            fabsf(ay - prevAy) < TARE_ACC_DELTA_G  &&
            fabsf(az - prevAz) < TARE_ACC_DELTA_G  &&
            fabsf(gx - prevGx) < TARE_GYRO_DELTA_DPS &&
            fabsf(gy - prevGy) < TARE_GYRO_DELTA_DPS &&
            fabsf(gz - prevGz) < TARE_GYRO_DELTA_DPS;

        prevAx = ax; prevAy = ay; prevAz = az;
        prevGx = gx; prevGy = gy; prevGz = gz;

        if (!stable) {
            stableStart = osKernelGetTickCount();
        } else if (osKernelGetTickCount() - stableStart >= TARE_STABILITY_MS) {
            g_request_tare = true;
            vTaskSuspend(NULL); // tare requested; no further work to do
        }
    }
}
