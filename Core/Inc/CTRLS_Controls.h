// Created by admin on 5/10/2026.

#ifndef KINGFISHER_SW_CONTROLS_H
#define KINGFISHER_SW_CONTROLS_H

#include <cstdint>
#include "CTRLS_ControlAlgorithm.h"
#include "CTRLS_SensorFusion.h"
#include "cmsis_os.h"
#include "FlightComputer_SENSORS.h"

// ─── Sensor snapshot shared between the main FC task and ctrlsTask ───────────
// The FC task writes this after each UpdateSensors(). ctrlsTask reads it.
// Body frame convention: x=lateral1, y=lateral2, z=axial (along rocket, +ve forward)
// BMI ang_vel is in deg/s — convert to rad/s before writing here.
struct CtrlsSensorSnapshot {
    float    accel_g[3]     = {};   // body-frame accelerometer reading (g units)
    float    gyro_rad_s[3]  = {};   // body-frame gyro (rad/s)
    float    altitude_m     = 0.0f; // AGL altitude from barometer (m)
    float    temperature_K  = 288.15f; // sensor temperature, telemetry/diagnostics only
    float    flight_time_s  = 0.0f; // seconds since liftoff (0 = pre-launch)
    STAGE    stage          = PREFLIGHT;
    uint32_t timestamp_ms   = 0;    // HAL_GetTick() when written
    bool     fresh          = false;// true = new data not yet consumed by ctrlsTask
};

// ─── Globals (defined in CTRLS_Controls.cpp) ─────────────────────────────────
// Call ctrlsInit() from MX_FREERTOS_Init (before osKernelStart) to create mutexes.
extern CtrlsSensorSnapshot g_SensorData;
extern osMutexId_t         g_ctrls_sensor_mutex;
extern float               g_ctrls_canard_cmd_deg;   // output: canard command (degrees)
extern osMutexId_t         g_ctrls_output_mutex;
extern volatile bool       g_ctrls_enabled;           // set true to activate control output
extern volatile bool       g_ctrls_test_mode;         // ground dynamics-test command/state flag

// Create RTOS resources — call once from MX_FREERTOS_Init before any task starts.
extern "C" void ctrlsInit(void);

// ─── Gain-scheduled state-feedback roll controller ───────────────────────────
class ControlLaw {
public:
    ControlLaw(const Physics& phys, const RocketConfig& cfg);

    // Returns canard deflection command (rad). Call after EKF update.
    float computeControl(float t, const StateVec& xhat);

    // Update roll effectiveness sign. Call once per loop with the CURRENT gyro
    // w3 reading and the command (rad) applied in the PREVIOUS step, and the
    // current air-relative speed estimate from the EKF.
    void  updateRollEffectivenessSign(float t, float w3_meas,
                                       float u_prev_rad, float vmag);

    float rollEffectivenessSign() const { return rem_sign_; }

    void reset();

private:
    const Physics&      phys_;
    const RocketConfig& cfg_;
    float last_u_  = 0.0f;
    float last_t_  = 0.0f;

    // Roll effectiveness monitor state
    float rem_sign_          =  1.0f;
    int   rem_flip_count_    =  0;
    int   rem_mismatch_count_=  0;
    float rem_prev_t_        = -1.0f;  // negative = uninitialized
    float rem_prev_w3_       =  0.0f;

    // Compute scalar gain K for current flight condition (incorporates rem_sign_).
    float gainSchedule_(float t, const StateVec& xhat) const;
};

// ─── FreeRTOS controls task ───────────────────────────────────────────────────
// Registered in freertos.c via osThreadNew. Notification-driven by updateDataTask.
// Runs EKF + ControlLaw and writes servo targets to g_telemNow.
extern "C" void CTRLs(void* arg);

#endif // KINGFISHER_SW_CONTROLS_H
