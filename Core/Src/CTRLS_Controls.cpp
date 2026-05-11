// Created by admin on 5/10/2026.

#include "CTRLS_Controls.h"
#include "timing.h"   // for millis()
#include "cmsis_os.h"

// ─── Shared globals ───────────────────────────────────────────────────────────
CtrlsSensorSnapshot g_ctrls_sensor        = {};
osMutexId_t         g_ctrls_sensor_mutex  = nullptr;
float               g_ctrls_canard_cmd_deg = 0.0f;
osMutexId_t         g_ctrls_output_mutex  = nullptr;

static const osMutexAttr_t k_sensor_mutex_attr = {
    "ctrlsSensorMtx", osMutexPrioInherit, nullptr, 0U
};
static const osMutexAttr_t k_output_mutex_attr = {
    "ctrlsOutputMtx", osMutexPrioInherit, nullptr, 0U
};

// Call once from MX_FREERTOS_Init before the scheduler starts
extern "C" void ctrlsInit(void) {
    g_ctrls_sensor_mutex = osMutexNew(&k_sensor_mutex_attr);
    g_ctrls_output_mutex = osMutexNew(&k_output_mutex_attr);
}

// ─── ControlLaw ───────────────────────────────────────────────────────────────
ControlLaw::ControlLaw(const Physics& phys, const RocketConfig& cfg)
    : phys_(phys), cfg_(cfg) {}

void ControlLaw::reset() {
    last_u_ = 0.0f;
    last_t_ = 0.0f;
}

float ControlLaw::gainSchedule_(float t, const StateVec& xhat) const {
    // Air-relative speed
    const float v1=xhat(3,0), v2=xhat(4,0), v3=xhat(5,0);
    const float vmag = sqrtf(v1*v1 + v2*v2 + v3*v3);
    if (vmag < cfg_.min_control_speed) return 0.0f;

    const float dM_dzeta = phys_.canardMomentJacobian(vmag);  // N·m/rad
    if (fabsf(dM_dzeta) < 1e-12f) return 0.0f;

    const float I3 = phys_.getInertiaRoll(t);
    return I3 * cfg_.roll_damping_lambda / dM_dzeta;
}

float ControlLaw::computeControl(float t, const StateVec& xhat, float T_K) {
    const float dt = t - last_t_;
    last_t_ = t;

    // Roll rate error (reference = 0)
    const float error = xhat(2, 0);  // w3

    // Gain-scheduled proportional output
    const float K    = gainSchedule_(t, xhat);
    float u_cmd = -K * error;

    // Rate limit
    if (dt > 0.0f && dt < 0.5f) {
        const float du_max = cfg_.max_deflection_rate * dt;
        if (u_cmd - last_u_ >  du_max) u_cmd = last_u_ + du_max;
        if (u_cmd - last_u_ < -du_max) u_cmd = last_u_ - du_max;
    }

    // Saturation
    if (u_cmd >  cfg_.max_deflection_rad) u_cmd =  cfg_.max_deflection_rad;
    if (u_cmd < -cfg_.max_deflection_rad) u_cmd = -cfg_.max_deflection_rad;

    // IREC compliance: inhibit during motor burn
    if (cfg_.irec_compliant && t < cfg_.t_burnout) u_cmd = 0.0f;

    // Pre-launch: no command
    if (t <= 0.0f) u_cmd = 0.0f;

    last_u_ = u_cmd;
    return u_cmd;
}

// ─── FreeRTOS controls task ───────────────────────────────────────────────────
// Runs at ~100 Hz. Maintains its own EKF and ControlLaw instances.
// Reads g_ctrls_sensor (written by main FC task) and writes g_ctrls_canard_cmd_deg.
extern "C" void ctrlsTask(void* /*arg*/) {
    // ── Build default config (caller should populate before osKernelStart) ──
    // The config is read from a globally accessible instance. For now we use
    // a local default; replace with a pointer/reference to the flight config
    // once the FlightComputer exposes it.
    static RocketConfig cfg;
    cfg.computeDerived();

    static Physics     phys(cfg);
    static EKF         ekf(phys, cfg);
    static ControlLaw  ctrl(phys, cfg);

    // Local copy of last sensor snapshot (used as fallback if no fresh data)
    CtrlsSensorSnapshot snap = {};
    snap.temperature_K = 288.15f;

    uint32_t last_tick = osKernelGetTickCount();

    for (;;) {
        // Wait until next 10 ms slot (100 Hz)
        osDelayUntil(last_tick + 10U);
        last_tick = osKernelGetTickCount();

        // ── Compute dt ──────────────────────────────────────────────────────
        static uint32_t prev_ms = 0U;
        const uint32_t  now_ms  = millis();
        const float     dt      = (prev_ms == 0U) ? 0.01f
                                  : static_cast<float>(now_ms - prev_ms) * 1e-3f;
        prev_ms = now_ms;

        if (dt <= 0.0f || dt > 0.5f) continue;  // guard: timer rollover / first call

        // ── Read sensor snapshot ─────────────────────────────────────────────
        bool fresh = false;
        if (osMutexAcquire(g_ctrls_sensor_mutex, 2U) == osOK) {
            if (g_ctrls_sensor.fresh) {
                snap  = g_ctrls_sensor;
                g_ctrls_sensor.fresh = false;
                fresh = true;
            }
            osMutexRelease(g_ctrls_sensor_mutex);
        }
        // If mutex unavailable or no fresh data, snap retains last known values
        // and fresh = false → predictOnly path below

        const float t   = snap.flight_time_s;
        const float alt = snap.altitude_m;
        const float T_K = snap.temperature_K;

        // ── EKF step ─────────────────────────────────────────────────────────
        // Read last canard command for the EOM input
        float u_last = 0.0f;
        if (osMutexAcquire(g_ctrls_output_mutex, 1U) == osOK) {
            u_last = g_ctrls_canard_cmd_deg * (M_PI / 180.0f);
            osMutexRelease(g_ctrls_output_mutex);
        }

        if (fresh) {
            // Build measurement vector [accel_g(3), gyro_rad_s(3)]
            MeasVec y;
            for (int i = 0; i < 3; i++) {
                y(i,   0) = snap.accel_g[i];
                y(3+i, 0) = snap.gyro_rad_s[i];
            }
            ekf.update(t, dt, y, u_last, alt, T_K);
        } else {
            ekf.predictOnly(t, dt, u_last, alt, T_K);
        }

        // ── Control law ───────────────────────────────────────────────────────
        const float u_rad = ctrl.computeControl(t, ekf.xhat, T_K);
        const float u_deg = u_rad * (180.0f / M_PI);

        // ── Write output ─────────────────────────────────────────────────────
        if (osMutexAcquire(g_ctrls_output_mutex, 2U) == osOK) {
            g_ctrls_canard_cmd_deg = u_deg;
            osMutexRelease(g_ctrls_output_mutex);
        }
    }
}
