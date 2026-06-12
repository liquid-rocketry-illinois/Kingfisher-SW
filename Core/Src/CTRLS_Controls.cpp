// Created by admin on 5/10/2026.

#include "CTRLS_Controls.h"
#include "timing.h"   // for millis()
#include "cmsis_os.h"

// Global vars. declare extern in other files to access, MUST use same name and type!
CtrlsSensorSnapshot g_SensorData           = {};
osMutexId_t         g_ctrls_sensor_mutex   = nullptr;
float               g_ctrls_canard_cmd_deg = 0.0f;
osMutexId_t         g_ctrls_output_mutex   = nullptr;
volatile bool       g_ctrls_enabled        = true;
volatile bool       g_ctrls_test_mode      = true;

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
    last_u_            = 0.0f;
    last_t_            = 0.0f;
    rem_sign_          = 1.0f;
    rem_flip_count_    = 0;
    rem_mismatch_count_= 0;
    rem_prev_t_        = -1.0f;
    rem_prev_w3_       = 0.0f;
}

float ControlLaw::gainSchedule_(float t, const StateVec& xhat, float alt) const {
    // Air-relative speed at the model/weather altitude, matching the EOM.
    const WeatherSample wx = phys_.weatherAtAltitude(alt);
    float va[3];
    phys_.airRelativeVelocity(xhat, wx.wind_x, wx.wind_y, va);
    const float vmag = sqrtf(va[0]*va[0] + va[1]*va[1] + va[2]*va[2]);
    if (vmag < cfg_.min_control_speed) return 0.0f;

    const float dM_dzeta = phys_.canardMomentJacobian(vmag);  // N·m/rad
    if (fabsf(dM_dzeta) < 1e-12f) return 0.0f;

    const float I3 = phys_.getInertiaRoll(t);
    return I3 * cfg_.roll_damping_lambda / dM_dzeta;
}

float ControlLaw::computeControl(float t, const StateVec& xhat, float alt) {
    const float dt = t - last_t_;
    last_t_ = t;

    // Roll rate error (reference = 0)
    const float error = xhat(2, 0);  // w3

    // Gain-scheduled proportional output (sign incorporates roll effectiveness)
    const float K    = gainSchedule_(t, xhat, alt);
    float u_cmd = -K * rem_sign_ * error;

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

// ─── Roll effectiveness monitor ───────────────────────────────────────────────
// Call once per loop with the CURRENT gyro w3, the PREVIOUS command in rad,
// and the current EKF speed estimate.  Flips rem_sign_ when the measured roll
// acceleration consistently opposes the modeled canard moment.
void ControlLaw::updateRollEffectivenessSign(float t, float w3_meas,
                                              float u_prev_rad, float vmag) {
    if (!cfg_.rem_enabled) return;

    const float prev_t  = rem_prev_t_;
    const float prev_w3 = rem_prev_w3_;
    rem_prev_t_  = t;
    rem_prev_w3_ = w3_meas;

    if (prev_t < 0.0f) return;  // first call — no previous sample yet

    const float dt = t - prev_t;
    if (dt <= 0.0f) { rem_mismatch_count_ = 0; return; }

    // Inhibit during burn and post-burn settling period
    if (t < cfg_.t_burnout + cfg_.rem_post_burn_delay_s) {
        rem_mismatch_count_ = 0; return;
    }

    // Skip if command is too small to produce a reliable signal
    if (fabsf(u_prev_rad) < cfg_.rem_min_cmd_rad) {
        rem_mismatch_count_ = 0; return;
    }

    // Expected roll acceleration from the canard moment model
    const float I3 = phys_.getInertiaRoll(t);
    const float M_cfd = phys_.canardMoment(vmag, u_prev_rad);
    const float expected_accel = (I3 > 1e-10f) ? (M_cfd / I3) : 0.0f;

    // Measured roll acceleration from consecutive gyro readings
    const float measured_accel = (w3_meas - prev_w3) / dt;

    // Require both sides to be significant before trusting the comparison
    if (fabsf(expected_accel) < cfg_.rem_min_expected_accel ||
        fabsf(measured_accel) < cfg_.rem_min_measured_accel) {
        rem_mismatch_count_ = 0; return;
        }

    // Accumulate or reset mismatch counter
    if (measured_accel * expected_accel < 0.0f) {
        rem_mismatch_count_++;
    } else {
        rem_mismatch_count_ = 0;
    }

    // Flip sign once threshold is reached
    const bool can_flip = cfg_.rem_allow_flip_back || (rem_flip_count_ == 0);
    if (can_flip && rem_mismatch_count_ >= cfg_.rem_required_mismatches) {
        rem_sign_ *= -1.0f;
        rem_flip_count_++;
        rem_mismatch_count_ = 0;
    }
}
