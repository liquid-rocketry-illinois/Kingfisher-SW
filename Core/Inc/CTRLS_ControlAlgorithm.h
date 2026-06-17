// Created by admin on 5/10/2026.

#ifndef KINGFISHER_SW_CONTROLALGORITHM_H
#define KINGFISHER_SW_CONTROLALGORITHM_H

#include <cmath>
#include <cstring>
#include <cstdint>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// ─── Fixed-size matrix library (zero heap, all stack) ────────────────────────
template<int R, int C>
struct Mat {
    float d[R][C];
    Mat() { memset(d, 0, sizeof(d)); }
    float&       operator()(int r, int c)       { return d[r][c]; }
    const float& operator()(int r, int c) const { return d[r][c]; }
};

template<int R, int K, int C>
inline Mat<R,C> operator*(const Mat<R,K>& A, const Mat<K,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) for(int k=0;k<K;k++)
        out(r,c) += A(r,k)*B(k,c);
    return out;
}

template<int R, int C>
inline Mat<R,C> operator+(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = A(r,c)+B(r,c);
    return out;
}

template<int R, int C>
inline Mat<R,C> operator-(const Mat<R,C>& A, const Mat<R,C>& B) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = A(r,c)-B(r,c);
    return out;
}

template<int R, int C>
inline Mat<C,R> Tr(const Mat<R,C>& A) {
    Mat<C,R> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(c,r) = A(r,c);
    return out;
}

template<int N>
inline Mat<N,N> eye() {
    Mat<N,N> I;
    for(int i=0;i<N;i++) I(i,i) = 1.0f;
    return I;
}

template<int R, int C>
inline Mat<R,C> operator*(float s, const Mat<R,C>& A) {
    Mat<R,C> out;
    for(int r=0;r<R;r++) for(int c=0;c<C;c++) out(r,c) = s*A(r,c);
    return out;
}

// LU inversion for 6×6 (used in EKF S^-1 step). Returns identity on singularity.
Mat<6,6> inv6(const Mat<6,6>& A);

// Type aliases matching the spec
using StateVec = Mat<10,1>;   // [w1,w2,w3, v1,v2,v3, qw,qx,qy,qz]
using StateMat = Mat<10,10>;
using MeasVec  = Mat<6,1>;    // [ax,ay,az, gx,gy,gz]
using MeasMat  = Mat<6,10>;
using InputMat = Mat<10,1>;   // B column (df/du), scalar u

struct WeatherSample {
    float rho           = 1.225f;   // kg/m^3
    float temperature_K = 288.15f;  // K
    float g             = 9.80665f; // m/s^2
    float wind_x        = 0.0f;     // world-frame m/s
    float wind_y        = 0.0f;     // world-frame m/s
};

// ─── Rocket configuration ─────────────────────────────────────────────────────
// Fallback defaults only. Call loadControlFreakRocketConfig() before flight.
struct RocketConfig {
    // Mass (kg)
    float m_0   = 14.0f;   // launch mass
    float m_f   = 11.5f;   // burnout mass
    float m_prop= 2.5f;    // propellant mass

    // Inertia (kg·m²)
    float I_0   = 0.85f;   // pitch/yaw at ignition
    float I_f   = 0.70f;   // pitch/yaw at burnout
    float I_3_0 = 0.008f;  // roll at ignition
    float I_3_f = 0.006f;  // roll at burnout

    // CG travel (m from nose)
    float x_CG_0 = 0.95f;
    float x_CG_f = 0.90f;

    // Geometry
    float d    = 0.098f;   // airframe diameter (m)
    float L_ne = 1.85f;    // nose-to-nozzle length (m)

    // Timing (s)
    float t_rail    = 0.6f;
    float t_burnout = 3.5f;
    float t_apogee  = 18.0f;

    // Barrowman aero
    float base_cn              = 3.2f;
    float base_cp              = 1.15f;  // m from nose
    float canard_cn            = 0.8f;
    float canard_cp            = 0.12f;  // m from nose (near tip)
    float canard_plane_angle_deg = 45.0f;

    // Fin roll geometry
    int   N_fins       = 4;
    float Cr           = 0.12f;  // root chord (m)
    float Ct           = 0.06f;  // tip chord (m)
    float s            = 0.07f;  // semi-span (m)
    float delta_deg    = 2.0f;   // fin cant angle (deg)
    float Cnalpha_fin  = 2.0f;   // fin normal force slope (per rad)

    // Canard CFD roll moment
    float canard_moment_coeff_per_deg = -2.23e-6f;  // N·m/(m²/s²·deg)

    // Controller
    float roll_damping_lambda  = 30.0f;    // desired closed-loop roll pole (1/s)
    float max_deflection_rad   = 0.2618f;  // 15 deg
    float max_deflection_rate  = 1.0472f;  // 60 deg/s in rad/s
    float min_control_speed    = 30.0f;    // m/s — below this gain = 0
    bool  irec_compliant       = true;     // inhibit control during motor burn

    // EKF process noise
    float Q_angular[3]    = {1e-4f, 1e-4f, 1e-4f};
    float Q_velocity[3]   = {1e-2f, 1e-2f, 1e-2f};
    float Q_quat[4]       = {1e-6f, 1e-6f, 1e-6f, 1e-6f};

    // EKF measurement noise
    float accel_noise_std_g  = 0.05f;   // g units, nominal flight
    float gyro_noise_std     = 0.01f;   // rad/s
    float accel_model_std_g  = 0.1f;    // g units, model mismatch
    float accel_burn_std_g   = 0.5f;    // g units, inflated during burn
    float P0_scale           = 0.1f;    // initial covariance scaling

    // Environment
    float rho_0     = 1.225f;   // kg/m³ sea level air density
    float g_0       = 9.80665f; // m/s²
    float v_wind_x  = 0.0f;     // world-frame wind (m/s)
    float v_wind_y  = 0.0f;

    // Initial state. Control Freak runtime overrides this from rail-angle setup.
    float x0[10] = {0,0,0, 0,0,0, 1,0,0,0};

    // Thrust curve table (time vs Newtons)
    static constexpr int MAX_TABLE = 132;
    float thrust_t[MAX_TABLE] = {};
    float thrust_N[MAX_TABLE] = {};
    int   thrust_n = 0;

    // Power-on drag table (Mach vs Cd)
    float drag_on_mach[MAX_TABLE] = {};
    float drag_on_cd[MAX_TABLE]   = {};
    int   drag_on_n = 0;

    // Power-off drag table
    float drag_off_mach[MAX_TABLE] = {};
    float drag_off_cd[MAX_TABLE]   = {};
    int   drag_off_n = 0;

    // ── Roll effectiveness monitor (REM) ──────────────────────────────────────
    // Detects reversed canard roll effectiveness at runtime and flips the
    // control gain sign.  Disabled by default.
    bool  rem_enabled             = false;
    float rem_post_burn_delay_s   = 0.25f;   // s after burnout before monitoring
    float rem_min_cmd_rad         = 0.0349f; // ~2 deg — skip tiny commands
    float rem_min_expected_accel  = 0.5f;    // rad/s² — minimum expected |w3dot|
    float rem_min_measured_accel  = 0.5f;    // rad/s² — minimum measured |w3dot|
    int   rem_required_mismatches = 5;       // consecutive sign disagreements to flip
    bool  rem_allow_flip_back     = true;    // allow more than one flip

    // Derived — call computeDerived() once after populating all fields above
    float delta_rad      = 0.0f;
    float canard_dir_1   = 1.0f;  // cos(canard_plane_angle)
    float canard_dir_2   = 0.0f;  // sin(canard_plane_angle)
    float A_ref          = 0.0f;  // π*(d/2)²

    void computeDerived() {
        delta_rad  = delta_deg * (M_PI / 180.0f);
        const float cpa = canard_plane_angle_deg * (M_PI / 180.0f);
        canard_dir_1 = cosf(cpa);
        canard_dir_2 = sinf(cpa);
        A_ref = M_PI * (d*0.5f) * (d*0.5f);
    }
};

// Populates RocketConfig from FV-Controls/rockets/Control Freak/sim setup.
void loadControlFreakRocketConfig(RocketConfig& cfg);

// Piecewise-linear interpolation with binary search. Clamps at endpoints.
float lerp(const float* xs, const float* ys, int n, float xi);

// ─── Physics ──────────────────────────────────────────────────────────────────
class Physics {
public:
    const RocketConfig& cfg; // Use global config (assigned later)

    explicit Physics(const RocketConfig& cfg);

    // Time-varying parameter accessors
    float getMass(float t) const;
    float getInertiaPitch(float t) const;
    float getInertiaRoll(float t) const;
    float getInertiaDotPitch(float t) const;
    float getInertiaDotRoll(float t) const;
    float getCG(float t) const;
    float getThrust(float t) const;
    float getDragCoeff(float mach, float t) const;
    WeatherSample weatherAtAltitude(float alt_m) const;
    static float speedOfSound(float T_kelvin);

    // Rotation helpers
    // R[3][3] = world-to-body DCM from unit quaternion
    static void R_BW(float qw, float qx, float qy, float qz, float R[3][3]);
    // Air-relative velocity in body frame (body velocity minus rotated wind)
    void airRelativeVelocity(const StateVec& x, float v_air[3]) const;
    void airRelativeVelocity(const StateVec& x, float wind_x, float wind_y,
                             float v_air[3]) const;

    // Canard roll moment (CFD numeric model)
    float canardMoment(float v_mag, float zeta_rad) const;
    float canardMomentJacobian(float v_mag) const;

    // Full nonlinear equations of motion
    StateVec eom(float t, const StateVec& x, float u_zeta, float alt = 0.0f) const;

    // Numerical Jacobian df/dx (20 eom calls via central differences)
    StateMat jacobianA(float t, const StateVec& x, float u, float alt = 0.0f) const;

    // Analytic B column (df/du) — only roll row is nonzero
    InputMat jacobianB(float t, const StateVec& x) const;

    // Measurement Jacobian dy/dx (6×10)
    MeasMat jacobianC(float t, const StateVec& x, float u,
                      const StateMat& A, float alt = 0.0f) const;

    // IMU measurement prediction (accel in g, gyro in rad/s)
    MeasVec predictMeasurement(float t, const StateVec& x, float u,
                               float alt = 0.0f) const;

private:
    // Precomputed fin roll geometry constants (computed in constructor)
    float K_f;   // fin cant roll forcing coefficient (N·m per unit dynamic pressure)
    float K_d;   // roll damping coefficient used as 0.5*rho*v*K_d*w3
    float y_ma;  // mean aerodynamic chord lateral offset (m)
    float C_ldw; // roll damping derivative
    float r_t;   // body radius at fin root (m)
    float tau;   // Barrowman span ratio (s + r_t) / r_t
};

#endif // KINGFISHER_SW_CONTROLALGORITHM_H
