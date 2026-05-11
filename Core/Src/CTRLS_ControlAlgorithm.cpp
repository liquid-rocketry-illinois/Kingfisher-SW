// Created by admin on 5/10/2026.

#include "CTRLS_ControlAlgorithm.h"
#include <cstdlib>  // for abs

// ─── inv6: LU decomposition with partial pivoting ────────────────────────────
Mat<6,6> inv6(const Mat<6,6>& A) {
    Mat<6,6> LU = A;
    int piv[6] = {0,1,2,3,4,5};

    for (int k = 0; k < 6; k++) {
        // partial pivot
        int   mx  = k;
        float mxv = fabsf(LU(k,k));
        for (int i = k+1; i < 6; i++) {
            if (fabsf(LU(i,k)) > mxv) { mxv = fabsf(LU(i,k)); mx = i; }
        }
        if (mx != k) {
            for (int j = 0; j < 6; j++) { float t = LU(k,j); LU(k,j)=LU(mx,j); LU(mx,j)=t; }
            int  tp = piv[k]; piv[k]=piv[mx]; piv[mx]=tp;
        }
        if (fabsf(LU(k,k)) < 1e-12f) return eye<6>(); // singular fallback
        for (int i = k+1; i < 6; i++) {
            LU(i,k) /= LU(k,k);
            for (int j = k+1; j < 6; j++) LU(i,j) -= LU(i,k)*LU(k,j);
        }
    }

    Mat<6,6> inv;
    for (int col = 0; col < 6; col++) {
        float b[6] = {};
        b[col] = 1.0f;
        // apply row permutation
        for (int k = 0; k < 6; k++) {
            if (piv[k] != k) { float t = b[piv[k]]; b[piv[k]]=b[k]; b[k]=t; }
        }
        // forward substitution (unit lower triangular L)
        for (int i = 1; i < 6; i++)
            for (int j = 0; j < i; j++) b[i] -= LU(i,j)*b[j];
        // back substitution (upper triangular U)
        for (int i = 5; i >= 0; i--) {
            for (int j = i+1; j < 6; j++) b[i] -= LU(i,j)*b[j];
            b[i] /= LU(i,i);
        }
        for (int i = 0; i < 6; i++) inv(i,col) = b[i];
    }
    return inv;
}

// ─── lerp ─────────────────────────────────────────────────────────────────────
float lerp(const float* xs, const float* ys, int n, float xi) {
    if (n <= 0) return 0.0f;
    if (xi <= xs[0]) return ys[0];
    if (xi >= xs[n-1]) return ys[n-1];
    // binary search for bracket
    int lo = 0, hi = n-1;
    while (hi - lo > 1) {
        int mid = (lo+hi) >> 1;
        if (xs[mid] <= xi) lo = mid; else hi = mid;
    }
    float t = (xi - xs[lo]) / (xs[hi] - xs[lo]);
    return ys[lo] + t * (ys[hi] - ys[lo]);
}

// ─── Physics constructor ──────────────────────────────────────────────────────
Physics::Physics(const RocketConfig& c) : cfg(c) {
    // Precompute fin roll geometry constants (from fin_roll_moment_numeric())
    tau = (cfg.Ct > 0.0f) ? (cfg.Ct / cfg.Cr) : 0.0f;

    // Mean aerodynamic chord lateral center of pressure position
    y_ma = (cfg.s / 3.0f) * (1.0f + 2.0f*tau) / (1.0f + tau);

    // Tip radius from rocket centreline
    r_t = cfg.d * 0.5f + cfg.s;

    // Roll damping coefficient (integrated over fin span using strip theory):
    // C_ldw = integral from r_root to r_t of Cnalpha * r^2 dr normalised by v
    float r_root = cfg.d * 0.5f;
    // K_d encodes: N_fins * rho * Cnalpha * integral(r^2, r_root, r_t) / 2
    // At runtime: M_roll_damp = -K_d * rho * v_mag * w3
    C_ldw = (cfg.Cnalpha_fin / 3.0f) * (r_t*r_t*r_t - r_root*r_root*r_root);
    K_d   = cfg.N_fins * C_ldw;          // multiply by 0.5*rho*v at runtime

    // Fin cant forcing: K_f * q_dyn = roll moment from canted fins
    // M_cant = N * Cnalpha * delta * A_ref * q_dyn * y_ma  (linearised in delta)
    K_f = (float)cfg.N_fins * cfg.Cnalpha_fin * sinf(cfg.delta_rad) * cfg.A_ref * y_ma;
}

// ─── Time-varying parameters ──────────────────────────────────────────────────
float Physics::getMass(float t) const {
    if (t >= cfg.t_burnout) return cfg.m_f;
    return cfg.m_0 - cfg.m_prop * (t / cfg.t_burnout);
}

float Physics::getInertiaPitch(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_f;
    return cfg.I_0 + (cfg.I_f - cfg.I_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaRoll(float t) const {
    if (t >= cfg.t_burnout) return cfg.I_3_f;
    return cfg.I_3_0 + (cfg.I_3_f - cfg.I_3_0) * (t / cfg.t_burnout);
}

float Physics::getInertiaDotPitch(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_f - cfg.I_0) / cfg.t_burnout;
}

float Physics::getInertiaDotRoll(float t) const {
    if (t >= cfg.t_burnout) return 0.0f;
    return (cfg.I_3_f - cfg.I_3_0) / cfg.t_burnout;
}

float Physics::getCG(float t) const {
    if (t >= cfg.t_burnout) return cfg.x_CG_f;
    return cfg.x_CG_0 + (cfg.x_CG_f - cfg.x_CG_0) * (t / cfg.t_burnout);
}

float Physics::getThrust(float t) const {
    if (t < 0.0f || t > cfg.t_burnout || cfg.thrust_n == 0) return 0.0f;
    return lerp(cfg.thrust_t, cfg.thrust_N, cfg.thrust_n, t);
}

float Physics::getDragCoeff(float mach, float t) const {
    if (t < cfg.t_burnout && cfg.drag_on_n > 0)
        return lerp(cfg.drag_on_mach, cfg.drag_on_cd, cfg.drag_on_n, mach);
    if (cfg.drag_off_n > 0)
        return lerp(cfg.drag_off_mach, cfg.drag_off_cd, cfg.drag_off_n, mach);
    return 0.45f; // fallback
}

float Physics::speedOfSound(float T_kelvin) {
    return sqrtf(1.4f * 287.05f * T_kelvin);
}

// ─── R_BW: world-to-body DCM from unit quaternion ────────────────────────────
void Physics::R_BW(float qw, float qx, float qy, float qz, float R[3][3]) {
    const float qx2=qx*qx, qy2=qy*qy, qz2=qz*qz;
    R[0][0] = 1.0f - 2.0f*(qy2+qz2);   R[0][1] = 2.0f*(qx*qy+qw*qz);    R[0][2] = 2.0f*(qx*qz-qw*qy);
    R[1][0] = 2.0f*(qx*qy-qw*qz);      R[1][1] = 1.0f - 2.0f*(qx2+qz2); R[1][2] = 2.0f*(qy*qz+qw*qx);
    R[2][0] = 2.0f*(qx*qz+qw*qy);      R[2][1] = 2.0f*(qy*qz-qw*qx);    R[2][2] = 1.0f - 2.0f*(qx2+qy2);
}

// ─── Air-relative velocity in body frame ─────────────────────────────────────
void Physics::airRelativeVelocity(const StateVec& x, float v_air[3]) const {
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);
    // wind vector in world frame
    const float wx=cfg.v_wind_x, wy=cfg.v_wind_y;
    // rotate to body frame
    float w_body[3] = {
        R[0][0]*wx + R[0][1]*wy,
        R[1][0]*wx + R[1][1]*wy,
        R[2][0]*wx + R[2][1]*wy
    };
    v_air[0] = x(3,0) - w_body[0];
    v_air[1] = x(4,0) - w_body[1];
    v_air[2] = x(5,0) - w_body[2];
}

// ─── Canard moment (CFD numeric) ──────────────────────────────────────────────
float Physics::canardMoment(float v_mag, float zeta_rad) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag
           * (zeta_rad * (180.0f / M_PI));
}

float Physics::canardMomentJacobian(float v_mag) const {
    return cfg.canard_moment_coeff_per_deg * v_mag * v_mag * (180.0f / M_PI);
}

// ─── Full nonlinear EOM ───────────────────────────────────────────────────────
StateVec Physics::eom(float t, const StateVec& x, float u_zeta,
                      float /*alt*/, float T_K) const {
    // Extract state
    float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    // Normalize quaternion
    float qn = sqrtf(qw*qw+qx*qx+qy*qy+qz*qz);
    if (qn > 1e-8f) { qw/=qn; qx/=qn; qy/=qn; qz/=qn; }

    // Time-varying params
    const float m    = getMass(t);
    const float I1   = getInertiaPitch(t);
    const float I3   = getInertiaRoll(t);
    const float I1d  = getInertiaDotPitch(t);
    const float I3d  = getInertiaDotRoll(t);
    const float xCG  = getCG(t);
    const float Thr  = getThrust(t);

    // Rotation matrix
    float R[3][3];
    R_BW(qw, qx, qy, qz, R);

    // Air-relative velocity
    float va[3];
    airRelativeVelocity(x, va);

    // Airspeed
    const float vmag2 = va[0]*va[0]+va[1]*va[1]+va[2]*va[2];
    const float vmag  = sqrtf(vmag2);
    const float vtrans= sqrtf(va[0]*va[0]+va[1]*va[1]);

    // Angle of attack (from axial z-body), clamped to ±15 deg
    static constexpr float AoA_MAX = 15.0f * (M_PI / 180.0f);
    float alpha = 0.0f;
    if (vmag > 0.1f)
        alpha = atan2f(vtrans, fabsf(va[2]));
    if (alpha >  AoA_MAX) alpha =  AoA_MAX;
    if (alpha < -AoA_MAX) alpha = -AoA_MAX;

    // Mach, Cd, dynamic pressure
    const float c_snd = speedOfSound(T_K);
    const float mach  = (c_snd > 0.0f) ? vmag / c_snd : 0.0f;
    const float Cd    = getDragCoeff(mach, t);
    const float rho   = cfg.rho_0;
    const float q_dyn = 0.5f * rho * vmag2;

    // Rail: no forces/moments before release
    const bool on_rail = (t < cfg.t_rail);

    // ── Forces (body frame) ──────────────────────────────────────────────────
    float F[3] = {0.0f, 0.0f, 0.0f};

    if (!on_rail) {
        // Gravity: g_world = [0,0,-g0]; g_body = R_BW * g_world = -g0 * R[:,2]
        F[0] += m * (-cfg.g_0 * R[0][2]);
        F[1] += m * (-cfg.g_0 * R[1][2]);
        F[2] += m * (-cfg.g_0 * R[2][2]);

        // Thrust along +z body axis
        F[2] += Thr;

        // Aerodynamic drag (along -v_air)
        if (vmag > 0.1f) {
            const float D = Cd * cfg.A_ref * q_dyn / vmag;
            F[0] -= D * va[0];
            F[1] -= D * va[1];
            F[2] -= D * va[2];
        }

        // Base + canard Barrowman normal force (transverse, creates pitch/yaw restoring)
        if (vmag > 0.1f && vtrans > 1e-6f) {
            const float sin_a = sinf(alpha);
            const float N = (cfg.base_cn + cfg.canard_cn) * sin_a * q_dyn * cfg.A_ref;
            F[0] += N * (va[0] / vtrans);
            F[1] += N * (va[1] / vtrans);
        }
    }

    // ── Moments (body frame) ─────────────────────────────────────────────────
    float M[3] = {0.0f, 0.0f, 0.0f};

    if (!on_rail) {
        const float sin_a = sinf(alpha);

        // Pitch/yaw restoring moments from base and canard CP-CG separation
        if (vmag > 0.1f && vtrans > 1e-6f) {
            const float N_base = cfg.base_cn   * sin_a * q_dyn * cfg.A_ref;
            const float N_can  = cfg.canard_cn * sin_a * q_dyn * cfg.A_ref;
            const float M_base = N_base * (cfg.base_cp  - xCG);  // +ve = destabilising
            const float M_can  = N_can  * (cfg.canard_cp - xCG);
            const float M_tot  = M_base + M_can;
            // Moment direction: perpendicular to axial axis, in transverse velocity plane
            M[0] += M_tot * (-va[1] / vtrans);
            M[1] += M_tot * ( va[0] / vtrans);
        }

        // Pitch/yaw aerodynamic damping (pitch rate × dynamic pressure × geometry)
        const float Cdamp = 0.5f * (cfg.base_cn + cfg.canard_cn)
                          * cfg.A_ref * cfg.L_ne * cfg.L_ne * q_dyn / fmaxf(vmag, 1.0f);
        M[0] -= Cdamp * w1;
        M[1] -= Cdamp * w2;

        // Fin cant roll forcing (proportional to dynamic pressure)
        M[2] += K_f * q_dyn;

        // Roll aerodynamic damping: M = -0.5 * rho * v * K_d * w3
        M[2] -= 0.5f * rho * vmag * K_d * w3;

        // Canard roll moment (CFD numeric)
        M[2] += canardMoment(vmag, u_zeta);
    }

    // ── Euler rigid-body (I1=I2 for symmetric rocket) ────────────────────────
    const float I2 = I1;
    StateVec xdot;
    xdot(0,0) = ((I2-I3)*w2*w3 + M[0] - I1d*w1) / I1;
    xdot(1,0) = ((I3-I1)*w3*w1 + M[1] - I1d*w2) / I1;
    xdot(2,0) = ((I1-I2)*w1*w2 + M[2] - I3d*w3) / I3;

    // ── Velocity: vdot = F/m - omega×v ───────────────────────────────────────
    xdot(3,0) = F[0]/m - (w2*v3 - w3*v2);
    xdot(4,0) = F[1]/m - (w3*v1 - w1*v3);
    xdot(5,0) = F[2]/m - (w1*v2 - w2*v1);

    // ── Quaternion kinematics: qdot = 0.5 * Omega(w) * q ─────────────────────
    xdot(6,0) = 0.5f*(-w1*qx - w2*qy - w3*qz);
    xdot(7,0) = 0.5f*( w1*qw + w3*qy - w2*qz);
    xdot(8,0) = 0.5f*( w2*qw - w3*qx + w1*qz);
    xdot(9,0) = 0.5f*( w3*qw + w2*qx - w1*qy);

    return xdot;
}

// ─── Numerical Jacobian A = df/dx (central finite differences) ───────────────
StateMat Physics::jacobianA(float t, const StateVec& x, float u,
                             float alt, float T_K) const {
    static constexpr float EPS = 1e-5f;
    StateMat A;
    for (int j = 0; j < 10; j++) {
        StateVec xp = x, xm = x;
        xp(j,0) += EPS;
        xm(j,0) -= EPS;
        StateVec fp = eom(t, xp, u, alt, T_K);
        StateVec fm = eom(t, xm, u, alt, T_K);
        for (int i = 0; i < 10; i++)
            A(i,j) = (fp(i,0) - fm(i,0)) / (2.0f * EPS);
    }
    return A;
}

// ─── Analytic B = df/du ───────────────────────────────────────────────────────
// Only roll row (index 2) is nonzero: dw3dot/du = dM_canard/du / I3
InputMat Physics::jacobianB(float t, const StateVec& x) const {
    float va[3];
    airRelativeVelocity(x, va);
    const float vmag = sqrtf(va[0]*va[0]+va[1]*va[1]+va[2]*va[2]);
    const float I3   = getInertiaRoll(t);
    InputMat B;
    B(2,0) = (I3 > 1e-10f) ? (canardMomentJacobian(vmag) / I3) : 0.0f;
    return B;
}

// ─── Measurement Jacobian C = dy/dx (6×10) ───────────────────────────────────
MeasMat Physics::jacobianC(float t, const StateVec& x, float u,
                            const StateMat& A) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);
    const float g0 = cfg.g_0;

    // J_cross = ∂(ω×v)/∂x  [3×10]
    // ω×v = [w2*v3-w3*v2, w3*v1-w1*v3, w1*v2-w2*v1]
    Mat<3,10> Jcross;
    // ∂/∂w1: [0, -v3, v2]
    Jcross(0,0)=0.0f;  Jcross(1,0)=-v3;   Jcross(2,0)= v2;
    // ∂/∂w2: [v3, 0, -v1]
    Jcross(0,1)= v3;   Jcross(1,1)=0.0f;  Jcross(2,1)=-v1;
    // ∂/∂w3: [-v2, v1, 0]
    Jcross(0,2)=-v2;   Jcross(1,2)= v1;   Jcross(2,2)=0.0f;
    // ∂/∂v1: [0, w3, -w2]
    Jcross(0,3)=0.0f;  Jcross(1,3)= w3;   Jcross(2,3)=-w2;
    // ∂/∂v2: [-w3, 0, w1]
    Jcross(0,4)=-w3;   Jcross(1,4)=0.0f;  Jcross(2,4)= w1;
    // ∂/∂v3: [w2, -w1, 0]
    Jcross(0,5)= w2;   Jcross(1,5)=-w1;   Jcross(2,5)=0.0f;
    // ∂/∂q = 0 (columns 6-9 already zero)

    // J_g = ∂(g_body)/∂x  [3×10]
    // g_body = R_BW * [0,0,-g0]; only q-columns nonzero
    // R_BW col-2: [2(qx*qz-qw*qy), 2(qy*qz+qw*qx), 1-2(qx²+qy²)]
    Mat<3,10> Jg;
    // ∂g_body/∂qw
    Jg(0,6) = -g0*(-2.0f*qy);     // ∂(-g0*2(qxqz-qwqy))/∂qw = 2*g0*qy? let me re-check
    // g_body[0] = -g0 * 2*(qx*qz - qw*qy)
    // ∂g_body[0]/∂qw = -g0 * 2*(-qy) = 2*g0*qy
    Jg(0,6) = 2.0f*g0*qy;
    Jg(0,7) = -2.0f*g0*qz;         // ∂/∂qx = -g0*2*qz
    Jg(0,8) = 2.0f*g0*qw;          // ∂/∂qy = -g0*2*(-qw) = 2*g0*qw
    Jg(0,9) = -2.0f*g0*qx;         // ∂/∂qz = -g0*2*qx

    // g_body[1] = -g0 * 2*(qy*qz + qw*qx)
    Jg(1,6) = -2.0f*g0*qx;         // ∂/∂qw = -g0*2*qx
    Jg(1,7) = -2.0f*g0*qw;         // ∂/∂qx = -g0*2*qw
    Jg(1,8) = -2.0f*g0*qz;         // ∂/∂qy = -g0*2*qz
    Jg(1,9) = -2.0f*g0*qy;         // ∂/∂qz = -g0*2*qy

    // g_body[2] = -g0 * (1 - 2*(qx²+qy²))
    // ∂/∂qw = 0
    Jg(2,7) = 4.0f*g0*qx;          // ∂/∂qx = -g0*(-4*qx) = 4*g0*qx
    Jg(2,8) = 4.0f*g0*qy;          // ∂/∂qy = 4*g0*qy

    // Build C matrix
    MeasMat C;

    // Accel rows 0-2: C[0:3,:] = (A[3:6,:] + J_cross - J_g) / g0
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 10; j++) {
            C(i, j) = (A(3+i, j) + Jcross(i, j) - Jg(i, j)) / g0;
        }
    }
    // Zero angular-rate columns on accel rows (accel doesn't directly observe w)
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C(i, j) = 0.0f;

    // Gyro rows 3-5: identity on angular-rate columns
    C(3,0) = 1.0f;
    C(4,1) = 1.0f;
    C(5,2) = 1.0f;

    return C;
}

// ─── Measurement prediction ───────────────────────────────────────────────────
// Returns [accel_g (3), gyro_rad_s (3)]
MeasVec Physics::predictMeasurement(float t, const StateVec& x, float u,
                                    float alt, float T_K) const {
    const float w1=x(0,0), w2=x(1,0), w3=x(2,0);
    const float v1=x(3,0), v2=x(4,0), v3=x(5,0);
    const float qw=x(6,0), qx=x(7,0), qy=x(8,0), qz=x(9,0);

    StateVec xdot = eom(t, x, u, alt, T_K);

    // vdot (body) = xdot[3:6]
    // Specific force = vdot + omega×v - g_body, divided by g to get g units
    // omega × v
    float oxv[3] = {
        w2*v3 - w3*v2,
        w3*v1 - w1*v3,
        w1*v2 - w2*v1
    };

    // g_body = R_BW * [0,0,-g0]
    float R[3][3];
    Physics::R_BW(qw, qx, qy, qz, R);
    float g_body[3] = {
        R[0][2] * (-cfg.g_0),
        R[1][2] * (-cfg.g_0),
        R[2][2] * (-cfg.g_0)
    };

    MeasVec y;
    for (int i = 0; i < 3; i++)
        y(i, 0) = (xdot(3+i, 0) + oxv[i] - g_body[i]) / cfg.g_0;

    // Gyro: direct angular rate observation
    y(3,0) = w1;
    y(4,0) = w2;
    y(5,0) = w3;

    return y;
}
