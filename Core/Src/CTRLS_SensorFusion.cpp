// Created by admin on 5/10/2026.

#include "CTRLS_SensorFusion.h"

// ─── Constructor ──────────────────────────────────────────────────────────────
EKF::EKF(const Physics& phys, const RocketConfig& cfg)
    : phys_(phys), cfg_(cfg) {
    reset(cfg);
}

void EKF::reset(const RocketConfig& cfg) {
    // Initial state from config
    for (int i = 0; i < 10; i++) xhat(i,0) = cfg.x0[i];

    // Initial covariance: scaled identity
    P_ = cfg.P0_scale * eye<10>();

    // Process noise Q (diagonal)
    Q_ = Mat<10,10>();
    for (int i = 0; i < 3; i++) Q_(i,   i)   = cfg.Q_angular[i];
    for (int i = 0; i < 3; i++) Q_(3+i, 3+i) = cfg.Q_velocity[i];
    for (int i = 0; i < 4; i++) Q_(6+i, 6+i) = cfg.Q_quat[i];

    // Measurement noise R (diagonal 6×6)
    R_ = Mat<6,6>();
    const float accel_var = cfg.accel_noise_std_g * cfg.accel_noise_std_g;
    const float gyro_var  = cfg.gyro_noise_std    * cfg.gyro_noise_std;
    for (int i = 0; i < 3; i++) R_(i,   i)   = accel_var;
    for (int i = 0; i < 3; i++) R_(3+i, 3+i) = gyro_var;
}

// ─── predict_ ─────────────────────────────────────────────────────────────────
void EKF::predict_(float t, float dt, float u, float alt, StateMat& A_out) {
    if (dt <= 0.0f || dt > 0.5f) return;

    // Euler integration
    StateVec xdot = phys_.eom(t, xhat, u, alt);
    for (int i = 0; i < 10; i++) xhat(i,0) += xdot(i,0) * dt;

    normalizeQuaternion_();

    // Linearise at updated state
    A_out = phys_.jacobianA(t, xhat, u, alt);

    // Discrete F = I + A*dt
    StateMat F = eye<10>() + dt * A_out;

    // P = F*P*F^T + Q
    P_ = F * P_ * Tr(F) + Q_;
}

// ─── correct_ ─────────────────────────────────────────────────────────────────
void EKF::correct_(float t, float u, const MeasVec& y_meas,
                   float alt, const StateMat& A,
                   bool burning, bool gyro_only) {
    // Build effective R (inflate accel during burn)
    Mat<6,6> R_eff = R_;
    if (burning) {
        const float bv = cfg_.accel_burn_std_g * cfg_.accel_burn_std_g;
        for (int i = 0; i < 3; i++) R_eff(i,i) = bv;
    }

    // Measurement Jacobian
    MeasMat C;
    if (gyro_only) {
        C(3,0) = 1.0f;
        C(4,1) = 1.0f;
        C(5,2) = 1.0f;
    } else {
        C = phys_.jacobianC(t, xhat, u, A, alt);
    }

    // Innovation: y_pred from noiseless sensor model
    MeasVec y_pred = phys_.predictMeasurement(t, xhat, u, alt);
    MeasVec innov;
    for (int i = 0; i < 6; i++) innov(i,0) = y_meas(i,0) - y_pred(i,0);

    // Innovation covariance S = C*P*C^T + R_eff
    Mat<6,6> S = C * P_ * Tr(C) + R_eff;

    // Kalman gain K = P*C^T*S^-1  (10×6)
    Mat<10,6> K = P_ * Tr(C) * inv6(S);

    // Gain limiting per spec:
    // - Zero accel columns (0-2) for rate rows (0-2) and quat rows (6-9)
    // - Zero gyro columns (3-5) for velocity rows (3-5) and quat rows (6-9)
    // - Zero all accel columns during burn
    for (int j = 0; j < 3; j++) {
        for (int i = 0; i < 3;  i++) K(i, j) = 0.0f;   // rate rows, accel cols
        for (int i = 6; i < 10; i++) K(i, j) = 0.0f;   // quat rows, accel cols
        if (burning)
            for (int i = 0; i < 10; i++) K(i, j) = 0.0f; // all rows, accel cols
    }
    for (int j = 3; j < 6; j++) {
        for (int i = 3; i < 6;  i++) K(i, j) = 0.0f;   // vel rows, gyro cols
        for (int i = 6; i < 10; i++) K(i, j) = 0.0f;   // quat rows, gyro cols
    }

    // State update
    StateVec dx = K * innov;
    for (int i = 0; i < 10; i++) xhat(i,0) += dx(i,0);

    // Covariance update (Joseph form: guarantees PSD)
    StateMat IKC = eye<10>() - K * C;
    P_ = IKC * P_ * Tr(IKC) + K * R_eff * Tr(K);
}

// ─── Public API ───────────────────────────────────────────────────────────────
StateVec EKF::update(float t, float dt, const MeasVec& y_meas, float u,
                     float alt) {
    const bool on_rail = (t < cfg_.t_rail);
    const bool burning = (t >= 0.0f && t < cfg_.t_burnout);

    if (on_rail) {
        applyRailConstraint_();
        // On rail: only gyro rows active in C (simplified 3×10 gyro-identity block)
        // We still run predict + partial correct for gyro channels
        StateMat A;
        predict_(t, dt, u, alt, A);
        applyRailConstraint_();
        // Gyro-only correction (use only rows 3-5 of y_meas)
        correct_(t, u, y_meas, alt, A, false, true);
        applyRailConstraint_();
    } else {
        StateMat A;
        predict_(t, dt, u, alt, A);
        correct_(t, u, y_meas, alt, A, burning);
    }

    normalizeQuaternion_();
    return xhat;
}

StateVec EKF::updateGyroOnly(float t, float dt, const MeasVec& y_meas, float u,
                             float alt) {
    StateMat A;
    predict_(t, dt, u, alt, A);
    correct_(t, u, y_meas, alt, A, false, true);
    normalizeQuaternion_();
    return xhat;
}

void EKF::predictOnly(float t, float dt, float u, float alt) {
    StateMat A;
    predict_(t, dt, u, alt, A);
    normalizeQuaternion_();
}

// ─── Private helpers ──────────────────────────────────────────────────────────
void EKF::applyRailConstraint_() {
    // Zero pitch/yaw rates and lateral velocities; lock quaternion to initial
    xhat(0,0) = 0.0f;  // w1 (pitch rate)
    xhat(1,0) = 0.0f;  // w2 (yaw rate)
    xhat(3,0) = 0.0f;  // v1 (lateral)
    xhat(4,0) = 0.0f;  // v2 (lateral)
    if (xhat(5,0) < 0.0f) xhat(5,0) = 0.0f;  // v3 >= 0 on rail
    // Lock quaternion to initial orientation
    xhat(6,0) = cfg_.x0[6];
    xhat(7,0) = cfg_.x0[7];
    xhat(8,0) = cfg_.x0[8];
    xhat(9,0) = cfg_.x0[9];
}

void EKF::normalizeQuaternion_() {
    const float n = sqrtf(xhat(6,0)*xhat(6,0) + xhat(7,0)*xhat(7,0)
                        + xhat(8,0)*xhat(8,0) + xhat(9,0)*xhat(9,0));
    if (n > 1e-8f) {
        xhat(6,0) /= n; xhat(7,0) /= n; xhat(8,0) /= n; xhat(9,0) /= n;
    } else {
        xhat(6,0) = 1.0f; xhat(7,0) = 0.0f; xhat(8,0) = 0.0f; xhat(9,0) = 0.0f;
    }
}
