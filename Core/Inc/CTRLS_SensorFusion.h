// Created by admin on 5/10/2026.

#ifndef KINGFISHER_SW_SENSORFUSION_H
#define KINGFISHER_SW_SENSORFUSION_H

#include "CTRLS_ControlAlgorithm.h"

// ─── EKF — Extended Kalman Filter ─────────────────────────────────────────────
// State: x = [w1,w2,w3, v1,v2,v3, qw,qx,qy,qz]  (body-frame rates, velocity, quaternion)
// Measurement: y = [ax,ay,az, gx,gy,gz]           (accel in g, gyro in rad/s)
class EKF {
public:
    StateVec xhat;  // current state estimate (readable by ControlLaw)

    EKF(const Physics& phys, const RocketConfig& cfg);

    // Full predict + correct step. Call when fresh IMU data is available.
    StateVec update(float t, float dt, const MeasVec& y_meas, float u,
                    float alt = 0.0f);

    // Predict only — no correction. Call when no new IMU sample is available.
    void predictOnly(float t, float dt, float u, float alt = 0.0f);

    // Reset to initial conditions
    void reset(const RocketConfig& cfg);

private:
    const Physics&      phys_;
    const RocketConfig& cfg_;

    StateMat P_;   // state covariance
    StateMat Q_;   // process noise
    Mat<6,6> R_;   // measurement noise (nominal)

    // Predict step: Euler integration + covariance propagation
    void predict_(float t, float dt, float u, float alt, StateMat& A_out);

    // Correct step: standard EKF update with gain limiting.
    void correct_(float t, float u, const MeasVec& y_meas,
                  float alt, const StateMat& A,
                  bool burning);

    // Enforce rail constraints (zero pitch/yaw, lock quaternion)
    void applyRailConstraint_();

    // Renormalize quaternion component of xhat
    void normalizeQuaternion_();
};

#endif // KINGFISHER_SW_SENSORFUSION_H
