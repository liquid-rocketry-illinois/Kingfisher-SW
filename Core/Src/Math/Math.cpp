//
// Created by dyrel on 2/21/2026.
//

#include "Math/Math.h"

// Map an input value in an original range to a different range
float MATHEMATICS::Map(float in, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) return outMin; // in case of x/0
    return outMin + ( (in - inMin) * (outMax - outMin) ) / (inMax - inMin);
}

// No input for mag if only accel and gyro inputs
void MATHEMATICS::Quaternion_Madgwick( Quaternion* QuatIn,
                                    Vector3D<float> accel,
                                    Vector3D<float> gyro,
                                    Vector3D<float> mag) {
    float ax = accel.x, ay = accel.y, az = accel.z;
    float gx = gyro.x,  gy = gyro.y,  gz = gyro.z;
    float mx = mag.x,   my = mag.y,   mz = mag.z;

    Q q = QuatIn->q;
    float beta = QuatIn->beta;
    float sampleFreq = QuatIn->sampleFreq;

    float q1 = q.w, q2 = q.x, q3 = q.y, q4 = q.z;

    float norm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _4q3 = 4.0f * q3;
    float _8q2 = 8.0f * q2;
    float _8q3 = 8.0f * q3;

    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;
    float q4q4 = q4*q4;

    /* Normalize accelerometer */
    float accelNormSq = ax*ax + ay*ay + az*az;
    if(accelNormSq == 0.0f) return;
    norm = 1.0F / sqrtf(accelNormSq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    bool useMag = !(mx == 0.0f && my == 0.0f && mz == 0.0f);

    if(useMag)
    {
        float magNormSq = mx*mx + my*my + mz*mz;
        if(magNormSq == 0.0f) useMag = false;
        else
        {
            norm = 1.0F / sqrtf(magNormSq);
            mx *= norm;
            my *= norm;
            mz *= norm;
        }
    }

    float hx, hy, _2bx, _2bz;

    if(useMag)
    {
        float _2q1mx = 2.0f * q1 * mx;
        float _2q1my = 2.0f * q1 * my;
        float _2q1mz = 2.0f * q1 * mz;
        float _2q2mx = 2.0f * q2 * mx;

        hx = mx*q1q1 - _2q1my*q4 + _2q1mz*q3 + mx*q2q2 +
             _2q2*my*q3 + _2q2*mz*q4 - mx*q3q3 - mx*q4q4;

        hy = _2q1mx*q4 + my*q1q1 - _2q1mz*q2 + _2q2mx*q3 -
             my*q2q2 + my*q3q3 + _2q3*mz*q4 - my*q4q4;

        _2bx = sqrtf(hx*hx + hy*hy);
        _2bz = -_2q1mx*q3 + _2q1my*q2 + mz*q1q1 +
               _2q2mx*q4 - mz*q2q2 + _2q3*my*q4 -
               mz*q3q3 + mz*q4q4;
    }

    /* Gradient descent correction */

    s1 = _4q1*q3q3 + _2q3*ax + _4q1*q2q2 - _2q2*ay;
    s2 = _4q2*q4q4 - _2q4*ax + 4.0f*q1q1*q2 - _2q1*ay - _4q2 +
         _8q2*q2q2 + _8q2*q3q3 + _4q2*az;
    s3 = 4.0f*q1q1*q3 + _2q1*ax + _4q3*q4q4 - _2q4*ay - _4q3 +
         _8q3*q2q2 + _8q3*q3q3 + _4q3*az;
    s4 = 4.0f*q2q2*q4 - _2q2*ax + 4.0f*q3q3*q4 - _2q3*ay;

    norm = 1.0F / sqrtf(s1*s1 + s2*s2 + s3*s3 + s4*s4);
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    /* Quaternion derivative */

    qDot1 = 0.5f * (-q2*gx - q3*gy - q4*gz) - beta*s1;
    qDot2 = 0.5f * ( q1*gx + q3*gz - q4*gy) - beta*s2;
    qDot3 = 0.5f * ( q1*gy - q2*gz + q4*gx) - beta*s3;
    qDot4 = 0.5f * ( q1*gz + q2*gy - q3*gx) - beta*s4;

    /* Integrate */

    float dt = 1.0f / sampleFreq;

    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    q4 += qDot4 * dt;

    /* Normalize quaternion */

    norm = 1.0F / sqrtf(q1*q1 + q2*q2 + q3*q3 + q4*q4);

    QuatIn->q.w = q1 * norm;
    QuatIn->q.x = q2 * norm;
    QuatIn->q.y = q3 * norm;
    QuatIn->q.z = q4 * norm;
}