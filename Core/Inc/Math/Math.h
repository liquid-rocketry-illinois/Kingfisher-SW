//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_MATH_H
#define KINGFISHER_SW_MATH_H

#include <cmath>
#include <stddef.h>
#include "Vector3D.h"

struct Q {
    float w = 1.0f;  // identity quaternion — avoids all-zero division in Madgwick
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct Quaternion {
    Q q;
    float beta = 0.1f;     // filter gain
    float sampleFreq = 1000.0f;  // Hz
};

struct MATHEMATICS {
    // Map an input value in an original range to a different range
    static float Map(float in, float inMin, float inMax, float outMin, float outMax);
    static void Quaternion_Madgwick(   Quaternion* QuatIn,
                                    Vector3D<float> accel,
                                    Vector3D<float> gyro,
                                    Vector3D<float> mag = Vector3D<float>(0.0f,0.0f,0.0f));
};

#endif //KINGFISHER_SW_MATH_H