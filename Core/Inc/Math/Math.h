//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_MATH_H
#define KINGFISHER_SW_MATH_H

#include <cmath>
#include <stddef.h>
#include "Vector3D.h"

struct Q {
    float w;
    float x;
    float y;
    float z;
};

struct Quaternion {
    Q q;
    float beta = 0.1f;     // filter gain
    float sampleFreq = 1000.0f;  // Hz
};

struct MATHEMATICS {
    // Map an input value in an original range to a different range
    static float Map(float in, float inMin, float inMax, float outMin, float outMax);
    static Q Quaternion_Madgwick(Quaternion* QuatIn, Vector3D<float> accel, Vector3D<float> gyro, Vector3D<float> mag);
};

#endif //KINGFISHER_SW_MATH_H