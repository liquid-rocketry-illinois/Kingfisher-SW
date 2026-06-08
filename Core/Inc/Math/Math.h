//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_MATH_H
#define KINGFISHER_SW_MATH_H

#include <cmath>
#include <stddef.h>
#include "Vector3D.h"

struct MATHEMATICS {
    // Map an input value in an original range to a different range
    static float Map(float in, float inMin, float inMax, float outMin, float outMax);
};

#endif //KINGFISHER_SW_MATH_H