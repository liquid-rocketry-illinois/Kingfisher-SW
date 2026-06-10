//
// Created by dyrel on 2/21/2026.
//

#include "Math/Math.h"

// Map an input value in an original range to a different range
float MATHEMATICS::Map(float in, float inMin, float inMax, float outMin, float outMax) {
    if (inMax == inMin) return outMin; // in case of x/0
    return outMin + ( (in - inMin) * (outMax - outMin) ) / (inMax - inMin);
}

