//
// Created by dyrel on 4/11/2026.
//

#include "Math/FloatTripleModRedundancy.h"

float TMRFloat::Vote(float v1, float v2, float v3) {
    float diff_12 = fabsf(v1 - v2);
    float diff_13 = fabsf(v1 - v3);
    float diff_23 = fabsf(v2 - v3);

    // If found value differs by 5% of average, flag a fault
    float threshold = 0.05f * ((v1 + v2 + v3) / 3.0f); // 5% of mean
    float best_diff = fminf(diff_12, fminf(diff_13, diff_23));
    if (best_diff > threshold) {
        // Tolerance exceeded code
        // TODO add code to log error, etc
    }

    // Find the pair that agrees most closely
    if (diff_12 <= diff_13 && diff_12 <= diff_23) {
        // v1 and v2 agree best — v3 is the outlier
        return (v1 + v2) / 2.0f;
    } else if (diff_13 <= diff_12 && diff_13 <= diff_23) {
        // v1 and v3 agree best — v2 is the outlier
        return (v1 + v3) / 2.0f;
    } else {
        // v2 and v3 agree best — v1 is the outlier
        return (v2 + v3) / 2.0f;
    }
}