//
// Created by admin on 5/19/2026.
//

#ifndef KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H
#define KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H

#include "Math/Math.h"
#include "Telemetry.h"
#include "timing.h"

extern float g_dt; // seconds since last update cycle, computed by ComputeDt()

namespace DataUpdate
{
    void  ComputeDt();          // call once at the top of each update cycle
    void  FuseQuat(Quaternion* QObj);
    void  QuatToYPR(Q* quaternionIn);
    float getVerticalVelocity();
}

#endif //KINGFISHER_SW_FLIGHTCOMPUTER_DATAUPDATE_H