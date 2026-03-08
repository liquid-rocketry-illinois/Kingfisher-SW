//
// Created by dyrel on 3/5/2026.
//

#include "Math/TripleModRedundancy.h"

Vector3D<float> TMR::Vote(const Vector3D<float>& v1,
                           const Vector3D<float>& v2,
                           const Vector3D<float>& v3)
{
    float d12 = distance(v1, v2);
    float d13 = distance(v1, v3);
    float d23 = distance(v2, v3);

    // Find the closest pair
    if(d12 <= d13 && d12 <= d23)
    {
        return average(v1, v2);
    }
    else if(d13 <= d12 && d13 <= d23)
    {
        return average(v1, v3);
    }
    else
    {
        return average(v2, v3);
    }
}

float TMR::distance(const Vector3D<float>& a, const Vector3D<float>& b)
{
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;

    return sqrtf(dx*dx + dy*dy + dz*dz);
}

Vector3D<float> TMR::average(const Vector3D<float>& a, const Vector3D<float>& b)
{
    Vector3D<float> out;

    out.x = (a.x + b.x) * 0.5f;
    out.y = (a.y + b.y) * 0.5f;
    out.z = (a.z + b.z) * 0.5f;

    return out;
}