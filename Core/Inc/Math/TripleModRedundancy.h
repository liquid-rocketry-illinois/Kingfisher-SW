//
// Created by dyrel on 3/5/2026.
//

#ifndef KINGFISHER_SW_TRIPLEMODREDUNDANCY_H
#define KINGFISHER_SW_TRIPLEMODREDUNDANCY_H

#include "Vector3D.h"

class TMR
{
public:

    static Vector3D<float> Vote(const Vector3D<float>& v1,
                            const Vector3D<float>& v2,
                            const Vector3D<float>& v3);

private:

    static float distance(const Vector3D<float>& a, const Vector3D<float>& b);
    static Vector3D<float> average(const Vector3D<float>& a, const Vector3D<float>& b);
};

#endif //KINGFISHER_SW_TRIPLEMODREDUNDANCY_H