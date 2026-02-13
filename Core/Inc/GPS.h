//
// Created by dyrel on 2/12/2026.
//

#ifndef KINGFISHER_SW_GPS_H
#define KINGFISHER_SW_GPS_H

#include <cstdint>
#include <cstddef>

struct GnssData
{
double longitude;   // degrees
double latitude;    // degrees
};

class GnssSensor
{
private:
    void* gnssHandle;
    GnssData currentData;
    bool initialized;
public:
    GnssSensor();
    ~GnssSensor();

    bool Init();
    bool Update();
    GnssData GetData() const;
};

#endif //KINGFISHER_SW_GPS_H