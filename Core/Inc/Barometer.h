//
// Created by dyrel on 4/11/2026.
//

#ifndef KINGFISHER_SW_BAROMETER_H
#define KINGFISHER_SW_BAROMETER_H
#include <cstdint>
#include "BMP390.h"
#include "spi.h"
#include "Math/FloatTripleModRedundancy.h"

typedef struct {
    BMP_Data SensorA, SensorB, SensorC = {};
    BMP_Data Filtered = {};
} BARO_DATA;

class Baro_Unified {
    Barometer SensorA, SensorB, SensorC;
    BARO_DATA data;
    bool TMR_on = false;

    typedef struct {
        int8_t A, B, C;
    } BMP_Status;

    BMP_Status status;

public:
    Baro_Unified();
    BMP_Status Init(bool TMR_Toggle = false);
    BMP_Status Update();
    BARO_DATA getData();
};

#endif //KINGFISHER_SW_BAROMETER_H