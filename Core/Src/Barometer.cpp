//
// Created by dyrel on 4/11/2026.
//

#include "Barometer.h"

Baro_Unified::Baro_Unified() :  SensorA(&hspi2, Barometer::SENSOR1_I),
                                SensorB(&hspi2, Barometer::SENSOR2_I),
                                SensorC(&hspi2, Barometer::SENSOR3_I) {}

// true = tmr on
Baro_Unified::BMP_Status Baro_Unified::Init(bool TMR_Toggle) {
    TMR_on = TMR_Toggle;

    status.A = SensorA.Init();
    if (TMR_Toggle) {
        status.B = SensorB.Init();
        status.C = SensorC.Init();
    }

    return status;
}

Baro_Unified::BMP_Status Baro_Unified::Update() {
    if (status.A == BMP3_OK) {
        status.A = SensorA.Update();
        data.SensorA = SensorA.getRawData();

        if (TMR_on && status.B == BMP3_OK && status.C == BMP3_OK) {
            status.B = SensorB.Update();
            status.C = SensorC.Update();
            data.SensorB = SensorB.getRawData();
            data.SensorC = SensorC.getRawData();

            data.Filtered.Pressure    = TMRFloat::Vote(data.SensorA.Pressure,     data.SensorB.Pressure,     data.SensorC.Pressure);
            data.Filtered.heightMeters = TMRFloat::Vote(data.SensorA.heightMeters, data.SensorB.heightMeters, data.SensorC.heightMeters);
            data.Filtered.Temperature  = TMRFloat::Vote(data.SensorA.Temperature,  data.SensorB.Temperature,  data.SensorC.Temperature);
        } else {
            // single-sensor mode: Filtered mirrors primary sensor
            data.Filtered = data.SensorA;
        }
    }
    return status;
}

BARO_DATA Baro_Unified::getData() {
    return data;
}