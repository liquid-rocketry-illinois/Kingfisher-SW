//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "bmi3.h"
#include "main.h"
#include "Math/Math.h"

struct BMI_Data
{
    Vector3D<float> accel_linear;
    Vector3D<float> ang_vel;
};

class SIMU_BMI323
{
public:
    enum BMI_INDEX {
        SENSOR1_I,
        SENSOR2_I,
        SENSOR3_I
    };

    SIMU_BMI323(SPI_HandleTypeDef* spi, BMI_INDEX DeviceNum);
    int Init();
    int Update();
    BMI_Data getRawData();

private:
    SPI_HandleTypeDef* _spi;
    bmi3_dev device;
    bool _sensor_active;
    BMI_INDEX InitDev;

    BMI_Data _raw;

    int8_t SPI_Read(uint8_t reg, uint8_t* data, uint16_t len);
    int8_t SPI_Write(uint8_t reg, const uint8_t* data, uint16_t len);
    void CS_Select();
    void CS_Deselect();
};

#endif //KINGFISHER_SW_SIMU_BMI323_H