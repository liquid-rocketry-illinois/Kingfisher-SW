//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "bmi3.h"
#include "main.h"
#include "Math/Math.h"

struct SensorData
{
    float a_x;
    float a_y;
    float a_z;
    float omega_x;
    float omega_y;
    float omega_z;
};

class SIMU_BMI323
{
public:
    SIMU_BMI323(SPI_HandleTypeDef* spi);

    int Init();
    int Update();

    SensorData getRawData();

private:
    SPI_HandleTypeDef* _spi;
    bmi3_dev device;
    bool _sensor_active;

    SensorData _raw;

    int8_t SPI_Read(uint8_t cs_index, uint8_t reg, uint8_t* data, uint16_t len);
    int8_t SPI_Write(uint8_t cs_index, uint8_t reg, const uint8_t* data, uint16_t len);
    void CS_Select(uint8_t index);
    void CS_Deselect(uint8_t index);
};

#endif //KINGFISHER_SW_SIMU_BMI323_H