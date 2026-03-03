//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "bmi3.h"
#include "main.h"

#define BMI1_CS_GPIO_Port CS1_Pin
#define BMI1_CS_Pin       GPIO_PIN_4

#define BMI2_CS_GPIO_Port GPIOE
#define BMI2_CS_Pin       GPIO_PIN_5

#define BMI3_CS_GPIO_Port GPIOE
#define BMI3_CS_Pin       GPIO_PIN_6

struct BMI323_Data
{
    float a_x;
    float a_y;
    float a_z;
    float omega_x;
    float omega_y;
    float omega_z;
};

struct BMI323_Quat
{
    float w;
    float x;
    float y;
    float z;
};

template<typename T>
struct SensorData
{
    T sensor1;
    T sensor2;
    T sensor3;
};

class SIMU_BMI323
{
public:
    SIMU_BMI323(SPI_HandleTypeDef* spi);

    int Init(bool TMR_Toggle);
    SensorData<int> Update();

    SensorData<BMI323_Data> getRawData();
    SensorData<BMI323_Quat> getQuat();

private:
    SPI_HandleTypeDef* _spi;
    bool _tmr_enabled;

    bmi3_dev _dev[3];
    bool _sensor_active[3];

    SensorData<BMI323_Data> _raw;
    SensorData<BMI323_Quat> _quat;

    int8_t SPI_Read(uint8_t cs_index, uint8_t reg, uint8_t* data, uint16_t len);
    int8_t SPI_Write(uint8_t cs_index, uint8_t reg, const uint8_t* data, uint16_t len);
    void CS_Select(uint8_t index);
    void CS_Deselect(uint8_t index);
};

#endif //KINGFISHER_SW_SIMU_BMI323_H