//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "main.h"
#include "spi.h"
#include "Math/Math.h"
#include "BMI323_Platform.h"

typedef enum {
    SENSOR1_I,
    SENSOR2_I,
    SENSOR3_I
} BMI_INDEX;

struct BMI_Data
{
    Vector3D<float> accel_linear;
    Vector3D<float> ang_vel;
};

typedef struct {
    uint8_t TXDat[512], RXDat[2048] = {0};
    SPI_HandleTypeDef* SPIbus;
    BMI_INDEX index;
    GPIO_TypeDef* cs_port;
    uint16_t cs_pin;
} BMIobjPtr;

class SIMU_BMI323
{
public:
    SIMU_BMI323(BMI_INDEX DeviceNum);
    int Init();
    int Update();
    BMI_Data getRawData();

private:
    bmi3_dev device;
    bool _sensor_active;
    BMI_INDEX InitDev;
    BMI_Data _raw;
    BMIobjPtr _obj;
};

#endif //KINGFISHER_SW_SIMU_BMI323_H