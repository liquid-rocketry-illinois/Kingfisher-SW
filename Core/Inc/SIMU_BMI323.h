//
// Created by dyrel on 3/3/2026.
//

#ifndef KINGFISHER_SW_SIMU_BMI323_H
#define KINGFISHER_SW_SIMU_BMI323_H

#include "bmi323.h"
#include "main.h"
#include "spi.h"
#include "Math/Math.h"

#define _spi &hspi4

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

    static int8_t SPI_Read(uint8_t reg, uint8_t* data, uint32_t len, void *intf_ptr);
    static int8_t SPI_Write(uint8_t reg, const uint8_t* data, uint32_t len, void *intf_ptr);
    void CS_Select();
    void CS_Deselect();
    static void platform_Delay(uint32_t microseconds, void *intf_ptr);
};

#endif //KINGFISHER_SW_SIMU_BMI323_H