//
// Created by dyrel on 3/5/2026.
//

#include "SIMU_BMI323.h"

extern SPI_HandleTypeDef hspi4;

SIMU_BMI323::SIMU_BMI323(BMI_INDEX DeviceNum)
{
    _sensor_active = false;
    InitDev = DeviceNum;

    device.intf = BMI3_SPI_INTF;
    device.read = BMI3_SPI_Read;
    device.write = BMI3_SPI_Write;
    device.intf_ptr = &_obj;
    device.delay_us = BMI323_delay_us;
}

int SIMU_BMI323::Init()
{
    if(bmi323_init(&device) == BMI3_OK)
    {
        _sensor_active = true;
    }

    return 0;
}

int SIMU_BMI323::Update()
{
    int status = 0;

    if(!_sensor_active)
    {
        return -1;
    }

    bmi3_sensor_data BMI323[2];

    BMI323[0].type = BMI323_ACCEL;
    BMI323[1].type = BMI323_GYRO;

    if(bmi323_get_sensor_data(BMI323, 2, &device) == BMI3_OK)
    {
        BMI_Data data;
        data.accel_linear.x = BMI323[0].sens_data.acc.x;
        data.accel_linear.y = BMI323[0].sens_data.acc.y;
        data.accel_linear.z = BMI323[0].sens_data.acc.z;
        data.ang_vel.x = BMI323[1].sens_data.gyr.x;
        data.ang_vel.y = BMI323[1].sens_data.gyr.y;
        data.ang_vel.z = BMI323[1].sens_data.gyr.z;

        _raw = data;
    }
    else {
        status = -1;
    }

    return status;
}

// Return the raw data calculated
BMI_Data SIMU_BMI323::getRawData()
{
    return _raw;
}