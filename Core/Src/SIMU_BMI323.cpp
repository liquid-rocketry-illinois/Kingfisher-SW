//
// Created by dyrel on 3/5/2026.
//

#include "SIMU_BMI323.h"
#include <cstring>

extern SPI_HandleTypeDef hspi4;

SIMU_BMI323::SIMU_BMI323(SPI_HandleTypeDef* spi)
{
    _spi = spi;
    _tmr_enabled = false;

    for(int i = 0; i < 3; i++)
        _sensor_active[i] = false;
}

void SIMU_BMI323::CS_Select(uint8_t index)
{
    switch(index)
    {
        case 0: HAL_GPIO_WritePin(SIMU_CS1_GPIO_Port, SIMU_CS1_Pin, GPIO_PIN_RESET); break;
        case 1: HAL_GPIO_WritePin(SIMU_CS2_GPIO_Port, SIMU_CS2_Pin, GPIO_PIN_RESET); break;
        case 2: HAL_GPIO_WritePin(SIMU_CS3_GPIO_Port, SIMU_CS3_Pin, GPIO_PIN_RESET); break;
    }
}

void SIMU_BMI323::CS_Deselect(uint8_t index)
{
    switch(index)
    {
        case 0: HAL_GPIO_WritePin(SIMU_CS1_GPIO_Port, SIMU_CS1_Pin, GPIO_PIN_SET); break;
        case 1: HAL_GPIO_WritePin(SIMU_CS2_GPIO_Port, SIMU_CS2_Pin, GPIO_PIN_SET); break;
        case 2: HAL_GPIO_WritePin(SIMU_CS3_GPIO_Port, SIMU_CS3_Pin, GPIO_PIN_SET); break;
    }
}

int8_t SIMU_BMI323::SPI_Read(uint8_t cs_index, uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t addr = reg | 0x80;

    CS_Select(cs_index);
    HAL_SPI_Transmit(_spi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(_spi, data, len, HAL_MAX_DELAY);
    CS_Deselect(cs_index);

    return 0;
}

int8_t SIMU_BMI323::SPI_Write(uint8_t cs_index, uint8_t reg, const uint8_t* data, uint16_t len)
{
    uint8_t addr = reg & 0x7F;

    CS_Select(cs_index);
    HAL_SPI_Transmit(_spi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);
    CS_Deselect(cs_index);

    return 0;
}

int SIMU_BMI323::Init(bool TMR_Toggle)
{
    _tmr_enabled = TMR_Toggle;

    int sensors_to_try = (_tmr_enabled) ? 3 : 1;

    for(int i = 0; i < sensors_to_try; i++)
    {
        _dev[i].intf = BMI3_SPI_INTF;
        _dev[i].read = [](uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr)
        {
            return ((SIMU_BMI323*)intf_ptr)->SPI_Read((uint32_t)intf_ptr >> 16, reg, data, len);
        };

        _dev[i].write = [](uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr)
        {
            return ((SIMU_BMI323*)intf_ptr)->SPI_Write((uint32_t)intf_ptr >> 16, reg, data, len);
        };

        _dev[i].intf_ptr = this;
        _dev[i].delay_us = [](uint32_t period, void*) { HAL_Delay(period / 1000); };

        if(bmi323_init(&_dev[i]) == BMI3_OK)
        {
            _sensor_active[i] = true;
        }
    }

    return 0;
}

SensorData<int> SIMU_BMI323::Update()
{
    SensorData<int> status = {0,0,0};

    for(int i = 0; i < 3; i++)
    {
        if(!_sensor_active[i])
        {
            if(i==0) status.sensor1 = -1;
            if(i==1) status.sensor2 = -1;
            if(i==2) status.sensor3 = -1;
            continue;
        }

        bmi3_sensor_data BMI323A, BMI323G;

        BMI323A.type = BMI323_ACCEL;
        BMI323G.type = BMI323_GYRO;

        if(bmi323_get_sensor_data(&BMI323A, 1, &_dev[i]) == BMI3_OK)
        {
            BMI323_Data data;
            data.a_x = BMI323A.sens_data.acc.x;
            data.a_y = BMI323A.sens_data.acc.y;
            data.a_z = BMI323A.sens_data.acc.z;
            data.omega_x = BMI323G.sens_data.gyr.x;
            data.omega_y = BMI323G.sens_data.gyr.y;
            data.omega_z = BMI323G.sens_data.gyr.z;

            if(i==0) _raw.sensor1 = data;
            if(i==1) _raw.sensor2 = data;
            if(i==2) _raw.sensor3 = data;

            status.sensor1 = 0;
        }
        else
        {
            if(i==0) status.sensor1 = -1;
            if(i==1) status.sensor2 = -1;
            if(i==2) status.sensor3 = -1;
        }
    }

    return status;
}

// Return the raw data calculated
SensorData<BMI323_Data> SIMU_BMI323::getRawData()
{
    return _raw;
}

// Return the calculated quaternion data with an option to use external
// magnetometer data.
SensorData<BMI323_Quat> SIMU_BMI323::getQuat(Vector3D<float> mag = Vector3D<float>(0.0f,0.0f,0.0f))
{
    // NOTE:
    // BMI323 does NOT natively output quaternions.
    // This is placeholder. You must implement fusion (e.g., Madgwick).

    return _quat;
}