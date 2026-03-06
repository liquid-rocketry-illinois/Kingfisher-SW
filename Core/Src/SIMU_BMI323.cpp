//
// Created by dyrel on 3/5/2026.
//

#include "SIMU_BMI323.h"

extern SPI_HandleTypeDef hspi4;

SIMU_BMI323::SIMU_BMI323(SPI_HandleTypeDef* spi)
{
    _spi = spi;
    _sensor_active = false;
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

int SIMU_BMI323::Init()
{
    device.intf = BMI3_SPI_INTF;
    device.read = [](uint8_t reg, uint8_t* data, uint32_t len, void* intf_ptr)
    {
        return static_cast<SIMU_BMI323 *>(intf_ptr)->SPI_Read(0, reg, data, len);
    };

    device.write = [](uint8_t reg, const uint8_t* data, uint32_t len, void* intf_ptr)
    {
        return static_cast<SIMU_BMI323 *>(intf_ptr)->SPI_Write(0, reg, data, len);
    };

    device.intf_ptr = this;
    device.delay_us = [](uint32_t period, void*) { HAL_Delay(period / 1000); };

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
            status = -1;
        }

        bmi3_sensor_data BMI323[2];

        BMI323[0].type = BMI323_ACCEL;
        BMI323[1].type = BMI323_GYRO;

        if(bmi323_get_sensor_data(BMI323, 2, &device) == BMI3_OK)
        {
            SensorData data;
            data.a_x = BMI323[0].sens_data.acc.x;
            data.a_y = BMI323[0].sens_data.acc.y;
            data.a_z = BMI323[0].sens_data.acc.z;
            data.omega_x = BMI323[1].sens_data.gyr.x;
            data.omega_y = BMI323[1].sens_data.gyr.y;
            data.omega_z = BMI323[1].sens_data.gyr.z;

            _raw = data;

            status = 0;
        }
        else {
            status = -1;
        }

    return status;
}

// Return the raw data calculated
SensorData SIMU_BMI323::getRawData()
{
    return _raw;
}