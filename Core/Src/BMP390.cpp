//
// Created by admin on 3/16/2026.
//

#include "BMP390.h"

#include <cstring>

#include "main.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
#include "Math/Math.h"

/* Sea level reference pressure for altitude calculation */
static constexpr float SEA_LEVEL_PRESSURE = 101325.0f;

/* -------------------------------------------------------------------------- */
/* Constructor                                                                */
/* -------------------------------------------------------------------------- */

Barometer::Barometer(SPI_HandleTypeDef* spi, BMP_INDEX DeviceNum)
{
    _spi = spi;
    InitDev = DeviceNum;
    _sensor_active = false;

    _raw.heightMeters = 0.0f;
    _raw.Pressure = 0.0f;
    _raw.Temperature = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

uint8_t Barometer::Init()
{
    spi_intf.spi_handle = _spi;

    /* Map sensor index to chip select pins */
    switch(InitDev)
    {
        case SENSOR1_I:
            spi_intf.cs_port = BMP390_CS1_GPIO_Port;
            spi_intf.cs_pin  = BMP390_CS1_Pin;
            break;

        case SENSOR2_I:
            spi_intf.cs_port = BMP390_CS2_GPIO_Port;
            spi_intf.cs_pin  = BMP390_CS2_Pin;
            break;

        case SENSOR3_I:
            spi_intf.cs_port = BMP390_CS3_GPIO_Port;
            spi_intf.cs_pin  = BMP390_CS3_Pin;
            break;

        default:
            return 1;
    }

    device.intf = BMP3_SPI_INTF;
    device.intf_ptr = &spi_intf;

    device.read = BMP_SPI_Read;
    device.write = BMP_SPI_Write;
    device.delay_us = BMP_delay_us;


    if(bmp3_init(&device) != BMP3_OK)
        return 2;

    /* Sensor configuration */
    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.temp_os = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.odr = BMP3_ODR_100_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR | BMP3_SEL_DRDY_EN;

    if(bmp3_set_sensor_settings(
        settings_sel,
        &settings,
        &device) != BMP3_OK)
    {
        return 3;
    }

    // now set operation mode
    settings.op_mode = BMP3_MODE_NORMAL;
    if(bmp3_set_op_mode(&settings, &device) != BMP3_OK)
        return 4;

    _sensor_active = true;

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Update sensor measurement                                                  */
/* -------------------------------------------------------------------------- */

uint8_t Barometer::Update()
{
    if(!_sensor_active || (bmp3_get_status(&status, &device) != BMP3_OK))
        return 1;

    bmp3_data data = { 0 };

    if(bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device) != BMP3_OK)
        return 2;
    else {
        if (status.intr.drdy == BMP3_ENABLE) {
            if (bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device) != BMP3_OK)
                return 3;
            if (bmp3_get_status(&status, &device) != BMP3_OK)
                return 4;
        }
    }

    _raw.Pressure = static_cast<float>(data.pressure);
    _raw.Temperature = static_cast<float>(data.temperature);

    /* Calculate altitude from pressure */
    _raw.heightMeters =
        44330.0F *
        (1.0f - powf(data.pressure / SEA_LEVEL_PRESSURE, 0.1903f));

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Return latest measurement                                                  */
/* -------------------------------------------------------------------------- */

BMP_Data Barometer::getRawData()
{
    return _raw;
}