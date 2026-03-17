//
// Created by admin on 3/16/2026.
//

#include "BMP390.h"
#include "Math/Math.h"

/* Sea level reference pressure for altitude calculation */
static constexpr float SEA_LEVEL_PRESSURE = 101325.0f;

/* SPI interface object used by Bosch driver */
static bmp3_spi_intf spi_intf;

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
            spi_intf.cs_port = GPIOA;
            spi_intf.cs_pin  = GPIO_PIN_4;
            break;

        case SENSOR2_I:
            spi_intf.cs_port = GPIOA;
            spi_intf.cs_pin  = GPIO_PIN_5;
            break;

        case SENSOR3_I:
            spi_intf.cs_port = GPIOA;
            spi_intf.cs_pin  = GPIO_PIN_6;
            break;

        default:
            return 1;
    }

    device.intf = BMP3_SPI_INTF;
    device.intf_ptr = &spi_intf;

    device.read = bmp3_spi_read;
    device.write = bmp3_spi_write;
    device.delay_us = bmp3_delay_us;

    if(bmp3_init(&device) != BMP3_OK)
        return 2;

    /* Sensor configuration */
    bmp3_settings settings;

    settings.press_en = BMP3_ENABLE;
    settings.temp_en  = BMP3_ENABLE;
    settings.op_mode = BMP3_MODE_NORMAL;

    settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
    settings.odr_filter.temp_os  = BMP3_OVERSAMPLING_2X;
    settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;

    if(bmp3_set_sensor_settings(
        BMP3_SEL_PRESS_EN |
        BMP3_SEL_TEMP_EN |
        BMP3_SEL_PRESS_OS |
        BMP3_SEL_TEMP_OS |
        BMP3_SEL_IIR_FILTER,
        &settings,
        &device) != BMP3_OK)
    {
        return 3;
    }

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
    if(!_sensor_active)
        return 1;

    bmp3_data data;

    if(bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &device) != BMP3_OK)
        return 2;

    _raw.Pressure = (float)data.pressure;
    _raw.Temperature = (float)data.temperature;

    /* Calculate altitude from pressure */
    _raw.heightMeters =
        44330.0f *
        (1.0f - powf(_raw.Pressure / SEA_LEVEL_PRESSURE, 0.1903f));

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Return latest measurement                                                  */
/* -------------------------------------------------------------------------- */

BMP_Data Barometer::getRawData()
{
    return _raw;
}

/* -------------------------------------------------------------------------- */
/* Optional direct SPI helpers (not required by Bosch driver)                */
/* -------------------------------------------------------------------------- */

int8_t Barometer::SPI_Read(uint8_t reg, uint8_t* data, uint16_t len)
{
    uint8_t addr = reg | 0x80;

    CS_Select();

    HAL_SPI_Transmit(_spi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(_spi, data, len, HAL_MAX_DELAY);

    CS_Deselect();

    return 0;
}

int8_t Barometer::SPI_Write(uint8_t reg, const uint8_t* data, uint16_t len)
{
    uint8_t addr = reg & 0x7F;

    CS_Select();

    HAL_SPI_Transmit(_spi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);

    CS_Deselect();

    return 0;
}

void Barometer::CS_Select()
{
    HAL_GPIO_WritePin(spi_intf.cs_port, spi_intf.cs_pin, GPIO_PIN_RESET);
}

void Barometer::CS_Deselect()
{
    HAL_GPIO_WritePin(spi_intf.cs_port, spi_intf.cs_pin, GPIO_PIN_SET);
}