//
// Created by admin on 3/16/2026.
//

#include "BMP390.h"

#include "main.h"
#include "FreeRTOS.h"
#include "projdefs.h"
#include "task.h"
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

    device.read = bmp3_spi_read;
    device.write = bmp3_spi_write;
    device.delay_us = bmp3_delay_us;


    if(bmp3_init(&device) != BMP3_OK)
        return 2;

    /* Sensor configuration */
    fifo_settings.mode = BMP3_ENABLE;
    fifo_settings.press_en = BMP3_ENABLE;
    fifo_settings.temp_en = BMP3_ENABLE;
    fifo_settings.filter_en = BMP3_ENABLE;
    fifo_settings.down_sampling = BMP3_FIFO_NO_SUBSAMPLING;
    fifo_settings.ffull_en = BMP3_ENABLE;

    fifo.buffer = fifo_data;
    fifo.req_frames = 50;

    settings.press_en = BMP3_ENABLE;
    settings.temp_en = BMP3_ENABLE;
    settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
    settings.odr_filter.odr = BMP3_ODR_50_HZ;

    settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN | BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR;

    settings_fifo = BMP3_SEL_FIFO_MODE | BMP3_SEL_FIFO_PRESS_EN | BMP3_SEL_FIFO_TEMP_EN | BMP3_SEL_FIFO_FULL_EN |
                    BMP3_SEL_FIFO_DOWN_SAMPLING | BMP3_SEL_FIFO_FILTER_EN;

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

    if (bmp3_set_fifo_settings(settings_fifo, &fifo_settings, &device) != BMP3_OK)
        return 5;

    return 0;
}

/* -------------------------------------------------------------------------- */
/* Update sensor measurement                                                  */
/* -------------------------------------------------------------------------- */

uint8_t Barometer::Update()
{
    if(!_sensor_active)
        return 1;

    if (bmp3_get_fifo_length(&fifo_length, &device) != BMP3_OK)
        return 2;
    if(bmp3_get_fifo_data(&fifo, &fifo_settings, &device) != BMP3_OK)
        return 3;
    if (bmp3_get_status(&status, &device) != BMP3_OK)
        return 4;
    if (bmp3_extract_fifo_data(fifo_pt_data, &fifo, &device) != BMP3_OK)
        return 5;

    for (int i = 0; i < fifo.parsed_frames; i++) {
#ifdef BMP3_FLOAT_COMPENSATION
        _raw.Pressure += (float)fifo_pt_data[i].pressure;
        _raw.Temperature += (float)fifo_pt_data[i].temperature;
#else
        _raw.Pressure += (float)fifo_pt_data[i].pressure / 100;
        _raw.Temperature += (float)fifo_pt_data[i].temperature / 100;
#endif // BMP3_FLOAT_COMPENSATION
    }

    _raw.Pressure /= (float)fifo.parsed_frames;
    _raw.Temperature /= (float)fifo.parsed_frames;

    /* Calculate altitude from pressure */
    _raw.heightMeters =
        44330.0 *
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
    if (InitDev == SENSOR1_I) {
        HAL_GPIO_WritePin(BMP390_CS2_GPIO_Port, BMP390_CS2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMP390_CS3_GPIO_Port, BMP390_CS3_Pin, GPIO_PIN_SET);
    }
    if (InitDev == SENSOR2_I) {
        HAL_GPIO_WritePin(BMP390_CS1_GPIO_Port, BMP390_CS1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMP390_CS3_GPIO_Port, BMP390_CS3_Pin, GPIO_PIN_SET);
    }
    if (InitDev == SENSOR3_I) {
        HAL_GPIO_WritePin(BMP390_CS2_GPIO_Port, BMP390_CS2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BMP390_CS1_GPIO_Port, BMP390_CS1_Pin, GPIO_PIN_SET);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
    HAL_GPIO_WritePin(spi_intf.cs_port, spi_intf.cs_pin, GPIO_PIN_RESET);
}

void Barometer::CS_Deselect()
{
    // Easier to just deselect everything
    HAL_GPIO_WritePin(BMP390_CS1_GPIO_Port, BMP390_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS2_GPIO_Port, BMP390_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS3_GPIO_Port, BMP390_CS3_Pin, GPIO_PIN_SET);

    vTaskDelay(pdMS_TO_TICKS(5));
}