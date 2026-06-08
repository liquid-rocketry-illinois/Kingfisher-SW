//
// Created by admin on 3/16/2026.
//

#include "LIS2MDL.h"

// ─── CS control ──────────────────────────────────────────────────────────────

void Magnetometer::cs_low()
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET);
}

void Magnetometer::cs_high()
{
    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
}

// ─── Platform callbacks ───────────────────────────────────────────────────────

void Magnetometer::platform_delay(uint32_t millisec)
{
    HAL_Delay(millisec);
}

int32_t Magnetometer::platform_write(void* handle, uint8_t reg,
                                      const uint8_t* bufp, uint16_t len)
{
    uint8_t tx[len + 1];
    tx[0] = reg & 0x7F;                 // MSB = 0 → write
    for (uint16_t i = 0; i < len; i++)
        tx[i + 1] = bufp[i];

    cs_low();
    HAL_SPI_Transmit(static_cast<SPI_HandleTypeDef*>(handle), tx, len + 1, HAL_MAX_DELAY);
    cs_high();
    return 0;
}

int32_t Magnetometer::platform_read(void* handle, uint8_t reg,
                                     uint8_t* bufp, uint16_t len)
{
    uint8_t reg_addr = reg | 0x80;      // MSB = 1 → read

    cs_low();
    HAL_SPI_Transmit(static_cast<SPI_HandleTypeDef*>(handle), &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive (static_cast<SPI_HandleTypeDef*>(handle), bufp,      len, HAL_MAX_DELAY);
    cs_high();
    return 0;
}

// ─── Constructor ──────────────────────────────────────────────────────────────

Magnetometer::Magnetometer(SPI_HandleTypeDef* spi) : _spi(spi)
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = _spi;
}

// ─── Init ─────────────────────────────────────────────────────────────────────

uint8_t Magnetometer::Init()
{
    uint8_t whoamI = 0;
    lis2mdl_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2MDL_ID)
        return 1;

    lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);
    uint8_t rst;
    do {
        lis2mdl_reset_get(&dev_ctx, &rst);
    } while (rst);

    lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_100Hz);
    lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);
    lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);

    _sensor_active = true;
    return 0;
}

// ─── Update ───────────────────────────────────────────────────────────────────

uint8_t Magnetometer::Update()
{
    if (!_sensor_active) return 1;

    uint8_t drdy = 0;
    if (lis2mdl_mag_data_ready_get(&dev_ctx, &drdy) != 0)
        return 2;

    if (!drdy) return 0;    // no new sample — retain last valid _raw

    int16_t mag_raw[3] = {};
    if (lis2mdl_magnetic_raw_get(&dev_ctx, mag_raw) != 0)
        return 3;

    int16_t temp_raw = 0;
    if (lis2mdl_temperature_raw_get(&dev_ctx, &temp_raw) != 0)
        return 4;

    // lis2mdl_from_lsb_to_mgauss returns milli-Gauss; convert to Gauss so
    // the attitude estimator receives "preferably Gauss" units directly.
    _raw.B_x = lis2mdl_from_lsb_to_mgauss(mag_raw[0]) * 0.001f;
    _raw.B_y = lis2mdl_from_lsb_to_mgauss(mag_raw[1]) * 0.001f;
    _raw.B_z = lis2mdl_from_lsb_to_mgauss(mag_raw[2]) * 0.001f;
    _raw.LIS2_Temperature = lis2mdl_from_lsb_to_celsius(temp_raw);

    return 0;
}

// ─── getRawData ───────────────────────────────────────────────────────────────

LIS2_Data Magnetometer::getRawData()
{
    return _raw;
}
