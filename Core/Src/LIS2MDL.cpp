//
// Created by admin on 3/16/2026.
//

#include "LIS2MDL.h"
#include "stm32h7xx_hal.h"   // or your specific HAL header

// To use memcpy()
#include <string.h>

// Define static context
stmdev_ctx_t Magnetometer::dev_ctx;

// Chip select pin (EDIT to match your board)
#define LIS2MDL_CS_GPIO_Port GPIOA
#define LIS2MDL_CS_Pin       GPIO_PIN_4

//---------------------------------------------
// Platform functions (HAL bindings)
//---------------------------------------------
void Magnetometer::platform_delay(uint32_t millisec)
{
    HAL_Delay(millisec);
}

int32_t Magnetometer::platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len)
{
    uint8_t tx_buf[len + 1];
    tx_buf[0] = reg & 0x7F; // write = MSB = 0
    memcpy(&tx_buf[1], bufp, len);

    cs_low();
    HAL_StatusTypeDef ret = HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, tx_buf, len + 1, HAL_MAX_DELAY);
    cs_high();

    return (ret == HAL_OK) ? 0 : -1;
}

int32_t Magnetometer::platform_read(void* handle, uint8_t reg, uint8_t* bufp, uint16_t len)
{
    uint8_t reg_addr = reg | 0x80; // read = MSB = 1

    cs_low();
    HAL_SPI_Transmit((SPI_HandleTypeDef*)handle, &reg_addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive((SPI_HandleTypeDef*)handle, bufp, len, HAL_MAX_DELAY);
    cs_high();

    return 0;
}

//---------------------------------------------
// Chip Select control
//---------------------------------------------
void Magnetometer::cs_low()
{
    HAL_GPIO_WritePin(LIS2MDL_CS_GPIO_Port, LIS2MDL_CS_Pin, GPIO_PIN_RESET);
}

void Magnetometer::cs_high()
{
    HAL_GPIO_WritePin(LIS2MDL_CS_GPIO_Port, LIS2MDL_CS_Pin, GPIO_PIN_SET);
}

//---------------------------------------------
// Constructor
//---------------------------------------------
Magnetometer::Magnetometer()
{
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg  = platform_read;
    dev_ctx.mdelay    = platform_delay;
    dev_ctx.handle    = &SENSOR_BUS;
}

//---------------------------------------------
// Initialization
//---------------------------------------------
uint8_t Magnetometer::Init()
{
    uint8_t whoamI = 0;

    // Check device ID
    lis2mdl_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2MDL_ID)
        return 1; // fail

    // Reset device
    lis2mdl_reset_set(&dev_ctx, PROPERTY_ENABLE);

    uint8_t rst;
    do {
        lis2mdl_reset_get(&dev_ctx, &rst);
    } while (rst);

    // Enable Block Data Update
    lis2mdl_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

    // Set output data rate (e.g. 10 Hz)
    lis2mdl_data_rate_set(&dev_ctx, LIS2MDL_ODR_10Hz);

    // Enable temperature compensation
    lis2mdl_offset_temp_comp_set(&dev_ctx, PROPERTY_ENABLE);

    // Continuous conversion mode
    lis2mdl_operating_mode_set(&dev_ctx, LIS2MDL_CONTINUOUS_MODE);

    return 0; // success
}

//---------------------------------------------
// Update (read new data)
//---------------------------------------------
uint8_t Magnetometer::Update()
{
    uint8_t drdy;
    lis2mdl_mag_data_ready_get(&dev_ctx, &drdy);

    if (!drdy)
        return 1; // no new data

    return 0;
}

//---------------------------------------------
// Get raw data
//---------------------------------------------
LIS2_Data Magnetometer::getRawData()
{
    LIS2_Data data;

    int16_t mag_raw[3];
    int16_t temp_raw;

    // Read magnetometer
    lis2mdl_magnetic_raw_get(&dev_ctx, mag_raw);

    // Read temperature
    lis2mdl_temperature_raw_get(&dev_ctx, &temp_raw);

    // Convert to physical units
    data.B_x = lis2mdl_from_lsb_to_mgauss(mag_raw[0]);
    data.B_y = lis2mdl_from_lsb_to_mgauss(mag_raw[1]);
    data.B_z = lis2mdl_from_lsb_to_mgauss(mag_raw[2]);

    data.LIS2_Temperature = lis2mdl_from_lsb_to_celsius(temp_raw);

    return data;
}