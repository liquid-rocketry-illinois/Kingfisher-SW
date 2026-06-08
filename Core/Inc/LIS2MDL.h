//
// Created by admin on 3/16/2026.
//

#ifndef KINGFISHER_SW_LIS2MDL_H
#define KINGFISHER_SW_LIS2MDL_H

#include "lis2mdl_reg.h"
#include "spi.h"
#include "main.h"

// Magnetic field in Gauss; temperature in degrees C.
typedef struct {
    float B_x              = 0.0F;
    float B_y              = 0.0F;
    float B_z              = 0.0F;
    float LIS2_Temperature = 0.0F;
} LIS2_Data;

class Magnetometer {
public:
    explicit Magnetometer(SPI_HandleTypeDef* spi);

    uint8_t   Init();
    uint8_t   Update();      // non-blocking DRDY check; retains last data if no new sample
    LIS2_Data getRawData();  // returns data cached by the most recent Update()

private:
    SPI_HandleTypeDef* _spi;
    stmdev_ctx_t       dev_ctx{};
    bool               _sensor_active = false;
    LIS2_Data          _raw;

    static int32_t platform_write(void* handle, uint8_t reg, const uint8_t* bufp, uint16_t len);
    static int32_t platform_read (void* handle, uint8_t reg, uint8_t* bufp,       uint16_t len);
    static void    platform_delay(uint32_t millisec);

    static void cs_low();
    static void cs_high();
};

#endif //KINGFISHER_SW_LIS2MDL_H
