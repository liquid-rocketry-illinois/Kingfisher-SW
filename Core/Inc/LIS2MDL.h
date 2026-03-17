//
// Created by admin on 3/16/2026.
//

#ifndef KINGFISHER_SW_LIS2MDL_H
#define KINGFISHER_SW_LIS2MDL_H

// We need this for the stmdev_ctx_t typedef
#include "ism6hg256x_reg.h"

// Now MEMS_SHARED_TYPES is defined
#include "lis2mdl_reg.h"
#include "spi.h"

#define SENSOR_BUS hspi1

typedef struct
{
    float B_x = 0.0F;
    float B_y = 0.0F;
    float B_z = 0.0F;
    float LIS2_Temperature = 0.0F;
} LIS2_Data;

class Magnetometer
{
    static int32_t platform_write( void *handle,
                            uint8_t reg,
                            const uint8_t *bufp,
                            uint16_t len);
    static int32_t platform_read(  void *handle,
                            uint8_t reg,
                            uint8_t *bufp,
                            uint16_t len);

    static void platform_delay(uint32_t millisec);

    static stmdev_ctx_t dev_ctx;

    static void cs_low();
    static void cs_high();

public:
    Magnetometer();
    uint8_t Init();
    uint8_t Update();
    LIS2_Data getRawData();
};

#endif //KINGFISHER_SW_LIS2MDL_H