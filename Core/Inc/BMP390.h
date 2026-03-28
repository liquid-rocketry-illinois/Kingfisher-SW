//
// Created by admin on 3/16/2026.
//

#ifndef KINGFISHER_SW_BMP390_H
#define KINGFISHER_SW_BMP390_H

#include "BMP3xx_platform.h"
#include "bmp3.h"

typedef struct
{
    float heightMeters = 0.0F;
    float Pressure = 0.0F;
    float Temperature = 0.0F;
} BMP_Data;

class Barometer
{
public:
    enum BMP_INDEX
    {
        SENSOR1_I,
        SENSOR2_I,
        SENSOR3_I
    };

    Barometer(SPI_HandleTypeDef* spi, BMP_INDEX DeviceNum);
    uint8_t Init();
    uint8_t Update();
    BMP_Data getRawData();

private:
    SPI_HandleTypeDef* _spi;
    bmp3_spi_intf spi_intf;
    bmp3_dev device;
    bool _sensor_active;
    BMP_INDEX InitDev;

    // config stuff in init and in update too
    bmp3_settings settings = {0};
    uint16_t settings_sel;
    bmp3_status status = {{0}};

    BMP_Data _raw;
};

#endif //KINGFISHER_SW_BMP390_H