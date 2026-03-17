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
    bmp3_dev device;
    bool _sensor_active;
    BMP_INDEX InitDev;

    BMP_Data _raw;

    int8_t SPI_Read(uint8_t reg, uint8_t* data, uint16_t len);
    int8_t SPI_Write(uint8_t reg, const uint8_t* data, uint16_t len);
    void CS_Select();
    void CS_Deselect();
};

#endif //KINGFISHER_SW_BMP390_H