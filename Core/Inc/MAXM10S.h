//
// Created by chris on 4/1/2026.
//

#ifndef KINGFISHER_SW_MAXM10S_H
#define KINGFISHER_SW_MAXM10S_H

#include "stdint.h"
#include "stm32h7xx_hal.h"



class MAXM10S {
public:
    MAXM10S(I2C_HandleTypeDef *hi2c);
    uint16_t getAvailableBytes();
    uint16_t readGPS(uint16_t numBytes, uint8_t* buffer);
private:
    I2C_HandleTypeDef *_hi2c;
    static const uint16_t I2C_ADDRESS = 0x84;
    static const uint16_t REG_BYTES_AVAIL = 0xFD;
    static const uint16_t REG_DATA_STREAM = 0xFF;
};

struct RocketData
{
    double latitude=0.0;
    double longitude=0.0;
    double altitude=0.0;
    bool islocated=false;
    bool isaltituded=false;
};
#endif //KINGFISHER_SW_MAXM10S_H