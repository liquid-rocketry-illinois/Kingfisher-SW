//
// Created by chris on 4/1/2026.
//

#ifndef KINGFISHER_SW_MAXM10S_H
#define KINGFISHER_SW_MAXM10S_H

#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "TinyGPSPlus.h"



class MAXM10S {

public:
    struct gpsData {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;

        uint8_t hour = 0;
        uint8_t minute = 0;
        uint8_t second = 0;

        uint8_t isLocated = false;
        uint8_t isAltituded = false;
        uint8_t isTimed = false;
    };

    MAXM10S();
    uint8_t Init(I2C_HandleTypeDef *hi2c);  // sets I2C handle and releases reset
    uint8_t update();
    gpsData getData();
    uint8_t gpsBuffer[2048];



private:
    I2C_HandleTypeDef *_hi2c;
    TinyGPSPlus gpsParser;
    static const uint16_t I2C_ADDRESS = 0x84;
    static const uint16_t REG_BYTES_AVAIL = 0xFD;
    static const uint16_t REG_DATA_STREAM = 0xFF;
    uint16_t getAvailableBytes();
    uint16_t readGPS(uint16_t numBytes, uint8_t* buffer);



    gpsData gpsDataStruct;
};




#endif //KINGFISHER_SW_MAXM10S_H