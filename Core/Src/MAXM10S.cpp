//
// Created by chris on 4/1/2026.
//

#include "MAXM10S.h"
#include "i2c.h"

// update returns int8_t 0 if functional or something else
// getdata returns a struct of time, longitude, latitude, altitude


MAXM10S::MAXM10S(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;
    HAL_GPIO_WritePin(GPS_RST_GPIO_Port, GPS_RST_Pin, GPIO_PIN_SET);
}

uint16_t MAXM10S::getAvailableBytes() {
    uint8_t buffer[2] = {0}; // To store high and low bits
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_BYTES_AVAIL, I2C_MEMADD_SIZE_8BIT, buffer, 2, 1000);
    if (status == HAL_OK) {
        return (buffer[0] << 8) | buffer[1];
    }
return 0;
}

uint16_t MAXM10S::readGPS(uint16_t numBytes, uint8_t *buffer) {
    if (numBytes == 0) {
        return 2;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_DATA_STREAM, I2C_MEMADD_SIZE_8BIT, buffer, numBytes+1, 1000);

    if (status == HAL_OK) {
        return 1; // The buffer is now full of GPS data!
    }
    return 0; // should not get this
}

uint8_t MAXM10S::update() {
    uint8_t newData = 0;
    uint16_t bytesWaiting = getAvailableBytes();
    if (bytesWaiting > 50) {
        if (bytesWaiting > sizeof(gpsBuffer)) {
            bytesWaiting = sizeof(gpsBuffer)-1;
        }
        if (readGPS(bytesWaiting, gpsBuffer) > 0) {
            for (int i = 0; i < bytesWaiting; i++) {
                if (gpsBuffer[i] != 0xff && gpsBuffer[i] != 0x00) {
                    gpsParser.encode(gpsBuffer[i]);
                }
            }
            gpsDataStruct.isLocated = gpsParser.location.isValid();
            gpsDataStruct.isAltituded = gpsParser.altitude.isValid();
            gpsDataStruct.isTimed = gpsParser.time.isValid();

            // location
            if (gpsParser.location.isUpdated() && gpsDataStruct.isLocated) {
                gpsDataStruct.latitude = gpsParser.location.lat();
                gpsDataStruct.longitude = gpsParser.location.lng();
                newData = 1;
            }

            // altitude
            if (gpsParser.altitude.isUpdated() && gpsDataStruct.isAltituded) {
                gpsDataStruct.altitude = gpsParser.altitude.meters();
                newData = 1;
            }

            // time
            if (gpsParser.time.isUpdated() && gpsDataStruct.isTimed) {
                gpsDataStruct.hour = gpsParser.time.hour();
                gpsDataStruct.minute = gpsParser.time.minute();
                gpsDataStruct.second = gpsParser.time.second();
                newData = 1;
            }
            if (newData == 1) {
                return 0; // new data read!
            }
            return 3; // valid read, no complete sentences

        }

        return 1;  // readGPS failure
    }
    return 2; // not enough bytes
}

MAXM10S::gpsData MAXM10S::readGPS() {
    return gpsDataStruct;
}








