//
// Created by chris on 4/1/2026.
//

#include "MAXM10S.h"




MAXM10S::MAXM10S(I2C_HandleTypeDef *hi2c) {
    _hi2c = hi2c;
}

uint16_t MAXM10S::getAvailableBytes() {
    uint8_t buffer[2] = {0}; // To store high and low bits
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(_hi2c, I2C_ADDRESS, REG_BYTES_AVAIL, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
    if (status == HAL_OK) {
        return (buffer[0] << 8) | buffer[1];
    }
return 0;
}

uint16_t MAXM10S::readGPS(uint16_t numBytes, uint8_t *buffer) {
    if (numBytes == 0) {
        return -1;
    }

    HAL_I2C_Mem_Read(_hi2c, I2C_ADDRESS, REG_DATA_STREAM, I2C_MEMADD_SIZE_8BIT, buffer, numBytes, 1000);
    return 0; // For successful read
}





