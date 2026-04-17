//
// Created by dyrel on 4/4/2026.
//

#ifndef KINGFISHER_SW_ISM6_PLATFORM_H
#define KINGFISHER_SW_ISM6_PLATFORM_H

#include <stdint.h>
#include <cstring>
#include <spi.h>

inline uint8_t  ISM_GTX[512] = {0},
                ISM_GRX[2048] = {0};

void ISM_CS_Low();
void ISM_CS_High();
void Platform_Init();
int32_t ISM_SPI_Write(  void *handle,
                        uint8_t reg,
                        const uint8_t *bufp,
                        uint16_t len);
int32_t ISM_SPI_Read(   void *handle,
                        uint8_t reg,
                        uint8_t *bufp,
                        uint16_t len);
void ISM_platform_MSdelay(uint32_t ms);

#endif //KINGFISHER_SW_ISM6_PLATFORM_H