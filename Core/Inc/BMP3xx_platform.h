#pragma once
#ifndef BMP3XX_PLATFORM_H
#define BMP3XX_PLATFORM_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

typedef struct
{
    SPI_HandleTypeDef* spi_handle;     // Replace with SPI_HandleTypeDef*
    GPIO_TypeDef* cs_port;        // Replace with GPIO_TypeDef*
    uint16_t cs_pin;      // Replace with actual pin number
} bmp3_spi_intf;

/*
 * Function prototypes required by Bosch BMP390 driver
 */
int8_t BMP_SPI_Read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *intf_ptr);

int8_t BMP_SPI_Write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *intf_ptr);

void BMP_delay_us(uint32_t period, void *intf_ptr);

#endif