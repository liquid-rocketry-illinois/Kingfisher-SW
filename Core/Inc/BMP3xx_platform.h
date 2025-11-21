#pragma once
#ifndef BMP3XX_PLATFORM_H
#define BMP3XX_PLATFORM_H

#include "..\..\Drivers\BMP390\bmp3.h"
#include <stdint.h>

/*
 * PLATFORM-SPECIFIC INTERFACE STRUCT
 * -----------------------------------
 * Fill in the actual members when hardware is decided.
 * For now this is an opaque pointer used by SPI read/write methods.
 */

typedef struct
{
    void *spi_handle;     // Replace with SPI_HandleTypeDef*
    void *cs_port;        // Replace with GPIO_TypeDef*
    uint16_t cs_pin;      // Replace with actual pin number
} bmp3_spi_intf;

/*
 * Function prototypes required by Bosch BMP390 driver
 */
int8_t bmp3_spi_read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *intf_ptr);

int8_t bmp3_spi_write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *intf_ptr);

void bmp3_delay_us(uint32_t period, void *intf_ptr);

#endif