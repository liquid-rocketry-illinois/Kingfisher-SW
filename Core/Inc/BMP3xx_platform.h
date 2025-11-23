#pragma once
#ifndef BMP3XX_PLATFORM_H
#define BMP3XX_PLATFORM_H

#include "..\..\Drivers\BMP390\bmp3.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C"
{
    #endif


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

/**
 * @brief Reads data from BMP390 sensor via SPI.
 * @param reg_addr Register address to read from.
 * @param data Pointer to buffer to store read data.
 * @param len Number of bytes to read.
 * @param intf_ptr Pointer to platform-specific SPI interface struct.
 * @return Status of read operation (0 for success, negative for error).
 */
int8_t bmp3_spi_read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *intf_ptr);

/**
 * @brief Writes data to BMP390 sensor via SPI.
 * @param reg_addr Register address to write to.
 * @param data Pointer to buffer containing data to write.
 * @param len Number of bytes to write.
 * @param intf_ptr Pointer to platform-specific SPI interface struct.
 * @return Status of write operation (0 for success, negative for error).
 */
int8_t bmp3_spi_write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *intf_ptr);

/**
 * @brief Delays execution for a specified number of microseconds.
 * @param period Number of microseconds to delay.
 * @param intf_ptr Pointer to platform-specific interface struct (unused).
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr);

/**
 * @brief Initializes the BMP390 interface.
 * @param dev Pointer to BMP390 device struct.
 * @param intf Interface type (e.g., SPI or I2C).
 * @return Status of initialization (0 for success, negative for error).
 */
int8_t bmp3_interface_init(struct bmp3_dev *dev, uint8_t intf);
/** 
 * @brief Initializes the BMP390 device interface.
 * @param dev Pointer to BMP390 device structure.
 * @param intf Interface type (e.g., SPI or I2C).
 * @return Status of initialization (0 for success, negative for error).
 */
int8_t bmp3_interface_init(struct bmp3_dev *dev, uint8_t intf);

#ifdef __cplusplus
}
#endif
#endif 