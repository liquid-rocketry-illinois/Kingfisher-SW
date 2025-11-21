#include <cstdint>
#include "..\Inc\BMP3xx_platform.h"   // not .hpp
extern "C" {
#include "../../Drivers/BMP390/bmp3.h"
}



/*
 * Note:
 *  - Replace the TODO blocks with your actual SPI transmit/receive calls
 *    and your CS pin toggling once hardware is finalized.
 *  - The logic is correct for the BMP390 SPI interface.
 */

int8_t bmp3_spi_read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *intf_ptr)
{
    bmp3_spi_intf *spi = (bmp3_spi_intf *)intf_ptr;

    // SPI read: MSB = 1
    uint8_t addr = reg_addr | 0x80;

    // --- Chip Select LOW ---
    // TODO: set CS pin LOW using your GPIO API

    // TODO: Send addr over SPI using your SPI transmit function
    // Example (when implemented):
    // platform_spi_transmit(spi->spi_handle, &addr, 1);

    // TODO: Receive bytes from SPI
    // platform_spi_receive(spi->spi_handle, data, len);

    // --- Chip Select HIGH ---
    // TODO: set CS pin HIGH

    // Return success for now (replace with real error code later)
    return BMP3_OK;
}


int8_t bmp3_spi_write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *intf_ptr)
{
    bmp3_spi_intf *spi = (bmp3_spi_intf *)intf_ptr;

    // SPI write: MSB = 0
    uint8_t addr = reg_addr & 0x7F;

    // --- Chip Select LOW ---
    // TODO: set CS LOW

    // TODO: transmit address
    // platform_spi_transmit(spi->spi_handle, &addr, 1);

    // TODO: transmit payload
    // platform_spi_transmit(spi->spi_handle, data, len);

    // --- Chip Select HIGH ---
    // TODO: set CS HIGH

    return BMP3_OK;
}


void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    // TODO:
    // Provide a microsecond-level delay function.
    //
    // If unavailable, you can approximate using milliseconds:
    //
    // platform_delay_ms((period + 999) / 1000);
    //
    // For now this is empty.
}
