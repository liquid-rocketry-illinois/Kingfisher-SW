#include "BMP3xx_platform.h"

extern "C" {
#include "bmp3.h"
}

/*
 * SPI read
 * Bosch SPI protocol:
 * bit7 = 1 for read
 */
int8_t bmp3_spi_read(uint8_t reg_addr,
                     uint8_t *data,
                     uint32_t len,
                     void *intf_ptr)
{
    if(intf_ptr == nullptr || data == nullptr || len == 0)
        return BMP3_E_NULL_PTR;

    bmp3_spi_intf *spi = (bmp3_spi_intf*)intf_ptr;

    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)spi->spi_handle;
    GPIO_TypeDef *cs_port   = (GPIO_TypeDef*)spi->cs_port;

    uint8_t addr = reg_addr | 0x80;   // read bit

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_RESET);

    if(HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);
        return BMP3_E_COMM_FAIL;
    }

    if(HAL_SPI_Receive(hspi, data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);
        return BMP3_E_COMM_FAIL;
    }

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);

    return BMP3_OK;
}

/*
 * SPI write
 * bit7 = 0 for write
 */
int8_t bmp3_spi_write(uint8_t reg_addr,
                      const uint8_t *data,
                      uint32_t len,
                      void *intf_ptr)
{
    if(intf_ptr == nullptr || data == nullptr || len == 0)
        return BMP3_E_NULL_PTR;

    bmp3_spi_intf *spi = (bmp3_spi_intf*)intf_ptr;

    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)spi->spi_handle;
    GPIO_TypeDef *cs_port   = (GPIO_TypeDef*)spi->cs_port;

    uint8_t addr = reg_addr & 0x7F;   // write bit

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_RESET);

    if(HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);
        return BMP3_E_COMM_FAIL;
    }

    if(HAL_SPI_Transmit(hspi, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);
        return BMP3_E_COMM_FAIL;
    }

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);

    return BMP3_OK;
}


/*
 * Microsecond delay function
 * Uses DWT cycle counter (best option on STM32H7)
 */
void bmp3_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000);

    while((DWT->CYCCNT - start) < ticks);
}