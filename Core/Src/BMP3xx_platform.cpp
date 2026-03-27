#include "BMP3xx_platform.h"
#include <cstring>

extern "C" {
#include "bmp3.h"
}

// To handle input
uint8_t addr[512];
uint8_t GRXBuffer[2048];

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

    addr[0] = reg_addr | 0x80;   // read bit

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_RESET);

    // timeout of 1000ms is still very generous
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, addr, GRXBuffer, len+1, 1000);
    while(hspi->State == HAL_SPI_STATE_BUSY);  // wait for xmission complete
    if(status != HAL_OK)
    {
        HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);
        return BMP3_E_COMM_FAIL;
    }

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_SET);

    memcpy(data, GRXBuffer+1, len);
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

    addr[0] = reg_addr & 0x7F;   // write bit
    memcpy(&addr[1], data, len);

    HAL_GPIO_WritePin(cs_port, spi->cs_pin, GPIO_PIN_RESET);

    // timeout delay of 1000ms is pretty generous
    int8_t status = HAL_SPI_Transmit(hspi, addr, len+1, 1000);
    while(hspi->State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    if(status != HAL_OK)
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
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = period * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}