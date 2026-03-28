#include "BMP3xx_platform.h"
#include <cstring>
#include "FreeRTOS.h"
#include "projdefs.h"
#include "spi.h"
#include "task.h"

// tx and rx data buffers
uint8_t GTXBuffer[512], GRXBuffer[2048];

const void BMP_CS_Select(bmp3_spi_intf* intf_ptr) {
    // ensure everything deselected before selecting only one. this is just a guard
    HAL_GPIO_WritePin(BMP390_CS1_GPIO_Port, BMP390_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS2_GPIO_Port, BMP390_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS3_GPIO_Port, BMP390_CS3_Pin, GPIO_PIN_SET);

    vTaskDelay(pdMS_TO_TICKS(5));
    HAL_GPIO_WritePin(intf_ptr->cs_port, intf_ptr->cs_pin, GPIO_PIN_RESET);
}

const void BMP_CS_Deselect(bmp3_spi_intf* intf_ptr) {
    // Easier to just deselect everything
    HAL_GPIO_WritePin(BMP390_CS1_GPIO_Port, BMP390_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS2_GPIO_Port, BMP390_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMP390_CS3_GPIO_Port, BMP390_CS3_Pin, GPIO_PIN_SET);

    vTaskDelay(pdMS_TO_TICKS(5));
}

/*
 * SPI read
 * Bosch SPI protocol:
 * bit7 = 1 for read
 */ // TODO: make read and write discoverable within bmp390 class init()
int8_t BMP_SPI_Read(uint8_t reg, uint8_t* data, uint32_t len, bmp3_spi_intf *intf_ptr)
{
    GTXBuffer[0] = reg | 0x80;

    BMP_CS_Select(intf_ptr);
    HAL_SPI_TransmitReceive(intf_ptr->spi_handle, GTXBuffer, data, len+1, 5000);
    while(intf_ptr->spi_handle->State == HAL_SPI_STATE_BUSY) {}
    BMP_CS_Deselect(intf_ptr);

    memcpy(data, GRXBuffer+1, len);

    return 0;
}

int8_t BMP_SPI_Write(uint8_t reg, const uint8_t* data, uint32_t len, bmp3_spi_intf *intf_ptr) { // bmp3_spi_intf *spi_ptr?
    GTXBuffer[0] = reg & 0x7F;
    memcpy(&GTXBuffer[1], data, len);

    BMP_CS_Select(intf_ptr);
    HAL_SPI_Transmit(intf_ptr->spi_handle, GTXBuffer, len+1, 5000);
    while(intf_ptr->spi_handle->State == HAL_SPI_STATE_BUSY) {}
    //HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);
    BMP_CS_Deselect(intf_ptr);

    return 0;
}

void BMP_delay_us(uint32_t us, void *intf_ptr) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}