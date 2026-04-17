//
// Created by dyrel on 3/31/2026.
//

#include "BMI323_Platform.h"
#include "SIMU_BMI323.h"
#include <cstring>
#include "FreeRTOS.h"
#include "projdefs.h"
#include "spi.h"
#include "task.h"

// tx and rx data buffer pair for each sensor
uint8_t BMI3_GTXBufferA[512], BMI3_GRXBufferA[2048];
uint8_t BMI3_GTXBufferB[512], BMI3_GRXBufferB[2048];
uint8_t BMI3_GTXBufferC[512], BMI3_GRXBufferC[2048];

void BMI_CS_Select(BMIobjPtr* intf_ptr) {
    // ensure everything deselected before selecting only one. this is just a guard
    HAL_GPIO_WritePin(SIMU_CS1_GPIO_Port, SIMU_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIMU_CS2_GPIO_Port, SIMU_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIMU_CS3_GPIO_Port, SIMU_CS3_Pin, GPIO_PIN_SET);

    HAL_GPIO_WritePin(intf_ptr->cs_port, intf_ptr->cs_pin, GPIO_PIN_RESET);
}

void BMI_CS_Deselect(BMIobjPtr* intf_ptr) {
    // Easier to just deselect everything
    HAL_GPIO_WritePin(SIMU_CS1_GPIO_Port, SIMU_CS1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIMU_CS2_GPIO_Port, SIMU_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SIMU_CS3_GPIO_Port, SIMU_CS3_Pin, GPIO_PIN_SET);
}

int8_t BMI3_SPI_Read(uint8_t reg, uint8_t* data, uint32_t len, void *ptr)
{
    auto *intf_ptr = static_cast<BMIobjPtr*>(ptr);

    if (intf_ptr->cs_pin == SIMU_CS1_Pin) {
        BMI3_GTXBufferA[0] = reg | 0x80;

        BMI_CS_Select(intf_ptr);
        HAL_SPI_TransmitReceive(intf_ptr->SPIbus, BMI3_GTXBufferA, BMI3_GRXBufferA, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        BMI_CS_Deselect(intf_ptr);

        memcpy(data, BMI3_GRXBufferA+1, len);
    }
    else if (intf_ptr->cs_pin == SIMU_CS2_Pin) {
        BMI3_GTXBufferB[0] = reg | 0x80;

        BMI_CS_Select(intf_ptr);
        HAL_SPI_TransmitReceive(intf_ptr->SPIbus, BMI3_GTXBufferB, BMI3_GRXBufferB, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        BMI_CS_Deselect(intf_ptr);

        memcpy(data, BMI3_GRXBufferB+1, len);
    }
    else if (intf_ptr->cs_pin == SIMU_CS3_Pin) {
        BMI3_GTXBufferC[0] = reg | 0x80;

        BMI_CS_Select(intf_ptr);
        HAL_SPI_TransmitReceive(intf_ptr->SPIbus, BMI3_GTXBufferC, BMI3_GRXBufferC, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        BMI_CS_Deselect(intf_ptr);

        memcpy(data, BMI3_GRXBufferC+1, len);
    }
    return 0;
}

int8_t BMI3_SPI_Write(uint8_t reg, const uint8_t* data, uint32_t len, void *ptr) {
    auto *intf_ptr = static_cast<BMIobjPtr*>(ptr);

    if (intf_ptr->index == SENSOR1_I) {
        BMI3_GTXBufferA[0] = reg & 0x7F;
        memcpy(&BMI3_GTXBufferA[1], data, len);

        BMI_CS_Select(intf_ptr);
        HAL_SPI_Transmit(intf_ptr->SPIbus, BMI3_GTXBufferA, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        //HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);
        BMI_CS_Deselect(intf_ptr);
    }
    else if (intf_ptr->index == SENSOR2_I) {
        BMI3_GTXBufferB[0] = reg & 0x7F;
        memcpy(&BMI3_GTXBufferB[1], data, len);

        BMI_CS_Select(intf_ptr);
        HAL_SPI_Transmit(intf_ptr->SPIbus, BMI3_GTXBufferB, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        //HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);
        BMI_CS_Deselect(intf_ptr);
    }
    else if (intf_ptr->index == SENSOR3_I) {
        BMI3_GTXBufferC[0] = reg & 0x7F;
        memcpy(&BMI3_GTXBufferC[1], data, len);

        BMI_CS_Select(intf_ptr);
        HAL_SPI_Transmit(intf_ptr->SPIbus, BMI3_GTXBufferC, len+1, 5000);
        while(intf_ptr->SPIbus->State == HAL_SPI_STATE_BUSY) {}
        //HAL_SPI_Transmit(_spi, (uint8_t*)data, len, HAL_MAX_DELAY);
        BMI_CS_Deselect(intf_ptr);
    }
    return 0;
}

void BMI323_delay_us(uint32_t us, void *intf_ptr) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);

    while ((DWT->CYCCNT - start) < ticks);
}