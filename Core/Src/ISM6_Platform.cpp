//
// Created by dyrel on 4/4/2026.
//

#include "ISM6_Platform.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

void ISM_CS_Low() {
    HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_RESET);
}

void ISM_CS_High() {
    HAL_GPIO_WritePin(CS_ISM_GPIO_Port, CS_ISM_Pin, GPIO_PIN_SET);
}

void Platform_Init() {
    // Nothing required if SPI and GPIO already initialized in CubeMX.
    // This is here in case init code is needed in addition to everything
    // here but nothing usually is needed... Continuing to monitor.
}

int32_t ISM_SPI_Write(void *handle,
                                    uint8_t reg,
                                    const uint8_t *bufp,
                                    uint16_t len) {
    SPI_HandleTypeDef *hspi = (SPI_HandleTypeDef*)handle;

    ISM_GTX[0] = reg & 0x7F;   // Write
    if (len > 1)
        ISM_GTX[0] |= 0x40;

    memcpy(&ISM_GTX[1], bufp, len);// Auto-increment

    ISM_CS_Low();
    HAL_SPI_Transmit(hspi, ISM_GTX, len+1, 5000);
    while(hspi->State == HAL_SPI_STATE_BUSY) {}
    ISM_CS_High();

    return 0;
}

int32_t ISM_SPI_Read(void *handle,
                                   uint8_t reg,
                                   uint8_t *bufp,
                                   uint16_t len) {
    auto *hspi = static_cast<SPI_HandleTypeDef *>(handle);

    ISM_GTX[0] = reg | 0x80;   // Read
    if (len > 1)
        ISM_GTX[0] |= 0x40;            // Auto-increment

    ISM_CS_Low();
    HAL_SPI_TransmitReceive(hspi, ISM_GTX, ISM_GRX, len+1, 5000);
    while(hspi->State == HAL_SPI_STATE_BUSY) {}
    ISM_CS_High();

    memcpy(bufp, ISM_GRX + 1, len);
    return 0;
}

void ISM_platform_MSdelay(uint32_t ms) {
    osDelay(ms);  // FreeRTOS delay, yields to scheduler
}