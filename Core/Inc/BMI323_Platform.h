//
// Created by dyrel on 3/31/2026.
//

#ifndef KINGFISHER_SW_BMI323_PLATFORM_H
#define KINGFISHER_SW_BMI323_PLATFORM_H

#include "bmi323.h"

void BMI_CS_Select();
void BMI_CS_DeSelect();

int8_t BMI3_SPI_Read(uint8_t reg, uint8_t* data, uint32_t len, void *ptr);
int8_t BMI3_SPI_Write(uint8_t reg, const uint8_t* data, uint32_t len, void *ptr);
void BMI323_delay_us(uint32_t microseconds, void *intf_ptr);

#endif //KINGFISHER_SW_BMI323_PLATFORM_H