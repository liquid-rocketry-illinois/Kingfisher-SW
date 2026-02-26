#ifndef KINGFISHER_SW_MILLIS_H
#define KINGFISHER_SW_MILLIS_H

#include "stm32h7xx_hal.h"
#include "core_cm7.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

    void MICROS_DWT_DWT_Init(void);
    void MICROS_DWT_Timebase_Init(void);
    uint32_t millis(void);
    uint32_t micros(void);

#ifdef __cplusplus
}
#endif

#endif