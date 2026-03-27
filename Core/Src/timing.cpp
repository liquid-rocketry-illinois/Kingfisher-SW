//
// Created by dyrel on 2/21/2026.
//
#include "timing.h"
#include "main.h"

uint32_t us_f;

void MICROS_DWT_DWT_Init() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT
    DWT->CYCCNT = 0;                                 // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              // Enable cycle counter
}

void MICROS_DWT_Timebase_Init() {
    us_f = HAL_RCC_GetHCLKFreq() / 1000000; // Precompute factor
    MICROS_DWT_DWT_Init(); // Init DWT
}

uint32_t millis(){
    return HAL_GetTick();
}

uint32_t micros()
{
    return DWT->CYCCNT / us_f;
}