//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "Buzzer.h"
#include "cmsis_os.h"
#include "TEST_FUNCTIONS.h"
#include "timing.h"

void buzzerTOGGLE() {}

extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer

    TEST test;
    /* Infinite loop */
    for(;;)
    {
        //final loop goes here!!
    }
    /* USER CODE END StartDefaultTask */
}