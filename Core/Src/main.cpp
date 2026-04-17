//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "Buzzer.h"
#include "cmsis_os.h"
#include "TEST_FUNCTIONS.h"
#include "timing.h"
#include "fatfs.h"

void buzzerTOGGLE() {}

extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer

    TEST test;
    /* Infinite loop */
    for(;;)
    {
        test.SD_TEST();
        // wont get past here for tests
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}