//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "Buzzer.h"
#include "cmsis_os.h"
#include "TEST_FUNCTIONS.h"

void buzzerTOGGLE() {}

extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    TEST test;

    for(;;)
    {
        test.ISM_TEST();
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}