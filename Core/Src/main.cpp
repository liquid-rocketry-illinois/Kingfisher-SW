//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "cmsis_os.h"

#include "Buzzer.h"
#include "GPS.h"

void buzzerTOGGLE() {}

extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        HAL_Delay(500L);

        TIM3->ARR = 19999L;
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

        for (int i = 500L; i < 2500; i+= 10) {
            TIM3->CCR2 = i;
            TIM3->CCR3 = i;
            HAL_Delay(1);
        }
        for (int i = 2500L; i > 500; i-= 10) {
            TIM3->CCR2 = i;
            TIM3->CCR3 = i;
            HAL_Delay(1);
        }


        //buzzerTOGGLE();
    }
    /* USER CODE END StartDefaultTask */
}