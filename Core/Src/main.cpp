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
    GnssSensor MAXM10S = GnssSensor();
    static GnssData GPS_Data;
    MAXM10S.Init();
    osDelay(1000); // Give time for inits
    /* Infinite loop */
    for(;;)
    {
        osDelay(1);

        // LED test code
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        HAL_Delay(500L);

        // GPS test code
        if (MAXM10S.Update()) GPS_Data = MAXM10S.GetData();

        // Servo test code

        // TIM3->ARR = 19999L;
        // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        //
        // for (int i = 500L; i < 2500; i+= 10) {
        //     TIM3->CCR2 = i;
        //     TIM3->CCR3 = i;
        //     HAL_Delay(1);
        // }
        // for (int i = 2500L; i > 500; i-= 10) {
        //     TIM3->CCR2 = i;
        //     TIM3->CCR3 = i;
        //     HAL_Delay(1);
        // }


        //buzzerTOGGLE();
    }
    /* USER CODE END StartDefaultTask */
}