//
// Created by dyrel on 2/7/2026.
//

#include "tim.h"
#include "Buzzer.h"
#include "cmsis_os.h"
#include "TEST_FUNCTIONS.h"
#include "timing.h"
#include "MAXM10S.h"
#include "i2c.h"
#include "TinyGPSPlus.h"
void buzzerTOGGLE() {}
extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer
    TEST test;
    MAXM10S gps(&hi2c4);
    MAXM10S::gpsData testingStruct;

    /* Infinite loop */
    for(;;)
    {
        if (gps.update() == 0) {
            testingStruct = gps.getData();
        }

        // wont get past here for tests
        vTaskDelay(100);
    }
    /* USER CODE END StartDefaultTask */
}