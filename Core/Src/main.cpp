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
void buzzerTOGGLE() {}

extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer
    TEST test;
    MAXM10S gps(&hi2c4);
    uint8_t gpsBuffer[1024]; // Buffer to hold the incoming data

    /* Infinite loop */
    for(;;)
    {
        for (int i = 0; i < 3; i++) {
            HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        }
        uint16_t bytesWaiting = gps.getAvailableBytes();
        if (bytesWaiting > 0) {
            if (bytesWaiting > sizeof(gpsBuffer)) {
                bytesWaiting = sizeof(gpsBuffer);
            }
            gps.readGPS(bytesWaiting, gpsBuffer);
        }
        // wont get past here for tests
        osDelay(1);
    }
    /* USER CODE END StartDefaultTask */
}