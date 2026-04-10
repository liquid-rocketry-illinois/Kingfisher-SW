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
    TinyGPSPlus gpsParser;
    osDelay(1000);
    static uint8_t gpsBuffer[2048]; // Buffer to hold the incoming data

    /* Infinite loop */
    for(;;)
    {
        uint16_t bytesWaiting = gps.getAvailableBytes();
        if (bytesWaiting > 50) {
            if (bytesWaiting > sizeof(gpsBuffer)) {
                bytesWaiting = sizeof(gpsBuffer);
            }
            if (gps.readGPS(bytesWaiting, gpsBuffer) > 0)
            {
                for (int i = 0; i < bytesWaiting; i++) {
                    gpsParser.encode(gpsBuffer[i]);

                    if (gpsParser.location.isUpdated() && gpsParser.location.isValid())
                    {
                        double latitude = gpsParser.location.lat();
                        double longitude = gpsParser.location.lng();
                    }
                    if (gpsParser.altitude.isUpdated() && gpsParser.altitude.isValid())
                    {
                        double altitudeMeters = gpsParser.altitude.meters();
                    }
                }
            }
        }
        // wont get past here for tests
        vTaskDelay(100);
    }
    /* USER CODE END StartDefaultTask */
}