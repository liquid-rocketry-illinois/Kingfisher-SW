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
#include "Flight_Procedures.h"

void buzzerTOGGLE() {}
extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer
    //TEST test;

//#define FLIGHT_MODE_GLOBAL
#define GROUND_MODE_GLOBAL

#ifdef FLIGHT_MODE_GLOBAL
    /* Init code */

    FlightComputer FC;

    FC.Init();

    /* Infinite loop */
    for(;;)
    {
        FC.Update();
    }
#endif

#ifdef GROUND_MODE_GLOBAL
    /* Init code */

    GroundStation GS;

    GS.Init();

    /* Infinite loop */
    for(;;)
    {
        GS.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        osDelay(1);
    }
#endif

    /* USER CODE END StartDefaultTask */
}