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

// ── Servo canard trim offsets — edit these before flashing the GND unit ───────
// These are transmitted to the FC once on GND reconnect and applied as the
// servo zero-point (degrees).  Positive = clockwise offset.
float GND_SERVO_OFFSET_S1 = 20.0f;
float GND_SERVO_OFFSET_S2 = 20.0f;
// ─────────────────────────────────────────────────────────────────────────────

void buzzerTOGGLE() {}
extern "C" void task(void*) {
    /* USER CODE BEGIN StartDefaultTask */
    MICROS_DWT_Timebase_Init(); // Initialize micros() timer
    TEST test;

    test.NoiseTest();

    FlightComputer FC;
    HAL_GPIO_WritePin(USR_LED_GPIO_Port, USR_LED_Pin, GPIO_PIN_SET);
    int8_t status = FC.Init();

    /* Infinite loop */
    for(;;)
    {
        status = FC.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    }
    /* USER CODE END StartDefaultTask */
}

extern "C" void task2(void*) {}