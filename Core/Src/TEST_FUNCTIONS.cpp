//
// Created by dyrel on 2/23/2026.
//

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "TEST_FUNCTIONS.h"

#include "task.h"

TEST::TEST(){;}

//TODO
//Need to change code of PWM generation. verify correct pwm signal
int TEST::SERVO_TEST() {
    Servo_Axon_Mini_MKII servoSet;
    bool initState = servoSet.Init({0,0}, PRECISION::TENTH_DEGREE, true);
    vTaskDelay(500);

    // Track status of servos
    static int status = 0;
    if (status == 0 && initState) {
        while (1) {
            servoSet.Update(90.0F, -33.0F);
            vTaskDelay(1000);
            servoSet.Update(-45.0F, 45.0F);
            vTaskDelay(1000);
            servoSet.Update(0.0F, 0.0F);
        }
    }

    return status;
}

int TEST::IMU_TEST() {
    IMUs IMU_ENGINE(true);
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    IMUsStatus sensorStatuses = IMU_ENGINE.Init();

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        sensorStatuses = IMU_ENGINE.Update();
        vTaskDelay(1);
    }
}

int TEST::BARO_TEST()
{
    Baro_Unified BaroEngine;
    auto init = BaroEngine.Init(true);

    while (1)
    {
        auto status = BaroEngine.Update();
        auto data = BaroEngine.getData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        //vTaskDelay(100); TODO: vTaskDelay causing hard fault- memory bad? overflow? div by 0?
        HAL_Delay(100);
    }
}

int TEST::LIS2_TEST()
{
    Magnetometer mag;
    LIS2_Data data;
    uint8_t status = 0;
    uint8_t init = mag.Init();

    while (1)
    {
        status = mag.Update();
        data = mag.getRawData();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(100);
    }
}

int TEST::RADIO_TEST() {
    Telemetry telem = Telemetry(TelemetryMode::TELEMETRY_MODE_FLIGHT); // or TelemetryMode::TELEMETRY_MODE_FLIGHT
    uint8_t status = 0;
    uint8_t init = telem.Init();

    while (1) {
        status = telem.Update();
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(100);
    }
}

int TEST::PYRO_TEST()
{

    vTaskDelay(pdMS_TO_TICKS(1000));

    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
    vTaskDelay(pdMS_TO_TICKS(5000));
    HAL_GPIO_WritePin(DROUGE_MAIN_GPIO_Port, DROUGE_MAIN_Pin, GPIO_PIN_RESET);

    while (1) {
        HAL_GPIO_TogglePin(USR_LED_GPIO_Port, USR_LED_Pin);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

}

